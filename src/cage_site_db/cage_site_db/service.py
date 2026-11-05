import cgi
import csv
import io
import json
import math
import os
import shutil
import tempfile
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image, ImageDraw, ImageFont

from . import SAFETY_MODE, __version__, config
from .db import dump, init_db, load, now, row, rows, transaction, uid
from .importers.coordinate_csv import import_coordinate_csv, import_subcage_csv
from .importers.exif_importer import import_photo
from .processing.geometry import enu_xy, haversine_m, valid_coordinate

WWW = os.path.join(os.path.dirname(os.path.dirname(__file__)), "www")


def _decode(record):
    if not record:
        return record
    for key in ("metadata_json", "quality_flags", "evidence_json", "warnings_json", "result_json"):
        if key in record:
            record[key[:-5] if key.endswith("_json") else key] = load(record[key], [] if key in ("quality_flags", "warnings_json") else {})
    return record


def _site_map():
    with transaction() as conn:
        photos = rows(conn.execute("SELECT s.*,a.original_name,a.source_role FROM gps_samples s JOIN source_assets a ON a.id=s.source_asset_id WHERE s.origin_type='PHOTO_EXIF'"))
        approved = rows(conn.execute("""SELECT p.*,c.label,c.site_id,q.calculation_method AS source_method
            FROM approved_point_versions p JOIN subcages c ON c.id=p.subcage_id
            JOIN sites g ON g.id=c.site_id
            LEFT JOIN candidate_points q ON q.id=p.candidate_point_id
            WHERE p.active=1 AND c.status='ACTIVE' AND g.status='ACTIVE'"""))
        routes = rows(conn.execute("SELECT * FROM route_previews ORDER BY created_at DESC LIMIT 20"))
        for route in routes:
            route["points"] = rows(conn.execute("SELECT * FROM route_preview_points WHERE route_id=? ORDER BY seq", (route["id"],)))
    return {"mode": SAFETY_MODE, "photos": photos, "approved": approved, "routes": routes}


def _route(route_id):
    with transaction() as conn:
        route = row(conn.execute("SELECT * FROM route_previews WHERE id=?", (route_id,)))
        if not route:
            return None
        route = _decode(route)
        route["preview_only"] = bool(route["preview_only"])
        route["realflight_eligible"] = False
        route["include_return"] = bool(route["include_return"])
        route["points"] = rows(conn.execute("SELECT * FROM route_preview_points WHERE route_id=? ORDER BY seq", (route_id,)))
        return route


def create_route(payload):
    subcage_ids = payload.get("subcage_ids") or []
    if not isinstance(subcage_ids, list) or not subcage_ids:
        raise ValueError("ROUTE_TARGETS_REQUIRED")
    home = payload.get("home") or {}
    home_ok = valid_coordinate(home.get("lat"), home.get("lon"))
    include_return = bool(payload.get("include_return", True))
    points, warnings = [], ["PREVIEW_ONLY", "REALFLIGHT_INELIGIBLE"]
    site_id = payload.get("site_id")
    with transaction() as conn:
        for subcage_id in subcage_ids:
            approved = row(conn.execute("SELECT p.*,s.label,s.site_id FROM approved_point_versions p JOIN subcages s ON s.id=p.subcage_id WHERE p.subcage_id=? AND p.active=1 ORDER BY p.version DESC LIMIT 1", (subcage_id,)))
            if not approved:
                raise ValueError("APPROVED_POINT_REQUIRED:%s" % subcage_id)
            if site_id and approved["site_id"] != site_id:
                raise ValueError("ROUTE_SITE_MISMATCH")
            site_id = approved["site_id"]
            points.append({"type": "TARGET", "subcage_id": subcage_id, "label": approved["label"],
                           "lat": approved["lat"], "lon": approved["lon"], "approved_version_id": approved["id"]})
    ordered = []
    if home_ok:
        ordered.append({"type": "HOME", "subcage_id": None, "label": "HOME", "lat": float(home["lat"]), "lon": float(home["lon"]), "approved_version_id": None})
    else:
        warnings.append("HOME_NOT_SET_NO_RETURN_SEGMENT")
    ordered.extend(points)
    if include_return and home_ok:
        ordered.append({"type": "RETURN_PREVIEW", "subcage_id": None, "label": "RETURN HOME", "lat": float(home["lat"]), "lon": float(home["lon"]), "approved_version_id": None})
    total = 0.0
    for index, point in enumerate(ordered):
        point["segment_distance_m"] = 0.0 if index == 0 else haversine_m(ordered[index-1]["lat"], ordered[index-1]["lon"], point["lat"], point["lon"])
        total += point["segment_distance_m"]
    route_id = uid("route")
    with transaction() as conn:
        conn.execute("INSERT INTO route_previews VALUES (?,?,?,?,?,?,?,?,?,?,?)", (
            route_id, site_id, str(payload.get("name") or "K9 route preview"),
            float(home["lat"]) if home_ok else None, float(home["lon"]) if home_ok else None,
            int(include_return), 1, 0, total, dump(warnings), now()))
        for seq, point in enumerate(ordered):
            conn.execute("INSERT INTO route_preview_points VALUES (?,?,?,?,?,?,?,?,?)", (
                route_id, seq, point["type"], point["subcage_id"], point["label"], point["lat"], point["lon"],
                point["segment_distance_m"], point["approved_version_id"]))
    return _route(route_id)


def route_csv(route):
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["PREVIEW_ONLY", "realflight_eligible=false", "route_id", route["id"]])
    writer.writerow(["seq", "point_type", "subcage_id", "label", "latitude", "longitude", "segment_distance_m", "approved_version_id"])
    for point in route["points"]:
        writer.writerow([point[key] for key in ("seq", "point_type", "subcage_id", "label", "lat", "lon", "segment_distance_m", "approved_version_id")])
    return output.getvalue().encode("utf-8-sig")


def route_geojson(route):
    coords = [[p["lon"], p["lat"]] for p in route["points"]]
    features = [{"type": "Feature", "properties": {"route_id": route["id"], "preview_only": True,
                 "realflight_eligible": False, "total_distance_m": route["total_distance_m"]},
                 "geometry": {"type": "LineString", "coordinates": coords}}]
    for point in route["points"]:
        features.append({"type": "Feature", "properties": {"seq": point["seq"], "label": point["label"], "point_type": point["point_type"], "preview_only": True},
                         "geometry": {"type": "Point", "coordinates": [point["lon"], point["lat"]]}})
    return json.dumps({"type": "FeatureCollection", "features": features}, ensure_ascii=False).encode("utf-8")


def route_png(route):
    width, height, margin = 1280, 800, 90
    image = Image.new("RGB", (width, height), "#071019")
    draw = ImageDraw.Draw(image)
    points = route["points"]
    if not points:
        draw.text((50, 50), "PREVIEW_ONLY - EMPTY", fill="#ff5c65")
    else:
        ref_lat = sum(p["lat"] for p in points) / len(points); ref_lon = sum(p["lon"] for p in points) / len(points)
        xy = [enu_xy(p["lat"], p["lon"], ref_lat, ref_lon) for p in points]
        xs, ys = [p[0] for p in xy], [p[1] for p in xy]
        span_x, span_y = max(max(xs)-min(xs), 10), max(max(ys)-min(ys), 10)
        scale = min((width-2*margin)/span_x, (height-2*margin)/span_y)
        pixel = [(width/2 + x*scale, height/2 - y*scale) for x, y in xy]
        for n in range(-5, 6):
            x = width/2+n*50; y=height/2+n*50
            draw.line((x, margin, x, height-margin), fill="#183044"); draw.line((margin, y, width-margin, y), fill="#183044")
        if len(pixel) > 1:
            draw.line(pixel, fill="#ffb020", width=5)
        for idx, ((x, y), point) in enumerate(zip(pixel, points)):
            color = "#2bd98f" if point["point_type"] == "TARGET" else "#4eb5ff"
            draw.ellipse((x-8,y-8,x+8,y+8), fill=color)
            draw.text((x+12,y-10), "%d %s" % (idx, point["label"]), fill="#eaf2f8")
        draw.line((width-70,100,width-70,45),fill="#fff",width=3); draw.polygon([(width-70,30),(width-78,48),(width-62,48)],fill="#fff")
        draw.text((width-84,110),"N",fill="#fff")
    draw.text((30, 25), "CAGE SITE DB  |  PREVIEW_ONLY  |  NOT FOR FLIGHT", fill="#ff6972")
    draw.text((30, height-40), "route=%s  distance=%.1f m  realflight_eligible=false" % (route["id"], route["total_distance_m"]), fill="#8fa5b5")
    output = io.BytesIO(); image.save(output, "PNG"); return output.getvalue()


class Handler(BaseHTTPRequestHandler):
    server_version = "CageSiteDB/%s" % __version__

    def send_body(self, code, ctype, body, headers=None):
        if isinstance(body, str): body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.send_header("X-Safety-Mode", SAFETY_MODE)
        self.send_header("X-Content-Type-Options", "nosniff")
        for key, value in (headers or {}).items(): self.send_header(key, value)
        self.end_headers(); self.wfile.write(body)

    def json(self, value, code=200):
        self.send_body(code, "application/json; charset=utf-8", json.dumps(value, ensure_ascii=False, default=str))

    def body_json(self):
        length = int(self.headers.get("Content-Length", 0))
        if length > 2 * 1024 * 1024: raise ValueError("JSON_BODY_TOO_LARGE")
        return json.loads(self.rfile.read(length).decode("utf-8")) if length else {}

    def multipart(self, max_bytes):
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0 or length > max_bytes: raise ValueError("UPLOAD_SIZE_INVALID")
        form = cgi.FieldStorage(fp=self.rfile, headers=self.headers, environ={"REQUEST_METHOD": "POST", "CONTENT_TYPE": self.headers.get("Content-Type"), "CONTENT_LENGTH": str(length)})
        fields = form["files"] if "files" in form else form["file"] if "file" in form else []
        return form, fields if isinstance(fields, list) else [fields]

    def import_files(self, kind):
        limits = {"photos": config.MAX_PHOTO_BYTES * config.MAX_BATCH_FILES}
        form, fields = self.multipart(limits[kind])
        if not fields or len(fields) > config.MAX_BATCH_FILES: raise ValueError("UPLOAD_BATCH_INVALID")
        job_id = uid("job"); results = []
        with transaction() as conn:
            conn.execute("INSERT INTO import_jobs VALUES (?,?,?,?,?,?,?,?,?)", (job_id, kind, "RUNNING", now(), None, len(fields), 0, 0, "{}"))
        for field in fields:
            name = os.path.basename(field.filename or "upload")
            suffix = os.path.splitext(name)[1].lower()
            fd, temp = tempfile.mkstemp(prefix="upload_", suffix=suffix, dir=os.path.join(config.ROOT, "incoming", "photos"))
            os.close(fd)
            try:
                with open(temp, "wb") as target: shutil.copyfileobj(field.file, target, 1024 * 1024)
                source_role = form.getfirst("source_role") or "WEB_UPLOAD"
                result = import_photo(temp, name, source_role, form.getfirst("session_id") or None)
                results.append({"file": name, "ok": result.get("import_status") not in ("FAILED", "BLOCKED"), "result": result})
            except Exception as exc:
                results.append({"file": name, "ok": False, "error": str(exc)})
            finally:
                try: os.unlink(temp)
                except OSError: pass
        succeeded = sum(1 for result in results if result["ok"]); failed = len(results)-succeeded
        with transaction() as conn:
            conn.execute("UPDATE import_jobs SET state=?,finished_at=?,succeeded=?,failed=?,result_json=? WHERE id=?",
                         ("COMPLETED" if failed == 0 else "COMPLETED_WITH_ERRORS", now(), succeeded, failed, dump(results), job_id))
        return {"job_id": job_id, "state": "COMPLETED" if failed == 0 else "COMPLETED_WITH_ERRORS", "results": results}

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path); path = parsed.path; query = urllib.parse.parse_qs(parsed.query)
        try:
            if path in ("/site-db", "/site-db/"):
                with open(os.path.join(WWW, "index.html"), "rb") as handle: return self.send_body(200, "text/html; charset=utf-8", handle.read())
            if path == "/api/site-db/health":
                return self.json({"ok": True, "version": __version__, "mode": SAFETY_MODE, "flight_control_effect": False,
                                  "coordinate_sources": ["PHOTO_EXIF", "MANUAL_WGS84", "MANUAL_WGS84_CSV"]})
            if path == "/api/site-db/map": return self.json(_site_map())
            if path == "/api/site-db/sites":
                with transaction() as conn: return self.json(rows(conn.execute("SELECT * FROM sites ORDER BY created_at")))
            if path == "/api/site-db/subcages":
                with transaction() as conn:
                    if query.get("site_id"): data = rows(conn.execute("SELECT * FROM subcages WHERE site_id=? ORDER BY label", (query["site_id"][0],)))
                    else: data = rows(conn.execute("SELECT * FROM subcages ORDER BY site_id,label"))
                return self.json(data)
            if path.startswith("/api/site-db/sites/") and path.endswith("/subcages"):
                site_id = path.split("/")[-2]
                with transaction() as conn:
                    data = rows(conn.execute("SELECT * FROM subcages WHERE site_id=? ORDER BY label", (site_id,)))
                return self.json(data)
            if path == "/api/site-db/assets":
                with transaction() as conn: data = rows(conn.execute("SELECT id,sha256,kind,original_name,source_role,byte_size,parser,parser_version,import_status,error_code,metadata_json,created_at FROM source_assets ORDER BY created_at DESC"))
                return self.json([_decode(item) for item in data])
            if path.startswith("/api/site-db/imports/"):
                job_id = path.rsplit("/",1)[-1]
                with transaction() as conn: data = row(conn.execute("SELECT * FROM import_jobs WHERE id=?", (job_id,)))
                return self.json(_decode(data), 200 if data else 404)
            if path.startswith("/api/site-db/photos/") and path.endswith("/metadata"):
                asset_id = path.split("/")[-2]
                with transaction() as conn:
                    asset = row(conn.execute("SELECT * FROM source_assets WHERE id=? AND kind='JPG'", (asset_id,)))
                    sample = row(conn.execute("SELECT * FROM gps_samples WHERE source_asset_id=? LIMIT 1", (asset_id,)))
                if not asset: return self.json({"error": "PHOTO_NOT_FOUND"},404)
                asset = _decode(asset); asset["sample"] = _decode(sample); return self.json(asset)
            if path.startswith("/api/site-db/photos/") and path.endswith("/thumbnail"):
                asset_id = path.split("/")[-2]
                with transaction() as conn: asset = row(conn.execute("SELECT sha256 FROM source_assets WHERE id=? AND kind='JPG'", (asset_id,)))
                if not asset: return self.json({"error": "PHOTO_NOT_FOUND"},404)
                target=os.path.join(config.THUMBNAILS,asset["sha256"]+".jpg")
                if not os.path.isfile(target): return self.json({"error":"THUMBNAIL_NOT_FOUND"},404)
                with open(target,"rb") as handle: return self.send_body(200,"image/jpeg",handle.read())
            if path.startswith("/api/site-db/large-cages/") and path.endswith("/mission-points"):
                site_id=path.split("/")[-2]
                with transaction() as conn:
                    data=rows(conn.execute("""SELECT p.id,p.subcage_id,p.version,p.lat,p.lon,p.approved_at,
                        s.label,s.row_index,s.col_index,q.calculation_method AS source_method
                        FROM approved_point_versions p JOIN subcages s ON s.id=p.subcage_id
                        JOIN sites g ON g.id=s.site_id
                        LEFT JOIN candidate_points q ON q.id=p.candidate_point_id
                        WHERE s.site_id=? AND s.status='ACTIVE' AND g.status='ACTIVE' AND p.active=1 ORDER BY s.label""",(site_id,)))
                return self.json({"site_id":site_id,"points":data})
            if path.startswith("/api/site-db/subcages/") and path.endswith("/point-versions"):
                subcage_id=path.split("/")[-2]
                with transaction() as conn:
                    data=rows(conn.execute("""SELECT p.*,q.calculation_method AS source_method
                        FROM approved_point_versions p LEFT JOIN candidate_points q ON q.id=p.candidate_point_id
                        WHERE p.subcage_id=? ORDER BY p.version DESC""",(subcage_id,)))
                return self.json({"subcage_id":subcage_id,"versions":[_decode(x) for x in data]})
            if path == "/api/site-db/points":
                with transaction() as conn:
                    if query.get("site_id"):
                        candidates=rows(conn.execute("SELECT c.*,s.label FROM candidate_points c JOIN subcages s ON s.id=c.subcage_id WHERE s.site_id=? ORDER BY c.created_at DESC", (query["site_id"][0],)))
                        approved=rows(conn.execute("SELECT p.*,s.label FROM approved_point_versions p JOIN subcages s ON s.id=p.subcage_id WHERE s.site_id=? ORDER BY p.approved_at DESC", (query["site_id"][0],)))
                    else:
                        candidates=rows(conn.execute("SELECT c.*,s.label FROM candidate_points c JOIN subcages s ON s.id=c.subcage_id ORDER BY c.created_at DESC"))
                        approved=rows(conn.execute("SELECT p.*,s.label FROM approved_point_versions p JOIN subcages s ON s.id=p.subcage_id ORDER BY p.approved_at DESC"))
                return self.json({"candidates":[_decode(x) for x in candidates],"approved":[_decode(x) for x in approved]})
            if path.startswith("/api/site-db/route-previews/"):
                parts=path.strip("/").split("/"); route=_route(parts[3]) if len(parts)>3 else None
                if not route: return self.json({"error":"ROUTE_NOT_FOUND"},404)
                if len(parts)==4: return self.json(route)
                fmt=parts[4]
                if fmt=="csv": return self.send_body(200,"text/csv; charset=utf-8",route_csv(route),{"Content-Disposition":"attachment; filename=route-preview.csv"})
                if fmt=="geojson": return self.send_body(200,"application/geo+json",route_geojson(route),{"Content-Disposition":"attachment; filename=route-preview.geojson"})
                if fmt=="png": return self.send_body(200,"image/png",route_png(route),{"Content-Disposition":"attachment; filename=route-preview.png"})
            return self.json({"error":"NOT_FOUND","path":path},404)
        except Exception as exc:
            return self.json({"status":"ERROR","error_code":str(exc).split(":",1)[0],"error":str(exc),
                              "impact":"requested data operation rejected; no flight-control effect",
                              "photos_still_available":True,"next_step":"correct the stated input/credential issue and retry"},400)

    def do_POST(self):
        path=urllib.parse.urlparse(self.path).path
        try:
            if path == "/api/site-db/imports/photos": return self.json(self.import_files("photos"),201)
            if path == "/api/site-db/imports/coordinate-csv":
                form, fields = self.multipart(config.MAX_COORDINATE_CSV_BYTES)
                if len(fields) != 1: raise ValueError("ONE_COORDINATE_CSV_REQUIRED")
                return self.json(import_coordinate_csv(fields[0].file.read(), form.getfirst("site_id")),201)
            if path == "/api/site-db/imports/subcage-csv":
                form, fields = self.multipart(config.MAX_COORDINATE_CSV_BYTES)
                if len(fields) != 1: raise ValueError("ONE_SUBCAGE_CSV_REQUIRED")
                return self.json(import_subcage_csv(fields[0].file.read(), form.getfirst("site_id")),201)
            payload=self.body_json()
            if path.startswith("/api/site-db/sites/") and path.endswith("/subcages"):
                payload["site_id"] = path.split("/")[-2]
                path = "/api/site-db/subcages"
            if path.startswith("/api/site-db/photos/") and path.endswith("/binding"):
                payload["photo_asset_id"] = path.split("/")[-2]
                path = "/api/site-db/bindings/photo"
            if path.startswith("/api/site-db/points/") and path.endswith("/approve"):
                payload["candidate_point_id"] = path.split("/")[-2]
                path = "/api/site-db/points/approve"
            if path.startswith("/api/site-db/sites/") and path.endswith("/status"):
                payload["site_id"] = path.split("/")[-2]; path = "/api/site-db/sites/status"
            if path.startswith("/api/site-db/subcages/") and path.endswith("/status"):
                payload["subcage_id"] = path.split("/")[-2]; path = "/api/site-db/subcages/status"
            if path == "/api/site-db/sites":
                if not str(payload.get("name","")).strip(): raise ValueError("SITE_NAME_REQUIRED")
                site_id=uid("site")
                with transaction() as conn: conn.execute("INSERT INTO sites VALUES (?,?,?,?,?,?)",(site_id,payload["name"].strip(),payload.get("site_type","CAGE_FARM"),now(),"ACTIVE",payload.get("notes")))
                return self.json({"id":site_id},201)
            if path == "/api/site-db/subcages":
                if not payload.get("site_id") or not str(payload.get("label","")).strip(): raise ValueError("SITE_AND_LABEL_REQUIRED")
                subcage_id=uid("subcage")
                with transaction() as conn:
                    if conn.execute("SELECT COUNT(*) FROM subcages WHERE site_id=?",(payload["site_id"],)).fetchone()[0] >= 200:
                        raise ValueError("SUBCAGE_LIMIT_200")
                    conn.execute("""INSERT INTO subcages
                        (id,site_id,label,row_index,col_index,notes,created_at,status)
                        VALUES (?,?,?,?,?,?,?,'ACTIVE')""",(subcage_id,payload["site_id"],payload["label"].strip(),payload.get("row_index"),payload.get("col_index"),payload.get("notes"),now()))
                return self.json({"id":subcage_id},201)
            if path == "/api/site-db/subcages/bulk":
                site_id=str(payload.get("site_id") or ""); prefix=str(payload.get("prefix") or "").strip()
                start=int(payload.get("start",1)); count=int(payload.get("count",0)); padding=int(payload.get("padding",3))
                if not site_id or not prefix or not 1 <= count <= 200 or not 1 <= padding <= 6:
                    raise ValueError("BULK_SUBCAGE_PARAMETERS_INVALID")
                created=[]
                with transaction() as conn:
                    if not row(conn.execute("SELECT id FROM sites WHERE id=? AND status='ACTIVE'",(site_id,))):
                        raise ValueError("ACTIVE_SITE_REQUIRED")
                    if conn.execute("SELECT COUNT(*) FROM subcages WHERE site_id=?",(site_id,)).fetchone()[0] + count > 200:
                        raise ValueError("SUBCAGE_LIMIT_200")
                    for number in range(start,start+count):
                        subcage_id=uid("subcage"); label=("%s%0*d"%(prefix,padding,number))
                        conn.execute("""INSERT INTO subcages
                            (id,site_id,label,row_index,col_index,notes,created_at,status)
                            VALUES (?,?,?,?,?,?,?,'ACTIVE')""",
                            (subcage_id,site_id,label,None,None,payload.get("notes"),now()))
                        created.append({"id":subcage_id,"label":label})
                return self.json({"site_id":site_id,"created":created,"count":len(created)},201)
            if path == "/api/site-db/sites/status":
                status=str(payload.get("status") or "").upper()
                if status not in ("ACTIVE","INACTIVE"): raise ValueError("STATUS_INVALID")
                with transaction() as conn: conn.execute("UPDATE sites SET status=? WHERE id=?",(status,payload.get("site_id")))
                return self.json({"ok":True,"status":status})
            if path == "/api/site-db/subcages/status":
                status=str(payload.get("status") or "").upper()
                if status not in ("ACTIVE","INACTIVE"): raise ValueError("STATUS_INVALID")
                with transaction() as conn: conn.execute("UPDATE subcages SET status=? WHERE id=?",(status,payload.get("subcage_id")))
                return self.json({"ok":True,"status":status})
            if path == "/api/site-db/points/manual-candidate":
                for key in ("site_id","subcage_id","reviewer","lat","lon"):
                    if payload.get(key) in (None,""): raise ValueError("MANUAL_%s_REQUIRED"%key.upper())
                lat,lon=float(payload["lat"]),float(payload["lon"])
                if not valid_coordinate(lat,lon): raise ValueError("GPS_SAMPLE_INVALID")
                candidate_id=uid("candidate")
                with transaction() as conn:
                    sub=row(conn.execute("SELECT id FROM subcages WHERE id=? AND site_id=? AND status='ACTIVE'",(payload["subcage_id"],payload["site_id"])))
                    if not sub: raise ValueError("ACTIVE_SUBCAGE_SITE_MISMATCH")
                    evidence={"reviewer":payload["reviewer"],"note":payload.get("note","")}
                    conn.execute("INSERT INTO candidate_points VALUES (?,?,?,?,?,?,?,?,?)",
                                 (candidate_id,sub["id"],"MANUAL_WGS84",lat,lon,dump(evidence),"USER_REVIEW_REQUIRED","PENDING",now()))
                return self.json({"candidate_point_id":candidate_id,"status":"PENDING"},201)
            if path == "/api/site-db/bindings/photo":
                for key in ("photo_asset_id","subcage_id","reviewer"):
                    if not payload.get(key): raise ValueError("BINDING_%s_REQUIRED"%key.upper())
                binding_id=uid("binding")
                with transaction() as conn:
                    if not row(conn.execute("SELECT id FROM subcages WHERE id=? AND status='ACTIVE'",(payload["subcage_id"],))):
                        raise ValueError("ACTIVE_SUBCAGE_REQUIRED")
                    sample=row(conn.execute("SELECT * FROM gps_samples WHERE source_asset_id=? AND origin_type='PHOTO_EXIF' LIMIT 1",(payload["photo_asset_id"],)))
                    conn.execute("INSERT INTO photo_bindings VALUES (?,?,?,?,?,?,?)",(binding_id,payload["photo_asset_id"],payload["subcage_id"],payload.get("capture_marker"),payload.get("binding_confidence","USER_CONFIRMED"),payload["reviewer"],now()))
                    candidate_id=None
                    if sample:
                        candidate_id=uid("candidate")
                        conn.execute("INSERT INTO candidate_points VALUES (?,?,?,?,?,?,?,?,?)",(candidate_id,payload["subcage_id"],"PHOTO_EXIF",sample["latitude"],sample["longitude"],dump([sample["id"]]),"PHOTO_CENTER_PROXY_UNVERIFIED","PENDING",now()))
                return self.json({"binding_id":binding_id,"candidate_point_id":candidate_id,"warning":"PHOTO GPS is camera/drone position; manual approval required"},201)
            if path == "/api/site-db/points/approve":
                if not payload.get("candidate_point_id") or not payload.get("reviewer"): raise ValueError("CANDIDATE_AND_REVIEWER_REQUIRED")
                with transaction() as conn:
                    candidate=row(conn.execute("SELECT * FROM candidate_points WHERE id=?",(payload["candidate_point_id"],)))
                    if not candidate: raise ValueError("CANDIDATE_NOT_FOUND")
                    if "SIMULATED" in candidate["quality"]: raise ValueError("SIMULATED_EVIDENCE_CANNOT_BE_APPROVED")
                    lat=float(payload.get("lat",candidate["lat"])); lon=float(payload.get("lon",candidate["lon"]))
                    if not valid_coordinate(lat,lon): raise ValueError("GPS_SAMPLE_INVALID")
                    version=conn.execute("SELECT COALESCE(MAX(version),0)+1 FROM approved_point_versions WHERE subcage_id=?",(candidate["subcage_id"],)).fetchone()[0]
                    conn.execute("UPDATE approved_point_versions SET active=0 WHERE subcage_id=?",(candidate["subcage_id"],))
                    approved_id=uid("approved")
                    evidence={"candidate_point_id":candidate["id"],"source_sample_ids":load(candidate["source_sample_ids"],[]),"original_quality":candidate["quality"],"coordinate_edited":lat!=candidate["lat"] or lon!=candidate["lon"]}
                    conn.execute("INSERT INTO approved_point_versions VALUES (?,?,?,?,?,?,?,?,?,?,?)",(approved_id,candidate["subcage_id"],candidate["id"],version,lat,lon,dump(evidence),payload["reviewer"],payload.get("review_note"),now(),1))
                    conn.execute("UPDATE candidate_points SET status='APPROVED' WHERE id=?",(candidate["id"],))
                return self.json({"id":approved_id,"version":version,"coordinate_approved":True,"mode":"DATA_ONLY"},201)
            if path == "/api/site-db/mission-snapshots":
                required=("mission_id","input_profile","home","parameters","preview_hash")
                if any(key not in payload for key in required): raise ValueError("SNAPSHOT_FIELDS_REQUIRED")
                home=payload["home"]; params=payload["parameters"]; snapshot_id=uid("snapshot"); stamp=now()
                with transaction() as conn:
                    existing=row(conn.execute("SELECT id,preview_hash,state FROM mission_snapshots WHERE mission_id=?",(payload["mission_id"],)))
                    if existing:
                        if existing["preview_hash"] == payload["preview_hash"]:
                            return self.json({"id":existing["id"],"mission_id":payload["mission_id"],"idempotent":True},200)
                        raise ValueError("TASK_ID_ALREADY_USED_CHANGE_TASK_ID")
                    conn.execute("""INSERT INTO mission_snapshots
                        (id,mission_id,input_profile,site_id,home_lat,home_lon,home_alt,
                         takeoff_relative_alt,transit_relative_alt,feed_relative_alt,feeding_dwell_sec,
                         waypoint_hold_sec,speed,acceptance_radius,preview_hash,state,details_json,created_at,updated_at)
                        VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                        (snapshot_id,payload["mission_id"],payload["input_profile"],payload.get("site_id"),
                         home["lat"],home["lon"],home.get("alt"),params["takeoff_relative_alt"],
                         params["transit_relative_alt"],params["feed_relative_alt"],params["feeding_dwell_sec"],
                         params["waypoint_hold_sec"],params["speed"],params["acceptance_radius"],
                         payload["preview_hash"],payload.get("state","PREVIEW_CONFIRMED"),dump(payload.get("details",{})),stamp,stamp))
                    for seq,target in enumerate(payload.get("targets",[])):
                        conn.execute("INSERT INTO mission_snapshot_targets VALUES (?,?,?,?,?,?,?)",
                                     (snapshot_id,seq,target.get("subcage_id"),target.get("approved_version_id"),
                                      target.get("label",str(seq)),target["lat"],target["lon"]))
                return self.json({"id":snapshot_id,"mission_id":payload["mission_id"]},201)
            if path.startswith("/api/site-db/mission-snapshots/") and path.endswith("/state"):
                mission_id=urllib.parse.unquote(path.split("/")[-2]); state=str(payload.get("state") or "")
                if not state: raise ValueError("SNAPSHOT_STATE_REQUIRED")
                with transaction() as conn: conn.execute("UPDATE mission_snapshots SET state=?,details_json=?,updated_at=? WHERE mission_id=?",(state,dump(payload.get("details",{})),now(),mission_id))
                return self.json({"ok":True,"mission_id":mission_id,"state":state})
            if path == "/api/site-db/route-previews": return self.json(create_route(payload),201)
            return self.json({"error":"NOT_FOUND","path":path},404)
        except Exception as exc:
            return self.json({"status":"ERROR","error_code":str(exc).split(":",1)[0],"error":str(exc),
                              "impact":"requested data operation rejected; no flight-control effect",
                              "photos_still_available":True,"next_step":"correct the stated input/credential issue and retry"},400)

    def log_message(self, fmt, *args):
        print("site-db", self.address_string(), fmt % args, flush=True)


def run(host="127.0.0.1", port=8090):
    init_db()
    server=ThreadingHTTPServer((host,port),Handler)
    print("cage_site_db %s %s http://%s:%d/site-db/"%(__version__,SAFETY_MODE,host,port),flush=True)
    server.serve_forever()
