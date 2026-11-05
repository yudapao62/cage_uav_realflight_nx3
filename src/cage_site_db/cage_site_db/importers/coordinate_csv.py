import csv
import io

from ..db import dump, now, row, transaction, uid
from ..processing.geometry import valid_coordinate


REQUIRED = ("small_cage_label", "latitude", "longitude")


def import_subcage_csv(data, site_id):
    if not site_id: raise ValueError("SITE_REQUIRED")
    text = data.decode("utf-8-sig") if isinstance(data, bytes) else str(data)
    reader = csv.DictReader(io.StringIO(text))
    if not reader.fieldnames or "small_cage_label" not in reader.fieldnames:
        raise ValueError("SUBCAGE_CSV_COLUMNS_INVALID")
    source_rows = list(reader)
    results=[]; seen=set()
    with transaction() as conn:
        if not row(conn.execute("SELECT id FROM sites WHERE id=? AND status='ACTIVE'",(site_id,))):
            raise ValueError("ACTIVE_SITE_REQUIRED")
        existing = conn.execute("SELECT COUNT(*) FROM subcages WHERE site_id=?",(site_id,)).fetchone()[0]
        if existing + len(source_rows) > 200: raise ValueError("SUBCAGE_LIMIT_200")
        for line,source in enumerate(source_rows,2):
            label=str(source.get("small_cage_label") or "").strip()
            try:
                if not label: raise ValueError("LABEL_REQUIRED")
                if label in seen: raise ValueError("DUPLICATE_LABEL_IN_FILE")
                seen.add(label)
                cage_id=uid("subcage")
                row_index=int(source["row_index"]) if source.get("row_index") else None
                col_index=int(source["col_index"]) if source.get("col_index") else None
                conn.execute("""INSERT INTO subcages
                    (id,site_id,label,row_index,col_index,notes,created_at,status)
                    VALUES (?,?,?,?,?,?,?,'ACTIVE')""",
                    (cage_id,site_id,label,row_index,col_index,source.get("note"),now()))
                results.append({"line":line,"label":label,"ok":True,"id":cage_id})
            except Exception as exc:
                results.append({"line":line,"label":label,"ok":False,"error":str(exc)})
    return {"total":len(results),"succeeded":sum(1 for x in results if x["ok"]),
            "failed":sum(1 for x in results if not x["ok"]),"rows":results}


def import_coordinate_csv(data, site_id):
    if not site_id:
        raise ValueError("SITE_REQUIRED")
    text = data.decode("utf-8-sig") if isinstance(data, bytes) else str(data)
    reader = csv.DictReader(io.StringIO(text))
    if not reader.fieldnames or any(name not in reader.fieldnames for name in REQUIRED):
        raise ValueError("COORDINATE_CSV_COLUMNS_INVALID")
    results = []; seen_labels = set()
    with transaction() as conn:
        if not row(conn.execute("SELECT id FROM sites WHERE id=? AND status='ACTIVE'", (site_id,))):
            raise ValueError("ACTIVE_SITE_REQUIRED")
        for line, source in enumerate(reader, 2):
            label = str(source.get("small_cage_label") or "").strip()
            try:
                lat, lon = float(source.get("latitude")), float(source.get("longitude"))
                if not label or not valid_coordinate(lat, lon):
                    raise ValueError("INVALID_COORDINATE")
                if label in seen_labels:
                    raise ValueError("DUPLICATE_LABEL_IN_FILE")
                seen_labels.add(label)
                subcage = row(conn.execute(
                    "SELECT id FROM subcages WHERE site_id=? AND label=? AND status='ACTIVE'",
                    (site_id, label)))
                if not subcage:
                    raise ValueError("ACTIVE_SUBCAGE_NOT_FOUND")
                row_index = int(source["row_index"]) if source.get("row_index") else None
                col_index = int(source["col_index"]) if source.get("col_index") else None
                if row_index is not None or col_index is not None:
                    conn.execute("""UPDATE subcages SET
                        row_index=COALESCE(?,row_index),col_index=COALESCE(?,col_index)
                        WHERE id=?""", (row_index, col_index, subcage["id"]))
                candidate_id = uid("candidate")
                evidence = {"csv_line": line, "note": source.get("note", ""),
                            "row_index": row_index, "col_index": col_index}
                conn.execute("""INSERT INTO candidate_points
                    (id,subcage_id,calculation_method,lat,lon,source_sample_ids,quality,status,created_at)
                    VALUES (?,?,?,?,?,?,?,?,?)""",
                    (candidate_id, subcage["id"], "MANUAL_WGS84_CSV", lat, lon,
                     dump(evidence), "USER_REVIEW_REQUIRED", "PENDING", now()))
                results.append({"line": line, "label": label, "ok": True,
                                "candidate_point_id": candidate_id})
            except Exception as exc:
                results.append({"line": line, "label": label, "ok": False,
                                "error": str(exc)})
    return {"total": len(results), "succeeded": sum(1 for item in results if item["ok"]),
            "failed": sum(1 for item in results if not item["ok"]), "rows": results}
