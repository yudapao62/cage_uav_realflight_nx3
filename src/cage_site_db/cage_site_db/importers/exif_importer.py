import datetime as dt
import json
import os
import subprocess

from PIL import Image

from .. import config
from ..assets import disk_has_room, immutable_copy, sha256_file, sniff_jpeg
from ..db import dump, now, row, transaction, uid
from ..processing.geometry import valid_coordinate


def _tag(meta, *suffixes):
    for suffix in suffixes:
        for key, value in meta.items():
            if key == suffix or key.endswith(":" + suffix):
                return value
    return None


def _signed(value, ref):
    if value is None:
        return None
    result = float(value)
    if str(ref or "").upper() in ("S", "W"):
        result = -abs(result)
    return result


def _time(meta):
    original = _tag(meta, "DateTimeOriginal", "CreateDate")
    offset = _tag(meta, "OffsetTimeOriginal", "OffsetTime")
    if not original:
        return None, None, "PHOTO_TIME_MISSING"
    if not offset:
        return str(original), None, "EXIF_TIMEZONE_UNVERIFIED"
    try:
        value = dt.datetime.strptime(str(original) + str(offset), "%Y:%m:%d %H:%M:%S%z")
        return str(original), value.astimezone(dt.timezone.utc).isoformat(), "EXIF_TIMEZONE_VERIFIED"
    except ValueError:
        return str(original), None, "EXIF_TIME_PARSE_FAILED"


def read_exif(path):
    try:
        proc = subprocess.run(["exiftool", "-j", "-G1", "-n", "-s", path], stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=20, check=False)
    except FileNotFoundError:
        raise RuntimeError("EXIFTOOL_NOT_INSTALLED")
    if proc.returncode != 0:
        raise RuntimeError("EXIF_READ_FAILED: " + proc.stderr.decode("utf-8", "replace")[-500:])
    parsed = json.loads(proc.stdout.decode("utf-8"))
    return parsed[0] if parsed else {}


def import_photo(path, original_name, source_role="WEB_UPLOAD", session_id=None):
    size = os.path.getsize(path)
    if size > config.MAX_PHOTO_BYTES:
        raise ValueError("PHOTO_TOO_LARGE")
    if not disk_has_room(config.ROOT, size):
        raise ValueError("DISK_RESERVE_REACHED")
    if not sniff_jpeg(path):
        raise ValueError("PHOTO_INVALID_JPEG")
    digest = sha256_file(path)
    with transaction() as conn:
        existing = row(conn.execute("SELECT * FROM source_assets WHERE sha256=?", (digest,)))
        if existing:
            existing["idempotent"] = True
            return existing
    target = immutable_copy(path, digest, ".jpg")
    flags, status, error = [], "SUCCESS", None
    try:
        with Image.open(target) as image:
            image.verify()
        with Image.open(target) as image:
            width, height = image.size
            thumb_path = os.path.join(config.THUMBNAILS, digest + ".jpg")
            if not os.path.exists(thumb_path):
                image.thumbnail((640, 640))
                image.convert("RGB").save(thumb_path, "JPEG", quality=82)
                os.chmod(thumb_path, 0o440)
    except Exception as exc:
        raise ValueError("PHOTO_DECODE_FAILED: %s" % exc)
    meta = read_exif(target)
    lat = _signed(_tag(meta, "GPSLatitude"), _tag(meta, "GPSLatitudeRef"))
    lon = _signed(_tag(meta, "GPSLongitude"), _tag(meta, "GPSLongitudeRef"))
    original_time, utc_time, time_quality = _time(meta)
    flags.append(time_quality)
    if lat is None or lon is None:
        flags.append("EXIF_NO_GPS"); status = "PARTIAL"; error = "EXIF_NO_GPS"
    elif not valid_coordinate(lat, lon):
        flags.append("GPS_SAMPLE_INVALID"); status = "FAILED"; error = "GPS_SAMPLE_INVALID"
        lat = lon = None
    else:
        flags.extend(["PHOTO_CENTER_PROXY_UNVERIFIED", "GPS_PRESENT"])
    altitude = _tag(meta, "GPSAltitude")
    altitude_ref_raw = _tag(meta, "GPSAltitudeRef")
    altitude_reference = None
    if altitude is not None:
        altitude = float(altitude)
        if str(altitude_ref_raw) in ("1", "Below Sea Level"):
            altitude = -abs(altitude); altitude_reference = "EXIF_BELOW_SEA_LEVEL_UNVERIFIED"
            flags.append("EXIF_ALTITUDE_SUSPICIOUS")
        else:
            altitude_reference = "EXIF_ABOVE_SEA_LEVEL_UNVERIFIED"
    asset_id = uid("asset")
    metadata = {"width": width, "height": height, "raw_exiftool": meta, "quality_flags": flags,
                "thumbnail": "/api/site-db/photos/%s/thumbnail" % asset_id}
    with transaction() as conn:
        conn.execute("INSERT INTO source_assets VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                     (asset_id, digest, "JPG", original_name, target, source_role, size, "image/jpeg",
                      "ExifTool", None, status, error, None, dump(metadata), now()))
        if lat is not None:
            conn.execute("INSERT INTO gps_samples VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                         (uid("sample"), asset_id, session_id, 0, original_time, utc_time, time_quality,
                          lat, lon, altitude, altitude_reference, 1, None, "WGS84_ASSUMED_FROM_EXIF",
                          None, None, None, "PHOTO_EXIF", dump(flags), dump({"direction": _tag(meta, "GPSImgDirection")})))
    return {"id": asset_id, "sha256": digest, "import_status": status, "error_code": error,
            "latitude": lat, "longitude": lon, "quality_flags": flags, "idempotent": False}
