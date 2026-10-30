import csv
import os

from .. import config
from ..assets import disk_has_room, immutable_copy, sha256_file
from ..db import dump, now, row, rows, transaction, uid
from ..processing.geometry import valid_coordinate
from ..processing.tracks import parse_time, segment_distance, segment_samples

ALIASES = {
    "time": ("sample_time_utc", "timestamp_utc", "CUSTOM.dateTime", "datetime", "time"),
    "lat": ("latitude", "lat", "OSD.latitude"),
    "lon": ("longitude", "lon", "lng", "OSD.longitude"),
    "alt": ("altitude_m", "altitude", "OSD.altitude", "OSD.height"),
    "speed": ("ground_speed_mps", "speed_mps", "speed"),
    "valid": ("gps_valid", "OSD.isGPSUsed"),
    "satellites": ("satellites", "gps_num", "OSD.gpsNum"),
    "source_id": ("source_sample_id", "frame_id", "index"),
}


def _column(fieldnames, key):
    for candidate in ALIASES[key]:
        if candidate in fieldnames:
            return candidate
    return None


def _bool(value):
    return str(value).strip().lower() not in ("", "0", "false", "no", "none", "null")


def _float(value):
    try:
        return float(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None


def normalize_csv(path):
    with open(path, newline="", encoding="utf-8-sig", errors="strict") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        mapping = {key: _column(fields, key) for key in ALIASES}
        if not mapping["time"] or not mapping["lat"] or not mapping["lon"]:
            raise ValueError("CSV_REQUIRED_FIELDS_MISSING")
        normalized = []
        for index, source in enumerate(reader):
            stamp = source.get(mapping["time"])
            parsed = parse_time(stamp)
            lat, lon = _float(source.get(mapping["lat"])), _float(source.get(mapping["lon"]))
            valid = _bool(source.get(mapping["valid"])) if mapping["valid"] else True
            flags = []
            if parsed is None:
                flags.append("TIME_MISSING_OR_UNVERIFIED")
            if not valid_coordinate(lat, lon) or not valid:
                flags.append("GPS_SAMPLE_INVALID")
            normalized.append({
                "sample_index": index, "source_sample_id": source.get(mapping["source_id"]) if mapping["source_id"] else str(index),
                "original_timestamp": stamp, "timestamp_utc": parsed.isoformat() if parsed else None,
                "time_quality": "UTC_VERIFIED" if parsed else "TIME_UNVERIFIED",
                "latitude": lat, "longitude": lon, "altitude_m": _float(source.get(mapping["alt"])) if mapping["alt"] else None,
                "speed_mps": _float(source.get(mapping["speed"])) if mapping["speed"] else None,
                "satellites": int(_float(source.get(mapping["satellites"]))) if mapping["satellites"] and _float(source.get(mapping["satellites"])) is not None else None,
                "gps_valid": bool(valid and valid_coordinate(lat, lon)), "quality_flags": flags,
            })
    return normalized, mapping


def _store_samples(conn, asset_id, session_id, normalized, origin_type):
    sample_ids = []
    for item in normalized:
        if not item["gps_valid"] or not item["timestamp_utc"]:
            continue
        sample_id = uid("sample"); sample_ids.append(sample_id)
        conn.execute("INSERT INTO gps_samples VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", (
            sample_id, asset_id, session_id, item["sample_index"], item["original_timestamp"], item["timestamp_utc"],
            item["time_quality"], item["latitude"], item["longitude"], item["altitude_m"],
            "SOURCE_UNSPECIFIED", 1, None, "WGS84_DECLARED_BY_ADAPTER", item["satellites"], None,
            item["speed_mps"], origin_type, dump(item["quality_flags"]), dump({"source_sample_id": item["source_sample_id"]})))
    return sample_ids


def _build_segments(conn, asset_id, session_id):
    samples = rows(conn.execute("SELECT * FROM gps_samples WHERE source_asset_id=? ORDER BY timestamp_utc,sample_index", (asset_id,)))
    segments, diagnostics = segment_samples(samples)
    for index, (group, break_reason) in enumerate(segments):
        segment_id = uid("segment")
        conn.execute("INSERT INTO track_segments VALUES (?,?,?,?,?,?,?,?,?,?)", (
            segment_id, session_id, asset_id, index, group[0]["timestamp_utc"], group[-1]["timestamp_utc"], len(group),
            segment_distance(group), break_reason, dump([d for d in diagnostics if d.get("reason") == break_reason])))
        for seq, item in enumerate(group):
            conn.execute("INSERT INTO track_segment_samples VALUES (?,?,?)", (segment_id, item["id"], seq))
    return len(segments), diagnostics


def import_csv(path, original_name, site_id=None, source_role="EXTERNAL_CSV_UNVERIFIED", parser="normalized-csv", parser_version="1", existing_asset=None, origin_type="EXTERNAL_CSV"):
    size = os.path.getsize(path)
    if size > config.MAX_CSV_BYTES:
        raise ValueError("CSV_TOO_LARGE")
    if not disk_has_room(config.ROOT, size):
        raise ValueError("DISK_RESERVE_REACHED")
    normalized, mapping = normalize_csv(path)
    if not normalized:
        raise ValueError("CSV_NO_ROWS")
    digest = sha256_file(path)
    session_id = uid("session")
    if existing_asset:
        asset_id = existing_asset["id"]
        target = existing_asset["original_path"]
    else:
        with transaction() as conn:
            prior = row(conn.execute("SELECT * FROM source_assets WHERE sha256=?", (digest,)))
            if prior:
                prior["idempotent"] = True
                return prior
        target = immutable_copy(path, digest, ".csv")
        asset_id = uid("asset")
    valid_count = sum(1 for item in normalized if item["gps_valid"] and item["timestamp_utc"])
    status = "SUCCESS" if valid_count else "FAILED"
    with transaction() as conn:
        conn.execute("INSERT INTO capture_sessions VALUES (?,?,?,?,?,?,?)", (
            session_id, site_id, "DJI Fly" if origin_type == "DJI_TXT" else "External converter",
            next((x["timestamp_utc"] for x in normalized if x["timestamp_utc"]), None), "UTC_FROM_SOURCE", source_role, now()))
        if not existing_asset:
            conn.execute("INSERT INTO source_assets VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", (
                asset_id, digest, "CSV", original_name, target, source_role, size, "text/csv", parser, parser_version,
                status, None if valid_count else "GPS_SAMPLE_INVALID", None, dump({"mapping": mapping, "rows": len(normalized)}), now()))
        _store_samples(conn, asset_id, session_id, normalized, origin_type)
        segment_count, diagnostics = _build_segments(conn, asset_id, session_id)
        conn.execute("UPDATE source_assets SET import_status=?, metadata_json=? WHERE id=?", (
            status, dump({"mapping": mapping, "rows": len(normalized), "valid_rows": valid_count,
                          "segment_count": segment_count, "diagnostics": diagnostics}), asset_id))
    return {"id": asset_id, "sha256": digest, "session_id": session_id, "rows": len(normalized),
            "valid_rows": valid_count, "segments": segment_count, "import_status": status, "idempotent": False}
