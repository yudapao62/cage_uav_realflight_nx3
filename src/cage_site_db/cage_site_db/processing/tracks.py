import datetime as dt
import re

from .geometry import haversine_m, valid_coordinate

DEFAULT_MAX_GAP_S = 5.0
DEFAULT_MAX_SPEED_MPS = 45.0


def parse_time(value):
    if not value:
        return None
    text = str(value).strip().replace("Z", "+00:00")
    match = re.search(r"\.(\d+)([+-]\d\d:\d\d)$", text)
    if match and len(match.group(1)) < 6:
        text = text[:match.start(1)] + match.group(1).ljust(6, "0") + match.group(2)
    try:
        parsed = dt.datetime.fromisoformat(text)
    except ValueError:
        for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S", "%Y/%m/%d %H:%M:%S"):
            try:
                parsed = dt.datetime.strptime(text, fmt)
                break
            except ValueError:
                parsed = None
        if parsed is None:
            return None
    if parsed.tzinfo is None:
        return None
    return parsed.astimezone(dt.timezone.utc)


def segment_samples(samples, max_gap_s=DEFAULT_MAX_GAP_S, max_speed_mps=DEFAULT_MAX_SPEED_MPS):
    ordered = sorted(samples, key=lambda item: (item.get("timestamp_utc") or "", item.get("sample_index", 0)))
    segments, current, diagnostics = [], [], []
    last = None
    seen = set()
    for item in ordered:
        lat, lon = item.get("latitude"), item.get("longitude")
        stamp = parse_time(item.get("timestamp_utc"))
        if not valid_coordinate(lat, lon) or stamp is None or not item.get("gps_valid", True):
            diagnostics.append({"sample": item.get("id"), "reason": "GPS_SAMPLE_INVALID"})
            if current:
                segments.append((current, "INVALID_GPS")); current = []
            last = None
            continue
        key = (stamp.isoformat(), round(float(lat), 9), round(float(lon), 9))
        if key in seen:
            diagnostics.append({"sample": item.get("id"), "reason": "DUPLICATE_SAMPLE"})
            continue
        seen.add(key)
        break_reason = None
        if last:
            delta = (stamp - last[0]).total_seconds()
            distance = haversine_m(last[1], last[2], float(lat), float(lon))
            speed = distance / delta if delta > 0 else float("inf")
            if delta <= 0:
                break_reason = "TIME_NOT_INCREASING"
            elif delta > max_gap_s:
                break_reason = "TIME_GAP"
            elif speed > max_speed_mps:
                break_reason = "DIAGNOSTIC_SPEED_JUMP"
            if break_reason:
                diagnostics.append({"sample": item.get("id"), "reason": break_reason, "gap_s": delta, "distance_m": distance, "speed_mps": speed})
                if current:
                    segments.append((current, break_reason)); current = []
        current.append(item)
        last = (stamp, float(lat), float(lon))
    if current:
        segments.append((current, None))
    return segments, diagnostics


def segment_distance(samples):
    return sum(haversine_m(float(a["latitude"]), float(a["longitude"]), float(b["latitude"]), float(b["longitude"])) for a, b in zip(samples, samples[1:]))
