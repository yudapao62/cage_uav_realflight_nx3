"""Pure planning helpers for the three real-flight Web task sources.

This module has no ROS publishers and never talks to MAVROS.  It only turns
validated geometry into MissionWaypoint-shaped dictionaries for preview or for
the existing, separately gated mission FSM.
"""
import math

from cage_geo.geodesy import LocalENU


TASK_SOURCES = ("GPS_CENTER", "GPS_GRID", "PHOTO_POINTS")


def battery_status(percentage, voltage, received_at, now, timeout, threshold):
    """Normalize MAVROS telemetry without inventing a percentage from voltage."""
    age = max(0.0, float(now)-float(received_at)) if received_at else None
    percentage = (float(percentage) if percentage is not None and
                  math.isfinite(float(percentage)) and 0.0 <= float(percentage) <= 1.0
                  else None)
    voltage = (float(voltage) if voltage is not None and math.isfinite(float(voltage)) and
               float(voltage) > 0.0 else None)
    return {"percentage": percentage, "voltage": voltage,
            "fresh": age is not None and age <= float(timeout), "age_sec": age,
            "low": percentage is not None and percentage <= float(threshold),
            "threshold": float(threshold)}


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value


def _coordinate(lat, lon, name):
    lat, lon = _finite(lat, name + ".lat"), _finite(lon, name + ".lon")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError("{} is outside WGS84 range".format(name))
    if lat == 0.0 and lon == 0.0:
        raise ValueError("{} is zero".format(name))
    return lat, lon


def _waypoint(seq, lat, lon, alt, yaw, radius, speed, hold, kind, cell_id=""):
    return {"seq": int(seq), "lat": float(lat), "lon": float(lon),
            "alt": float(alt), "yaw": float(yaw),
            "acceptance_radius": float(radius), "speed": float(speed),
            "hold_time": float(hold), "type": str(kind),
            "cell_id": str(cell_id)}


def _common(home, takeoff_alt, transit_alt, feed_alt, dwell, radius, speed,
            max_leg_distance):
    hlat, hlon = _coordinate(home[0], home[1], "home")
    halt = _finite(home[2], "home.alt")
    values = {
        "takeoff_alt": _finite(takeoff_alt, "takeoff_relative_alt"),
        "transit_alt": _finite(transit_alt, "transit_relative_alt"),
        "feed_alt": _finite(feed_alt, "feed_relative_alt"),
        "dwell": _finite(dwell, "waypoint_hold_sec"),
        "radius": _finite(radius, "acceptance_radius"),
        "speed": _finite(speed, "speed"),
        "max_leg": _finite(max_leg_distance, "max_leg_distance"),
    }
    if any(values[key] <= 0.0 for key in values):
        raise ValueError("heights, dwell, radius, speed and max leg must be positive")
    return (hlat, hlon, halt), values


def _validate_route(local, xy, max_leg):
    previous = (0.0, 0.0)
    for point in xy:
        if math.hypot(point[0], point[1]) > max_leg:
            raise ValueError("planned target exceeds max_leg_distance")
        if math.hypot(point[0] - previous[0], point[1] - previous[1]) > max_leg:
            raise ValueError("planned mission leg exceeds max_leg_distance")
        previous = point


def _grid_mission(home, targets, takeoff_alt, transit_alt, feed_alt, dwell,
                  radius, speed, max_leg_distance):
    home, cfg = _common(home, takeoff_alt, transit_alt, feed_alt, dwell,
                        radius, speed, max_leg_distance)
    if not targets:
        raise ValueError("at least one target is required")
    if len(targets) > 200:
        raise ValueError("target count exceeds 200")
    local = LocalENU(*home)
    xy = []
    clean = []
    for index, target in enumerate(targets):
        lat, lon = _coordinate(target["lat"], target["lon"], "target[{}]".format(index))
        east, north, _ = local.to_enu(lat, lon, home[2])
        xy.append((east, north))
        clean.append((lat, lon, str(target.get("cell_id") or "P{:03d}".format(index + 1))))
    _validate_route(local, xy, cfg["max_leg"])
    first, last = xy[0], xy[-1]
    yaw = math.degrees(math.atan2(last[0] - first[0], last[1] - first[1])) % 360.0
    result = [
        _waypoint(0, home[0], home[1], cfg["takeoff_alt"], yaw, cfg["radius"], cfg["speed"], 0.0, "TAKEOFF"),
        _waypoint(1, clean[0][0], clean[0][1], cfg["transit_alt"], yaw, cfg["radius"], cfg["speed"], 0.0, "TRANSIT"),
        _waypoint(2, clean[0][0], clean[0][1], cfg["feed_alt"], yaw, cfg["radius"], cfg["speed"], 0.0, "GRID_ENTER"),
    ]
    for lat, lon, cell_id in clean:
        result.append(_waypoint(len(result), lat, lon, cfg["feed_alt"], yaw,
                                cfg["radius"], cfg["speed"], cfg["dwell"],
                                "CELL_TARGET", cell_id))
    # Do not append a duplicate GRID_EXIT at the final cell coordinates.  K9
    # can complete a zero-hold duplicate before publishing its sequence, which
    # prevents the FSM from seeing the event and requesting RTL.  The FSM now
    # completes the grid after the final CELL_TARGET's logical hold timer.
    return result


def plan_gps_grid(home, endpoint_a, endpoint_b, rows, cols,
                  takeoff_alt, transit_alt, feed_alt, dwell, radius=3.0,
                  speed=5.0, max_leg_distance=5000.0):
    """Use A and B as opposite corners of a square in the local ENU plane.

    The physical cage is approximately square and A/B are its diagonal
    vertices.  A diagonal uniquely defines that square: rotate A->B by 90
    degrees, then combine the two vectors to obtain two perpendicular,
    equal-length side vectors whose sum is A->B.  Columns and rows divide
    those side vectors.  Each target is the exact cell center and the route
    is row-wise serpentine.
    """
    rows, cols = int(rows), int(cols)
    if not (1 <= rows <= 50 and 1 <= cols <= 50 and rows * cols <= 200):
        raise ValueError("grid rows*cols must be 1..200 (each dimension 1..50)")
    home_coord, _ = _common(home, takeoff_alt, transit_alt, feed_alt, dwell,
                            radius, speed, max_leg_distance)
    local = LocalENU(*home_coord)
    alat, alon = _coordinate(endpoint_a[0], endpoint_a[1], "anchor_a")
    blat, blon = _coordinate(endpoint_b[0], endpoint_b[1], "anchor_b")
    ax, ay, _ = local.to_enu(alat, alon, home_coord[2])
    bx, by, _ = local.to_enu(blat, blon, home_coord[2])
    dx, dy = bx - ax, by - ay
    diagonal = math.hypot(dx, dy)
    if diagonal < 1.0:
        raise ValueError("GPS A/B diagonal must be at least 1.0 m")

    # p is A->B rotated 90 degrees counter-clockwise.  col_vec + row_vec is
    # exactly A->B, while col_vec and row_vec are perpendicular and equal.
    px, py = -dy, dx
    col_vec = ((dx + px) * 0.5, (dy + py) * 0.5)
    row_vec = ((dx - px) * 0.5, (dy - py) * 0.5)
    side = diagonal / math.sqrt(2.0)
    cell_col, cell_row = side / cols, side / rows
    targets = []
    for row in range(1, rows + 1):
        columns = range(1, cols + 1) if row % 2 else range(cols, 0, -1)
        for col in columns:
            col_fraction = (col - 0.5) / cols
            row_fraction = (row - 0.5) / rows
            east = ax + col_vec[0] * col_fraction + row_vec[0] * row_fraction
            north = ay + col_vec[1] * col_fraction + row_vec[1] * row_fraction
            lat, lon, _ = local.to_wgs84(east, north, 0.0)
            targets.append({"lat": lat, "lon": lon,
                            "cell_id": "R{:02d}C{:02d}".format(row, col)})
    mission = _grid_mission(home_coord, targets, takeoff_alt, transit_alt,
                            feed_alt, dwell, radius, speed, max_leg_distance)
    return mission, {"rows": rows, "cols": cols,
                     "geometry_definition": "A_B_OPPOSITE_CORNERS_SQUARE",
                     "diagonal_m": diagonal,
                     "length_m": side, "width_m": side,
                     "cell_length_m": cell_col, "cell_width_m": cell_row,
                     "column_vector_east_m": col_vec[0],
                     "column_vector_north_m": col_vec[1],
                     "row_vector_east_m": row_vec[0],
                     "row_vector_north_m": row_vec[1],
                     "target_count": len(targets)}


def plan_photo_points(home, photo_points, takeoff_alt, transit_alt, feed_alt,
                      dwell, radius=3.0, speed=5.0,
                      max_leg_distance=5000.0, preserve_order=False):
    """Plan immutable photo/database XY points; altitude comes from Web parameters."""
    home_coord, _ = _common(home, takeoff_alt, transit_alt, feed_alt, dwell,
                            radius, speed, max_leg_distance)
    local = LocalENU(*home_coord)
    remaining = []
    for index, point in enumerate(photo_points):
        lat, lon = _coordinate(point["lat"], point["lon"], "photo_point[{}]".format(index))
        east, north, _ = local.to_enu(lat, lon, home_coord[2])
        remaining.append({"lat": lat, "lon": lon, "x": east, "y": north,
                          "cell_id": str(point.get("label") or point.get("id") or "PHOTO-{:03d}".format(index + 1)),
                          "source_id": str(point.get("asset_id") or point.get("id") or ""),
                          "source_asset_id": str(point.get("asset_id") or ""),
                          "approved_version_id": str(point.get("id") or "") if not point.get("asset_id") else ""})
    ordered, x, y = [], 0.0, 0.0
    if preserve_order:
        ordered = remaining
    else:
        while remaining:
            index = min(range(len(remaining)),
                        key=lambda i: (math.hypot(remaining[i]["x"] - x,
                                                  remaining[i]["y"] - y),
                                       remaining[i]["cell_id"]))
            point = remaining.pop(index)
            ordered.append(point)
            x, y = point["x"], point["y"]
    mission = _grid_mission(home_coord, ordered, takeoff_alt, transit_alt,
                            feed_alt, dwell, radius, speed, max_leg_distance)
    return mission, {"target_count": len(ordered),
                     "order": [{"source_id": p["source_id"],
                                "source_asset_id": p["source_asset_id"],
                                "approved_version_id": p["approved_version_id"],
                                "label": p["cell_id"], "lat": p["lat"],
                                "lon": p["lon"]} for p in ordered],
                     "altitude_source": "WEB_PARAMETER_RELATIVE_ALT",
                     "photo_altitude_ignored": True,
                     "order_mode": "MANUAL" if preserve_order else "NEAREST"}
