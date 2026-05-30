import math


def clamp_int(value, default=0, minimum=None, maximum=None):
    try:
        out = int(round(float(value)))
    except (TypeError, ValueError):
        out = default
    if minimum is not None and out < minimum:
        out = minimum
    if maximum is not None and out > maximum:
        out = maximum
    return out


def heading_deg_to_etsi(value):
    if value is None:
        return 3601
    deg = float(value) % 360.0
    scaled = int(round(deg * 10.0))
    return 0 if scaled >= 3600 else clamp_int(scaled, 3601, 0, 3601)


def normalize_heading_deg(value):
    if value is None:
        return None
    try:
        heading = float(value)
    except (TypeError, ValueError):
        return None
    if int(round(heading)) == 3601:
        return None
    if abs(heading) > 360.0:
        heading /= 10.0
    return heading % 360.0


def xy_to_latlon(x, y, origin_lat, origin_lon):
    lat = origin_lat + (y / 111320.0)
    lon = origin_lon + (x / (111320.0 * math.cos(math.radians(origin_lat))))
    return {"latitude": lat, "longitude": lon}


def latlon_to_xy(lat, lon, origin_lat, origin_lon):
    return {
        "x": (lon - origin_lon) * 111320.0 * math.cos(math.radians(origin_lat)),
        "y": (lat - origin_lat) * 111320.0,
    }


def parse_lane_index(lane_id):
    try:
        return int(lane_id.split("_")[-1])
    except (AttributeError, TypeError, ValueError):
        return None


def edge_id_from_lane(lane_id):
    if not lane_id:
        return ""
    parts = lane_id.rsplit("_", 1)
    return parts[0] if len(parts) == 2 and parts[1].isdigit() else lane_id
