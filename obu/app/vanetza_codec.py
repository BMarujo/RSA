def vanetza_station_id(payload):
    for key in ["stationID", "stationId"]:
        if key in payload:
            return int(payload[key])
    for path in [
        ["itsPduHeader", "stationID"],
        ["itsPduHeader", "stationId"],
        ["fields", "header", "stationId"],
        ["fields", "header", "stationID"],
    ]:
        cursor = payload
        for segment in path:
            if not isinstance(cursor, dict):
                break
            cursor = cursor.get(segment)
        if isinstance(cursor, (int, str)):
            return int(cursor)
    return None


def unwrap_vanetza_cam(payload):
    cam = payload.get("fields", {}).get("cam")
    return cam if isinstance(cam, dict) else payload


def unwrap_vanetza_mcm(payload):
    mcm = payload.get("fields", {}).get("payload")
    return mcm if isinstance(mcm, dict) else payload
