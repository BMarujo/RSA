import copy
import json
import math
import os
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt


MCM_TYPE_DEFAULT = 8
MCM_ACTION_REQUEST = 1
MCM_ACTION_ACCEPT = 2
MCM_ACTION_REJECT = 3

STATE_CRUISE = "CRUISE"
STATE_NEGOTIATING = "NEGOTIATING"
STATE_YIELDING = "YIELDING"
STATE_MERGING = "MERGING"
STATE_ABORT = "ABORT"


def env(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value is not None and value != "" else default


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def ms_since_minute() -> int:
    return int(time.time() * 1000) % 65536


def meters_per_deg_lon(lat_deg: float) -> float:
    return 111320.0 * math.cos(math.radians(lat_deg))


def xy_to_latlon(x: float, y: float, origin_lat: float, origin_lon: float) -> Dict[str, float]:
    lat = origin_lat + (y / 111320.0)
    lon = origin_lon + (x / max(1.0, meters_per_deg_lon(origin_lat)))
    return {"latitude": lat, "longitude": lon}


def latlon_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> Dict[str, float]:
    x = (lon - origin_lon) * meters_per_deg_lon(origin_lat)
    y = (lat - origin_lat) * 111320.0
    return {"x": x, "y": y}


def parse_lane_index(lane_id: str) -> Optional[int]:
    if not lane_id:
        return None
    parts = lane_id.split("_")
    try:
        return int(parts[-1])
    except ValueError:
        return None


def parse_csv(value: str) -> set[str]:
    return {item.strip() for item in value.split(",") if item.strip()}


def parse_bbox(value: str) -> Optional[tuple[float, float, float, float]]:
    if not value.strip():
        return None
    parts = [float(item.strip()) for item in value.split(",") if item.strip()]
    if len(parts) != 4:
        return None
    x1, y1, x2, y2 = parts
    return (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))


def edge_id_from_lane(lane_id: str) -> str:
    if not lane_id:
        return ""
    parts = lane_id.rsplit("_", 1)
    if len(parts) == 2 and parts[1].isdigit():
        return parts[0]
    return lane_id


def get_path(data: Dict[str, Any], *keys: str) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def vanetza_station_id(payload: Dict[str, Any]) -> Optional[int]:
    station_id = payload.get("stationID")
    if station_id is None:
        station_id = payload.get("stationId")
    if station_id is None:
        station_id = get_path(payload, "itsPduHeader", "stationID")
    if station_id is None:
        station_id = get_path(payload, "itsPduHeader", "stationId")
    if station_id is None:
        station_id = get_path(payload, "fields", "header", "stationId")
    if station_id is None:
        station_id = get_path(payload, "fields", "header", "stationID")
    if station_id is None:
        return None
    return int(station_id)


def unwrap_vanetza_cam(payload: Dict[str, Any]) -> Dict[str, Any]:
    cam = get_path(payload, "fields", "cam")
    if isinstance(cam, dict):
        return cam
    return payload


def unwrap_vanetza_mcm(payload: Dict[str, Any]) -> Dict[str, Any]:
    mcm = get_path(payload, "fields", "payload")
    if isinstance(mcm, dict):
        return mcm
    return payload


class OBUApp:
    def __init__(self) -> None:
        self.vehicle_id = env("VEHICLE_ID", "vehicle_1")
        self.station_id = int(env("STATION_ID", "1"))
        self.station_type = int(env("STATION_TYPE", "5"))
        self.mcm_station_type = int(env("MCM_STATION_TYPE", "1"))
        self.itss_role = int(env("ITSS_ROLE", "1"))
        self.role = env("VEHICLE_ROLE", "host").lower()
        self.role_mode = env("ROLE_MODE", "static").lower()
        if self.role == "auto":
            self.role_mode = "auto"
            self.role = "host"

        self.local_mqtt_host = env("LOCAL_MQTT_HOST", "127.0.0.1")
        self.local_mqtt_port = int(env("LOCAL_MQTT_PORT", "1883"))

        self.origin_lat = float(env("ORIGIN_LAT", "40.0"))
        self.origin_lon = float(env("ORIGIN_LON", "-8.0"))

        self.vehicle_length = float(env("VEHICLE_LENGTH", "4.5"))
        self.vehicle_width = float(env("VEHICLE_WIDTH", "1.9"))

        self.cruise_speed = float(env("CRUISE_SPEED", "15.0"))
        self.merge_speed_bonus = float(env("MERGE_SPEED_BONUS", "1.0"))
        self.lead_speed_bonus = float(env("LEAD_SPEED_BONUS", "1.0"))
        self.priority_merge = env("MERGE_PRIORITY", "true").lower() == "true"
        merge_station_id = int(env("MERGE_STATION_ID", "0"))
        self.merge_station_id = merge_station_id if merge_station_id > 0 else None
        self.default_speed_mode = int(env("DEFAULT_SPEED_MODE", "31"))
        self.priority_speed_mode = int(env("PRIORITY_SPEED_MODE", "0"))
        self.priority_distance = float(env("PRIORITY_DISTANCE", "40.0"))

        self.cam_period_s = int(env("CAM_PERIOD_MS", "100")) / 1000.0
        self.fsm_period_s = int(env("FSM_PERIOD_MS", "100")) / 1000.0
        self.actuator_period_s = int(env("ACTUATOR_PERIOD_MS", "100")) / 1000.0
        self.status_period_s = int(env("STATUS_PERIOD_MS", "250")) / 1000.0

        self.merge_point_x = float(env("MERGE_POINT_X", "0"))
        self.merge_point_y = float(env("MERGE_POINT_Y", "0"))
        self.merge_lane_index = int(env("MERGE_LANE_INDEX", "0"))
        self.eta_threshold_s = float(env("ETA_THRESHOLD_S", "5.0"))
        self.safe_headway_s = float(env("SAFE_HEADWAY_S", "1.5"))
        self.negotiation_timeout_s = float(env("NEGOTIATION_TIMEOUT_S", "2.0"))
        self.request_retry_s = float(env("REQUEST_RETRY_S", "0.5"))
        self.response_period_s = float(env("RESPONSE_PERIOD_S", "0.5"))
        self.neighbor_timeout_s = float(env("NEIGHBOR_TIMEOUT_S", "1.0"))
        self.yield_speed_delta = float(env("YIELD_SPEED_DELTA", "3.0"))
        self.abort_speed = float(env("ABORT_SPEED", "2.0"))
        self.min_speed = float(env("MIN_SPEED", "0.5"))
        self.min_clearance_m = float(env("MIN_CLEARANCE_M", "8.0"))
        self.ramp_edge_ids = parse_csv(env("RAMP_EDGE_IDS", "ramp_in"))
        self.main_edge_ids = parse_csv(env("MAIN_EDGE_IDS", "main_in,main_out"))
        self.ramp_y_threshold = float(env("RAMP_Y_THRESHOLD", "-1.0"))
        self.ramp_bbox = parse_bbox(env("RAMP_BBOX", ""))
        self.role_detection_distance = float(env("ROLE_DETECTION_DISTANCE", str(max(self.priority_distance * 2.0, 180.0))))

        self.desired_speed = env("DESIRED_SPEED", "")
        self.enable_mcm = env("ENABLE_MCM", "true").lower() == "true"
        self.enable_denm = env("ENABLE_DENM", "false").lower() == "true"
        self.publish_idle_actuators = env("PUBLISH_IDLE_ACTUATORS", "true").lower() == "true"

        self.sensor_topic = f"car/{self.vehicle_id}/sensors/gps"
        self.actuator_speed_topic = f"car/{self.vehicle_id}/actuators/speed"
        self.actuator_lane_topic = f"car/{self.vehicle_id}/actuators/lane"
        self.actuator_speed_mode_topic = f"car/{self.vehicle_id}/actuators/speed_mode"
        self.status_topic = f"car/{self.vehicle_id}/status/fsm"

        self.cam_in_topic = "vanetza/in/cam"
        self.mcm_in_topic = "vanetza/in/mcm"
        self.denm_in_topic = "vanetza/in/denm"

        self.cam_out_topic = "vanetza/out/cam"
        self.mcm_out_topic = "vanetza/out/mcm"
        self.denm_out_topic = "vanetza/out/denm"

        base_dir = os.path.dirname(os.path.abspath(__file__))
        template_dir = os.path.join(base_dir, "templates")
        self.cam_template = load_json(os.path.join(template_dir, "in_cam.json"))
        self.mcm_template = load_json(os.path.join(template_dir, "in_mcm.json"))
        self.denm_template = load_json(os.path.join(template_dir, "in_denm.json"))

        self.client = mqtt.Client(client_id=f"obu-{self.vehicle_id}-{os.getpid()}")
        self.client.on_message = self.on_message

        self.sensor_state: Optional[Dict[str, Any]] = None
        self.last_position: Optional[Dict[str, float]] = None
        self.last_heading: Optional[float] = None
        self.last_cam_sent = 0.0
        self.last_mcm_sent = 0.0
        self.last_fsm_step = 0.0
        self.last_actuator_sent = 0.0
        self.last_status_sent = 0.0

        self.neighbors: Dict[int, Dict[str, Any]] = {}
        self.mcm_messages: Dict[int, Dict[str, Any]] = {}
        self.pending_request: Optional[Dict[str, Any]] = None
        self.last_mcm_response: Dict[int, float] = {}
        self.mcm_seq = 0
        self.denm_seq = 0

        self.target_speed: Optional[float] = None
        self.target_lane_index: Optional[int] = None
        self.target_speed_mode: int = self.default_speed_mode

        self.fsm_state = STATE_CRUISE
        self.fsm_state_since = time.time()
        self.effective_role = self.role

    def connect(self) -> None:
        self.client.connect(self.local_mqtt_host, self.local_mqtt_port, 60)
        self.client.subscribe(self.sensor_topic)
        self.client.subscribe(self.cam_out_topic)
        self.client.subscribe(self.mcm_out_topic)
        self.client.subscribe(self.denm_out_topic)
        self.client.loop_start()

    def on_message(self, _client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except json.JSONDecodeError:
            return

        if msg.topic == self.sensor_topic:
            self.sensor_state = payload
            return

        if msg.topic == self.cam_out_topic:
            self._handle_cam(payload)
            return

        if msg.topic == self.mcm_out_topic:
            self._handle_mcm(payload)
            return

        if msg.topic == self.denm_out_topic:
            return

    def _set_state(self, state: str) -> None:
        if self.fsm_state != state:
            self.fsm_state = state
            self.fsm_state_since = time.time()

    def _current_speed(self) -> Optional[float]:
        if not self.sensor_state:
            return None
        return float(self.sensor_state.get("speed", 0.0))

    def _base_cruise_speed(self) -> float:
        if self.effective_role == "merge":
            return self.cruise_speed + self.merge_speed_bonus
        return self.cruise_speed

    def _distance_to_merge(self, x: float, y: float) -> float:
        return math.hypot(self.merge_point_x - x, self.merge_point_y - y)

    def _self_distance_to_merge(self) -> Optional[float]:
        if not self.sensor_state:
            return None
        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        return self._distance_to_merge(x, y)

    def _merge_candidate_id(self) -> Optional[int]:
        if self.merge_station_id is not None and self.merge_station_id in self.neighbors:
            return self.merge_station_id

        candidates = []
        for station_id, data in self.neighbors.items():
            if not self._neighbor_is_merge_candidate(station_id):
                continue
            eta = self._neighbor_eta(station_id)
            if eta is None:
                continue
            candidates.append((eta, station_id))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][1]

    def _set_target_speed(self, speed: float) -> None:
        self.target_speed = max(speed, self.min_speed)

    def _prune_neighbors(self) -> None:
        now = time.time()
        stale = [sid for sid, data in self.neighbors.items() if now - data.get("timestamp", 0) > self.neighbor_timeout_s]
        for sid in stale:
            self.neighbors.pop(sid, None)

    def _handle_cam(self, payload: Dict[str, Any]) -> None:
        station_id = vanetza_station_id(payload)
        if station_id is None:
            return
        if station_id == self.station_id:
            return

        try:
            cam_payload = unwrap_vanetza_cam(payload)
            cam_params = cam_payload.get("camParameters", {})
            basic = cam_params.get("basicContainer", {})
            pos = basic.get("referencePosition", {})
            high = cam_params.get("highFrequencyContainer", {})
            veh = high.get("basicVehicleContainerHighFrequency", {})
            speed = veh.get("speed", {}).get("speedValue")
            heading = veh.get("heading", {}).get("headingValue")
        except AttributeError:
            return

        lat = pos.get("latitude")
        lon = pos.get("longitude")
        if lat is None or lon is None:
            return

        xy = latlon_to_xy(float(lat), float(lon), self.origin_lat, self.origin_lon)

        self.neighbors[station_id] = {
            "x": xy["x"],
            "y": xy["y"],
            "speed": speed,
            "heading": heading,
            "timestamp": time.time(),
        }

    def _handle_mcm(self, payload: Dict[str, Any]) -> None:
        mcm_payload = unwrap_vanetza_mcm(payload)
        basic = mcm_payload.get("basicContainer", {})
        station_id = vanetza_station_id(payload)
        if station_id is None:
            station_id = basic.get("stationID")
        if station_id is None:
            station_id = basic.get("stationId")
        if station_id is None:
            return
        station_id = int(station_id)
        if station_id == self.station_id:
            return

        rational = basic.get("rational", {})
        action = rational.get("manoeuvreCooperationCost")
        manoeuvre_id = basic.get("manoeuvreId")

        self.mcm_messages[station_id] = {
            "action": action,
            "manoeuvre_id": manoeuvre_id,
            "timestamp": time.time(),
        }

    def _estimate_heading(self, x: float, y: float) -> Optional[float]:
        if self.last_position is None:
            return None
        dx = x - self.last_position["x"]
        dy = y - self.last_position["y"]
        if abs(dx) < 1e-3 and abs(dy) < 1e-3:
            return None
        return math.degrees(math.atan2(dx, dy)) % 360.0

    def _build_cam(self) -> Dict[str, Any]:
        cam = copy.deepcopy(self.cam_template)
        if not self.sensor_state:
            return cam

        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        speed = float(self.sensor_state.get("speed", 0.0))

        latlon = xy_to_latlon(x, y, self.origin_lat, self.origin_lon)

        cam["stationId"] = self.station_id
        cam_params = cam.setdefault("camParameters", {})
        basic = cam_params.setdefault("basicContainer", {})
        ref = basic.setdefault("referencePosition", {})
        ref["latitude"] = latlon["latitude"]
        ref["longitude"] = latlon["longitude"]
        basic["stationType"] = self.station_type

        high = cam_params.setdefault("highFrequencyContainer", {})
        veh = high.setdefault("basicVehicleContainerHighFrequency", {})
        veh_speed = veh.setdefault("speed", {})
        veh_speed["speedValue"] = speed

        heading = self._estimate_heading(x, y)
        if heading is not None:
            veh_heading = veh.setdefault("heading", {})
            veh_heading["headingValue"] = heading
            self.last_heading = heading

        vehicle_length = veh.setdefault("vehicleLength", {})
        vehicle_length["vehicleLengthValue"] = self.vehicle_length
        veh["vehicleWidth"] = self.vehicle_width

        cam["generationDeltaTime"] = ms_since_minute()
        self.last_position = {"x": x, "y": y}
        return cam

    def _build_mcm(self, action: int, manoeuvre_id: int) -> Dict[str, Any]:
        mcm = copy.deepcopy(self.mcm_template)
        if not self.sensor_state:
            return mcm

        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        speed = float(self.sensor_state.get("speed", 0.0))
        latlon = xy_to_latlon(x, y, self.origin_lat, self.origin_lon)

        mcm["stationId"] = self.station_id
        basic = mcm.setdefault("basicContainer", {})
        basic["generationDeltaTime"] = ms_since_minute()
        basic["stationID"] = self.station_id
        basic["stationType"] = self.mcm_station_type
        basic["itssRole"] = self.itss_role
        basic["mcmType"] = MCM_TYPE_DEFAULT
        basic["manoeuvreId"] = manoeuvre_id

        rational = basic.setdefault("rational", {})
        rational["manoeuvreCooperationCost"] = action

        position = basic.setdefault("position", {})
        position["latitude"] = latlon["latitude"]
        position["longitude"] = latlon["longitude"]

        container = mcm.setdefault("mcmContainer", {})
        veh = container.setdefault("vehicleManoeuvreContainer", {})
        state = veh.setdefault("vehicleCurrentStateContainer", {})
        vehicle_speed = state.setdefault("vehicleSpeed", {})
        vehicle_speed["speedValue"] = speed
        if self.last_heading is not None:
            vehicle_heading = state.setdefault("vehicleHeading", {})
            vehicle_heading["value"] = self.last_heading

        vehicle_size = state.setdefault("vehicleSize", {})
        vehicle_size["vehicleWidth"] = self.vehicle_width
        vehicle_length = vehicle_size.setdefault("vehicleLenth", {})
        vehicle_length["vehicleLengthValue"] = self.vehicle_length
        return mcm

    def _build_denm(self) -> Dict[str, Any]:
        denm = copy.deepcopy(self.denm_template)
        if not self.sensor_state:
            return denm

        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        latlon = xy_to_latlon(x, y, self.origin_lat, self.origin_lon)

        management = denm.setdefault("management", {})
        action_id = management.setdefault("actionId", {})
        self.denm_seq += 1
        action_id["originatingStationId"] = self.station_id
        action_id["sequenceNumber"] = self.denm_seq
        now = time.time()
        management["referenceTime"] = now
        management["detectionTime"] = now
        management["stationType"] = self.station_type
        event_position = management.setdefault("eventPosition", {})
        event_position["latitude"] = latlon["latitude"]
        event_position["longitude"] = latlon["longitude"]
        return denm

    def _publish_json(self, topic: str, payload: Dict[str, Any]) -> None:
        self.client.publish(topic, json.dumps(payload))

    def _publish_actuators(self) -> None:
        if self.target_speed is None:
            if not self.publish_idle_actuators or not self.sensor_state:
                return
            self.target_speed = float(self.sensor_state.get("speed", 0.0))

        payload = {"target_speed": float(self.target_speed), "timestamp": time.time()}
        self._publish_json(self.actuator_speed_topic, payload)

        if self.target_lane_index is not None:
            lane_payload = {"target_lane_index": int(self.target_lane_index), "timestamp": time.time()}
            self._publish_json(self.actuator_lane_topic, lane_payload)

        if self.target_speed_mode is not None:
            mode_payload = {"speed_mode": int(self.target_speed_mode), "timestamp": time.time()}
            self._publish_json(self.actuator_speed_mode_topic, mode_payload)

    def _publish_status(self) -> None:
        distance = self._self_distance_to_merge()
        eta = self._merge_eta()
        payload: Dict[str, Any] = {
            "vehicle_id": self.vehicle_id,
            "station_id": self.station_id,
            "role": self.role,
            "role_mode": self.role_mode,
            "effective_role": self.effective_role,
            "fsm_state": self.fsm_state,
            "fsm_state_age_s": time.time() - self.fsm_state_since,
            "distance_to_merge_m": distance,
            "merge_eta_s": eta,
            "neighbor_count": len(self.neighbors),
            "target_speed": self.target_speed,
            "target_lane_index": self.target_lane_index,
            "target_speed_mode": self.target_speed_mode,
            "pending_request": self.pending_request is not None,
            "timestamp": time.time(),
        }
        if self.sensor_state:
            payload["lane_id"] = self.sensor_state.get("lane_id")
            payload["speed"] = self.sensor_state.get("speed")
        self._publish_json(self.status_topic, payload)

    def _merge_eta(self) -> Optional[float]:
        if not self.sensor_state:
            return None
        speed = float(self.sensor_state.get("speed", 0.0))
        distance = self._self_distance_to_merge()
        if distance is None:
            return None
        return distance / max(speed, 0.1)

    def _neighbor_eta(self, station_id: int) -> Optional[float]:
        data = self.neighbors.get(station_id)
        if not data:
            return None
        speed = data.get("speed")
        if speed is None:
            return None
        distance = self._distance_to_merge(data["x"], data["y"])
        return distance / max(float(speed), 0.1)

    def _neighbor_distance(self, station_id: int) -> Optional[float]:
        data = self.neighbors.get(station_id)
        if not data or not self.sensor_state:
            return None
        sx = float(self.sensor_state.get("x", 0.0))
        sy = float(self.sensor_state.get("y", 0.0))
        dx = data["x"] - sx
        dy = data["y"] - sy
        return math.hypot(dx, dy)

    def _neighbor_etas(self) -> list[tuple[float, int]]:
        etas: list[tuple[float, int]] = []
        for station_id in self.neighbors:
            eta = self._neighbor_eta(station_id)
            if eta is None:
                continue
            etas.append((eta, station_id))
        etas.sort(key=lambda item: (item[0], item[1]))
        return etas

    def _lane_edge_id(self) -> str:
        if not self.sensor_state:
            return ""
        return edge_id_from_lane(str(self.sensor_state.get("lane_id", "")))

    def _self_is_on_ramp(self) -> bool:
        if not self.sensor_state:
            return False
        edge_id = self._lane_edge_id()
        if edge_id in self.ramp_edge_ids:
            return True
        y = float(self.sensor_state.get("y", 0.0))
        distance = self._self_distance_to_merge()
        return y <= self.ramp_y_threshold and (distance is None or distance <= self.role_detection_distance)

    def _neighbor_is_merge_candidate(self, station_id: int) -> bool:
        data = self.neighbors.get(station_id)
        if not data:
            return False
        if self.merge_station_id is not None and station_id == self.merge_station_id:
            return True
        x = float(data["x"])
        y = float(data["y"])
        distance = self._distance_to_merge(x, y)
        if distance > self.role_detection_distance:
            return False
        if self.ramp_bbox is not None:
            min_x, min_y, max_x, max_y = self.ramp_bbox
            return min_x <= x <= max_x and min_y <= y <= max_y
        return y <= self.ramp_y_threshold

    def _arrives_before(self, eta_a: float, station_a: int, eta_b: float, station_b: int) -> bool:
        if abs(eta_a - eta_b) > 1e-3:
            return eta_a < eta_b
        return station_a < station_b

    def _resolve_role(self) -> str:
        if self.role_mode != "auto":
            return self.role
        if self._self_is_on_ramp():
            return "merge"

        self_eta = self._merge_eta()
        merge_id = self._merge_candidate_id()
        merge_eta = self._neighbor_eta(merge_id) if merge_id is not None else None
        if self_eta is None or merge_id is None or merge_eta is None:
            return "host"

        if self._arrives_before(self_eta, self.station_id, merge_eta, merge_id):
            return "lead"
        return "host"

    def _select_host_candidate(self, self_eta: float) -> Optional[int]:
        candidates = []
        for eta, station_id in self._neighbor_etas():
            delta = eta - self_eta
            if delta >= 0:
                candidates.append((delta, station_id))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][1]

    def _select_lead_candidate(self, self_eta: float) -> Optional[int]:
        candidates = []
        for eta, station_id in self._neighbor_etas():
            delta = self_eta - eta
            if delta > 0:
                candidates.append((delta, station_id))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][1]

    def _send_mcm(self, action: int, manoeuvre_id: int) -> None:
        if not self.enable_mcm:
            return
        payload = self._build_mcm(action, manoeuvre_id)
        self._publish_json(self.mcm_in_topic, payload)
        self.last_mcm_sent = time.time()

    def _send_denm(self) -> None:
        if not self.enable_denm:
            return
        payload = self._build_denm()
        self._publish_json(self.denm_in_topic, payload)

    def step(self) -> None:
        now = time.time()
        if now - self.last_cam_sent >= self.cam_period_s:
            cam_payload = self._build_cam()
            self._publish_json(self.cam_in_topic, cam_payload)
            self.last_cam_sent = now

        if now - self.last_fsm_step >= self.fsm_period_s:
            self._step_fsm()
            self.last_fsm_step = now

        if now - self.last_actuator_sent >= self.actuator_period_s:
            self._publish_actuators()
            self.last_actuator_sent = now

        if now - self.last_status_sent >= self.status_period_s:
            self._publish_status()
            self.last_status_sent = now

    def _step_fsm(self) -> None:
        if not self.sensor_state:
            return

        self._prune_neighbors()

        # Each FSM method is responsible for setting its own targets.
        # We set safe defaults here but do NOT reset mid-operation.
        self.effective_role = self._resolve_role()
        default_speed = self._base_cruise_speed()
        if self.desired_speed != "":
            try:
                default_speed = float(self.desired_speed)
            except ValueError:
                pass

        self.target_speed = max(default_speed, self.min_speed)
        self.target_lane_index = None
        self.target_speed_mode = self.default_speed_mode
        if self.effective_role == "merge":
            self._fsm_merge()
        elif self.effective_role == "host":
            self._fsm_host()
        elif self.effective_role == "lead":
            self._fsm_lead()

    def _fsm_merge(self) -> None:
        eta = self._merge_eta()
        if eta is None:
            return

        distance_to_merge = self._self_distance_to_merge()
        if distance_to_merge is None:
            return

        # --- Check if already merged (on the target lane) ---
        lane_id = self.sensor_state.get("lane_id", "")
        lane_index = parse_lane_index(lane_id)
        if lane_index is not None and lane_index == self.merge_lane_index:
            # On main road already — we merged successfully
            on_main = "main_out" in lane_id or "main_in" in lane_id or ":merge_point" in lane_id
            if on_main:
                self._set_state(STATE_CRUISE)
                self._set_target_speed(self.cruise_speed)
                self.target_speed_mode = self.default_speed_mode
                self.pending_request = None
                return

        # --- Identify neighbors by ETA to merge point ---
        lead_id = self._select_lead_candidate(eta)
        lead_eta = self._neighbor_eta(lead_id) if lead_id is not None else None

        host_id = self._select_host_candidate(eta)
        host_eta = None
        if host_id is not None:
            host_eta = self._neighbor_eta(host_id)
            if host_eta is None:
                host_id = None

        lead_distance = self._neighbor_distance(lead_id) if lead_id is not None else None
        host_distance = self._neighbor_distance(host_id) if host_id is not None else None

        # --- Compute the safe ETA window [min_eta, max_eta] ---
        min_eta = lead_eta + self.safe_headway_s if lead_eta is not None else None
        max_eta = host_eta - self.safe_headway_s if host_eta is not None else None

        gap_possible = True
        if min_eta is not None and max_eta is not None and max_eta < min_eta:
            gap_possible = False

        # --- Adjust speed to aim for the gap ---
        desired_eta = eta
        if gap_possible:
            if min_eta is not None and desired_eta < min_eta:
                desired_eta = min_eta
            if max_eta is not None and desired_eta > max_eta:
                desired_eta = max_eta
        else:
            # No gap — aim to arrive AFTER the host car passes
            if host_eta is not None:
                desired_eta = max(desired_eta, host_eta + self.safe_headway_s)
            elif min_eta is not None:
                desired_eta = max(desired_eta, min_eta + self.safe_headway_s)

        # --- Speed adjustment based on desired ETA ---
        if desired_eta > eta + 0.05:
            # Need to slow down to arrive later
            adjusted_speed = distance_to_merge / max(desired_eta, 0.1)
            # Graduated deceleration: don't drop below 40% of cruise
            floor_speed = max(self.cruise_speed * 0.4, self.min_speed)
            self._set_target_speed(max(adjusted_speed, floor_speed))
        elif desired_eta < eta - 0.05:
            # Need to speed up to arrive earlier (gap is ahead of us)
            adjusted_speed = distance_to_merge / max(desired_eta, 0.1)
            ceiling_speed = self.cruise_speed + 2.0 * self.merge_speed_bonus
            self._set_target_speed(min(adjusted_speed, ceiling_speed))
        else:
            # On track — maintain merge speed
            self._set_target_speed(self.cruise_speed + self.merge_speed_bonus)

        # --- Clearance checks ---
        gap_ahead_ok = min_eta is None or eta >= min_eta
        gap_behind_ok = max_eta is None or eta <= max_eta
        clearance_ok = True
        if lead_distance is not None and lead_distance <= self.min_clearance_m:
            clearance_ok = False
        if host_distance is not None and host_distance <= self.min_clearance_m:
            clearance_ok = False
        # NOTE: no lead car visible = road is clear ahead (removed inverted check)

        can_merge = gap_possible and gap_ahead_ok and gap_behind_ok and clearance_ok

        # --- Set priority speed mode when approaching merge zone ---
        # Enable earlier and more aggressively so SUMO doesn't block acceleration
        if self.priority_merge and distance_to_merge <= self.priority_distance:
            self.target_speed_mode = self.priority_speed_mode

        # --- Far from merge point: just cruise ---
        if eta > self.eta_threshold_s:
            if self.fsm_state in (STATE_NEGOTIATING, STATE_MERGING, STATE_YIELDING):
                self._set_state(STATE_CRUISE)
            return

        # --- Close but can't merge: graduated slowdown, not hard stop ---
        if not can_merge and distance_to_merge <= self.cruise_speed * self.safe_headway_s:
            self._set_state(STATE_YIELDING)
            # Proportional deceleration based on distance remaining
            ratio = max(distance_to_merge / (self.cruise_speed * self.safe_headway_s), 0.0)
            slow_speed = max(self.cruise_speed * 0.3 * ratio + self.min_speed, self.min_speed)
            self._set_target_speed(slow_speed)
            # Keep priority speed mode so we can accelerate away quickly
            if self.priority_merge:
                self.target_speed_mode = self.priority_speed_mode
            return

        # --- MCM Negotiation with host ---
        response_action = None
        if host_id is not None:
            if self.pending_request is None or self.pending_request.get("host_id") != host_id:
                self.mcm_seq += 1
                self.pending_request = {
                    "host_id": host_id,
                    "manoeuvre_id": self.mcm_seq,
                    "timestamp": time.time(),
                }
                self._send_mcm(MCM_ACTION_REQUEST, self.mcm_seq)
                self._set_state(STATE_NEGOTIATING)
            elif time.time() - self.last_mcm_sent >= self.request_retry_s:
                self._send_mcm(MCM_ACTION_REQUEST, self.pending_request["manoeuvre_id"])

            response = self.mcm_messages.get(host_id)
            if response is not None and self.pending_request is not None:
                # Accept response if manoeuvre_id matches OR if priority merge
                mid_match = response.get("manoeuvre_id") == self.pending_request.get("manoeuvre_id")
                if mid_match or self.priority_merge:
                    response_action = response.get("action")
                    if response_action == MCM_ACTION_ACCEPT:
                        self.pending_request = None
                    elif response_action == MCM_ACTION_REJECT and not self.priority_merge:
                        self._set_state(STATE_ABORT)
                        self._set_target_speed(max(self.abort_speed, self.min_speed))
                        self._send_denm()
                        self.pending_request = None
                        return

            if self.pending_request and time.time() - self.pending_request["timestamp"] > self.negotiation_timeout_s:
                if not self.priority_merge:
                    self._set_state(STATE_ABORT)
                    self._set_target_speed(max(self.abort_speed, self.min_speed))
                    self._send_denm()
                    self.pending_request = None
                    return
                # Priority merge: proceed without response
                self.pending_request = None

        # --- Execute merge or continue negotiating ---
        allowed_by_mcm = self.priority_merge or host_id is None or response_action == MCM_ACTION_ACCEPT
        if can_merge and allowed_by_mcm:
            self._set_state(STATE_MERGING)
            merge_target_speed = self.cruise_speed + 2.0 * self.merge_speed_bonus
            self._set_target_speed(merge_target_speed)
            self.target_lane_index = self.merge_lane_index
            self.target_speed_mode = self.priority_speed_mode
        elif not can_merge:
            self._set_state(STATE_NEGOTIATING if host_id is not None else STATE_YIELDING)

    def _latest_request(self) -> Optional[Dict[str, Any]]:
        now = time.time()
        merge_id = self._merge_candidate_id()
        if merge_id is not None:
            merge_eta = self._neighbor_eta(merge_id)
            if merge_eta is not None and merge_eta <= self.eta_threshold_s:
                # Look up actual manoeuvre_id from MCM messages if available
                mcm_data = self.mcm_messages.get(merge_id, {})
                manoeuvre_id = mcm_data.get("manoeuvre_id", 0)
                return {
                    "station_id": merge_id,
                    "manoeuvre_id": manoeuvre_id,
                    "timestamp": now,
                    "eta": merge_eta,
                }
        requests = []
        for station_id, data in self.mcm_messages.items():
            if data.get("action") != MCM_ACTION_REQUEST:
                continue
            if now - data.get("timestamp", 0) > self.neighbor_timeout_s:
                continue
            eta = self._neighbor_eta(station_id)
            if eta is None:
                continue
            requests.append((eta, station_id, data))
        if not requests:
            return None
        requests.sort(key=lambda item: (item[0], item[1]))
        eta, station_id, data = requests[0]
        data = data.copy()
        data["station_id"] = station_id
        data["eta"] = eta
        return data

    def _fsm_host(self) -> None:
        merge_id = self._merge_candidate_id()
        request = self._latest_request()

        # Determine the merge car's station ID and manoeuvre_id
        req_station_id = None
        req_manoeuvre_id = 0
        if request:
            req_station_id = request["station_id"]
            req_manoeuvre_id = request.get("manoeuvre_id") or 0
        elif merge_id is not None:
            req_station_id = merge_id

        if req_station_id is None:
            self._set_state(STATE_CRUISE)
            return

        merge_eta = self._neighbor_eta(req_station_id)
        if merge_eta is None:
            self._set_state(STATE_CRUISE)
            return

        merge_distance = None
        if req_station_id in self.neighbors:
            merge_distance = self._distance_to_merge(
                self.neighbors[req_station_id]["x"],
                self.neighbors[req_station_id]["y"],
            )

        # Only yield if the merge car is within the priority zone
        if merge_distance is not None and merge_distance > self.priority_distance:
            self._set_state(STATE_CRUISE)
            return

        distance = self._self_distance_to_merge()
        if distance is None:
            return

        # Compute the speed needed to arrive AFTER the merge car + headway
        gap_buffer = self.safe_headway_s * 0.5
        target_eta = merge_eta + self.safe_headway_s + gap_buffer
        required_speed = distance / max(target_eta, 0.1)

        # Proportional yield — don't drop too far below cruise
        current_speed = self._current_speed() or self.cruise_speed
        if self.priority_merge:
            # Hard yield: merge car has priority
            yield_floor = max(self.cruise_speed - self.yield_speed_delta, self.min_speed)
            required_speed = min(required_speed, yield_floor)
        required_speed = min(required_speed, current_speed)
        # Never go below 30% of cruise speed
        speed_floor = max(self.cruise_speed * 0.3, self.min_speed)
        required_speed = max(required_speed, speed_floor)

        if required_speed < self.target_speed or self.priority_merge:
            self._set_state(STATE_YIELDING)
            self._set_target_speed(required_speed)
        else:
            self._set_state(STATE_CRUISE)

        # Send MCM ACCEPT with the CORRECT manoeuvre_id
        last_sent = self.last_mcm_response.get(req_station_id, 0)
        if time.time() - last_sent >= self.response_period_s:
            self._send_mcm(MCM_ACTION_ACCEPT, req_manoeuvre_id)
            self.last_mcm_response[req_station_id] = time.time()

    def _fsm_lead(self) -> None:
        merge_id = self._merge_candidate_id()
        if merge_id is None:
            self._set_state(STATE_CRUISE)
            return

        merge_eta = self._neighbor_eta(merge_id)
        merge_distance = None
        if merge_id in self.neighbors:
            merge_distance = self._distance_to_merge(self.neighbors[merge_id]["x"], self.neighbors[merge_id]["y"])
        if merge_eta is None or merge_distance is None:
            self._set_state(STATE_CRUISE)
            return

        if merge_distance <= self.priority_distance:
            distance = self._self_distance_to_merge()
            if distance is None:
                return
            gap_buffer = self.safe_headway_s * 0.5
            target_eta = max(merge_eta - (self.safe_headway_s + gap_buffer), 0.1)
            required_speed = distance / target_eta
            current_speed = self._current_speed() or self.cruise_speed
            base_speed = self.cruise_speed + 2.0 * self.lead_speed_bonus
            final_speed = max(required_speed, current_speed, base_speed)
            self._set_state(STATE_CRUISE)
            self._set_target_speed(final_speed)
            # Override SUMO speed checks so lead can actually accelerate
            self.target_speed_mode = self.priority_speed_mode
        else:
            self._set_state(STATE_CRUISE)

    def run(self) -> None:
        self.connect()
        try:
            while True:
                self.step()
                time.sleep(0.01)
        finally:
            self.client.loop_stop()


def main() -> None:
    app = OBUApp()
    app.run()


if __name__ == "__main__":
    main()
