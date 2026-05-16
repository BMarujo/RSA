import copy
import json
import logging
import math
import os
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt

log = logging.getLogger("obu")
logging.basicConfig(
    level=logging.DEBUG,
    format="%(message)s",
)


MCM_TYPE_DEFAULT = 8
MCM_ACTION_REQUEST = 1
MCM_ACTION_ACCEPT = 2
MCM_ACTION_REJECT = 3
MAX_MANOEUVRE_ID = 255

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


def clamp_int(
    value: Any,
    default: int = 0,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    try:
        out = int(round(float(value)))
    except (TypeError, ValueError):
        out = default

    if minimum is not None and out < minimum:
        out = minimum
    if maximum is not None and out > maximum:
        out = maximum
    return out


def heading_deg_to_etsi(value: Optional[float]) -> int:
    if value is None:
        return 3601

    deg = float(value) % 360.0
    scaled = int(round(deg * 10.0))

    if scaled >= 3600:
        scaled = 0

    return clamp_int(scaled, default=3601, minimum=0, maximum=3601)


def normalize_heading_deg(value: Any) -> Optional[float]:
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
        self.default_speed_mode = int(env("DEFAULT_SPEED_MODE", "0"))
        self.priority_speed_mode = int(env("PRIORITY_SPEED_MODE", "0"))
        self.priority_distance = float(env("PRIORITY_DISTANCE", "40.0"))

        self.cam_period_s = int(env("CAM_PERIOD_MS", "100")) / 1000.0
        self.fsm_period_s = int(env("FSM_PERIOD_MS", "100")) / 1000.0
        self.actuator_period_s = int(env("ACTUATOR_PERIOD_MS", "100")) / 1000.0
        self.status_period_s = int(env("STATUS_PERIOD_MS", "250")) / 1000.0

        self.merge_point_x = float(env("MERGE_POINT_X", "0"))
        self.merge_point_y = float(env("MERGE_POINT_Y", "0"))
        self.merge_lane_index = int(env("MERGE_LANE_INDEX", "0"))
        self.merge_zone_clearance_m = float(env("MERGE_ZONE_CLEARANCE_M", "45.0"))
        self.merge_stop_margin_m = float(env("MERGE_STOP_MARGIN_M", "18.0"))
        self.merge_blocked_approach_s = float(env("MERGE_BLOCKED_APPROACH_S", "4.0"))
        self.eta_threshold_s = float(env("ETA_THRESHOLD_S", "5.0"))
        self.safe_headway_s = float(env("SAFE_HEADWAY_S", "1.5"))
        self.negotiation_timeout_s = float(env("NEGOTIATION_TIMEOUT_S", "2.0"))
        self.request_retry_s = float(env("REQUEST_RETRY_S", "0.5"))
        self.response_period_s = float(env("RESPONSE_PERIOD_S", "0.5"))
        self.neighbor_timeout_s = float(env("NEIGHBOR_TIMEOUT_S", "1.0"))
        self.yield_speed_delta = float(env("YIELD_SPEED_DELTA", "3.0"))
        self.abort_speed = float(env("ABORT_SPEED", "2.0"))
        self.abort_cooldown_s = float(env("ABORT_COOLDOWN_S", "3.0"))
        self.min_speed = float(env("MIN_SPEED", "0.5"))
        self.emergency_min_speed = float(env("EMERGENCY_MIN_SPEED", "0.0"))
        self.min_clearance_m = float(env("MIN_CLEARANCE_M", "8.0"))
        self.final_merge_guard_m = float(env("FINAL_MERGE_GUARD_M", "28.0"))
        self.final_merge_clearance_m = float(env("FINAL_MERGE_CLEARANCE_M", "10.0"))
        self.merge_occupancy_s = float(env("MERGE_OCCUPANCY_S", "3.0"))
        self.min_merge_entry_speed = float(env("MIN_MERGE_ENTRY_SPEED", "5.0"))
        self.merge_entry_speed_guard_m = float(env("MERGE_ENTRY_SPEED_GUARD_M", "35.0"))
        self.max_speed_step_up = float(env("MAX_SPEED_STEP_UP", "2.5"))
        self.max_speed_step_down = float(env("MAX_SPEED_STEP_DOWN", "0.45"))
        self.max_speed_step_emergency = float(env("MAX_SPEED_STEP_EMERGENCY", "1.5"))
        self.merge_yield_floor_ratio = float(env("MERGE_YIELD_FLOOR_RATIO", "0.2"))
        self.host_yield_floor_ratio = float(env("HOST_YIELD_FLOOR_RATIO", "0.2"))
        self.host_reject_distance_m = float(env("HOST_REJECT_DISTANCE_M", "20.0"))
        self.host_same_lane_guard_gap = float(env("HOST_SAME_LANE_GUARD_GAP", "14.0"))
        self.ramp_platoon_headway_s = float(env("RAMP_PLATOON_HEADWAY_S", "1.4"))
        self.ramp_platoon_min_gap = float(env("RAMP_PLATOON_MIN_GAP", "14.0"))
        self.ramp_platoon_speed_delta = float(env("RAMP_PLATOON_SPEED_DELTA", "0.8"))
        self.merge_queue_release_gap = float(env("MERGE_QUEUE_RELEASE_GAP", "34.0"))
        self.enable_cam_following = env("ENABLE_CAM_FOLLOWING", "true").lower() == "true"
        self.cam_follow_headway_s = float(env("CAM_FOLLOW_HEADWAY_S", "1.2"))
        self.cam_follow_min_gap = float(env("CAM_FOLLOW_MIN_GAP", "10.0"))
        self.cam_follow_lookahead = float(env("CAM_FOLLOW_LOOKAHEAD", "50.0"))
        self.cam_follow_lateral_tolerance = float(env("CAM_FOLLOW_LATERAL_TOLERANCE_M", "3.8"))
        self.cam_follow_speed_delta = float(env("CAM_FOLLOW_SPEED_DELTA", "0.8"))
        self.cam_follow_critical_gap = float(env("CAM_FOLLOW_CRITICAL_GAP_M", "6.0"))
        self.cam_follow_brake_decel = float(env("CAM_FOLLOW_BRAKE_DECEL", "4.5"))
        self.cam_follow_emergency_decel = float(env("CAM_FOLLOW_EMERGENCY_DECEL", "9.0"))
        self.ramp_edge_ids = parse_csv(env("RAMP_EDGE_IDS", "ramp_in"))
        self.main_edge_ids = parse_csv(env("MAIN_EDGE_IDS", "main_in,main_out"))
        self.ramp_station_ids = {int(item) for item in parse_csv(env("RAMP_STATION_IDS", "")) if item.isdigit()}
        self.is_ramp_vehicle = self.station_id in self.ramp_station_ids
        self.merge_completed = False
        self.merge_committed = False
        self.merge_committed_since = 0.0
        self.merge_commit_timeout_s = float(env("MERGE_COMMIT_TIMEOUT_S", "8.0"))
        
        self.main_station_ids = {int(item) for item in parse_csv(env("MAIN_STATION_IDS", "")) if item.isdigit()}
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
        self.fsm_state_since = self._sim_time()
        self.effective_role = self.role
        self.following_active = False
        self.following_station_id: Optional[int] = None
        self.following_gap_m: Optional[float] = None
        self.following_reason = ""

        self.first_sensor_time: Optional[float] = None
        self.merge_neighbor_warmup_s = float(env("MERGE_NEIGHBOR_WARMUP_S", "1.0"))
        self.merge_min_neighbors_before_merge = int(env("MERGE_MIN_NEIGHBORS_BEFORE_MERGE", "0"))
        self.merge_conflict_follow_distance_m = float(env("MERGE_CONFLICT_FOLLOW_DISTANCE_M", "55.0"))
        self.merge_conflict_floor_ratio = float(env("MERGE_CONFLICT_FLOOR_RATIO", "0.55"))

    def connect(self) -> None:
        last_error = None

        for attempt in range(40):
            try:
                self.client.connect(self.local_mqtt_host, self.local_mqtt_port, 60)
                break
            except OSError as exc:
                last_error = exc
                log.debug(
                    "MQTT local broker not ready for %s, retry=%d error=%s",
                    self.vehicle_id,
                    attempt + 1,
                    exc,
                )
                time.sleep(0.25)
        else:
            raise last_error if last_error is not None else RuntimeError("MQTT connect failed")

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
            if self.first_sensor_time is None:
                try:
                    self.first_sensor_time = float(payload.get("time", 0.0))
                except (TypeError, ValueError):
                    self.first_sensor_time = self._sim_time()
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
            self.fsm_state_since = self._sim_time()

    def _current_speed(self) -> Optional[float]:
        if not self.sensor_state:
            return None
        return float(self.sensor_state.get("speed", 0.0))

    def _current_heading(self) -> Optional[float]:
        if self.sensor_state:
            heading = normalize_heading_deg(self.sensor_state.get("heading"))
            if heading is not None:
                return heading
        return normalize_heading_deg(self.last_heading)

    def _sim_time(self) -> float:
        return float(self.sensor_state.get("time", 0.0)) if self.sensor_state else 0.0

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

    def _set_target_speed(self, speed: float, emergency: bool = False) -> None:
        target = max(speed, self.emergency_min_speed if emergency else self.min_speed)
        current = self._current_speed()
        if current is not None:
            upper = max(current + self.max_speed_step_up, self.min_speed)
            step_down = self.max_speed_step_emergency if emergency else self.max_speed_step_down
            lower = max(current - step_down, self.emergency_min_speed if emergency else self.min_speed)
            target = min(max(target, lower), upper)
        self.target_speed = target

    def _prune_neighbors(self) -> None:
        now = self._sim_time()
        stale = [sid for sid, data in self.neighbors.items() if now - data.get("timestamp", 0) > self.neighbor_timeout_s]
        for sid in stale:
            self.neighbors.pop(sid, None)

    def _prune_mcm_messages(self) -> None:
        now = self._sim_time()
        ttl = max(self.neighbor_timeout_s, self.negotiation_timeout_s)
        stale = [
            sid for sid, data in self.mcm_messages.items()
            if now - float(data.get("timestamp", 0.0)) > ttl
        ]
        for sid in stale:
            self.mcm_messages.pop(sid, None)

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
            heading = normalize_heading_deg(veh.get("heading", {}).get("headingValue"))
        except AttributeError:
            return

        lat = pos.get("latitude")
        lon = pos.get("longitude")
        if lat is None or lon is None:
            return

        xy = latlon_to_xy(float(lat), float(lon), self.origin_lat, self.origin_lon)
        distance_to_merge = self._distance_to_merge(xy["x"], xy["y"])
        previous = self.neighbors.get(station_id)
        previous_distance = previous.get("distance_to_merge") if previous else None
        distance_delta = None
        if previous_distance is not None:
            distance_delta = distance_to_merge - float(previous_distance)

        self.neighbors[station_id] = {
            "x": xy["x"],
            "y": xy["y"],
            "speed": speed,
            "heading": heading,
            "distance_to_merge": distance_to_merge,
            "distance_delta": distance_delta,
            "timestamp": self._sim_time(),
        }

    def _parse_mcm_action(self, value: Any) -> Optional[int]:
        try:
            action = int(value)
        except (TypeError, ValueError):
            return None
        if action not in (MCM_ACTION_REQUEST, MCM_ACTION_ACCEPT, MCM_ACTION_REJECT):
            return None
        return action

    def _parse_received_manoeuvre_id(self, value: Any) -> Optional[int]:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            return None
        if parsed < 0 or parsed > MAX_MANOEUVRE_ID:
            return None
        return parsed

    def _handle_mcm(self, payload: Dict[str, Any]) -> None:
        if not self.sensor_state:
            return

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
        action = self._parse_mcm_action(rational.get("manoeuvreCooperationCost"))
        manoeuvre_id = self._parse_received_manoeuvre_id(basic.get("manoeuvreId"))

        if action is None or manoeuvre_id is None:
            return

        if action in (MCM_ACTION_ACCEPT, MCM_ACTION_REJECT):
            pending = self.pending_request
            if (
                pending is not None
                and station_id == pending.get("host_id")
                and manoeuvre_id == pending.get("manoeuvre_id")
            ):
                log.debug(
                    "[%.1f] %s MCM_RX: from=%d action=%s manoeuvre=%s",
                    self._sim_time(), self.vehicle_id, station_id, action, manoeuvre_id,
                )

        self.mcm_messages[station_id] = {
            "action": action,
            "manoeuvre_id": manoeuvre_id,
            "timestamp": self._sim_time(),
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

        heading = self._current_heading()
        if heading is None:
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

    def _build_mcm(self, action: int, manoeuvre_id: Optional[int]) -> Dict[str, Any]:
        mcm = copy.deepcopy(self.mcm_template)
        if not self.sensor_state:
            return mcm

        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        speed = float(self.sensor_state.get("speed", 0.0))
        latlon = xy_to_latlon(x, y, self.origin_lat, self.origin_lon)

        manoeuvre_id = self._normalize_manoeuvre_id(manoeuvre_id)
        action = clamp_int(action, default=MCM_ACTION_REQUEST)

        mcm["stationId"] = self.station_id
        basic = mcm.setdefault("basicContainer", {})
        basic["generationDeltaTime"] = clamp_int(ms_since_minute(), minimum=0, maximum=65535)
        basic["stationID"] = self.station_id
        basic["stationType"] = self.mcm_station_type
        basic["itssRole"] = self.itss_role
        basic["mcmType"] = MCM_TYPE_DEFAULT
        basic["manoeuvreId"] = manoeuvre_id

        rational = basic.setdefault("rational", {})
        rational["manoeuvreCooperationCost"] = action

        position = basic.setdefault("position", {})
        position["latitude"] = float(latlon["latitude"])
        position["longitude"] = float(latlon["longitude"])

        container = mcm.setdefault("mcmContainer", {})
        veh = container.setdefault("vehicleManoeuvreContainer", {})
        state = veh.setdefault("vehicleCurrentStateContainer", {})

        vehicle_speed = state.setdefault("vehicleSpeed", {})
        vehicle_speed["speedValue"] = clamp_int(speed, default=0, minimum=0)

        vehicle_heading = state.setdefault("vehicleHeading", {})
        vehicle_heading["value"] = heading_deg_to_etsi(self.last_heading)

        vehicle_size = state.setdefault("vehicleSize", {})
        vehicle_size["vehicleWidth"] = clamp_int(self.vehicle_width, default=1, minimum=1)
        vehicle_length = vehicle_size.setdefault("vehicleLenth", {})
        vehicle_length["vehicleLengthValue"] = clamp_int(self.vehicle_length, default=1, minimum=1)
        return mcm

    def _next_manoeuvre_id(self) -> int:
        self.mcm_seq = (self.mcm_seq + 1) % (MAX_MANOEUVRE_ID + 1)
        return self.mcm_seq

    def _normalize_manoeuvre_id(self, value: Optional[int]) -> int:
        if value is None:
            return self._next_manoeuvre_id()

        try:
            parsed = int(value)
        except (TypeError, ValueError):
            return self._next_manoeuvre_id()

        return max(0, min(MAX_MANOEUVRE_ID, parsed))

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
        now = self._sim_time()
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

        payload = {"target_speed": float(self.target_speed), "timestamp": self._sim_time()}
        self._publish_json(self.actuator_speed_topic, payload)

        if self.target_lane_index is not None:
            lane_payload = {"target_lane_index": int(self.target_lane_index), "timestamp": self._sim_time()}
            self._publish_json(self.actuator_lane_topic, lane_payload)

        if self.target_speed_mode is not None:
            mode_payload = {"speed_mode": int(self.target_speed_mode), "timestamp": self._sim_time()}
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
            "fsm_state_age_s": self._sim_time() - self.fsm_state_since,
            "distance_to_merge_m": distance,
            "merge_eta_s": eta,
            "neighbor_count": len(self.neighbors),
            "target_speed": self.target_speed,
            "target_lane_index": self.target_lane_index,
            "target_speed_mode": self.target_speed_mode,
            "following_active": self.following_active,
            "following_station_id": self.following_station_id,
            "following_gap_m": self.following_gap_m,
            "following_reason": self.following_reason,
            "pending_request": self.pending_request is not None,
            "timestamp": self._sim_time(),
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

    def _neighbor_is_approaching_merge(self, data: Dict[str, Any]) -> bool:
        distance_delta = data.get("distance_delta")
        if distance_delta is None:
            return True
        try:
            return float(distance_delta) <= 0.25
        except (TypeError, ValueError):
            return True

    def _neighbor_is_merge_candidate(self, station_id: int) -> bool:
        data = self.neighbors.get(station_id)
        if not data:
            return False

        x = float(data["x"])
        y = float(data["y"])
        distance = self._distance_to_merge(x, y)

        if distance > self.role_detection_distance:
            return False

        approaching = self._neighbor_is_approaching_merge(data)

        # A vehicle that came from the ramp is only a merge candidate while it is still
        # approaching the merge zone. Once it moves away from the merge point, treat it
        # as downstream/main traffic.
        if station_id in self.ramp_station_ids:
            if not approaching:
                return False

            if self.ramp_bbox is not None:
                min_x, min_y, max_x, max_y = self.ramp_bbox
                inside_ramp_box = min_x <= x <= max_x and min_y <= y <= max_y
                return inside_ramp_box or distance <= self.priority_distance

            return True

        if self.merge_station_id is not None and station_id == self.merge_station_id:
            return approaching

        if self.ramp_bbox is not None:
            min_x, min_y, max_x, max_y = self.ramp_bbox
            return approaching and min_x <= x <= max_x and min_y <= y <= max_y

        return approaching and y <= self.ramp_y_threshold

    def _neighbor_is_main_candidate(self, station_id: int) -> bool:
        data = self.neighbors.get(station_id)
        if not data:
            return False

        distance = self._distance_to_merge(float(data["x"]), float(data["y"]))
        if distance > self.role_detection_distance:
            return False

        approaching = self._neighbor_is_approaching_merge(data)

        if station_id in self.main_station_ids:
            return approaching

        if station_id in self.ramp_station_ids:
            return False

        return approaching and not self._neighbor_is_merge_candidate(station_id)

    def _all_main_clearance_ok(self) -> bool:
        """Physical hard-veto: block merge if any main-road neighbor
        that arrives at a similar time is dangerously close *ahead*
        on the merge trajectory.  Uses heading-based projection so
        vehicles on a different lane (laterally offset) don't block."""
        if not self.sensor_state:
            return False

        own_eta = self._merge_eta()
        if own_eta is None:
            return False

        own_heading = self._current_heading()
        if own_heading is None:
            return True  # No heading info — can't check, allow merge

        sx = float(self.sensor_state.get("x", 0.0))
        sy = float(self.sensor_state.get("y", 0.0))
        rad = math.radians(90 - own_heading)
        fwd_x = math.cos(rad)
        fwd_y = math.sin(rad)

        for station_id, data in self.neighbors.items():
            if not self._neighbor_is_main_candidate(station_id):
                continue

            neighbor_eta = self._neighbor_eta(station_id)
            if neighbor_eta is None:
                continue

            # Temporal gate: if ETAs are well separated, no conflict.
            if abs(neighbor_eta - own_eta) > self.safe_headway_s:
                continue

            # Directional gate: project onto our heading vector.
            dx = float(data["x"]) - sx
            dy = float(data["y"]) - sy
            longitudinal = dx * fwd_x + dy * fwd_y
            lateral = abs(-dx * fwd_y + dy * fwd_x)

            # Skip vehicles that are laterally far (different lane).
            if lateral > self.cam_follow_lateral_tolerance:
                continue

            # Only block if genuinely close ahead.
            if 0 < longitudinal <= self.min_clearance_m:
                return False

        return True

    def _merge_zone_clearance_ok(self, lead_id: Optional[int] = None, host_id: Optional[int] = None) -> bool:
        """ETA-based slot check: ensure the lead and host bounding
        our target slot are not within safe_headway_s of our own ETA.
        Only checks the two bounding vehicles, not every main neighbor,
        to avoid over-blocking in dense zipper scenarios."""
        own_eta = self._merge_eta()
        if own_eta is None:
            return False

        for station_id in (lead_id, host_id):
            if station_id is None:
                continue

            neighbor_eta = self._neighbor_eta(station_id)
            if neighbor_eta is None:
                continue

            if abs(neighbor_eta - own_eta) < self.safe_headway_s:
                return False

        return True

    def _ramp_leader(self, self_distance: float) -> Optional[tuple[int, float, float]]:
        leaders: list[tuple[float, int, float]] = []
        for station_id, data in self.neighbors.items():
            if not self._neighbor_is_merge_candidate(station_id):
                continue
            leader_distance = self._distance_to_merge(float(data["x"]), float(data["y"]))
            gap = self_distance - leader_distance
            if gap <= 0:
                continue
            leaders.append((gap, station_id, float(data.get("speed") or 0.0)))
        if not leaders:
            return None
        gap, station_id, speed = min(leaders, key=lambda item: (item[0], item[1]))
        return station_id, gap, speed

    def _arrives_before(self, eta_a: float, station_a: int, eta_b: float, station_b: int) -> bool:
        if abs(eta_a - eta_b) > 1e-3:
            return eta_a < eta_b
        return station_a < station_b

    def _self_merge_completed(self) -> bool:
        if not self.sensor_state:
            return False

        lane_id = str(self.sensor_state.get("lane_id", ""))
        lane_index = parse_lane_index(lane_id)
        edge_id = edge_id_from_lane(lane_id)

        return (
            edge_id in self.main_edge_ids
            and lane_index is not None
            and lane_index == self.merge_lane_index
        )

    def _resolve_role(self) -> str:
        if self.role_mode != "auto":
            return self.role

        if self.is_ramp_vehicle and not self.merge_completed:
            return "merge"

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
            if not self._neighbor_is_main_candidate(station_id):
                continue
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
            if not self._neighbor_is_main_candidate(station_id):
                continue
            delta = self_eta - eta
            if delta > 0:
                candidates.append((delta, station_id))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][1]

    def _send_mcm(self, action: int, manoeuvre_id: Optional[int] = None) -> None:
        if not self.enable_mcm:
            return
        payload = self._build_mcm(action, manoeuvre_id)
        self._publish_json(self.mcm_in_topic, payload)
        self.last_mcm_sent = self._sim_time()

    def _send_denm(self) -> None:
        if not self.enable_denm:
            return
        payload = self._build_denm()
        self._publish_json(self.denm_in_topic, payload)

    def step(self) -> None:
        now = self._sim_time()
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
        self._prune_mcm_messages()

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

        pre_state = self.fsm_state

        if self.effective_role == "merge":
            self._fsm_merge()
        elif self.effective_role == "host":
            self._fsm_host()
        elif self.effective_role == "lead":
            self._fsm_lead()
        self._apply_car_following()

        # --- Periodic debug log (every ~1s) ---
        now = self._sim_time()
        if not hasattr(self, '_last_debug_log'):
            self._last_debug_log = 0.0
        if now - self._last_debug_log >= 1.0:
            self._last_debug_log = now
            spd = self._current_speed()
            dist = self._self_distance_to_merge()
            eta_val = self._merge_eta()
            log.debug(
                "[%.1f] %s role=%s state=%s->%s speed=%.2f target=%.2f dist=%.1f eta=%.1f "
                "following=%s(sid=%s gap=%.1f reason=%s) neighbors=%d",
                now, self.vehicle_id, self.effective_role,
                pre_state, self.fsm_state,
                spd or 0, self.target_speed or 0,
                dist or 0, eta_val or 0,
                self.following_active, self.following_station_id,
                self.following_gap_m or 0, self.following_reason,
                len(self.neighbors),
            )

    def _final_merge_lane_clear(self, lead_id: Optional[int], host_id: Optional[int], distance_to_merge: float) -> bool:
        if distance_to_merge > self.final_merge_guard_m:
            return True

        if lead_id is None and host_id is None:
            log.debug(
                "[%.1f] %s FINAL_GUARD: REJECTED blind merge at dist=%.1f",
                self._sim_time(), self.vehicle_id, distance_to_merge
            )
            return False

        for station_id, data in self.neighbors.items():
            if not self._neighbor_is_main_candidate(station_id):
                continue
            
            n_dist = self._distance_to_merge(float(data["x"]), float(data["y"]))
            if n_dist < self.final_merge_clearance_m:
                log.debug(
                    "[%.1f] %s FINAL_GUARD: REJECTED clearance=%.1f < %.1f (sid=%d)",
                    self._sim_time(), self.vehicle_id, n_dist, self.final_merge_clearance_m, station_id
                )
                return False

        log.debug(
            "[%.1f] %s FINAL_GUARD: OK at dist=%.1f",
            self._sim_time(), self.vehicle_id, distance_to_merge
        )
        return True

    def _fsm_merge(self) -> None:
        eta = self._merge_eta()
        if eta is None:
            return

        distance_to_merge = self._self_distance_to_merge()
        if distance_to_merge is None:
            return

        # --- Check if already merged (on the target lane) ---
        lane_id = str(self.sensor_state.get("lane_id", ""))
        lane_index = parse_lane_index(lane_id)
        edge_id = edge_id_from_lane(lane_id)

        if self._self_merge_completed():
            self.merge_completed = True
            self.merge_committed = False
            self._set_state(STATE_CRUISE)
            self._set_target_speed(self.cruise_speed)
            self.target_speed_mode = self.default_speed_mode
            self.target_lane_index = None
            self.pending_request = None
            return

        # If a ramp-born vehicle has already committed to the merge, do not drop the
        # lane command just because the next FSM cycle recomputed can_merge=False.
        # Lane changes take multiple simulation steps.
        if self.merge_committed and not self.merge_completed:
            commit_age = self._sim_time() - self.merge_committed_since

            self._set_state(STATE_MERGING)
            self.target_lane_index = self.merge_lane_index
            self.target_speed_mode = self.priority_speed_mode

            keep_speed = max(
                self.min_merge_entry_speed,
                self.cruise_speed + self.merge_speed_bonus,
                self.min_speed,
            )
            self._set_target_speed(keep_speed)

            log.debug(
                "[%.1f] %s MERGE_COMMIT_KEEPALIVE: age=%.1f edge=%s lane=%s target_lane=%d",
                self._sim_time(),
                self.vehicle_id,
                commit_age,
                edge_id,
                lane_index,
                self.merge_lane_index,
            )

            if commit_age < self.merge_commit_timeout_s:
                return

            # Timeout: keep trying, but avoid permanent stale pending request.
            self.pending_request = None
            self.merge_committed_since = self._sim_time()
            return

        ramp_leader = self._ramp_leader(distance_to_merge)
        if ramp_leader is not None:
            _leader_id, ramp_gap, leader_speed = ramp_leader
            current_speed = self._current_speed() or self.cruise_speed
            desired_gap = max(
                self.ramp_platoon_min_gap,
                current_speed * self.ramp_platoon_headway_s,
                self.merge_queue_release_gap if distance_to_merge <= self.priority_distance else 0.0,
            )
            if ramp_gap < desired_gap:
                follow_speed = max(
                    leader_speed - self.ramp_platoon_speed_delta,
                    self.cruise_speed * self.merge_yield_floor_ratio,
                    self.min_speed,
                )

                self._set_state(STATE_YIELDING)
                self._set_target_speed(min(current_speed, follow_speed))
                self.target_lane_index = None

                log.debug(
                    "[%.1f] %s RAMP_PLATOON: leader=%d gap=%.1f desired=%.1f "
                    "leader_spd=%.2f follow_spd=%.2f",
                    self._sim_time(), self.vehicle_id, _leader_id,
                    ramp_gap, desired_gap, leader_speed, follow_speed,
                )

                return

        # --- ABORT cooldown: wait before retrying negotiation ---
        if self.fsm_state == STATE_ABORT:
            abort_age = self._sim_time() - self.fsm_state_since
            if abort_age < self.abort_cooldown_s:
                # Don't drop to 0 during abort — maintain a floor so
                # we can recover quickly after cooldown.
                abort_floor = max(self.cruise_speed * 0.4, self.min_speed)
                self._set_target_speed(abort_floor)
                return
            # Cooldown expired — allow re-evaluation
            self._set_state(STATE_CRUISE)

        # --- Identify neighbors by ETA to merge point ---
        lead_id = self._select_lead_candidate(eta)
        lead_eta = self._neighbor_eta(lead_id) if lead_id is not None else None

        host_id = self._select_host_candidate(eta)
        host_eta = None
        if host_id is not None:
            host_eta = self._neighbor_eta(host_id)
            if host_eta is None:
                host_id = None

        # --- Don't merge blind: after this vehicle starts receiving sensors,
        #     wait briefly for CAM/MCM neighbor state before allowing a merge.
        if distance_to_merge <= self.priority_distance and lead_id is None and host_id is None:
            first_sensor_time = self.first_sensor_time
            local_age = 0.0
            if first_sensor_time is not None:
                local_age = self._sim_time() - first_sensor_time

            too_few_neighbors = len(self.neighbors) < self.merge_min_neighbors_before_merge

            if first_sensor_time is None or local_age < self.merge_neighbor_warmup_s or too_few_neighbors:
                self._set_state(STATE_YIELDING)
                self._set_target_speed(max(self.cruise_speed * 0.5, self.min_speed))
                self.target_lane_index = None
                log.debug(
                    "[%.1f] %s WAIT_NEIGHBORS: age=%.2f neighbors=%d lead=None host=None",
                    self._sim_time(), self.vehicle_id, local_age, len(self.neighbors),
                )
                return

        lead_distance = self._neighbor_distance(lead_id) if lead_id is not None else None
        host_distance = self._neighbor_distance(host_id) if host_id is not None else None

        # --- Compute the safe ETA window [min_eta, max_eta] ---
        min_eta = lead_eta + self.safe_headway_s if lead_eta is not None else None

        # The host must arrive after the merge vehicle has reached the merge point
        # AND had time to clear the lane-change / junction conflict area.
        host_buffer_s = self.safe_headway_s + self.merge_occupancy_s
        max_eta = host_eta - host_buffer_s if host_eta is not None else None

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

        if not self._all_main_clearance_ok():
            clearance_ok = False

        # ETA-based slot check — only against our target slot's bounding vehicles
        slot_ok = self._merge_zone_clearance_ok(lead_id=lead_id, host_id=host_id)

        entry_speed_ok = True
        current_speed = self._current_speed() or 0.0

        if distance_to_merge <= self.merge_entry_speed_guard_m:
            entry_speed_ok = current_speed >= self.min_merge_entry_speed

        can_merge = (
            gap_possible
            and gap_ahead_ok
            and gap_behind_ok
            and clearance_ok
            and slot_ok
            and entry_speed_ok
        )

        log.debug(
            "[%.1f] %s MERGE_DECISION: eta=%.2f dist=%.1f "
            "lead=%s(eta=%s dist=%s) host=%s(eta=%s dist=%s) "
            "min_eta=%s max_eta=%s gap_possible=%s ahead=%s behind=%s "
            "clearance=%s slot=%s entry_speed=%s -> can_merge=%s",
            self._sim_time(), self.vehicle_id, eta, distance_to_merge,
            lead_id, f"{lead_eta:.2f}" if lead_eta else "None",
            f"{lead_distance:.1f}" if lead_distance else "None",
            host_id, f"{host_eta:.2f}" if host_eta else "None",
            f"{host_distance:.1f}" if host_distance else "None",
            f"{min_eta:.2f}" if min_eta else "None",
            f"{max_eta:.2f}" if max_eta else "None",
            gap_possible, gap_ahead_ok, gap_behind_ok,
            clearance_ok, slot_ok, entry_speed_ok, can_merge,
        )

        # --- Set priority speed mode when approaching merge zone ---
        if distance_to_merge <= self.priority_distance:
            self.target_speed_mode = self.priority_speed_mode

        if not clearance_ok and distance_to_merge <= self.priority_distance:
            self._set_state(STATE_YIELDING if host_id is None else STATE_NEGOTIATING)
            stop_distance = max(distance_to_merge - self.merge_stop_margin_m, 0.0)
            blocked_speed = stop_distance / max(self.merge_blocked_approach_s, 0.1)
            current_speed = self._current_speed() or self.cruise_speed
            self._set_target_speed(min(current_speed, blocked_speed), emergency=distance_to_merge <= self.merge_stop_margin_m + 12.0)
            self.target_lane_index = None
            log.debug(
                "[%.1f] %s BLOCKED: clearance_ok=False dist=%.1f blocked_spd=%.2f",
                self._sim_time(), self.vehicle_id, distance_to_merge, blocked_speed,
            )
            return

        # --- Far from merge point: just cruise ---
        if eta > self.eta_threshold_s:
            if self.fsm_state in (STATE_NEGOTIATING, STATE_MERGING, STATE_YIELDING):
                self._set_state(STATE_CRUISE)
            return

        # --- Close but can't merge: graduated slowdown ---
        if not can_merge and distance_to_merge <= self.cruise_speed * self.safe_headway_s:
            self._set_state(STATE_YIELDING)
            ratio = max(distance_to_merge / (self.cruise_speed * self.safe_headway_s), 0.0)
            floor_speed = max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed)
            slow_speed = max(self.cruise_speed * 0.7 * ratio + self.min_speed, floor_speed)
            self._set_target_speed(slow_speed)
            log.debug(
                "[%.1f] %s CLOSE_SLOWDOWN: dist=%.1f ratio=%.2f slow_spd=%.2f",
                self._sim_time(), self.vehicle_id, distance_to_merge, ratio, slow_speed,
            )
            return

        # --- MCM Negotiation with host ---
        response_action = None
        if host_id is not None:
            if self.pending_request is None or self.pending_request.get("host_id") != host_id:
                manoeuvre_id = self._next_manoeuvre_id()
                self.pending_request = {
                    "host_id": host_id,
                    "manoeuvre_id": manoeuvre_id,
                    "timestamp": self._sim_time(),
                }
                self._send_mcm(MCM_ACTION_REQUEST, manoeuvre_id)
                self._set_state(STATE_NEGOTIATING)
            elif self._sim_time() - self.last_mcm_sent >= self.request_retry_s:
                self._send_mcm(MCM_ACTION_REQUEST, self.pending_request["manoeuvre_id"])

            response = self.mcm_messages.get(host_id)

            if response is not None and self.pending_request is not None:
                response_time = float(response.get("timestamp", -1.0))
                request_time = float(self.pending_request.get("timestamp", 0.0))

                fresh_response = (
                    response_time >= request_time
                    and self._sim_time() - response_time <= self.neighbor_timeout_s
                )

                mid_match = (
                    fresh_response
                    and int(response.get("manoeuvre_id", -1)) == int(self.pending_request.get("manoeuvre_id", -2))
                )

                if mid_match:
                    response_action = response.get("action")
                    if response_action == MCM_ACTION_ACCEPT:
                        self.pending_request = None
                    elif response_action == MCM_ACTION_REJECT:
                        log.debug(
                            "[%.1f] %s MCM_REJECT: host=%s manoeuvre=%s -> recompute slot",
                            self._sim_time(),
                            self.vehicle_id,
                            host_id,
                            self.pending_request.get("manoeuvre_id"),
                        )
                        self.pending_request = None
                        self._set_state(STATE_NEGOTIATING)
                        self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed))
                        return

            if self.pending_request and self._sim_time() - self.pending_request["timestamp"] > self.negotiation_timeout_s:
                # Timeout — don't slam to 0, just give up this negotiation
                # and let the merge FSM re-evaluate next cycle.
                log.debug(
                    "[%.1f] %s MCM_TIMEOUT: host=%s giving up negotiation",
                    self._sim_time(), self.vehicle_id, host_id,
                )
                self.pending_request = None
                # Don't set ABORT — just go back to CRUISE to re-evaluate
                self._set_state(STATE_CRUISE)

        # --- Execute merge ONLY when gap is safe AND host accepted ---
        allowed_by_mcm = host_id is None or response_action == MCM_ACTION_ACCEPT
        final_guard_ok = self._final_merge_lane_clear(lead_id, host_id, distance_to_merge)
        
        if can_merge and allowed_by_mcm and final_guard_ok:
            was_merging = self.fsm_state == STATE_MERGING
            self._set_state(STATE_MERGING)

            if not self.merge_committed:
                self.merge_committed = True
                self.merge_committed_since = self._sim_time()

            merge_target_speed = self.cruise_speed + self.merge_speed_bonus
            self._set_target_speed(merge_target_speed)

            if not was_merging:
                log.debug(
                    "[%.1f] %s MERGING! target_spd=%.2f lane=%d",
                    self._sim_time(), self.vehicle_id, merge_target_speed, self.merge_lane_index,
                )

            self.target_lane_index = self.merge_lane_index
            self.target_speed_mode = self.priority_speed_mode
        elif not can_merge or not final_guard_ok:
            self._set_state(STATE_NEGOTIATING if host_id is not None else STATE_YIELDING)
            self.target_lane_index = None

            if distance_to_merge <= self.final_merge_guard_m:
                stop_distance = max(distance_to_merge - self.merge_stop_margin_m, 0.0)
                guard_speed = max(
                    stop_distance / max(self.merge_blocked_approach_s, 0.1),
                    self.cruise_speed * self.merge_yield_floor_ratio,
                    self.min_speed,
                )
                current_speed = self._current_speed() or self.cruise_speed
                self._set_target_speed(min(current_speed, guard_speed))

                log.debug(
                    "[%.1f] %s FINAL_GUARD_SLOWDOWN: dist=%.1f guard_spd=%.2f final=%s can_merge=%s",
                    self._sim_time(), self.vehicle_id, distance_to_merge, guard_speed,
                    final_guard_ok, can_merge,
                )


    def _is_on_main_road(self, station_id: int, x: float, y: float, heading: float) -> bool:
        """Determine if a vehicle is currently on the main road."""
        is_main_static = station_id in self.main_station_ids
        
        # Heading-based check for vehicles past the merge point
        dx = x - self.merge_point_x
        dy = y - self.merge_point_y
        rad = math.radians(90 - heading)
        passed_merge = (dx * math.cos(rad) + dy * math.sin(rad)) > 0
        
        return is_main_static or passed_merge

    def _apply_car_following(self) -> None:
        """Cap target_speed from CAM-only perception for relevant following/conflict pairs."""
        self.following_active = False
        self.following_station_id = None
        self.following_gap_m = None
        self.following_reason = ""

        if not self.enable_cam_following or not self.sensor_state:
            return

        own_heading = self._current_heading()
        if own_heading is None:
            return

        own_x = float(self.sensor_state.get("x", 0.0))
        own_y = float(self.sensor_state.get("y", 0.0))
        own_speed = self._current_speed() or 0.0
        own_dist = self._self_distance_to_merge() or 0.0
        own_is_merge = self.effective_role == "merge"

        rad = math.radians(90 - own_heading)
        fwd_x = math.cos(rad)
        fwd_y = math.sin(rad)

        min_follow_speed: Optional[float] = None
        follow_station_id: Optional[int] = None
        follow_gap: Optional[float] = None
        follow_reason = ""
        is_emergency = False

        for station_id, data in self.neighbors.items():
            nx = float(data.get("x", 0.0))
            ny = float(data.get("y", 0.0))
            if data.get("speed") is None:
                continue

            n_speed = float(data.get("speed", 0.0))
            n_dist = float(data.get("distance_to_merge", 0.0))
            neighbor_is_merge = self._neighbor_is_merge_candidate(station_id)
            neighbor_is_main = self._neighbor_is_main_candidate(station_id)

            dx = nx - own_x
            dy = ny - own_y

            longitudinal = dx * fwd_x + dy * fwd_y
            lateral = abs(-dx * fwd_y + dy * fwd_x)

            gap: Optional[float] = None
            reason = ""

            # Same-lane following: strict lateral gate only.
            if (
                0.0 < longitudinal <= self.cam_follow_lookahead
                and lateral <= self.cam_follow_lateral_tolerance
            ):
                gap = max(0.0, longitudinal - self.vehicle_length)
                reason = "same_lane_cam"

            # Merge-conflict following: MCM fallback for merge-vs-main pairs.
            # Merge-conflict following: MCM fallback for merge-vs-main pairs.
            elif own_is_merge and own_dist <= self.merge_conflict_follow_distance_m:
                # Merge conflict is NOT a physical bumper-to-bumper gap.
                # It is only a soft ETA correction for ramp vehicles near the actual merge.
                conflict_pair = neighbor_is_main
                if not conflict_pair:
                    continue

                own_eta_cf = self._merge_eta()
                neighbor_eta_cf = self._neighbor_eta(station_id)
                if own_eta_cf is None or neighbor_eta_cf is None:
                    continue

                # If ETAs are not close, this is not our conflict slot.
                if abs(own_eta_cf - neighbor_eta_cf) > self.safe_headway_s * 1.2:
                    continue

                desired_eta = neighbor_eta_cf + self.safe_headway_s

                if own_eta_cf < desired_eta:
                    soft_floor = max(self.cruise_speed * self.merge_conflict_floor_ratio, self.min_speed)
                    target = max(own_dist / max(desired_eta, 0.1), soft_floor)

                    if min_follow_speed is None or target < min_follow_speed:
                        min_follow_speed = target
                        follow_station_id = station_id
                        follow_gap = abs(own_eta_cf - neighbor_eta_cf)
                        follow_reason = "merge_conflict_eta"
                        is_emergency = False

                continue

            if gap is None:
                continue

            closing_speed = max(own_speed - n_speed, 0.0)
            brake_decel = max(self.cam_follow_brake_decel, 0.1)
            emergency_decel = max(self.cam_follow_emergency_decel, brake_decel)

            closing_buffer = (closing_speed * closing_speed) / (2.0 * brake_decel)
            safe_gap = self.cam_follow_min_gap + (own_speed * self.cam_follow_headway_s) + closing_buffer

            if gap < safe_gap:
                available_gap = max(gap - self.cam_follow_min_gap, 0.0)
                headway_speed = max(available_gap / max(self.cam_follow_headway_s, 0.1), 0.0)
                braking_speed = math.sqrt(max(0.0, (n_speed * n_speed) + (2.0 * brake_decel * available_gap)))
                emergency_speed = math.sqrt(max(0.0, (n_speed * n_speed) + (2.0 * emergency_decel * available_gap)))

                # Do not command a full stop unless the gap is physically critical.
                soft_floor = max(self.cruise_speed * 0.35, self.min_speed)
                hard_floor = max(self.cruise_speed * 0.30, self.emergency_min_speed)

                target = min(
                    max(n_speed - self.cam_follow_speed_delta, soft_floor),
                    max(headway_speed, soft_floor),
                    max(braking_speed, soft_floor),
                )

                if gap < self.cam_follow_critical_gap:
                    target = max(
                        min(target, own_speed * 0.65, emergency_speed),
                        hard_floor,
                    )

                if min_follow_speed is None or target < min_follow_speed:
                    min_follow_speed = target
                    follow_station_id = station_id
                    follow_gap = gap
                    follow_reason = reason
                    is_emergency = (
                        reason == "same_lane_cam"
                        and (
                            (gap < self.cam_follow_critical_gap * 0.75 and closing_speed > 1.4)
                            or (gap < self.cam_follow_min_gap * 0.55 and closing_speed > 1.5)
                        )
                    )

        if min_follow_speed is not None:
            self.following_active = True
            self.following_station_id = follow_station_id
            self.following_gap_m = follow_gap
            self.following_reason = follow_reason
            if self.fsm_state == STATE_CRUISE:
                self._set_state(STATE_YIELDING)
            self._set_target_speed(min_follow_speed, emergency=is_emergency)
            log.debug(
                "[%.1f] %s CAR_FOLLOW: sid=%d gap=%.1f reason=%s "
                "follow_spd=%.2f emergency=%s",
                self._sim_time(), self.vehicle_id, follow_station_id or 0,
                follow_gap or 0, follow_reason, min_follow_speed, is_emergency,
            )

    def _latest_request(self) -> Optional[Dict[str, Any]]:
        """Return the earliest-ETA real MCM REQUEST from any merge neighbor.
        Does NOT fabricate requests from CAM-only candidates."""
        now = self._sim_time()
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

        out = data.copy()
        out["station_id"] = station_id
        out["eta"] = eta
        return out

    def _fsm_host(self) -> None:
        merge_id = self._merge_candidate_id()
        request = self._latest_request()

        # Determine the merge car's station ID and manoeuvre_id
        has_real_request = request is not None
        req_station_id = None
        req_manoeuvre_id = None

        if request:
            req_station_id = request["station_id"]
            req_manoeuvre_id = request.get("manoeuvre_id") or 0
        elif merge_id is not None:
            # CAM-only yielding: can slow down physically,
            # but must NOT send ACCEPT/REJECT because there is no real manoeuvre_id.
            req_station_id = merge_id
            req_manoeuvre_id = None

        if req_station_id is None:
            self._set_state(STATE_CRUISE)
            return

        if (
            self.following_active
            and self.following_reason == "same_lane_cam"
            and self.following_gap_m is not None
            and self.following_gap_m < self.host_same_lane_guard_gap
        ):
            if has_real_request and req_station_id is not None and req_manoeuvre_id is not None:
                last_sent = self.last_mcm_response.get(req_station_id, 0)
                if self._sim_time() - last_sent >= self.response_period_s:
                    self._send_mcm(MCM_ACTION_REJECT, req_manoeuvre_id)
                    self.last_mcm_response[req_station_id] = self._sim_time()

            self._set_state(STATE_YIELDING)
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

        own_eta = self._merge_eta()
        if own_eta is None:
            return

        # --- Only yield if the merge car targets the gap directly
        #     ahead of this host (between us and our lead).
        lead_id = self._select_lead_candidate(own_eta)
        if lead_id is not None:
            lead_eta = self._neighbor_eta(lead_id)
            if lead_eta is not None and merge_eta < lead_eta:
                if has_real_request:
                    last_sent = self.last_mcm_response.get(req_station_id, 0)
                    if self._sim_time() - last_sent >= self.response_period_s:
                        self._send_mcm(MCM_ACTION_REJECT, req_manoeuvre_id)
                        self.last_mcm_response[req_station_id] = self._sim_time()

                log.debug(
                    "[%.1f] %s HOST_REJECT_NOT_MY_GAP: merge=%d merge_eta=%.2f < lead_eta=%.2f lead=%d real=%s",
                    self._sim_time(), self.vehicle_id, req_station_id, merge_eta, lead_eta, lead_id, has_real_request,
                )
                self._set_state(STATE_CRUISE)
                return

        # --- Check if yielding is safe; REJECT if not ---
        if distance <= self.host_reject_distance_m:
            if has_real_request:
                last_sent = self.last_mcm_response.get(req_station_id, 0)
                if self._sim_time() - last_sent >= self.response_period_s:
                    self._send_mcm(MCM_ACTION_REJECT, req_manoeuvre_id)
                    self.last_mcm_response[req_station_id] = self._sim_time()
            log.debug(
                "[%.1f] %s HOST_REJECT: dist=%.1f <= reject_dist=%.1f",
                self._sim_time(), self.vehicle_id, distance, self.host_reject_distance_m,
            )
            self._set_state(STATE_CRUISE)
            return

        # Compute the speed needed to arrive AFTER the merge car + headway + occupancy
        target_eta = merge_eta + self.safe_headway_s + self.merge_occupancy_s
        required_speed = distance / max(target_eta, 0.1)

        current_speed = self._current_speed() or self.cruise_speed
        required_speed = min(required_speed, current_speed)
        speed_floor = max(self.cruise_speed * self.host_yield_floor_ratio, self.min_speed)
        required_speed = max(required_speed, speed_floor)

        if required_speed < self.target_speed:
            self._set_state(STATE_YIELDING)
            self._set_target_speed(required_speed)
            log.debug(
                "[%.1f] %s HOST_YIELD: for merge=%d merge_eta=%.2f own_eta=%.2f "
                "target_eta=%.2f req_spd=%.2f dist=%.1f",
                self._sim_time(), self.vehicle_id, req_station_id,
                merge_eta, own_eta, target_eta, required_speed, distance,
            )
        else:
            self._set_state(STATE_CRUISE)

        if has_real_request:
            last_sent = self.last_mcm_response.get(req_station_id, 0)
            if self._sim_time() - last_sent >= self.response_period_s:
                self._send_mcm(MCM_ACTION_ACCEPT, req_manoeuvre_id)
                self.last_mcm_response[req_station_id] = self._sim_time()

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
