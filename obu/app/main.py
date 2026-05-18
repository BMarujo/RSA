import copy
import json
import logging
import math
import os
import time
from typing import Any, Dict, Optional, Tuple

import paho.mqtt.client as mqtt

log = logging.getLogger("obu")
logging.basicConfig(level=logging.DEBUG, format="%(message)s")

MCM_TYPE_DEFAULT = 8
MCM_ACTION_REQUEST = 1
MCM_ACTION_ACCEPT = 2
MCM_ACTION_REJECT = 3
MAX_MANOEUVRE_ID = 255

def mcm_action_name(action):
    if action == MCM_ACTION_REQUEST: return "REQUEST"
    if action == MCM_ACTION_ACCEPT: return "ACCEPT"
    if action == MCM_ACTION_REJECT: return "REJECT"
    return str(action)

STATE_CRUISE = "CRUISE"
STATE_NEGOTIATING = "NEGOTIATING"
STATE_YIELDING = "YIELDING"
STATE_MERGING = "MERGING"
STATE_ABORT = "ABORT"

def env(name, default):
    v = os.getenv(name)
    return v if v is not None and v != "" else default

def load_json(p):
    with open(p, "r", encoding="utf-8") as h: return json.load(h)

def ms_since_minute(): return int(time.time() * 1000) % 65536

def clamp_int(v, default=0, minimum=None, maximum=None):
    try: out = int(round(float(v)))
    except: out = default
    if minimum is not None and out < minimum: out = minimum
    if maximum is not None and out > maximum: out = maximum
    return out

def heading_deg_to_etsi(v):
    if v is None: return 3601
    deg = float(v) % 360.0
    scaled = int(round(deg * 10.0))
    return 0 if scaled >= 3600 else clamp_int(scaled, 3601, 0, 3601)

def normalize_heading_deg(v):
    if v is None: return None
    try: h = float(v)
    except: return None
    if int(round(h)) == 3601: return None
    if abs(h) > 360.0: h /= 10.0
    return h % 360.0

def xy_to_latlon(x, y, olat, olon):
    lat = olat + (y / 111320.0)
    lon = olon + (x / (111320.0 * math.cos(math.radians(olat))))
    return {"latitude": lat, "longitude": lon}

def latlon_to_xy(lat, lon, olat, olon):
    return {"x": (lon - olon) * 111320.0 * math.cos(math.radians(olat)), "y": (lat - olat) * 111320.0}

def parse_lane_index(lid):
    try: return int(lid.split("_")[-1])
    except: return None

def parse_csv(v): return {i.strip() for i in v.split(",") if i.strip()}

def parse_bbox(v):
    try:
        p = [float(i.strip()) for i in v.split(",") if i.strip()]
        if len(p) == 4: return (min(p[0], p[2]), min(p[1], p[3]), max(p[0], p[2]), max(p[1], p[3]))
    except: pass
    return None

def edge_id_from_lane(lid):
    if not lid: return ""
    p = lid.rsplit("_", 1)
    return p[0] if len(p) == 2 and p[1].isdigit() else lid

def vanetza_station_id(p):
    for k in ["stationID", "stationId"]:
        if k in p: return int(p[k])
    for path in [["itsPduHeader", "stationID"], ["itsPduHeader", "stationId"], ["fields", "header", "stationId"], ["fields", "header", "stationID"]]:
        c = p
        for s in path:
            if not isinstance(c, dict): break
            c = c.get(s)
        if isinstance(c, (int, str)): return int(c)
    return None

def unwrap_vanetza_cam(p):
    c = p.get("fields", {}).get("cam")
    return c if isinstance(c, dict) else p

def unwrap_vanetza_mcm(p):
    m = p.get("fields", {}).get("payload")
    return m if isinstance(m, dict) else p

class OBUApp:
    def __init__(self):
        self.active_merge_request_started_at = 0.0
        self.host_reservation_max_s = float(env("HOST_RESERVATION_MAX_S", "10.0"))
        self.remote_vehicle_status = {}
        self.vehicle_id = env("VEHICLE_ID", "v1")
        self.station_id = int(env("STATION_ID", "1"))
        self.station_type = int(env("STATION_TYPE", "5"))
        self.mcm_station_type = int(env("MCM_STATION_TYPE", "1"))
        self.itss_role = int(env("ITSS_ROLE", "1"))
        self.role, self.role_mode = env("VEHICLE_ROLE", "host").lower(), "static"
        if self.role == "auto": self.role, self.role_mode = "host", "auto"
        self.local_mqtt_host, self.local_mqtt_port = env("LOCAL_MQTT_HOST", "127.0.0.1"), int(env("LOCAL_MQTT_PORT", "1883"))
        self.origin_lat, self.origin_lon = float(env("ORIGIN_LAT", "40.0")), float(env("ORIGIN_LON", "-8.0"))
        self.vehicle_length, self.vehicle_width = float(env("VEHICLE_LENGTH", "4.5")), float(env("VEHICLE_WIDTH", "1.9"))
        self.cruise_speed = float(env("CRUISE_SPEED", "15.0"))
        self.merge_speed_bonus, self.lead_speed_bonus = float(env("MERGE_SPEED_BONUS", "1.0")), float(env("LEAD_SPEED_BONUS", "1.0"))
        self.priority_merge = env("MERGE_PRIORITY", "true").lower() == "true"
        msid = int(env("MERGE_STATION_ID", "0"))
        self.merge_station_id = msid if msid > 0 else None
        self.default_speed_mode, self.priority_speed_mode = int(env("DEFAULT_SPEED_MODE", "0")), int(env("PRIORITY_SPEED_MODE", "0"))
        self.priority_distance = float(env("PRIORITY_DISTANCE", "40.0"))
        self.cam_period_s, self.fsm_period_s = int(env("CAM_PERIOD_MS", "100"))/1000.0, int(env("FSM_PERIOD_MS", "100"))/1000.0
        self.actuator_period_s, self.status_period_s = int(env("ACTUATOR_PERIOD_MS", "100"))/1000.0, int(env("STATUS_PERIOD_MS", "250"))/1000.0
        self.merge_point_x, self.merge_point_y = float(env("MERGE_POINT_X", "0")), float(env("MERGE_POINT_Y", "0"))
        self.merge_lane_index, self.merge_zone_clearance_m = int(env("MERGE_LANE_INDEX", "0")), float(env("MERGE_ZONE_CLEARANCE_M", "45.0"))
        self.merge_stop_margin_m, self.merge_blocked_approach_s = float(env("MERGE_STOP_MARGIN_M", "18.0")), float(env("MERGE_BLOCKED_APPROACH_S", "4.0"))
        self.eta_threshold_s, self.safe_headway_s = float(env("ETA_THRESHOLD_S", "5.0")), float(env("SAFE_HEADWAY_S", "1.5"))
        self.negotiation_timeout_s, self.request_retry_s = float(env("NEGOTIATION_TIMEOUT_S", "2.0")), float(env("REQUEST_RETRY_S", "0.5"))
        self.response_period_s, self.neighbor_timeout_s = float(env("RESPONSE_PERIOD_S", "0.5")), float(env("NEIGHBOR_TIMEOUT_S", "1.0"))
        self.abort_speed, self.abort_cooldown_s = float(env("ABORT_SPEED", "2.0")), float(env("ABORT_COOLDOWN_S", "3.0"))
        self.min_speed, self.emergency_min_speed = float(env("MIN_SPEED", "0.5")), float(env("EMERGENCY_MIN_SPEED", "0.0"))
        self.min_clearance_m, self.final_merge_guard_m = float(env("MIN_CLEARANCE_M", "8.0")), float(env("FINAL_MERGE_GUARD_M", "28.0"))
        self.final_merge_clearance_m, self.merge_occupancy_s = float(env("FINAL_MERGE_CLEARANCE_M", "10.0")), float(env("MERGE_OCCUPANCY_S", "3.0"))
        self.merge_commit_headway_s = float(env("MERGE_COMMIT_HEADWAY_S", str(self.safe_headway_s)))
        self.min_merge_entry_speed, self.merge_entry_speed_guard_m = float(env("MIN_MERGE_ENTRY_SPEED", "5.0")), float(env("MERGE_ENTRY_SPEED_GUARD_M", "35.0"))
        self.max_speed_step_up, self.max_speed_step_down, self.max_speed_step_emergency = float(env("MAX_SPEED_STEP_UP", "2.5")), float(env("MAX_SPEED_STEP_DOWN", "0.45")), float(env("MAX_SPEED_STEP_EMERGENCY", "1.5"))
        self.merge_yield_floor_ratio, self.host_yield_floor_ratio = float(env("MERGE_YIELD_FLOOR_RATIO", "0.2")), float(env("HOST_YIELD_FLOOR_RATIO", "0.2"))
        self.host_reject_distance_m, self.host_same_lane_guard_gap = float(env("HOST_REJECT_DISTANCE_M", "20.0")), float(env("HOST_SAME_LANE_GUARD_GAP", "14.0"))
        self.ramp_platoon_headway_s, self.ramp_platoon_min_gap, self.ramp_platoon_speed_delta = float(env("RAMP_PLATOON_HEADWAY_S", "1.4")), float(env("RAMP_PLATOON_MIN_GAP", "14.0")), float(env("RAMP_PLATOON_SPEED_DELTA", "0.8"))
        self.merge_queue_release_gap = float(env("MERGE_QUEUE_RELEASE_GAP", "34.0"))
        self.enable_cam_following = env("ENABLE_CAM_FOLLOWING", "true").lower() == "true"
        self.cam_follow_headway_s, self.cam_follow_min_gap = float(env("CAM_FOLLOW_HEADWAY_S", "1.2")), float(env("CAM_FOLLOW_MIN_GAP", "10.0"))
        self.cam_follow_lookahead, self.cam_follow_lateral_tolerance = float(env("CAM_FOLLOW_LOOKAHEAD", "50.0")), float(env("CAM_FOLLOW_LATERAL_TOLERANCE_M", "3.8"))
        self.cam_follow_speed_delta, self.cam_follow_critical_gap = float(env("CAM_FOLLOW_SPEED_DELTA", "0.8")), float(env("CAM_FOLLOW_CRITICAL_GAP_M", "6.0"))
        self.cam_follow_brake_decel, self.cam_follow_emergency_decel = float(env("CAM_FOLLOW_BRAKE_DECEL", "4.5")), float(env("CAM_FOLLOW_EMERGENCY_DECEL", "9.0"))
        self.ramp_edge_ids, self.main_edge_ids = parse_csv(env("RAMP_EDGE_IDS", "ramp_in")), parse_csv(env("MAIN_EDGE_IDS", "main_in,main_out"))
        self.ramp_station_ids = {int(i) for i in parse_csv(env("RAMP_STATION_IDS", "")) if i.isdigit()}
        self.is_ramp_vehicle = self.station_id in self.ramp_station_ids
        self.merge_completed, self.merge_completed_since, self.past_merge_point, self.missed_merge_logged = False, 0.0, False, False
        self.min_distance_to_merge_seen, self.post_merge_lock_s = float("inf"), float(env("POST_MERGE_LOCK_S", "3.0"))
        self.merge_committed, self.merge_committed_since = False, 0.0
        self.merge_physical_started_once = False
        self.merge_commit_timeout_s = float(env("MERGE_COMMIT_TIMEOUT_S", "8.0"))
        self.merge_accept_timeout_s = float(env("MERGE_ACCEPT_TIMEOUT_S", str(self.merge_commit_timeout_s)))
        self.merge_commit_distance_m, self.merge_lane_prepare_distance_m = float(env("MERGE_COMMIT_DISTANCE_M", "35.0")), float(env("MERGE_LANE_PREPARE_DISTANCE_M", "95.0"))
        self.final_guard_stale_neighbor_s = float(env("FINAL_GUARD_STALE_NEIGHBOR_S", "4.0"))
        self.main_station_ids = {int(i) for i in parse_csv(env("MAIN_STATION_IDS", "")) if i.isdigit()}
        self.ramp_y_threshold = float(env("RAMP_Y_THRESHOLD", "-1.0"))
        self.ramp_bbox = parse_bbox(env("RAMP_BBOX", ""))
        self.role_detection_distance = float(env("ROLE_DETECTION_DISTANCE", "180.0"))
        self.desired_speed = env("DESIRED_SPEED", "")
        self.host_clear_lane_index, self.host_cooperative_lane_change = int(env("HOST_CLEAR_LANE_INDEX", "1")), env("HOST_COOPERATIVE_LANE_CHANGE", "false").lower() == "true"
        self.host_clear_lane_hold_s, self.host_clear_lane_until, self.host_clear_for_station = float(env("HOST_CLEAR_LANE_HOLD_S", "6.0")), 0.0, None
        self.host_return_lock_distance_m = float(env("HOST_RETURN_LOCK_DISTANCE_M", "90.0"))
        self.locked_slot, self.locked_slot_until, self.slot_lock_s, self.slot_blocked_since = None, 0.0, float(env("SLOT_LOCK_S", "4.0")), 0.0
        self.active_merge_request, self.active_merge_request_until = None, 0.0
        self.host_reservation_s, self.cooperative_host_lookback_s, self.slot_neighbor_grace_s = float(env("HOST_RESERVATION_S", "4.0")), float(env("COOPERATIVE_HOST_LOOKBACK_S", "5.0")), float(env("SLOT_NEIGHBOR_GRACE_S", "2.5"))
        self.allow_hostless_merge, self.host_min_yield_delta = env("ALLOW_HOSTLESS_MERGE", "false").lower() == "true", float(env("HOST_MIN_YIELD_DELTA", "1.0"))
        self.mcm_request_distance_m, self.mcm_retry_blocked_until = float(env("MCM_REQUEST_DISTANCE_M", "95.0")), 0.0
        self.mcm_timeout_cooldown_s, self.mcm_stale_host_grace_s = float(env("MCM_TIMEOUT_COOLDOWN_S", "1.0")), float(env("MCM_STALE_HOST_GRACE_S", "0.75"))
        self.mcm_late_host_lock_grace_s = float(env("MCM_LATE_HOST_LOCK_GRACE_S", "1.5"))
        self.mcm_late_host_lock_distance_m = float(env("MCM_LATE_HOST_LOCK_DISTANCE_M", "70.0"))
        self.merge_deadlock_since = 0.0
        self.pending_host_lost_since = 0.0
        self.merge_deadlock_timeout_s = float(env("MERGE_DEADLOCK_TIMEOUT_S", "4.0"))
        self.merge_authorized = False
        self.merge_authorized_since = 0.0
        self.merge_authorized_timeout_s = float(env("MERGE_AUTHORIZED_TIMEOUT_S", "10.0"))
        self.merge_accepted = False
        self.merge_accepted_since = 0.0
        self.accepted_slot_invalid_since = 0.0
        self.accepted_slot_invalid_timeout_s = 1.5
        self.last_accepted_wait_log = 0.0
        self.last_lclear_block_log = 0.0
        self.merge_safety_hold_since = 0.0
        self.merge_safety_hold_timeout_s = float(env("MERGE_SAFETY_HOLD_TIMEOUT_S", "2.5"))
        self.had_merge_timeout_this_attempt = False
        self.rejected_hosts_until = {}
        self.skip_car_following_this_step = False
        self.committed_lead_id, self.committed_host_id, self.committed_manoeuvre_id = None, None, None
        self.count_late_merge_recovery, self.count_merge_failed_no_gap, self.count_merge_completed, self.count_merge_completed_clean, self.recovery_triggered_this_merge = 0, 0, 0, 0, False
        self.enable_mcm, self.enable_denm, self.publish_idle_actuators = env("ENABLE_MCM", "true").lower() == "true", env("ENABLE_DENM", "false").lower() == "true", env("PUBLISH_IDLE_ACTUATORS", "true").lower() == "true"
        self.sensor_topic = f"car/{self.vehicle_id}/sensors/gps"
        self.lane_command_status_topic = f"car/{self.vehicle_id}/status/lane_command"
        self.actuator_speed_topic, self.actuator_lane_topic, self.actuator_speed_mode_topic = f"car/{self.vehicle_id}/actuators/speed", f"car/{self.vehicle_id}/actuators/lane", f"car/{self.vehicle_id}/actuators/speed_mode"
        self.status_topic, self.cam_in_topic, self.mcm_in_topic, self.denm_in_topic = f"car/{self.vehicle_id}/status/fsm", "vanetza/in/cam", "vanetza/in/mcm", "vanetza/in/denm"
        self.cam_out_topic, self.mcm_out_topic, self.denm_out_topic = "vanetza/out/cam", "vanetza/out/mcm", "vanetza/out/denm"
        base_dir = os.path.dirname(os.path.abspath(__file__))
        template_dir = os.path.join(base_dir, "templates")
        self.cam_template, self.mcm_template, self.denm_template = load_json(os.path.join(template_dir, "in_cam.json")), load_json(os.path.join(template_dir, "in_mcm.json")), load_json(os.path.join(template_dir, "in_denm.json"))
        self.client = mqtt.Client(client_id=f"obu-{self.vehicle_id}-{os.getpid()}")
        self.client.on_message = self.on_message
        self.sensor_state, self.last_position, self.last_heading = None, None, None
        self.lane_command_status, self.last_lane_command_status_key, self.last_commit_lane_apply_log_key = {}, None, None
        self.last_lane_clear_time = 0.0
        self.last_cam_sent, self.last_mcm_sent, self.last_fsm_step, self.last_actuator_sent, self.last_status_sent = 0.0, 0.0, 0.0, 0.0, 0.0
        self.neighbors, self.neighbor_memory, self.mcm_messages, self.pending_request, self.last_mcm_response = {}, {}, {}, None, {}
        self.mcm_seq, self.denm_seq, self.target_speed, self.target_lane_index, self.target_speed_mode = 0, 0, None, None, 0
        self.fsm_state, self.fsm_state_since, self.effective_role = STATE_CRUISE, 0.0, self.role
        self.following_active, self.following_station_id, self.following_gap_m, self.following_reason = False, None, None, ""
        self.first_sensor_time = None
        self.merge_neighbor_warmup_s, self.merge_min_neighbors_before_merge = float(env("MERGE_NEIGHBOR_WARMUP_S", "1.0")), int(env("MERGE_MIN_NEIGHBORS_BEFORE_MERGE", "0"))
        self.merge_conflict_follow_distance_m, self.merge_conflict_floor_ratio = float(env("MERGE_CONFLICT_FOLLOW_DISTANCE_M", "55.0")), float(env("MERGE_CONFLICT_FLOOR_RATIO", "0.55"))

    def connect(self):
        for a in range(40):
            try: self.client.connect(self.local_mqtt_host, self.local_mqtt_port, 60); break
            except: time.sleep(0.25)
        self.client.subscribe(self.sensor_topic)
        self.client.subscribe(self.lane_command_status_topic)
        self.client.subscribe(self.cam_out_topic)
        self.client.subscribe(self.mcm_out_topic)
        self.client.subscribe(self.denm_out_topic)
        self.client.subscribe("car/+/status/fsm")
        self.client.loop_start()

    def on_message(self, _c, _u, msg):
        try: p = json.loads(msg.payload.decode("utf-8"))
        except: return
        if msg.topic.startswith("car/") and msg.topic.endswith("/status/fsm"):
            try:
                sid = int(p.get("station_id"))
                vid = str(p.get("vehicle_id", ""))
            except (TypeError, ValueError):
                return
            if sid != self.station_id and vid != self.vehicle_id:
                self.remote_vehicle_status[sid] = p
            return
        if msg.topic == self.sensor_topic:
            self.sensor_state = p
            if self.first_sensor_time is None: self.first_sensor_time = float(p.get("time", self._sim_time()))
        elif msg.topic == self.lane_command_status_topic:
            self.lane_command_status = p
            state = str(p.get("state", ""))
            key = (state, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"))
            if key != self.last_lane_command_status_key:
                if state == "CLEAR": getattr(self, "_log_timeline_event", lambda *x,**y: None)("CLEAR")
                self.last_lane_command_status_key = key
                if state == "WAIT_EDGE":
                    log.debug("[%.1f] %s MERGE_LANE_WAIT_EDGE_CONFIRMED: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s", self._sim_time(), self.vehicle_id, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"), p.get("executable"))
                elif state == "APPLY":
                    log.debug("[%.1f] %s MERGE_LANE_APPLY_CONFIRMED: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s", self._sim_time(), self.vehicle_id, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"), p.get("executable"))
                elif state == "CLEAR":
                    self.last_lane_clear_time = self._sim_time()
                    log.debug("[%.1f] %s MERGE_LANE_CLEAR_CONFIRMED: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s", self._sim_time(), self.vehicle_id, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"), p.get("executable"))
        elif msg.topic == self.cam_out_topic: self._handle_cam(p)
        elif msg.topic == self.mcm_out_topic: self._handle_mcm(p)

    def _set_state(self, s):
        if self.fsm_state != s: self.fsm_state, self.fsm_state_since = s, self._sim_time()

    def _current_speed(self): return float(self.sensor_state.get("speed", 0.0)) if self.sensor_state else None
    def _current_heading(self):
        if self.sensor_state:
            h = normalize_heading_deg(self.sensor_state.get("heading"))
            if h is not None: return h
        return normalize_heading_deg(self.last_heading)
    def _sim_time(self): return float(self.sensor_state.get("time", 0.0)) if self.sensor_state else 0.0

    def _lane_command_status_fresh(self, max_age=0.5):
        if not self.lane_command_status: return False
        try: return self._sim_time() - float(self.lane_command_status.get("time", -999.0)) <= max_age
        except Exception: return False

    def _lane_command_waiting_edge(self):
        if not self._lane_command_status_fresh(): return False
        try: target_lane = int(self.lane_command_status.get("target_lane"))
        except Exception: return False
        return self.lane_command_status.get("state") == "WAIT_EDGE" and target_lane == self.merge_lane_index and self.lane_command_status.get("executable") is False

    def _lane_command_apply_active(self):
        if not self._lane_command_status_fresh(): return False
        try: target_lane = int(self.lane_command_status.get("target_lane"))
        except Exception: return False
        return self.lane_command_status.get("state") == "APPLY" and target_lane == self.merge_lane_index

    def _project_neighbor_data(self, d, now=None):
        p = d.copy(); now = now or self._sim_time()
        try:
            age = max(0.0, now - float(p.get("timestamp", now)))
            s, h = float(p.get("speed") or 0.0), p.get("heading")
            if age > 0 and h is not None and s > 0:
                rad = math.radians(90 - float(h))
                p["x"], p["y"] = float(p["x"]) + math.cos(rad) * s * age, float(p["y"]) + math.sin(rad) * s * age
                p["distance_to_merge"] = self._distance_to_merge(p["x"], p["y"])
        except: pass
        return p

    def _final_guard_neighbor_items(self):
        now, res = self._sim_time(), self.neighbors.copy()
        for s, d in self.neighbor_memory.items():
            if s not in res and now - float(d.get("timestamp", 0.0)) <= self.final_guard_stale_neighbor_s: res[s] = self._project_neighbor_data(d, now)
        return list(res.items())

    def _neighbor_eta_from_data(self, d):
        s = d.get("speed")
        return self._distance_to_merge(float(d["x"]), float(d["y"])) / max(float(s), 0.1) if s else None

    def _neighbor_eta(self, s):
        d = self.neighbors.get(s)
        return self._neighbor_eta_from_data(d) if d else None

    def _merge_eta(self):
        s, d = self._current_speed(), self._self_distance_to_merge()
        return d / max(float(s), 0.1) if s is not None and d is not None else None

    def _neighbor_recent(self, sid):
        if sid is None: return True
        d = self.neighbors.get(sid)
        return self._sim_time() - float(d.get("timestamp", 0.0)) <= self.slot_neighbor_grace_s if d else False

    def _has_active_host_reservation(self): return self.active_merge_request and self._sim_time() < self.active_merge_request_until

    def _has_any_main_neighbor_near_merge(self):
        for s, d in self._final_guard_neighbor_items():
            if s in self.main_station_ids and self._distance_to_merge(float(d["x"]), float(d["y"])) <= self.role_detection_distance: return True
        return False

    def _host_yield_effective(self, hid):
        if hid is None: return True
        d = self.neighbors.get(hid)
        if not d: return False
        s = float(d.get("speed", 999.0))
        if self.pending_request and self.pending_request.get("host_target_speed"): return s <= float(self.pending_request["host_target_speed"]) + 0.5
        return s <= self.cruise_speed - 0.8

    def _base_cruise_speed(self): return self.cruise_speed * 0.9 if self.effective_role == "merge" else self.cruise_speed
    def _distance_to_merge(self, x, y): return math.hypot(self.merge_point_x - x, self.merge_point_y - y)
    def _self_distance_to_merge(self): return self._distance_to_merge(float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0))) if self.sensor_state else None

    def _check_merge_finalized(self):
        if not self.is_ramp_vehicle or self.merge_completed: return False
        if self._self_merge_completed():
            curt = self._sim_time(); lid_s = str(self.sensor_state.get("lane_id", "")); lidx, eid = parse_lane_index(lid_s), edge_id_from_lane(lid_s)
            self._log_timeline_event("COMPLETED")
            self.merge_completed_since, self.count_merge_completed = curt, self.count_merge_completed + 1
            clean_merge = not getattr(self, 'recovery_triggered_this_merge', False) and not self.had_merge_timeout_this_attempt
            if clean_merge: self.count_merge_completed_clean += 1
            if self.had_merge_timeout_this_attempt: log.debug("[%.1f] %s MERGE_COMPLETED_AFTER_TIMEOUT: eid=%s lane=%d", curt, self.vehicle_id, eid, lidx)
            else: log.debug("[%.1f] %s MERGE_COMPLETED: eid=%s lane=%d clean=%s", curt, self.vehicle_id, eid, lidx, clean_merge)
            self.merge_completed, self.merge_committed, self.merge_authorized, self.merge_authorized_since, self.had_merge_timeout_this_attempt, self.merge_deadlock_since, self.merge_safety_hold_since, self.merge_accepted, self.merge_accepted_since, self.accepted_slot_invalid_since = True, False, False, 0.0, False, 0.0, 0.0, False, 0.0, 0.0
            self.merge_physical_started_once = False
            self.committed_lead_id, self.committed_host_id, self.committed_manoeuvre_id, self.recovery_triggered_this_merge = None, None, None, False
            self._set_state(STATE_CRUISE); self.target_lane_index, self.pending_request = None, None
            es = max(self.cruise_speed, self.min_merge_entry_speed); self._set_target_speed(es, force=True); self.target_speed_mode, self.skip_car_following_this_step = self.priority_speed_mode, True; return True
        return False

    def _update_self_merge_progress(self):
        d = self._self_distance_to_merge()
        if d is None: return
        if d < self.min_distance_to_merge_seen: self.min_distance_to_merge_seen = d; return
        if self.min_distance_to_merge_seen <= self.merge_stop_margin_m and d > self.min_distance_to_merge_seen + 6.0: self.past_merge_point = True; self._check_merge_finalized()

    def _merge_candidate_id(self):
        if self.merge_station_id in self.neighbors: return self.merge_station_id
        cs = []
        for s, d in self.neighbors.items():
            if self._neighbor_is_merge_candidate(s):
                e = self._neighbor_eta(s)
                if e is not None: cs.append((e, s))
        if not cs: return None
        cs.sort(); return cs[0][1]

    def _set_target_speed(self, s, emergency=False, force=False):
        t = max(s, self.emergency_min_speed if emergency else self.min_speed)
        if force: self.target_speed = t; return
        c = self._current_speed()
        if c is not None:
            u, d = c + self.max_speed_step_up, self.max_speed_step_emergency if emergency else self.max_speed_step_down
            t = min(max(t, c - d), u)
        self.target_speed = t

    def _prune_neighbors(self):
        n = self._sim_time()
        for s in [sid for sid, data in self.neighbors.items() if n - data.get("timestamp", 0) > self.neighbor_timeout_s]: self.neighbors.pop(s, None)
        for s in [sid for sid, data in self.neighbor_memory.items() if n - float(data.get("timestamp", 0.0)) > max(self.final_guard_stale_neighbor_s, self.neighbor_timeout_s)]: self.neighbor_memory.pop(s, None)

    def _prune_mcm_messages(self):
        n, t = self._sim_time(), max(self.neighbor_timeout_s, self.negotiation_timeout_s)
        for s in [sid for sid, data in self.mcm_messages.items() if n - float(data.get("timestamp", 0.0)) > t]: self.mcm_messages.pop(s, None)

    def _update_neighbor_observation(self, s, x, y, sp, h):
        d = self._distance_to_merge(x, y); pr = self.neighbors.get(s); dl = d - float(pr["distance_to_merge"]) if pr and "distance_to_merge" in pr else None
        self.neighbors[s] = self.neighbor_memory[s] = {"x": x, "y": y, "speed": sp, "heading": h, "distance_to_merge": d, "distance_delta": dl, "timestamp": self._sim_time()}

    def _handle_cam(self, p):
        s = vanetza_station_id(p)
        if s is None or s == self.station_id: return
        try:
            cp = unwrap_vanetza_cam(p)["camParameters"]; bc, hf = cp["basicContainer"], cp["highFrequencyContainer"]["basicVehicleContainerHighFrequency"]
            xy = latlon_to_xy(float(bc["referencePosition"]["latitude"]), float(bc["referencePosition"]["longitude"]), self.origin_lat, self.origin_lon)
            self._update_neighbor_observation(s, xy["x"], xy["y"], hf["speed"]["speedValue"], normalize_heading_deg(hf["heading"]["headingValue"]))
        except: pass

    def _handle_mcm(self, p):
        if not self.sensor_state: return
        m = unwrap_vanetza_mcm(p); s = vanetza_station_id(p) or m.get("basicContainer", {}).get("stationID")
        if s is None or int(s) == self.station_id: return
        s, bc = int(s), m.get("basicContainer", {})
        a, mid = self._parse_mcm_action(bc.get("rational", {}).get("manoeuvreCooperationCost")), int(bc.get("manoeuvreId", 0)); t = self._mcm_target_station_id(m)
        if a in (1, 2, 3) and (t is None or t != self.station_id): return
        if a is None: return
        log.debug("[%.1f] %s MCM_RX_%s: from=%d manoeuvre=%d target=%s", self._sim_time(), self.vehicle_id, mcm_action_name(a), s, mid, t)
        try:
            xy = latlon_to_xy(float(bc["position"]["latitude"]), float(bc["position"]["longitude"]), self.origin_lat, self.origin_lon)
            st = m["mcmContainer"]["vehicleManoeuvreContainer"]["vehicleCurrentStateContainer"]; self._update_neighbor_observation(s, xy["x"], xy["y"], st["vehicleSpeed"]["speedValue"], heading_deg_to_etsi(st["vehicleHeading"].get("value")))
        except: pass
        self.mcm_messages[s] = {"action": a, "manoeuvre_id": mid, "target_station_id": t, "timestamp": self._sim_time()}

    def _parse_mcm_action(self, v):
        try:
            a = int(v); return a if a in (1, 2, 3) else None
        except: return None

    def _mcm_target_station_id(self, p):
        adv = p.get("mcmContainer", {}).get("vehicleManoeuvreContainer", {}).get("manoeuvreAdvice", [])
        return int(adv[0]["executantID"]) if adv and adv[0].get("executantID") is not None else None

    def _build_cam(self):
        c = copy.deepcopy(self.cam_template)
        if not self.sensor_state: return c
        x, y, sp = float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0)), float(self.sensor_state.get("speed", 0.0))
        ll = xy_to_latlon(x, y, self.origin_lat, self.origin_lon); c["stationId"] = self.station_id
        ref = c.setdefault("camParameters", {}).setdefault("basicContainer", {}).setdefault("referencePosition", {})
        ref["latitude"], ref["longitude"] = ll["latitude"], ll["longitude"]
        c["camParameters"]["basicContainer"]["stationType"] = self.station_type
        hf = c["camParameters"].setdefault("highFrequencyContainer", {}).setdefault("basicVehicleContainerHighFrequency", {})
        hf["speed"]["speedValue"] = sp
        h = self._current_heading() or self._estimate_heading(x, y)
        if h is not None: hf.setdefault("heading", {})["headingValue"] = h; self.last_heading = h
        hf.setdefault("vehicleLength", {})["vehicleLengthValue"], hf["vehicleWidth"], c["generationDeltaTime"], self.last_position = self.vehicle_length, self.vehicle_width, ms_since_minute(), {"x": x, "y": y}; return c

    def _build_mcm(self, a, mid, target_station_id=None):
        m = copy.deepcopy(self.mcm_template)
        if not self.sensor_state: return m
        x, y, sp = float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0)), float(self.sensor_state.get("speed", 0.0))
        ll, mid = xy_to_latlon(x, y, self.origin_lat, self.origin_lon), self._normalize_manoeuvre_id(mid)
        m["stationId"] = self.station_id; bc = m.setdefault("basicContainer", {})
        bc["generationDeltaTime"], bc["stationID"], bc["stationType"], bc["itssRole"], bc["mcmType"], bc["manoeuvreId"] = clamp_int(ms_since_minute(), 0, 65535), self.station_id, self.mcm_station_type, self.itss_role, 8, mid
        bc.setdefault("rational", {})["manoeuvreCooperationCost"] = a
        bc.setdefault("position", {})["latitude"], bc["position"]["longitude"] = ll["latitude"], ll["longitude"]
        if target_station_id: m.setdefault("mcmContainer", {}).setdefault("vehicleManoeuvreContainer", {}).setdefault("manoeuvreAdvice", [{}])[0]["executantID"] = int(target_station_id)
        st = m.setdefault("mcmContainer", {}).setdefault("vehicleManoeuvreContainer", {}).setdefault("vehicleCurrentStateContainer", {})
        st.setdefault("vehicleSpeed", {})["speedValue"] = clamp_int(sp, 0); st.setdefault("vehicleHeading", {})["value"] = heading_deg_to_etsi(self.last_heading)
        sz = st.setdefault("vehicleSize", {}); sz["vehicleWidth"] = clamp_int(self.vehicle_width, 1); sz.setdefault("vehicleLenth", {})["vehicleLengthValue"] = clamp_int(self.vehicle_length, 1); return m

    def _build_denm(self):
        d = copy.deepcopy(self.denm_template)
        if not self.sensor_state: return d
        ll = xy_to_latlon(float(self.sensor_state["x"]), float(self.sensor_state["y"]), self.origin_lat, self.origin_lon)
        self.denm_seq += 1; m = d.setdefault("management", {}); aid = m.setdefault("actionId", {})
        aid["originatingStationId"], aid["sequenceNumber"] = self.station_id, self.denm_seq
        m["referenceTime"] = m["detectionTime"] = self._sim_time(); m["stationType"] = self.station_type
        ep = m.setdefault("eventPosition", {}); ep["latitude"], ep["longitude"] = ll["latitude"], ll["longitude"]; return d

    def _publish_json(self, t, p): self.client.publish(t, json.dumps(p))

    def _publish_actuators(self):
        if self.target_speed is None:
            if not self.publish_idle_actuators or not self.sensor_state: return
            self.target_speed = float(self.sensor_state.get("speed", 0.0))
        self._publish_json(self.actuator_speed_topic, {"target_speed": float(self.target_speed), "timestamp": self._sim_time()})
        if self.target_lane_index is not None: self._publish_json(self.actuator_lane_topic, {"target_lane_index": int(self.target_lane_index), "timestamp": self._sim_time()})
        if self.target_speed_mode is not None: self._publish_json(self.actuator_speed_mode_topic, {"speed_mode": int(self.target_speed_mode), "timestamp": self._sim_time()})

    def _publish_status(self):
        d, e = self._self_distance_to_merge(), self._merge_eta()
        lcs = self.lane_command_status or {}; lid_s = str(self.sensor_state.get("lane_id", "")) if self.sensor_state else ""; p = {"vehicle_id": self.vehicle_id, "station_id": self.station_id, "role": self.role, "merge_completed": getattr(self, "merge_completed", False), "merge_committed": getattr(self, "merge_committed", False), "lane_command_state": lcs.get("state", "NONE"), "edge_id": edge_id_from_lane(lid_s), "lane_index": parse_lane_index(lid_s), "role_mode": self.role_mode, "effective_role": self.effective_role, "fsm_state": self.fsm_state, "fsm_state_age_s": self._sim_time() - self.fsm_state_since, "distance_to_merge_m": d, "merge_eta_s": e, "neighbor_count": len(self.neighbors), "target_speed": self.target_speed, "target_lane_index": self.target_lane_index, "target_speed_mode": self.target_speed_mode, "following_active": self.following_active, "following_station_id": self.following_station_id, "following_gap_m": self.following_gap_m, "following_reason": self.following_reason, "pending_request": self.pending_request is not None, "count_late_merge_recovery": self.count_late_merge_recovery, "count_merge_failed_no_gap": self.count_merge_failed_no_gap, "count_merge_completed": self.count_merge_completed, "count_merge_completed_clean": self.count_merge_completed_clean, "timestamp": self._sim_time()}
        if self.sensor_state: p["lane_id"], p["speed"] = self.sensor_state.get("lane_id"), self.sensor_state.get("speed")
        self._publish_json(self.status_topic, p)

    def _next_manoeuvre_id(self):
        self.mcm_seq = (self.mcm_seq + 1) % (MAX_MANOEUVRE_ID + 1)
        return ((self.station_id * 31 + self.mcm_seq) % MAX_MANOEUVRE_ID) or 1

    def _normalize_manoeuvre_id(self, v):
        try: p = int(v or self._next_manoeuvre_id())
        except: p = self._next_manoeuvre_id()
        return max(0, min(MAX_MANOEUVRE_ID, p))

    def _neighbor_distance(self, s):
        d = self.neighbors.get(s)
        return math.hypot(d["x"] - float(self.sensor_state.get("x", 0.0)), d["y"] - float(self.sensor_state.get("y", 0.0))) if d and self.sensor_state else None

    def _neighbor_etas(self):
        res = []
        for s in self.neighbors:
            e = self._neighbor_eta(s)
            if e is not None: res.append((e, s))
        res.sort(); return res

    def _lane_edge_id(self): return edge_id_from_lane(str(self.sensor_state.get("lane_id", ""))) if self.sensor_state else ""

    def _self_is_on_ramp(self):
        if not self.sensor_state: return False
        if self._lane_edge_id() in self.ramp_edge_ids: return True
        return float(self.sensor_state.get("y", 0.0)) <= self.ramp_y_threshold and (self._self_distance_to_merge() or 0) <= self.role_detection_distance

    def _neighbor_is_approaching_merge(self, d):
        v = d.get("distance_delta"); return float(v) <= 0.1 if v is not None else True

    def _neighbor_is_merge_candidate(self, s):
        d = self.neighbors.get(s)
        if not d: return False
        dist = self._distance_to_merge(float(d["x"]), float(d["y"]))
        if dist > self.role_detection_distance: return False
        if d.get("distance_delta") is not None and float(d["distance_delta"]) > 0.5: return False
        app = self._neighbor_is_approaching_merge(d)
        if s in self.ramp_station_ids:
            if not app: return False
            if self.ramp_bbox:
                x1, y1, x2, y2 = self.ramp_bbox
                return (x1 <= float(d["x"]) <= x2 and y1 <= float(d["y"]) <= y2) or dist <= self.priority_distance
            return True
        return app and (s == self.merge_station_id if self.merge_station_id else float(d["y"]) <= self.ramp_y_threshold)

    def _neighbor_is_main_candidate(self, s):
        d = self.neighbors.get(s)
        if not d or self._distance_to_merge(float(d["x"]), float(d["y"])) > self.role_detection_distance: return False
        return self._neighbor_is_approaching_merge(d) and s in self.main_station_ids

    def _all_main_clearance_ok(self):
        if not self.sensor_state: return False
        oe, oh = self._merge_eta(), self._current_heading()
        if oe is None or oh is None: return True
        sx, sy = float(self.sensor_state["x"]), float(self.sensor_state["y"])
        rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
        for s, data in self.neighbors.items():
            if not self._neighbor_is_main_candidate(s): continue
            ne = self._neighbor_eta(s)
            if ne is None or abs(ne - oe) > self.safe_headway_s: continue
            dx, dy = float(data["x"]) - sx, float(data["y"]) - sy; lon, lat = dx * fx + dy * fy, abs(-dx * fy + dy * fx)
            if lat <= self.cam_follow_lateral_tolerance and 0 < lon <= self.min_clearance_m: return False
        return True

    def _merge_zone_clearance_ok(self, lid=None, hid=None):
        oe = self._merge_eta()
        if oe is None: return False
        for s in (lid, hid):
            if s is None: continue
            ne = self._neighbor_eta(s)
            if ne is not None and abs(ne - oe) < self.safe_headway_s: return False
        return True

    def _merge_start_gap_diag_values(self, sid, ox, oy, fx, fy, own_speed):
        if sid is None: return (None, None, None, None, None, None)
        d = self.neighbors.get(sid) or self.neighbor_memory.get(sid)
        if not d: return (None, None, None, None, None, None)
        dx, dy = float(d.get("x", 0.0)) - ox, float(d.get("y", 0.0)) - oy
        lon, lat, ns = dx * fx + dy * fy, abs(-dx * fy + dy * fx), float(d.get("speed", 0.0))
        gap = max(0.0, abs(lon) - self.vehicle_length)
        closing = own_speed - ns if lon > 0 else ns - own_speed
        def proj(t): return max(0.0, gap - max(closing, 0.0) * t)
        return (gap, lat, closing, proj(1.0), proj(2.0), proj(3.0))

    def _log_timeline_event(self, event, host=None, lead=None, manoeuvre=None):
        if not hasattr(self, '_timeline_logged'): self._timeline_logged = set()
        curt = self._sim_time()
        lcs = self.lane_command_status or {}
        lid_s = str(self.sensor_state.get("lane_id", "")) if self.sensor_state else ""
        lidx = parse_lane_index(lid_s)
        eid = edge_id_from_lane(lid_s)
        cspd = self._current_speed() or 0.0
        dtm = self._self_distance_to_merge() or 0.0
        tgt = self.target_speed

        ho = host or (self.committed_host_id if getattr(self, 'committed_host_id', None) else (self.pending_request.get("host_id") if getattr(self, 'pending_request', None) else None))
        le = lead or (self.committed_lead_id if getattr(self, 'committed_lead_id', None) else (self.pending_request.get("lead_id") if getattr(self, 'pending_request', None) else None))
        mo = manoeuvre or (self.pending_request.get("manoeuvre_id") if getattr(self, 'pending_request', None) else getattr(self, 'last_manoeuvre_id_timeline', None))
        if mo is not None: self.last_manoeuvre_id_timeline = mo
        
        ek = f"{mo}_{event}"
        if event not in ("POST_MERGE_CAR_FOLLOW", "POST_CLEAR_CAR_FOLLOW", "TIMELINE_TEST") and ek in self._timeline_logged: return
        self._timeline_logged.add(ek)

        lg, llat, lclosing, lg1, lg2, lg3 = None, None, None, None, None, None
        hg, hlat, hclosing, hg1, hg2, hg3 = None, None, None, None, None, None
        if self.sensor_state and self._current_heading() is not None:
            ox, oy, oh = float(self.sensor_state["x"]), float(self.sensor_state["y"]), self._current_heading()
            rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
            lg, llat, lclosing, lg1, lg2, lg3 = self._merge_start_gap_diag_values(int(le) if le else None, ox, oy, fx, fy, cspd)
            hg, hlat, hclosing, hg1, hg2, hg3 = self._merge_start_gap_diag_values(int(ho) if ho else None, ox, oy, fx, fy, cspd)

        def f(v): return f"{v:.2f}" if isinstance(v, float) else ("" if v is None else str(v))
        
        log.info(
            "MERGE_ATTEMPT_TIMELINE: vehicle=%s manoeuvre=%s host=%s lead=%s event=%s t=%.1f "
            "edge=%s lane=%s lane_cmd_state=%s speed=%.2f target=%.2f dtm=%.1f "
            "lead_gap=%s host_gap=%s lead_gap_t1=%s lead_gap_t2=%s lead_gap_t3=%s "
            "host_gap_t1=%s host_gap_t2=%s host_gap_t3=%s",
            self.vehicle_id, mo, ho, le, event, curt,
            eid, lidx, lcs.get("state", "NONE"), cspd, tgt, dtm,
            f(lg), f(hg), f(lg1), f(lg2), f(lg3), f(hg1), f(hg2), f(hg3)
        )

    def _log_merge_start_gap_diag(self, event_name, lid, hid, le, he, e, dtm, cspd, lidx, eid):
        oh = self._current_heading()
        if oh is None or not self.sensor_state: return
        ox, oy = float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0))
        rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
        lg, llat, lclosing, lg1, lg2, lg3 = self._merge_start_gap_diag_values(lid, ox, oy, fx, fy, cspd)
        hg, hlat, hclosing, hg1, hg2, hg3 = self._merge_start_gap_diag_values(hid, ox, oy, fx, fy, cspd)
        lcs = self.lane_command_status or {}
        log.debug(
            "[%.1f] %s MERGE_START_GAP_DIAG: event=%s lead=%s host=%s lead_eta=%s own_eta=%s host_eta=%s "
            "lead_gap=%s host_gap=%s lead_gap_p1=%s lead_gap_p2=%s lead_gap_p3=%s host_gap_p1=%s "
            "host_gap_p2=%s host_gap_p3=%s rel_to_lead=%s rel_to_host=%s lead_lat=%s host_lat=%s "
            "edge=%s lane=%s lane_cmd_state=%s lane_cmd_edge=%s lane_cmd_target=%s lane_cmd_executable=%s "
            "dtm=%.1f speed=%.2f",
            self._sim_time(), self.vehicle_id, event_name, lid, hid, le, e, he, lg, hg, lg1, lg2, lg3,
            hg1, hg2, hg3, lclosing, hclosing, llat, hlat, eid, lidx, lcs.get("state"), lcs.get("edge_id"),
            lcs.get("target_lane"), lcs.get("executable"), dtm, cspd
        )

    def _log_slot_quality_diag(self, event_name, lid, hid, le, he, e, dtm, source="", manoeuvre=None):
        if e is None or dtm is None: return
        cspd = self._current_speed() or 0.0
        lane_id = str(self.sensor_state.get("lane_id", "")) if self.sensor_state else ""
        lidx, eid = parse_lane_index(lane_id), edge_id_from_lane(lane_id)
        mine, maxe, gp = self._merge_slot_window(le, he)
        de = self._desired_eta_for_window(e, mine, maxe)
        lg, llat, lclosing, lg1, lg2, lg3 = None, None, None, None, None, None
        hg, hlat, hclosing, hg1, hg2, hg3 = None, None, None, None, None, None
        if self.sensor_state and self._current_heading() is not None:
            ox, oy, oh = float(self.sensor_state["x"]), float(self.sensor_state["y"]), self._current_heading()
            rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
            lg, llat, lclosing, lg1, lg2, lg3 = self._merge_start_gap_diag_values(lid, ox, oy, fx, fy, cspd)
            hg, hlat, hclosing, hg1, hg2, hg3 = self._merge_start_gap_diag_values(hid, ox, oy, fx, fy, cspd)
        lspd = float((self.neighbors.get(lid) or self.neighbor_memory.get(lid) or {}).get("speed", 0.0)) if lid else None
        hspd = float((self.neighbors.get(hid) or self.neighbor_memory.get(hid) or {}).get("speed", 0.0)) if hid else None
        pending_age = None
        accepted_age = None
        if self.pending_request:
            pending_age = self._sim_time() - float(self.pending_request.get("timestamp", self._sim_time()))
            if self.pending_request.get("accepted_at"):
                accepted_age = self._sim_time() - float(self.pending_request.get("accepted_at", self._sim_time()))
        lcs = self.lane_command_status or {}
        mid = manoeuvre
        if mid is None and self.pending_request:
            mid = self.pending_request.get("manoeuvre_id")
        def fmt(v): return f"{v:.2f}" if isinstance(v, float) else ("None" if v is None else str(v))
        log.info(
            "MERGE_SLOT_QUALITY_DIAG: vehicle=%s event=%s t=%.1f manoeuvre=%s source=%s "
            "lead=%s host=%s own_eta=%s lead_eta=%s host_eta=%s min_eta=%s max_eta=%s desired_eta=%s "
            "gap_possible=%s dtm=%.1f edge=%s lane=%s speed=%.2f lead_speed=%s host_speed=%s "
            "lead_gap=%s host_gap=%s lead_gap_t1=%s lead_gap_t2=%s lead_gap_t3=%s "
            "host_gap_t1=%s host_gap_t2=%s host_gap_t3=%s rel_to_lead=%s rel_to_host=%s "
            "lead_lat=%s host_lat=%s pending=%s accepted=%s authorized=%s committed=%s "
            "pending_age=%s accepted_age=%s lane_cmd_state=%s lane_cmd_target=%s",
            self.vehicle_id, event_name, self._sim_time(), mid, source,
            lid, hid, fmt(e), fmt(le), fmt(he), fmt(mine), fmt(maxe), fmt(de),
            gp, dtm, eid, lidx, cspd, fmt(lspd), fmt(hspd),
            fmt(lg), fmt(hg), fmt(lg1), fmt(lg2), fmt(lg3),
            fmt(hg1), fmt(hg2), fmt(hg3), fmt(lclosing), fmt(hclosing),
            fmt(llat), fmt(hlat), self.pending_request is not None,
            bool(self.pending_request and self.pending_request.get("accepted_at")),
            self.merge_authorized, self.merge_committed, fmt(pending_age), fmt(accepted_age),
            lcs.get("state", "NONE"), lcs.get("target_lane")
        )

    def _log_host_decision(self, ramp_id, manoeuvre_id, decision, reason):
        n = self._sim_time()
        rst = self.remote_vehicle_status.get(ramp_id, {})
        me, d, oe = self._neighbor_eta(ramp_id), self._self_distance_to_merge(), self._merge_eta()
        te = (me + self.safe_headway_s + self.merge_occupancy_s) if me else None
        rqs = (d / max(te, 0.1)) if d is not None and te else None
        cs = self._current_speed() or self.cruise_speed
        asid = int(self.active_merge_request["station_id"]) if self.active_merge_request else None
        arem = self.active_merge_request_until - n if self.active_merge_request else 0.0
        log.info(
            "HOST_REQUEST_DECISION: host=%s ramp=%s manoeuvre=%s decision=%s reason=%s "
            "ramp_eta=%s host_eta=%s required_speed=%s current_speed=%.2f "
            "active_request_ramp=%s active_request_remaining=%.1f "
            "ramp_remote_edge=%s ramp_remote_lane=%s ramp_remote_fsm=%s",
            self.vehicle_id, ramp_id, manoeuvre_id, decision, reason,
            f"{me:.2f}" if me else "None", f"{oe:.2f}" if oe else "None",
            f"{rqs:.2f}" if rqs else "None", cs, asid, arem,
            rst.get("edge_id", ""), rst.get("lane_index", ""), rst.get("fsm_state", "NONE")
        )

    def _ramp_leader(self, sd):
        ls = []
        for s, d in self.neighbors.items():
            if self._neighbor_is_merge_candidate(s):
                ld = self._distance_to_merge(float(d["x"]), float(d["y"]))
                if sd - ld > 0.1: ls.append((sd - ld, s, float(d.get("speed") or 0.0)))
        if not ls: return None
        ls.sort(); return ls[0][1], ls[0][0], ls[0][2]

    def _has_ramp_leader_close(self, sd):
        l = self._ramp_leader(sd)
        if not l: return False
        dg = max(self.ramp_platoon_min_gap, (self._current_speed() or self.cruise_speed) * self.ramp_platoon_headway_s, self.merge_queue_release_gap if sd <= self.priority_distance else 0.0)
        return l[1] < dg

    def _arrives_before(self, ea, sa, eb, sb): return ea < eb - 1e-3 or (abs(ea - eb) <= 1e-3 and sa < sb)

    def _self_merge_completed(self):
        if not self.sensor_state: return False
        if self.merge_completed: return True
        lid_str = str(self.sensor_state.get("lane_id", "")); lidx, eid = parse_lane_index(lid_str), edge_id_from_lane(lid_str)
        if eid in self.main_edge_ids:
            if eid == "1331698336": return lidx >= self.merge_lane_index
            return True
        return False

    def _resolve_role(self):
        if self.role_mode != "auto": return self.role
        if not self.is_ramp_vehicle and self._has_active_host_reservation(): return "host"
        if self.past_merge_point: return "merge" if (self.is_ramp_vehicle and not self.merge_completed) else "cruise"
        if self.is_ramp_vehicle: return "host" if self.merge_completed else "merge"
        if self._self_is_on_ramp(): return "merge"
        se, mi = self._merge_eta(), self._merge_candidate_id(); me = self._neighbor_eta(mi) if mi else None
        if se is None or mi is None or me is None: return "host"
        return "lead" if self._arrives_before(se, self.station_id, me, mi) else "host"

    def _main_candidate_etas(self):
        res = []
        for e, s in self._neighbor_etas():
            if self._neighbor_is_main_candidate(s): res.append((e, s))
        res.sort(); return res

    def _merge_slot_window(self, le, he):
        mine, maxe = le + self.safe_headway_s if le is not None else None, he - self.merge_commit_headway_s if he is not None else None
        return mine, maxe, (mine is None or maxe is None or maxe >= mine)

    def _desired_eta_for_window(self, e, mine, maxe):
        d = e
        if mine is not None and d < mine: d = mine
        if maxe is not None and d > maxe: d = maxe
        return d

    def _select_merge_slot(self, e):
        mes = self._main_candidate_etas()
        if not mes: return None, None, None, None, None, None, True, e, "no_main_neighbors"
        hi = next((i for i, (me, _) in enumerate(mes) if me >= e), None)
        if hi is not None:
            he, hid = mes[hi]; le, lid = mes[hi-1] if hi > 0 else (None, None); mine, maxe, gp = self._merge_slot_window(le, he)
            return lid, hid, le, he, mine, maxe, gp, self._desired_eta_for_window(e, mine, maxe), "selected"
        le, lid = mes[-1]; mine, maxe, gp = self._merge_slot_window(le, None); return lid, None, le, None, mine, maxe, gp, self._desired_eta_for_window(e, mine, maxe), "true_after_last_main"

    def _expire_pending_request(self):
        if not self.pending_request: return
        p_time = float(self.pending_request["accepted_at" if self.pending_request.get("accepted_at") else "timestamp"])
        tout = self.merge_accept_timeout_s if self.pending_request.get("accepted_at") else self.negotiation_timeout_s
        if self._sim_time() - p_time > tout:
            oh = self.pending_request.get("host_id"); self.pending_request, self.mcm_retry_blocked_until = None, self._sim_time() + self.mcm_timeout_cooldown_s
            if oh: self.mcm_messages.pop(int(oh), None)
            log.debug("[%.1f] %s MCM_TIMEOUT: host=%s", self._sim_time(), self.vehicle_id, oh); self._set_state(STATE_NEGOTIATING); self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed))

    def _send_mcm(self, a, mid=None, target_station_id=None):
        if self.enable_mcm:
            if a == 1: self._log_timeline_event("REQUEST_SENT", host=target_station_id, manoeuvre=self._normalize_manoeuvre_id(mid))
            self._publish_json(self.mcm_in_topic, self._build_mcm(a, mid, target_station_id)); self.last_mcm_sent = self._sim_time(); log.debug("[%.1f] %s MCM_TX_%s: manoeuvre=%d target=%s", self._sim_time(), self.vehicle_id, mcm_action_name(a), self._normalize_manoeuvre_id(mid), target_station_id)

    def _send_denm(self):
        if self.enable_denm: self._publish_json(self.denm_in_topic, self._build_denm())

    def _lock_left_lane_near_merge(self):
        if not self.host_cooperative_lane_change or not self.sensor_state: return
        lidx, d = parse_lane_index(str(self.sensor_state.get("lane_id", ""))), self._self_distance_to_merge()
        if d is not None and lidx == self.host_clear_lane_index and d <= self.host_return_lock_distance_m: self.target_lane_index = self.host_clear_lane_index

    def _hold_host_clear_lane(self, sid):
        if not self.host_cooperative_lane_change: return
        self.host_clear_lane_until, self.host_clear_for_station = max(self.host_clear_lane_until, self._sim_time() + self.host_clear_lane_hold_s), sid
        if parse_lane_index(str(self.sensor_state.get("lane_id", ""))) in (self.merge_lane_index, self.host_clear_lane_index): self.target_lane_index = self.host_clear_lane_index

    def step(self):
        now = self._sim_time()
        if now - self.last_cam_sent >= self.cam_period_s: self._publish_json(self.cam_in_topic, self._build_cam()); self.last_cam_sent = now
        if now - self.last_fsm_step >= self.fsm_period_s: self._step_fsm(); self.last_fsm_step = now
        if now - self.last_actuator_sent >= self.actuator_period_s: self._publish_actuators(); self.last_actuator_sent = now
        if now - self.last_status_sent >= self.status_period_s: self._publish_status(); self.last_status_sent = now

    def _step_fsm(self):
        if not self.sensor_state: return
        now = self._sim_time()
        self._update_self_merge_progress(); self._prune_neighbors(); self._prune_mcm_messages()
        self.effective_role, self.skip_car_following_this_step = self._resolve_role(), False; ds = self._base_cruise_speed()
        if self.desired_speed:
            try: ds = float(self.desired_speed)
            except: pass
        self.target_speed, self.target_lane_index, self.target_speed_mode, ps = max(ds, self.min_speed), None, self.default_speed_mode, self.fsm_state
        tr = self._latest_request() if not self.is_ramp_vehicle else None
        if not self.is_ramp_vehicle and (self._has_active_host_reservation() or tr): self.effective_role = "host"; self._fsm_host()
        elif self.effective_role == "cruise": self._set_state(STATE_CRUISE)
        elif self.effective_role == "merge": self._fsm_merge()
        elif self.effective_role == "host": self._fsm_host()
        elif self.effective_role == "lead": self._fsm_lead()
        if self.host_cooperative_lane_change and self.host_clear_lane_until > self._sim_time() and self.effective_role in ("host", "lead"): self.target_lane_index = self.host_clear_lane_index
        self._lock_left_lane_near_merge(); self._apply_car_following()
        if not hasattr(self, '_last_debug_log'): self._last_debug_log = 0.0
        if now - self._last_debug_log >= 1.0: self._last_debug_log = now; log.debug("[%.1f] %s role=%s state=%s->%s speed=%.2f target=%.2f neighbors=%d", now, self.vehicle_id, self.effective_role, ps, self.fsm_state, self._current_speed() or 0, self.target_speed or 0, len(self.neighbors))

    def _final_merge_lane_clear(self, lid, hid, dtm):
        if dtm > 45.0: return True
        if lid is None and hid is None and self._has_any_main_neighbor_near_merge():
            if self.merge_committed or (self.is_ramp_vehicle and self.past_merge_point): return True
            log.debug("[%.1f] %s FINAL_GUARD: REJECTED blind merge with main traffic", self._sim_time(), self.vehicle_id); return False
        if not self.sensor_state: return False
        oe, oh = self._merge_eta(), self._current_heading(); sx, sy = float(self.sensor_state["x"]), float(self.sensor_state["y"])
        rad = math.radians(90 - oh) if oh else 0; fx, fy = math.cos(rad), math.sin(rad); checked, gids = set(), []
        if lid: gids.append(lid)
        if hid: gids.append(hid)
        for s, _ in self._final_guard_neighbor_items():
            if s not in gids: gids.append(s)
        for s in gids:
            if s in checked: continue
            checked.add(s); data = self.neighbors.get(s) or (self._project_neighbor_data(self.neighbor_memory[s]) if s in self.neighbor_memory else None)
            if not data: continue
            ndist = self._distance_to_merge(float(data["x"]), float(data["y"]))
            if ndist > self.role_detection_distance: continue
            is_cand = self._neighbor_is_merge_candidate(s)
            tlt = s in self.main_station_ids or (s in self.ramp_station_ids and not is_cand)
            if not tlt: continue
            if s in self.ramp_station_ids and ndist > dtm + 0.1: continue
            dx, dy = float(data["x"]) - sx, float(data["y"]) - sy; pg, lat = math.hypot(dx, dy), abs(-dx * fy + dy * fx)
            if s in (lid, hid):
                if pg < self.final_merge_clearance_m: log.debug("[%.1f] %s FINAL_GUARD: REJECTED slot-boundary sid=%d gap=%.1f", self._sim_time(), self.vehicle_id, s, pg); return False
                ne = self._neighbor_eta(s) or self._neighbor_eta_from_data(data)
                if ne and oe and abs(ne - oe) < self.safe_headway_s * 0.75: log.debug("[%.1f] %s FINAL_GUARD: REJECTED slot-boundary ETA sid=%d", self._sim_time(), self.vehicle_id, s); return False
                continue
            ne = self._neighbor_eta(s) or self._neighbor_eta_from_data(data); ec = oe and ne and abs(ne - oe) < self.safe_headway_s
            cc = abs(ndist - dtm) < self.final_merge_clearance_m * 1.5 and lat <= self.cam_follow_lateral_tolerance * 2.0
            if pg < self.final_merge_clearance_m or (ec and cc): log.debug("[%.1f] %s FINAL_GUARD: REJECTED sid=%d gap=%.1f", self._sim_time(), self.vehicle_id, s, pg); return False
        return True

    def _negotiate_merge_slot(self, hid, lid=None, le=None, he=None):
        if hid is None: return None
        if self.pending_request is not None:
            pending_hid = int(self.pending_request["host_id"])
            if pending_hid != hid: hid = pending_hid
        if self.pending_request is None or int(self.pending_request.get("host_id", 0)) != hid:
            if self._sim_time() < self.rejected_hosts_until.get(hid, 0.0): return None
            self.mcm_messages.pop(hid, None); mid = self._next_manoeuvre_id(); ht = None
            if hid in self.neighbors:
                h = self.neighbors[hid]; dist = self._distance_to_merge(float(h["x"]), float(h["y"])); oe = self._merge_eta()
                if oe: ht = dist / max(oe + self.safe_headway_s + self.merge_occupancy_s, 0.1)
            self.pending_request = {"host_id": hid, "host_eta": he, "host_target_speed": ht, "lead_id": lid, "lead_eta": le, "manoeuvre_id": mid, "timestamp": self._sim_time(), "retry_count": 0}
            self.accepted_slot_invalid_since = 0.0
            self._log_slot_quality_diag("REQUEST_NEW", lid, hid, le, he, self._merge_eta(), self._self_distance_to_merge(), "new_request", mid)
            self._send_mcm(1, mid, target_station_id=hid); self._set_state(STATE_NEGOTIATING)
        elif not self.pending_request.get("accepted_at") and self._sim_time() - self.last_mcm_sent >= self.request_retry_s:
            self.pending_request["retry_count"] = self.pending_request.get("retry_count", 0) + 1
            curt = self._sim_time()
            e, dtm = self._merge_eta(), self._self_distance_to_merge()
            hid = self.pending_request.get("host_id")
            rst = self.remote_vehicle_status.get(hid, {})
            log.info(
                "MCM_REQUEST_RETRY_DIAG: vehicle=%s host=%s lead=%s manoeuvre=%s retry_count=%d pending_age=%.1f dtm=%.1f "
                "own_eta=%.2f host_eta_current=%s host_eta_at_request=%s lead_eta_current=%s lead_eta_at_request=%s "
                "last_response_action=%s last_response_age=%s host_remote_fsm=%s host_remote_edge=%s host_remote_lane=%s "
                "host_remote_merge_committed=%s host_remote_merge_completed=%s",
                self.vehicle_id, hid, self.pending_request.get("lead_id"), self.pending_request["manoeuvre_id"],
                self.pending_request["retry_count"], curt - float(self.pending_request["timestamp"]), dtm,
                e if e else 0.0, self._neighbor_eta(hid) or "None", self.pending_request.get("host_eta"),
                self._neighbor_eta(self.pending_request.get("lead_id")) or "None", self.pending_request.get("lead_eta"),
                self.mcm_messages.get(hid, {}).get("action", "None"), 
                curt - float(self.mcm_messages.get(hid, {}).get("timestamp", curt)) if hid in self.mcm_messages else "None",
                rst.get("fsm_state", "NONE"), rst.get("edge_id", ""), rst.get("lane_index", ""),
                rst.get("merge_committed", False), rst.get("merge_completed", False)
            )
            self._log_slot_quality_diag("REQUEST_RETRY", self.pending_request.get("lead_id"), hid, self.pending_request.get("lead_eta"), self._neighbor_eta(hid) or self.pending_request.get("host_eta"), self._merge_eta(), self._self_distance_to_merge(), "retry_request", self.pending_request["manoeuvre_id"])
            self._send_mcm(1, self.pending_request["manoeuvre_id"], target_station_id=hid)
        resp = self.mcm_messages.get(hid); ra = 2 if self.pending_request.get("accepted_at") else None
        if self.pending_request:
            for sid, data in list(self.mcm_messages.items()):
                if int(sid) == hid or data.get("action") != 2: continue
                fresh = float(data.get("timestamp", -1)) >= float(self.pending_request.get("timestamp", 0)) and self._sim_time() - float(data.get("timestamp", -1)) <= self.neighbor_timeout_s
                target_match = data.get("target_station_id") is None or int(data.get("target_station_id")) == self.station_id
                mid_match = int(data.get("manoeuvre_id", -1)) == int(self.pending_request["manoeuvre_id"])
                if fresh and target_match and mid_match:
                    log.debug("[%.1f] %s MCM_ACCEPT_WRONG_HOST: got=%d expected=%d manoeuvre=%d", self._sim_time(), self.vehicle_id, int(sid), hid, self.pending_request["manoeuvre_id"])
                    self.mcm_messages.pop(sid, None)
        if resp and self.pending_request:
            rt, rqt = float(resp.get("timestamp", -1)), float(self.pending_request.get("timestamp", 0))
            fresh = rt >= rqt and self._sim_time() - rt <= self.neighbor_timeout_s
            target_match = resp.get("target_station_id") is None or int(resp.get("target_station_id")) == self.station_id
            mid_match = int(resp.get("manoeuvre_id", -1)) == int(self.pending_request["manoeuvre_id"])
            if fresh and mid_match and target_match:
                ra = resp.get("action")
                if ra == 2: 
                    if not self.pending_request.get("accepted_at"):
                        self._log_timeline_event("ACCEPT_MATCHED", host=hid, manoeuvre=self.pending_request["manoeuvre_id"])
                        self._log_slot_quality_diag("ACCEPT_MATCHED", self.pending_request.get("lead_id"), hid, self.pending_request.get("lead_eta"), self._neighbor_eta(hid) or self.pending_request.get("host_eta"), self._merge_eta(), self._self_distance_to_merge(), "accept_matched", self.pending_request["manoeuvre_id"])
                        log.debug("[%.1f] %s MCM_ACCEPT_MATCHED: host=%d manoeuvre=%d", self._sim_time(), self.vehicle_id, hid, self.pending_request["manoeuvre_id"])
                        self.merge_accepted = True
                        self.merge_accepted_since = self._sim_time()
                        self.accepted_slot_invalid_since = 0.0
                    self.pending_request["accepted_at"] = self._sim_time()
                elif ra == 3: log.debug("[%.1f] %s MCM_REJECT: host=%d manoeuvre=%d", self._sim_time(), self.vehicle_id, hid, self.pending_request["manoeuvre_id"]); self.rejected_hosts_until[hid] = self._sim_time() + 1.5; self.pending_request, self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since = None, False, False, 0.0; self._set_state(STATE_NEGOTIATING); self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed)); return 3
            elif not fresh:
                if resp.get("action") == 2: log.debug("[%.1f] %s MCM_ACCEPT_STALE: host=%d manoeuvre=%s", self._sim_time(), self.vehicle_id, hid, resp.get("manoeuvre_id"))
                self.mcm_messages.pop(hid, None)
            else:
                if not mid_match: log.debug("[%.1f] %s MCM_ACCEPT_WRONG_MANOEUVRE: host=%d got=%s expected=%s", self._sim_time(), self.vehicle_id, hid, resp.get("manoeuvre_id"), self.pending_request["manoeuvre_id"])
                if not target_match: log.debug("[%.1f] %s MCM_ACCEPT_WRONG_TARGET: host=%d target=%s self=%d", self._sim_time(), self.vehicle_id, hid, resp.get("target_station_id"), self.station_id)
        if self.pending_request:
            p_time = float(self.pending_request["accepted_at" if self.pending_request.get("accepted_at") else "timestamp"])
            age, tout = self._sim_time() - p_time, self.merge_accept_timeout_s if self.pending_request.get("accepted_at") else self.negotiation_timeout_s
            if age > tout:
                oh = self.pending_request.get("host_id"); self.pending_request, self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since, self.mcm_retry_blocked_until = None, False, False, 0.0, self._sim_time() + self.mcm_timeout_cooldown_s
                if oh: self.mcm_messages.pop(int(oh), None)
                log.debug("[%.1f] %s MCM_TIMEOUT: host=%s giving up", self._sim_time(), self.vehicle_id, oh); self._set_state(STATE_NEGOTIATING); self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed)); return 3
        return ra

    def _fsm_merge(self):
        e, dtm = self._merge_eta(), self._self_distance_to_merge()
        if e is None or dtm is None: return
        curt, lid_s = self._sim_time(), str(self.sensor_state.get("lane_id", "")); lidx, eid, cspd = parse_lane_index(lid_s), edge_id_from_lane(lid_s), self._current_speed() or 0.0
        if self._check_merge_finalized(): return
        if self.merge_completed and curt - self.merge_completed_since < self.post_merge_lock_s:
            es = max(self.cruise_speed, self.min_merge_entry_speed); self._set_target_speed(es, force=True); self.target_speed_mode, self.skip_car_following_this_step = self.priority_speed_mode, True; return
        if self.merge_committed and not self.merge_completed:
            ca, ceid = curt - self.merge_committed_since, edge_id_from_lane(lid_s); self._set_state(STATE_MERGING); self.target_speed_mode, self.target_lane_index = self.priority_speed_mode, self.merge_lane_index
            if self._lane_command_waiting_edge():
                self._log_timeline_event("WAIT_EDGE")
                lcs = self.lane_command_status or {}
                log.debug("[%.1f] %s MERGE_COMMIT_WAIT_LANE_AVAILABLE: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s speed=%.2f dtm=%.1f", curt, self.vehicle_id, lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"), lcs.get("lane_count"), lcs.get("executable"), cspd, dtm)
            if self._lane_command_apply_active():
                lcs = self.lane_command_status or {}
                key = (lcs.get("state"), lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"), lcs.get("lane_count"))
                if key != self.last_commit_lane_apply_log_key:
                    self._log_timeline_event("APPLY")
                    self.last_commit_lane_apply_log_key = key
                    log.debug("[%.1f] %s MERGE_COMMIT_LANE_APPLY_ACTIVE: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s speed=%.2f dtm=%.1f", curt, self.vehicle_id, lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"), lcs.get("lane_count"), lcs.get("executable"), cspd, dtm)
            if ceid == "1331698336" and lidx != self.merge_lane_index: self.skip_car_following_this_step = True; rspd = max(self.min_merge_entry_speed, self.cruise_speed * 0.9); self._set_target_speed(rspd, force=True); return
            lid, hid = self.committed_lead_id, self.committed_host_id; le, he = self._neighbor_eta(lid) if lid else None, self._neighbor_eta(hid) if hid else None
            lgok, hgok, fgok = (e - le) >= self.safe_headway_s if le else True, (he - e) >= self.merge_commit_headway_s if he else True, self._final_merge_lane_clear(lid, hid, dtm)
            if not lgok or not hgok or not fgok: 
                lcs = self.lane_command_status or {}
                if self.merge_safety_hold_since <= 0.0: self.merge_safety_hold_since = curt
                if curt - self.merge_safety_hold_since > self.merge_safety_hold_timeout_s:
                    self._log_timeline_event("ABORT_SAFETY_HOLD")
                    log.debug("[%.1f] %s MERGE_COMMIT_ABORT_SAFETY_HOLD: lgok=%s hgok=%s fgok=%s edge=%s lane=%s target_lane=%s speed=%.2f lane_cmd_state=%s lane_cmd_executable=%s lane_cmd_edge=%s lane_cmd_lane_count=%s", curt, self.vehicle_id, lgok, hgok, fgok, ceid, lidx, self.target_lane_index, cspd, lcs.get("state"), lcs.get("executable"), lcs.get("edge_id"), lcs.get("lane_count")); self.merge_committed, self.merge_authorized, self.pending_request, self.merge_accepted, self.accepted_slot_invalid_since = False, False, None, False, 0.0; self._set_state(STATE_NEGOTIATING); return
                self._set_target_speed(max(self.min_merge_entry_speed * 0.5, self.min_speed), force=True); log.debug("[%.1f] %s MERGE_COMMIT_SAFETY_HOLD: lgok=%s hgok=%s fgok=%s edge=%s lane=%s target_lane=%s speed=%.2f lane_cmd_state=%s lane_cmd_executable=%s lane_cmd_edge=%s lane_cmd_lane_count=%s", curt, self.vehicle_id, lgok, hgok, fgok, ceid, lidx, self.target_lane_index, cspd, lcs.get("state"), lcs.get("executable"), lcs.get("edge_id"), lcs.get("lane_count")); return
            self.merge_safety_hold_since = 0.0; self._set_target_speed(max(self.min_merge_entry_speed, self.cruise_speed * 0.9)); self.skip_car_following_this_step = True
            if ca >= self.merge_commit_timeout_s: self._log_timeline_event("TIMEOUT"); log.debug("[%.1f] %s MERGE_COMMIT_TIMEOUT", curt, self.vehicle_id); self.had_merge_timeout_this_attempt, self.pending_request, self.merge_committed, self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since = True, None, False, False, False, 0.0; self._set_state(STATE_NEGOTIATING); return
            return
        if self.merge_authorized and not self.merge_committed:
            auth_age = curt - self.merge_authorized_since
            if auth_age > self.merge_authorized_timeout_s: self._log_timeline_event("TIMEOUT"); log.debug("[%.1f] %s MERGE_AUTHORIZED_TIMEOUT: age=%.1f pending=%s", curt, self.vehicle_id, auth_age, self.pending_request); self.merge_authorized, self.pending_request, self.merge_accepted, self.accepted_slot_invalid_since, self.locked_slot = False, None, False, 0.0, None; self._set_state(STATE_NEGOTIATING); return
        if self.pending_request is None and not self.merge_committed and self.fsm_state not in (STATE_NEGOTIATING, STATE_MERGING) and self._has_ramp_leader_close(dtm): self._set_state(STATE_YIELDING); self._set_target_speed(max(cspd - self.ramp_platoon_speed_delta, self.min_speed)); return
        if curt < self.mcm_retry_blocked_until: self._set_state(STATE_YIELDING); self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed)); return
        if self.pending_request is None and not self.merge_committed and dtm > self.mcm_request_distance_m: self._set_state(STATE_CRUISE); return
        if self.locked_slot_until < curt: self.locked_slot = None
        lc, hc, lec, hec, mic, mxc, gpc, dec, src = self._select_merge_slot(e)
        if self.pending_request:
            hid = int(self.pending_request["host_id"]); he = self._neighbor_eta(hid) or float(self.pending_request.get("host_eta") or 0.0)
            mes, lid, le = self._main_candidate_etas(), None, None
            for meta, mid in mes:
                if mid != hid and meta < e and (le is None or meta > le): le, lid = meta, mid
            if lid is None and self.pending_request.get("lead_id"): lid = int(self.pending_request["lead_id"]); le = float(self.pending_request.get("lead_eta") or self._neighbor_eta(lid) or 0.0)
            mine, maxe, gp = self._merge_slot_window(le, he); de, sreas = self._desired_eta_for_window(e, mine, maxe), "pending_request"
        else: lid, hid, le, he, mine, maxe, gp, de, sreas = lc, hc, lec, hec, mic, mxc, gpc, dec, src
        ra = self._negotiate_merge_slot(hid, lid, le, he)
        if self.pending_request and not self.pending_request.get("accepted_at"):
            phid = int(self.pending_request["host_id"]); phe = self._neighbor_eta(phid)
            if phe is None or phe <= e + 0.05:
                near_commit = dtm <= min(self.merge_commit_distance_m, self.mcm_late_host_lock_distance_m) or self.past_merge_point
                if near_commit:
                    if self.pending_host_lost_since <= 0.0: self.pending_host_lost_since = curt
                    lost_age = curt - self.pending_host_lost_since
                    if lost_age <= self.mcm_late_host_lock_grace_s:
                        lg, llat, lclosing, lg1, lg2, lg3 = (None, None, None, None, None, None)
                        hg, hlat, hclosing, hg1, hg2, hg3 = (None, None, None, None, None, None)
                        if self.sensor_state and self._current_heading() is not None:
                            ox, oy, oh = float(self.sensor_state["x"]), float(self.sensor_state["y"]), self._current_heading()
                            rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
                            le_id = self.pending_request.get("lead_id")
                            lg, llat, lclosing, lg1, lg2, lg3 = self._merge_start_gap_diag_values(int(le_id) if le_id else None, ox, oy, fx, fy, cspd)
                            hg, hlat, hclosing, hg1, hg2, hg3 = self._merge_start_gap_diag_values(phid, ox, oy, fx, fy, cspd)

                        log.debug(
                            "[%.1f] %s MCM_PENDING_HOLD_LATE_HOST: host=%d age=%.1f dtm=%.1f lead_gap=%s host_gap=%s pending_age=%.1f accepted_at=%s reason=%s",
                            curt, self.vehicle_id, phid, lost_age, dtm,
                            f"{lg:.2f}" if lg is not None else "None",
                            f"{hg:.2f}" if hg is not None else "None",
                            curt - float(self.pending_request["timestamp"]),
                            f"{self.pending_request.get('accepted_at'):.1f}" if self.pending_request.get("accepted_at") else "None",
                            "lost" if phe is None else f"ahead(eta={phe:.2f}<=own={e:.2f})"
                        )
                        self._set_state(STATE_NEGOTIATING)
                        self._set_target_speed(max(min(cspd, self.cruise_speed * 0.85), self.min_speed), force=True)
                        return
                self.pending_host_lost_since = 0.0
                self._log_timeline_event("TIMEOUT")
                
                rst = self.remote_vehicle_status.get(phid, {})
                mine, maxe, gp = self._merge_slot_window(le, he)
                lg, llat, lclosing, lg1, lg2, lg3 = None, None, None, None, None, None
                hg, hlat, hclosing, hg1, hg2, hg3 = None, None, None, None, None, None
                if self.sensor_state and self._current_heading() is not None:
                    ox, oy, oh = float(self.sensor_state["x"]), float(self.sensor_state["y"]), self._current_heading()
                    rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
                    lg, llat, lclosing, lg1, lg2, lg3 = self._merge_start_gap_diag_values(lid, ox, oy, fx, fy, self._current_speed() or 0.0)
                    hg, hlat, hclosing, hg1, hg2, hg3 = self._merge_start_gap_diag_values(phid, ox, oy, fx, fy, self._current_speed() or 0.0)

                log.info(
                    "MCM_PENDING_ABANDON_DIAG: vehicle=%s host=%s lead=%s manoeuvre=%s reason=%s dtm=%.1f "
                    "own_eta=%s host_eta=%s lead_eta=%s pending_age=%.1f last_mcm_sent_age=%.1f retry_count=%d "
                    "host_remote_fsm=%s host_remote_edge=%s host_remote_lane=%s host_remote_merge_committed=%s "
                    "host_remote_merge_completed=%s host_remote_lane_cmd_state=%s active_merge_request=%s "
                    "slot_source=%s lead_gap=%s host_gap=%s lead_gap_t1=%s lead_gap_t2=%s lead_gap_t3=%s "
                    "host_gap_t1=%s host_gap_t2=%s host_gap_t3=%s host_gap_possible=%s",
                    self.vehicle_id, phid, lid, self.pending_request.get("manoeuvre_id"), 
                    "lost" if phe is None else "ahead", dtm,
                    f"{e:.2f}" if e is not None else "None", f"{he:.2f}" if he is not None else "None", f"{le:.2f}" if le is not None else "None",
                    curt - float(self.pending_request["timestamp"]), curt - self.last_mcm_sent, 
                    self.pending_request.get("retry_count", 0),
                    rst.get("fsm_state", "NONE"), rst.get("edge_id", ""), rst.get("lane_index", ""),
                    rst.get("merge_committed", False), rst.get("merge_completed", False),
                    rst.get("lane_command_state", "NONE"), bool(rst.get("active_merge_request")),
                    sreas, f"{lg:.2f}" if lg is not None else "None", f"{hg:.2f}" if hg is not None else "None",
                    f"{lg1:.2f}" if lg1 is not None else "None", f"{lg2:.2f}" if lg2 is not None else "None", f"{lg3:.2f}" if lg3 is not None else "None",
                    f"{hg1:.2f}" if hg1 is not None else "None", f"{hg2:.2f}" if hg2 is not None else "None", f"{hg3:.2f}" if hg3 is not None else "None",
                    gp
                )

                log.debug("[%.1f] %s MCM_PENDING_ABANDON: host=%d %s", curt, self.vehicle_id, phid, "lost" if phe is None else f"ahead(eta={phe:.2f}<=own={e:.2f})")
                self.mcm_messages.pop(phid, None); self.pending_request, self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since = None, False, False, 0.0; lid, hid, le, he, mine, maxe, gp, de, sreas = lc, hc, lec, hec, mic, mxc, gpc, dec, src
            else:
                self.pending_host_lost_since = 0.0
        if self.locked_slot is None and hid is not None: self.locked_slot, self.locked_slot_until = (lid, hid), curt + self.slot_lock_s
        if maxe and maxe <= 0.0: self.locked_slot, self.pending_request, self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since = None, None, False, False, 0.0; self._set_state(STATE_NEGOTIATING); return
        if not gp: de = max(e, mine) if mine else e
        if de > e + 0.05: self._set_target_speed(max(dtm / max(de, 0.1), self.cruise_speed * 0.4))
        elif de < e - 0.05: self._set_target_speed(min(dtm / max(de, 0.1), self.cruise_speed + self.merge_speed_bonus))
        else: self._set_target_speed(self.cruise_speed * 0.9)
        if self.fsm_state == STATE_NEGOTIATING and cspd < 0.1:
            if not hasattr(self, '_stop_since'): self._stop_since = curt
            if curt - self._stop_since > 3.0: log.debug("[%.1f] %s STOPPED_TOO_LONG: host %s", curt, self.vehicle_id, hid); self.pending_request, self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since, self._stop_since = None, False, False, 0.0, curt
        else: self._stop_since = curt
        lgok, hgok, cok, fgok = (e - le) >= self.safe_headway_s if le else True, (he - e) >= self.merge_commit_headway_s if he else True, self._all_main_clearance_ok(), self._final_merge_lane_clear(lid, hid, dtm)
        esok, cready = dtm > self.merge_entry_speed_guard_m or cspd >= self.min_merge_entry_speed, (dtm <= self.merge_commit_distance_m or self.past_merge_point)
        lclear = True
        if lid:
            ldv = self._neighbor_distance(lid)
            if ldv is not None and abs(ldv - dtm) < self.final_merge_clearance_m:
                lclear = False
                if curt - self.last_lclear_block_log > 1.0:
                    fresh_data = self.neighbors.get(lid)
                    memory_data = self.neighbor_memory.get(lid)
                    source_data = fresh_data or memory_data or {}
                    age = curt - float(source_data.get("timestamp", curt))
                    log.debug(
                        "[%.1f] %s MERGE_LCLEAR_BLOCK: sid=%s gap=%.1f neighbor_age=%.1f from_memory=%s dtm=%.1f",
                        curt, self.vehicle_id, lid, abs(ldv - dtm), age, fresh_data is None and memory_data is not None, dtm
                    )
                    self.last_lclear_block_log = curt
        if self.fsm_state == STATE_ABORT and curt - self.fsm_state_since < self.abort_cooldown_s: self._set_target_speed(max(self.cruise_speed * 0.4, self.min_speed)); return
        elif self.fsm_state == STATE_ABORT: self._set_state(STATE_CRUISE)
        if ra == 3: return
        has_rm = dtm <= self.priority_distance and (len(self._main_candidate_etas()) > 0 or self._has_any_main_neighbor_near_merge())
        hlma = self.allow_hostless_merge and hid is None and self.pending_request is None and not has_rm; amcm = hlma or ra == 2
        if hid is None and not self.allow_hostless_merge:
            amcm = False
            if self.merge_authorized: log.debug("[%.1f] %s MERGE_AUTH_CLEAR_HOSTLESS_DISABLED", curt, self.vehicle_id); self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since = False, False, 0.0
        hyok = self._host_yield_effective(hid)
        if not hyok and ra == 2 and self.pending_request:
            aat = self.pending_request.get("accepted_at")
            if aat and curt - aat > 1.0 and hgok and fgok: hyok = True
        if ra == 2 and not hyok: self.target_lane_index = None; self._set_state(STATE_NEGOTIATING); self._set_target_speed(min(cspd, self.cruise_speed * 0.85), force=True); return
        if ra == 2 and hyok and (not cready or not fgok or (not lgok or not hgok or not cok)): self._set_target_speed(min(cspd, self.cruise_speed * 0.85), force=True)
        if ra == 2:
            if not lgok or not hgok:
                if self.slot_blocked_since <= 0.0: self.slot_blocked_since = curt
                if curt - self.slot_blocked_since > 2.0: log.debug("[%.1f] %s MERGE_SLOT_ABANDON_BLOCKED: lgok=%s hgok=%s", curt, self.vehicle_id, lgok, hgok); self.pending_request, self.merge_authorized, self.merge_accepted, self.slot_blocked_since = None, False, False, 0.0; self.mcm_messages.pop(int(hid), None); self._set_state(STATE_NEGOTIATING); return
            else: self.slot_blocked_since = 0.0

        lg_v, hg_v, lg1_v, lg2_v = None, None, None, None
        if self.sensor_state and self._current_heading() is not None:
            ox, oy, oh = float(self.sensor_state["x"]), float(self.sensor_state["y"]), self._current_heading()
            rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad)
            lg_v, _, _, lg1_v, lg2_v, _ = self._merge_start_gap_diag_values(lid, ox, oy, fx, fy, cspd)
            hg_v, _, _, _, _, _ = self._merge_start_gap_diag_values(hid, ox, oy, fx, fy, cspd)

        lgok_proj = (lg1_v is None or lg1_v > 1.0) and (lg2_v is None or lg2_v > 1.0)
        
        accepted_ready = (
            ra == 2
            and hyok
            and lgok
            and hgok
            and fgok
            and lclear
            and cok
            and lgok_proj
        )

        if amcm and not self.merge_authorized:
            if hlma:
                self.merge_authorized, self.merge_authorized_since = True, curt
                self._log_timeline_event("AUTHORIZED")
                log.debug("[%.1f] %s MERGE_AUTHORIZED_HOSTLESS", curt, self.vehicle_id)
            elif ra == 2:
                if accepted_ready:
                    self.merge_authorized, self.merge_authorized_since = True, curt
                    self._log_timeline_event("AUTHORIZED")
                    self._log_slot_quality_diag("AUTHORIZED", lid, hid, le, he, e, dtm, sreas, self.pending_request.get("manoeuvre_id") if self.pending_request else None)
                    log.debug("[%.1f] %s MERGE_AUTHORIZED_BY_MCM: host=%s manoeuvre=%s", curt, self.vehicle_id, hid, self.pending_request.get("manoeuvre_id") if self.pending_request else 0)
                else:
                    if curt - self.last_accepted_wait_log > 1.0:
                        def fmt_gap(v): return f"{v:.2f}" if v is not None else "None"
                        reason = []
                        if not hyok: reason.append("hyok=False")
                        if not lgok: reason.append(f"lgok=False(gap={fmt_gap(lg_v)})")
                        if not hgok: reason.append(f"hgok=False(gap={fmt_gap(hg_v)})")
                        if not fgok: reason.append("fgok=False")
                        if not lclear: reason.append("lclear=False")
                        if not cok: reason.append("cok=False")
                        if not lgok_proj: reason.append(f"lgok_proj=False(t1={fmt_gap(lg1_v)}, t2={fmt_gap(lg2_v)})")
                        self._log_slot_quality_diag("ACCEPTED_WAIT_SLOT_VALID", lid, hid, le, he, e, dtm, sreas, self.pending_request.get("manoeuvre_id") if self.pending_request else None)
                        log.debug("[%.1f] %s MERGE_ACCEPTED_WAIT_SLOT_VALID: reason=%s", curt, self.vehicle_id, ",".join(reason))
                        self.last_accepted_wait_log = curt

        if self.merge_accepted and not self.merge_authorized and ra == 2:
            if not accepted_ready:
                if self.accepted_slot_invalid_since <= 0.0: self.accepted_slot_invalid_since = curt
                inv_age = curt - self.accepted_slot_invalid_since
                if inv_age > self.accepted_slot_invalid_timeout_s:
                    self._log_timeline_event("SLOT_EXPIRED")
                    self._log_slot_quality_diag("SLOT_EXPIRED", lid, hid, le, he, e, dtm, sreas, self.pending_request.get("manoeuvre_id") if self.pending_request else None)
                    log.debug("[%.1f] %s MERGE_ACCEPTED_SLOT_EXPIRED: age=%.1f dtm=%.1f", curt, self.vehicle_id, inv_age, dtm)
                    self.pending_request, self.merge_accepted, self.merge_authorized, self.accepted_slot_invalid_since = None, False, False, 0.0
                    if hid: self.mcm_messages.pop(int(hid), None)
                    self._set_state(STATE_NEGOTIATING); return
            else:
                self.accepted_slot_invalid_since = 0.0

        physical_zone = (eid == "1331698336" or eid in self.main_edge_ids or dtm <= self.merge_commit_distance_m)
        log.debug("[%.1f] %s MERGE_DECISION: auth=%s phys=%s fgok=%s lgok=%s hgok=%s cok=%s esok=%s cready=%s dtm=%.1f past=%s committed=%s hid=%s ra=%s lclear=%s hyok=%s", curt, self.vehicle_id, self.merge_authorized, physical_zone, fgok, lgok, hgok, cok, esok, cready, dtm, self.past_merge_point, self.merge_committed, hid, ra, lclear, hyok)
        if self.merge_authorized and physical_zone and lgok and hgok and cok and fgok and lclear and cready and hyok:
            if not self.merge_committed and not self.merge_physical_started_once:
                self.merge_physical_started_once = True
                self._log_merge_start_gap_diag("start", lid, hid, le, he, e, dtm, cspd, lidx, eid)
                self.merge_committed, self.merge_committed_since, self.committed_lead_id, self.committed_host_id = True, curt, lid, hid
                self._log_timeline_event("PHYSICAL_START")
                log.debug("[%.1f] %s MERGE_PHYSICAL_START: host=%s manoeuvre=%s", curt, self.vehicle_id, hid, self.pending_request.get("manoeuvre_id") if self.pending_request else 0); log.debug("[%.1f] %s MERGING!", curt, self.vehicle_id)
            elif not self.merge_committed and self.merge_physical_started_once:
                self._log_merge_start_gap_diag("resume", lid, hid, le, he, e, dtm, cspd, lidx, eid)
                self.merge_committed, self.merge_committed_since, self.committed_lead_id, self.committed_host_id = True, curt, lid, hid; log.debug("[%.1f] %s MERGE_PHYSICAL_RESUME: host=%s manoeuvre=%s", curt, self.vehicle_id, hid, self.pending_request.get("manoeuvre_id") if self.pending_request else 0)
            self._set_state(STATE_MERGING); self._set_target_speed(max(self.min_merge_entry_speed, self.cruise_speed * 0.9)); self.target_lane_index, self.target_speed_mode = self.merge_lane_index, self.priority_speed_mode
        elif self.merge_authorized and not self.merge_committed: self._set_state(STATE_NEGOTIATING); self.target_lane_index = None; log.debug("[%.1f] %s MERGE_PREPARE_WAIT_PHYSICAL: dtm=%.1f", curt, self.vehicle_id, dtm)
        elif not self.merge_committed: self._set_state(STATE_NEGOTIATING if hid else STATE_CRUISE); self.target_lane_index = None
        if not self.merge_committed and dtm <= self.cruise_speed * self.safe_headway_s and not (self.merge_authorized and physical_zone):
            self._set_state(STATE_YIELDING); stopd = max(dtm - self.merge_stop_margin_m, 0.0); slsp = stopd / max(self.merge_blocked_approach_s, 0.1)
            if dtm > self.merge_stop_margin_m + 6.0: slsp = max(slsp, self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed)
            else: slsp = max(slsp, self.emergency_min_speed)
            self._set_target_speed(slsp, emergency=dtm <= self.merge_stop_margin_m + 8.0); return
        if self.is_ramp_vehicle and self.past_merge_point and not self.merge_completed:
            if not self.merge_committed and not self.merge_authorized:
                self._log_slot_quality_diag("LOST_AUTH_AFTER_POINT", lid, hid, le, he, e, dtm, sreas, self.pending_request.get("manoeuvre_id") if self.pending_request else None)
                log.debug("[%.1f] %s MERGE_FAILED_LOST_AUTH_AFTER_POINT: dtm=%.1f past=True hid=%s auth=False", curt, self.vehicle_id, dtm, hid)
                self._set_state(STATE_ABORT); self._set_target_speed(0.18, force=True); return
            lp = float(self.sensor_state.get("lane_pos", 0.0)); rem = 63.23 - lp
            if rem < 10.0:
                if not getattr(self, 'recovery_triggered_this_merge', False): self.count_late_merge_recovery += 1; self.recovery_triggered_this_merge = True
                self._set_target_speed(max(self.min_speed, rem / 2.0), force=True); self.target_lane_index = self.merge_lane_index; self.target_speed_mode, self.skip_car_following_this_step = self.priority_speed_mode, True
            elif rem < 2.0: self.count_merge_failed_no_gap += 1; self._set_target_speed(0.0, force=True); self._set_state(STATE_ABORT)

    def _latest_request(self):
        now, reqs = self._sim_time(), []
        for sid, data in self.mcm_messages.items():
            if data.get("action") == 1 and now - data.get("timestamp", 0) <= self.neighbor_timeout_s:
                e = self._neighbor_eta(sid)
                if e is not None: reqs.append((e, sid, data))
        if not reqs: return None
        reqs.sort(); e, sid, data = reqs[0]; out = data.copy(); out["station_id"], out["eta"] = sid, e; return out

    def _apply_car_following(self):
        if self.skip_car_following_this_step: self.skip_car_following_this_step = False; return
        self.following_active, self.following_station_id, self.following_gap_m, self.following_reason = False, None, None, ""
        if not self.enable_cam_following or not self.sensor_state: return
        pmw = self.merge_completed and self._sim_time() - self.merge_completed_since < self.post_merge_lock_s; oh = self._current_heading()
        if oh is None: return
        ox, oy, osp, od = float(self.sensor_state["x"]), float(self.sensor_state["y"]), self._current_speed() or 0.0, self._self_distance_to_merge() or 0.0
        rad = math.radians(90 - oh); fx, fy = math.cos(rad), math.sin(rad); mfsp, fsid, fgap, freas, is_em = None, None, None, "", False
        for sid, d in self.neighbors.items():
            nx, ny, ns = float(d.get("x", 0.0)), float(d.get("y", 0.0)), float(d.get("speed", 0.0)); dx, dy = nx - ox, ny - oy; lon, lat = dx * fx + dy * fy, abs(-dx * fy + dy * fx)
            gap, reason = None, ""
            if 0.0 < lon <= self.cam_follow_lookahead and lat <= self.cam_follow_lateral_tolerance:
                gap, reason = max(0.0, lon - self.vehicle_length), "same_lane_cam"; ir = (sid in self.ramp_station_ids or self._neighbor_is_merge_candidate(sid)) and not self.merge_completed
                if ir:
                    if pmw: continue
                    if gap < 4.0:
                        sf = max(self.cruise_speed * 0.4, self.min_speed)
                        if osp < 1.0: mfsp = max(mfsp or 0, sf)
                if self.effective_role in ("lead", "host", "cruise") and gap > self.cam_follow_critical_gap: continue
            elif self.effective_role == "merge" and od <= self.merge_conflict_follow_distance_m:
                if not self._neighbor_is_main_candidate(sid): continue
                ne, oe = self._neighbor_eta(sid), self._merge_eta()
                if ne and oe and abs(ne - oe) <= self.safe_headway_s * 1.2:
                    de = ne + self.safe_headway_s
                    if oe < de:
                        sf = max(self.cruise_speed * self.merge_conflict_floor_ratio, self.min_speed); t = max(od / max(de, 0.1), sf)
                        if mfsp is None or t < mfsp: mfsp, fsid, fgap, freas, is_em = t, sid, abs(oe - ne), "merge_conflict_eta", False
                continue
            if gap is None: continue
            clsp, bd = max(osp - ns, 0.0), max(self.cam_follow_brake_decel, 0.1); sg = self.cam_follow_min_gap + (osp * self.cam_follow_headway_s) + (clsp * clsp) / (2.0 * bd)
            if gap < sg:
                ag = max(gap - self.cam_follow_min_gap, 0.0); hsp = max(ag / max(self.cam_follow_headway_s, 0.1), 0.0); bsp = math.sqrt(max(0.0, (ns * ns) + (2.0 * bd * ag))); sf = max(self.cruise_speed * 0.35, self.min_speed)
                if self.merge_completed and gap < 3.0: sf = max(self.cruise_speed * 0.45, self.min_speed)
                t = min(max(ns - self.cam_follow_speed_delta, sf), max(hsp, sf), max(bsp, sf))
                if gap < self.cam_follow_critical_gap: t = max(min(t, osp * 0.65), max(self.cruise_speed * 0.30, self.emergency_min_speed))
                if mfsp is None or t < mfsp: mfsp, fsid, fgap, freas, is_em = t, sid, gap, reason, reason == "same_lane_cam" and ((gap < self.cam_follow_critical_gap * 0.75 and clsp > 1.4) or (gap < self.cam_follow_min_gap * 0.55 and clsp > 1.5))
        if mfsp is not None:
            now = self._sim_time()
            self_lid = str(self.sensor_state.get("lane_id", ""))
            self_edge, self_lane = edge_id_from_lane(self_lid), parse_lane_index(self_lid)
            fd = self.neighbors.get(fsid, {}) if fsid is not None else {}
            leader_lid = str(fd.get("lane_id", ""))
            leader_edge, leader_lane = edge_id_from_lane(leader_lid), parse_lane_index(leader_lid)
            if fsid in self.ramp_station_ids: leader_role_hint = "ramp"
            elif fsid in self.main_station_ids: leader_role_hint = "main"
            elif fsid is None: leader_role_hint = "unknown"
            else: leader_role_hint = "merge_candidate" if self._neighbor_is_merge_candidate(fsid) else "unknown"
            merge_completed_age = now - self.merge_completed_since if self.merge_completed else -1.0
            lcs = self.lane_command_status or {}
            target_before = self.target_speed
            self.following_active, self.following_station_id, self.following_gap_m, self.following_reason = True, fsid, fgap, freas
            if self.fsm_state == STATE_CRUISE: self._set_state(STATE_YIELDING)
            self._set_target_speed(mfsp, emergency=is_em)
            log.debug(
                "[%.1f] %s CAR_FOLLOW: sid=%d edge=%s lane=%s leader_edge=%s leader_lane=%s leader_role_hint=%s "
                "self_merge_completed=%s self_merge_completed_age=%.1f self_merge_committed=%s lane_cmd_state=%s "
                "lane_cmd_edge=%s lane_cmd_target=%s lane_cmd_executable=%s same_lane=%s reason=%s gap=%.1f "
                "follow_spd=%.2f emergency=%s target_before=%s target_after=%s",
                now, self.vehicle_id, fsid or 0, self_edge, self_lane, leader_edge, leader_lane, leader_role_hint,
                self.merge_completed, merge_completed_age, self.merge_committed, lcs.get("state"),
                lcs.get("edge_id"), lcs.get("target_lane"), lcs.get("executable"), freas == "same_lane_cam",
                freas, fgap or 0, mfsp, is_em, target_before, self.target_speed
            )
            if self.merge_completed:
                self._log_timeline_event("POST_MERGE_CAR_FOLLOW")
                log.debug(
                    "[%.1f] %s POST_MERGE_CAR_FOLLOW: age=%.1f sid=%d gap=%.1f reason=%s follow_spd=%.2f "
                    "self_edge=%s self_lane=%s leader_edge=%s leader_lane=%s lane_cmd_state=%s",
                    now, self.vehicle_id, merge_completed_age, fsid or 0, fgap or 0, freas, mfsp,
                    self_edge, self_lane, leader_edge, leader_lane, lcs.get("state")
                )
            if freas == "same_lane_cam" and self.last_lane_clear_time > 0.0 and now - self.last_lane_clear_time <= 3.0:
                self._log_timeline_event("POST_CLEAR_CAR_FOLLOW")
                log.debug(
                    "[%.1f] %s POST_CLEAR_CAR_FOLLOW: age=%.1f sid=%d gap=%.1f reason=%s follow_spd=%.2f "
                    "self_edge=%s self_lane=%s leader_edge=%s leader_lane=%s lane_cmd_state=%s",
                    now, self.vehicle_id, now - self.last_lane_clear_time, fsid or 0, fgap or 0, freas, mfsp,
                    self_edge, self_lane, leader_edge, leader_lane, lcs.get("state")
                )

    def _fsm_host(self):
        n = self._sim_time(); r = self._latest_request()
        if self.active_merge_request:
            asid, amid = int(self.active_merge_request["station_id"]), int(self.active_merge_request["manoeuvre_id"]); asp = float(self.active_merge_request.get("target_speed", self.cruise_speed * self.host_yield_floor_ratio))
            rst = self.remote_vehicle_status.get(asid, {})
            nlcs = rst.get("lane_command_state", "NONE")
            ne = rst.get("edge_id", "")
            nla = str(rst.get("lane_index", ""))
            rcmpl = rst.get("merge_completed", False)
            rcmt = rst.get("merge_committed", False)
            rfsm = rst.get("fsm_state", "")
            osp = self._current_speed() or 0.0

            time_since_start = n - getattr(self, 'active_merge_request_started_at', 0.0)
            max_s = getattr(self, 'host_reservation_max_s', 10.0)

            if rcmpl or nlcs == "CLEAR":
                log.info("HOST_RESERVATION_RELEASE_AFTER_CLEAR: host=%s ramp=%d manoeuvre=%d reason=completed ramp_fsm=%s ramp_lcs=%s ramp_edge=%s ramp_merge_completed=%s", self.vehicle_id, asid, amid, rst.get("fsm_state", ""), nlcs, ne, rcmpl)
                self.active_merge_request, self.active_merge_request_until = None, 0.0
            elif time_since_start > max_s:
                log.info("HOST_RESERVATION_RELEASE_MAX_TIMEOUT: host=%s ramp=%d manoeuvre=%d duration=%.1f ramp_fsm=%s ramp_lcs=%s ramp_edge=%s ramp_merge_completed=%s", self.vehicle_id, asid, amid, time_since_start, rst.get("fsm_state", ""), nlcs, ne, rcmpl)
                self.active_merge_request, self.active_merge_request_until = None, 0.0
            elif nlcs in ("WAIT_EDGE", "APPLY") or rfsm == STATE_MERGING or (rcmt and not rcmpl):
                new_until = min(getattr(self, 'active_merge_request_started_at', n) + max_s, n + self.host_reservation_s)
                if new_until > self.active_merge_request_until:
                    self.active_merge_request_until = new_until
                    log.info("HOST_RESERVATION_EXTEND_UNTIL_CLEAR: host=%s ramp=%d manoeuvre=%d new_remaining=%.1f ramp_lcs=%s ramp_edge=%s", self.vehicle_id, asid, amid, self.active_merge_request_until - n, nlcs, ne)
            elif n >= self.active_merge_request_until:
                log.info("HOST_RELEASE_BEFORE_CLEAR: host=%s ramp=%d manoeuvre=%d reason=timeout ramp_fsm=%s ramp_lane_cmd_state=%s ramp_edge=%s ramp_lane=%s ramp_merge_completed=%s ramp_merge_committed=%s", 
                         self.vehicle_id, asid, amid, rfsm, nlcs, ne, nla, rcmpl, rcmt)
                self.active_merge_request, self.active_merge_request_until = None, 0.0

            if self.active_merge_request:
                log.info("HOST_RESERVATION_HOLD: host=%s ramp=%d manoeuvre=%d remaining=%.1f own_speed=%.2f target_speed=%.2f ramp_fsm=%s ramp_lane_cmd_state=%s ramp_edge=%s ramp_lane=%s ramp_merge_completed=%s ramp_merge_committed=%s", 
                         self.vehicle_id, asid, amid, self.active_merge_request_until - n, osp, asp, rfsm, nlcs, ne, nla, rcmpl, rcmt)
                log.debug("[%.1f] %s HOST_HOLD_YIELD: for merge=%d spd=%.2f", n, self.vehicle_id, asid, asp)
                
                if r and int(r["station_id"]) != asid:
                    osid, omid = int(r["station_id"]), int(r.get("manoeuvre_id") or 0)
                    if n - self.last_mcm_response.get(osid, 0) >= self.response_period_s:
                        self._log_host_decision(osid, omid, "REJECT", "BUSY")
                        self._send_mcm(3, omid, target_station_id=osid); self.last_mcm_response[osid] = n
                    else:
                        self._log_host_decision(osid, omid, "IGNORE", "BUSY_PERIOD")
                self._set_state(STATE_YIELDING); self._set_target_speed(asp, force=True)
                if n - self.last_mcm_response.get(asid, 0) >= self.response_period_s: self._send_mcm(2, amid, target_station_id=asid); self.last_mcm_response[asid] = n
                return
        if not r: self._set_state(STATE_CRUISE); return
        rsid, rmid = int(r["station_id"]), int(r.get("manoeuvre_id") or 0); me, d, oe = self._neighbor_eta(rsid), self._self_distance_to_merge(), self._merge_eta()
        if me is None or d is None or oe is None: self._set_state(STATE_CRUISE); return
        if d <= self.host_reject_distance_m:
            if n - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
                self._log_host_decision(rsid, rmid, "REJECT", "DISTANCE")
                self._send_mcm(3, rmid, target_station_id=rsid); self.last_mcm_response[rsid] = n
            else:
                self._log_host_decision(rsid, rmid, "IGNORE", "DISTANCE_PERIOD")
            self._set_state(STATE_CRUISE); return
        te = me + self.safe_headway_s + self.merge_occupancy_s; rqs = d / max(te, 0.1); cs = self._current_speed() or self.cruise_speed
        sf = max(self.cruise_speed * self.host_yield_floor_ratio, self.min_speed); rqs = max(min(rqs, cs), sf)
        gd, asafe = oe - me, (oe - me) >= self.merge_commit_headway_s; dy = rqs < cs - 0.15; ns = gd >= (self.merge_commit_headway_s * 0.75); sa = rqs >= cs - 0.30
        if dy:
            yt = max(min(rqs, cs - self.host_min_yield_delta), sf); self._set_state(STATE_YIELDING); self._set_target_speed(yt, force=True); self.active_merge_request, self.active_merge_request_until = {"station_id": rsid, "manoeuvre_id": rmid, "target_speed": yt, "target_eta": te}, n + self.host_reservation_s; self.active_merge_request_started_at = n
            log.info("HOST_RESERVATION_START: host=%s ramp=%d manoeuvre=%d until=%.1f own_speed=%.2f target_speed=%.2f required_speed=%.2f gap_eta=%.2f", self.vehicle_id, rsid, rmid, self.active_merge_request_until, cs, yt, rqs, oe - me)
            log.debug("[%.1f] %s HOST_RESERVED: merge=%d manoeuvre=%d until=%.1f target_spd=%.2f", n, self.vehicle_id, rsid, rmid, self.active_merge_request_until, yt)
            log.debug("[%.1f] %s HOST_YIELD: for merge=%d req_spd=%.2f", n, self.vehicle_id, rsid, yt)
            if n - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
                self._log_host_decision(rsid, rmid, "ACCEPT", "YIELDING")
                self._send_mcm(2, rmid, target_station_id=rsid); self.last_mcm_response[rsid] = n
            else:
                self._log_host_decision(rsid, rmid, "IGNORE", "YIELDING_PERIOD")
            return
        if asafe or (ns and sa):
            self._set_state(STATE_CRUISE if asafe else STATE_YIELDING); hs = min(cs, max(rqs, self.cruise_speed * self.host_yield_floor_ratio))
            if not asafe: hs = max(min(hs, cs - self.host_min_yield_delta), sf); self._set_target_speed(hs, force=True)
            self.active_merge_request, self.active_merge_request_until = {"station_id": rsid, "manoeuvre_id": rmid, "target_speed": hs, "target_eta": te}, n + self.host_reservation_s; self.active_merge_request_started_at = n
            log.info("HOST_RESERVATION_START: host=%s ramp=%d manoeuvre=%d until=%.1f own_speed=%.2f target_speed=%.2f required_speed=%.2f gap_eta=%.2f", self.vehicle_id, rsid, rmid, self.active_merge_request_until, cs, hs, rqs, oe - me)
            log.debug("[%.1f] %s HOST_RESERVED: merge=%d manoeuvre=%d until=%.1f target_spd=%.2f", n, self.vehicle_id, rsid, rmid, self.active_merge_request_until, hs)
            if n - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
                self._log_host_decision(rsid, rmid, "ACCEPT", "SAFE_OR_NOMINAL")
                self._send_mcm(2, rmid, target_station_id=rsid); self.last_mcm_response[rsid] = n
            else:
                self._log_host_decision(rsid, rmid, "IGNORE", "NOMINAL_PERIOD")
            return
        self._set_state(STATE_CRUISE)
        if n - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
            self._log_host_decision(rsid, rmid, "REJECT", "UNSAFE_GAP")
            self._send_mcm(3, rmid, target_station_id=rsid); self.last_mcm_response[rsid] = n
        else:
            self._log_host_decision(rsid, rmid, "IGNORE", "UNSAFE_PERIOD")

    def _fsm_lead(self):
        if self._has_active_host_reservation(): self._fsm_host(); return
        mid = self._merge_candidate_id()
        if not mid: self._set_state(STATE_CRUISE); return
        me = self._neighbor_eta(mid)
        if not me: self._set_state(STATE_CRUISE); return
        md = self._distance_to_merge(self.neighbors[mid]["x"], self.neighbors[mid]["y"])
        if md <= self.priority_distance:
            cl = max(self.cruise_speed + self.lead_speed_bonus, self._current_speed() or self.cruise_speed)
            self._set_state(STATE_CRUISE); self._set_target_speed(cl, force=True); self.target_speed_mode = self.priority_speed_mode
        else: self._set_state(STATE_CRUISE)

    def run(self):
        self.connect()
        try:
            while True: self.step(); time.sleep(0.01)
        finally: self.client.loop_stop()

def main(): app = OBUApp(); app.run()
if __name__ == "__main__": main()
