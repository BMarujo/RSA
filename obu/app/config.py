import json
import os


def env(name, default):
    value = os.getenv(name)
    return value if value is not None and value != "" else default


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_csv(value):
    return {item.strip() for item in value.split(",") if item.strip()}


def parse_bbox(value):
    try:
        parts = [float(item.strip()) for item in value.split(",") if item.strip()]
        if len(parts) == 4:
            return (
                min(parts[0], parts[2]),
                min(parts[1], parts[3]),
                max(parts[0], parts[2]),
                max(parts[1], parts[3]),
            )
    except (TypeError, ValueError):
        pass
    return None


def env_bool(name, default):
    return env(name, default).lower() == "true"


def env_float(name, default):
    return float(env(name, str(default)))


def env_int(name, default):
    return int(env(name, str(default)))


def parse_int_csv(value):
    return {int(item) for item in parse_csv(value) if item.isdigit()}


class OBUConfig:
    @classmethod
    def from_env(cls):
        cfg = cls()

        cfg.host_reservation_max_s = env_float("HOST_RESERVATION_MAX_S", "10.0")
        cfg.vehicle_id = env("VEHICLE_ID", "v1")
        cfg.station_id = env_int("STATION_ID", "1")
        cfg.station_type = env_int("STATION_TYPE", "5")
        cfg.mcm_station_type = env_int("MCM_STATION_TYPE", "1")
        cfg.itss_role = env_int("ITSS_ROLE", "1")
        cfg.role, cfg.role_mode = env("VEHICLE_ROLE", "host").lower(), "static"
        if cfg.role == "auto":
            cfg.role, cfg.role_mode = "host", "auto"

        cfg.local_mqtt_host = env("LOCAL_MQTT_HOST", "127.0.0.1")
        cfg.local_mqtt_port = env_int("LOCAL_MQTT_PORT", "1883")
        cfg.origin_lat = env_float("ORIGIN_LAT", "40.0")
        cfg.origin_lon = env_float("ORIGIN_LON", "-8.0")
        cfg.vehicle_length = env_float("VEHICLE_LENGTH", "4.5")
        cfg.vehicle_width = env_float("VEHICLE_WIDTH", "1.9")
        cfg.cruise_speed = env_float("CRUISE_SPEED", "15.0")
        cfg.merge_speed_bonus = env_float("MERGE_SPEED_BONUS", "1.0")
        cfg.lead_speed_bonus = env_float("LEAD_SPEED_BONUS", "1.0")
        cfg.priority_merge = env_bool("MERGE_PRIORITY", "true")

        merge_station_id = env_int("MERGE_STATION_ID", "0")
        cfg.merge_station_id = merge_station_id if merge_station_id > 0 else None
        cfg.default_speed_mode = env_int("DEFAULT_SPEED_MODE", "0")
        cfg.priority_speed_mode = env_int("PRIORITY_SPEED_MODE", "0")
        cfg.priority_distance = env_float("PRIORITY_DISTANCE", "40.0")
        cfg.cam_period_s = env_int("CAM_PERIOD_MS", "100") / 1000.0
        cfg.fsm_period_s = env_int("FSM_PERIOD_MS", "100") / 1000.0
        cfg.actuator_period_s = env_int("ACTUATOR_PERIOD_MS", "100") / 1000.0
        cfg.status_period_s = env_int("STATUS_PERIOD_MS", "250") / 1000.0

        cfg.merge_point_x = env_float("MERGE_POINT_X", "0")
        cfg.merge_point_y = env_float("MERGE_POINT_Y", "0")
        cfg.merge_lane_index = env_int("MERGE_LANE_INDEX", "0")
        cfg.merge_zone_clearance_m = env_float("MERGE_ZONE_CLEARANCE_M", "45.0")
        cfg.merge_stop_margin_m = env_float("MERGE_STOP_MARGIN_M", "18.0")
        cfg.merge_blocked_approach_s = env_float("MERGE_BLOCKED_APPROACH_S", "4.0")
        cfg.eta_threshold_s = env_float("ETA_THRESHOLD_S", "5.0")
        cfg.safe_headway_s = env_float("SAFE_HEADWAY_S", "1.5")
        cfg.negotiation_timeout_s = env_float("NEGOTIATION_TIMEOUT_S", "2.0")
        cfg.request_retry_s = env_float("REQUEST_RETRY_S", "0.5")
        cfg.response_period_s = env_float("RESPONSE_PERIOD_S", "0.5")
        cfg.neighbor_timeout_s = env_float("NEIGHBOR_TIMEOUT_S", "1.0")
        cfg.abort_speed = env_float("ABORT_SPEED", "2.0")
        cfg.abort_cooldown_s = env_float("ABORT_COOLDOWN_S", "3.0")
        cfg.min_speed = env_float("MIN_SPEED", "0.5")
        cfg.emergency_min_speed = env_float("EMERGENCY_MIN_SPEED", "0.0")
        cfg.min_clearance_m = env_float("MIN_CLEARANCE_M", "8.0")
        cfg.final_merge_guard_m = env_float("FINAL_MERGE_GUARD_M", "28.0")
        cfg.final_merge_clearance_m = env_float("FINAL_MERGE_CLEARANCE_M", "10.0")
        cfg.merge_occupancy_s = env_float("MERGE_OCCUPANCY_S", "3.0")
        cfg.merge_commit_headway_s = env_float("MERGE_COMMIT_HEADWAY_S", cfg.safe_headway_s)
        cfg.min_merge_entry_speed = env_float("MIN_MERGE_ENTRY_SPEED", "5.0")
        cfg.merge_entry_speed_guard_m = env_float("MERGE_ENTRY_SPEED_GUARD_M", "35.0")
        cfg.max_speed_step_up = env_float("MAX_SPEED_STEP_UP", "2.5")
        cfg.max_speed_step_down = env_float("MAX_SPEED_STEP_DOWN", "0.45")
        cfg.max_speed_step_emergency = env_float("MAX_SPEED_STEP_EMERGENCY", "1.5")
        cfg.merge_yield_floor_ratio = env_float("MERGE_YIELD_FLOOR_RATIO", "0.2")
        cfg.host_yield_floor_ratio = env_float("HOST_YIELD_FLOOR_RATIO", "0.2")

        cfg.merge_stalled_recovery_enabled = env_bool("MERGE_STALLED_LEAD_ONLY_RECOVERY", "true")
        cfg.merge_stalled_recovery_speed = env_float("MERGE_STALLED_RECOVERY_SPEED", cfg.min_merge_entry_speed)
        cfg.merge_wait_edge_floor_ratio = env_float("MERGE_WAIT_EDGE_FLOOR_RATIO", "0.45")
        cfg.merge_wait_edge_hostless_timeout_s = env_float("MERGE_WAIT_EDGE_HOSTLESS_TIMEOUT_S", "75.0")
        cfg.merge_lost_auth_after_point_floor_enabled = env_bool("MERGE_LOST_AUTH_AFTER_POINT_ROLLING_FLOOR", "true")
        cfg.merge_lost_auth_after_point_floor_ratio = env_float("MERGE_LOST_AUTH_AFTER_POINT_FLOOR_RATIO", cfg.merge_wait_edge_floor_ratio)

        cfg.host_reject_distance_m = env_float("HOST_REJECT_DISTANCE_M", "20.0")
        cfg.host_same_lane_guard_gap = env_float("HOST_SAME_LANE_GUARD_GAP", "14.0")
        cfg.host_min_accept_gap_s = env_float("HOST_MIN_ACCEPT_GAP_S", cfg.merge_commit_headway_s)
        cfg.ramp_platoon_headway_s = env_float("RAMP_PLATOON_HEADWAY_S", "1.4")
        cfg.ramp_platoon_min_gap = env_float("RAMP_PLATOON_MIN_GAP", "14.0")
        cfg.ramp_platoon_speed_delta = env_float("RAMP_PLATOON_SPEED_DELTA", "0.8")
        cfg.merge_queue_release_gap = env_float("MERGE_QUEUE_RELEASE_GAP", "34.0")

        cfg.enable_cam_following = env_bool("ENABLE_CAM_FOLLOWING", "true")
        cfg.cam_follow_headway_s = env_float("CAM_FOLLOW_HEADWAY_S", "1.2")
        cfg.cam_follow_min_gap = env_float("CAM_FOLLOW_MIN_GAP", "10.0")
        cfg.cam_follow_lookahead = env_float("CAM_FOLLOW_LOOKAHEAD", "50.0")
        cfg.cam_follow_lateral_tolerance = env_float("CAM_FOLLOW_LATERAL_TOLERANCE_M", "3.8")
        cfg.cam_follow_speed_delta = env_float("CAM_FOLLOW_SPEED_DELTA", "0.8")
        cfg.cam_follow_critical_gap = env_float("CAM_FOLLOW_CRITICAL_GAP_M", "6.0")
        cfg.cam_follow_brake_decel = env_float("CAM_FOLLOW_BRAKE_DECEL", "4.5")
        cfg.cam_follow_emergency_decel = env_float("CAM_FOLLOW_EMERGENCY_DECEL", "9.0")
        cfg.final_guard_ttc_s = env_float("FINAL_GUARD_TTC_S", "3.0")
        cfg.final_guard_lateral_mult = env_float("FINAL_GUARD_LATERAL_MULT", "2.0")

        cfg.ramp_edge_ids = parse_csv(env("RAMP_EDGE_IDS", "ramp_in"))
        cfg.main_edge_ids = parse_csv(env("MAIN_EDGE_IDS", "main_in,main_out"))
        cfg.ramp_station_ids = parse_int_csv(env("RAMP_STATION_IDS", ""))
        cfg.is_ramp_vehicle = cfg.station_id in cfg.ramp_station_ids
        cfg.post_merge_lock_s = env_float("POST_MERGE_LOCK_S", "3.0")
        cfg.post_clear_rear_guard_enabled = env_bool("POST_CLEAR_REAR_GUARD", "true")
        cfg.post_clear_min_rear_gap_m = env_float("POST_CLEAR_MIN_REAR_GAP_M", "4.0")
        cfg.post_clear_rear_ttc_s = env_float("POST_CLEAR_REAR_TTC_S", "2.0")
        cfg.post_clear_rear_guard_max_s = env_float("POST_CLEAR_REAR_GUARD_MAX_S", "2.0")
        cfg.apply_rear_guard_enabled = env_bool("APPLY_REAR_GAP_GUARD", "true")
        cfg.apply_rear_min_gap_m = env_float("APPLY_REAR_MIN_GAP_M", "4.5")
        cfg.apply_rear_ttc_s = env_float("APPLY_REAR_TTC_S", "2.2")
        cfg.apply_rear_min_closing_mps = env_float("APPLY_REAR_MIN_CLOSING_MPS", "0.25")
        cfg.apply_rear_flow_floor_ratio = env_float("APPLY_REAR_FLOW_FLOOR_RATIO", "0.65")

        cfg.merge_commit_timeout_s = env_float("MERGE_COMMIT_TIMEOUT_S", "12.0")
        cfg.merge_accept_timeout_s = env_float("MERGE_ACCEPT_TIMEOUT_S", cfg.merge_commit_timeout_s)
        cfg.merge_commit_distance_m = env_float("MERGE_COMMIT_DISTANCE_M", "35.0")
        cfg.merge_lane_prepare_distance_m = env_float("MERGE_LANE_PREPARE_DISTANCE_M", "95.0")
        cfg.final_guard_stale_neighbor_s = env_float("FINAL_GUARD_STALE_NEIGHBOR_S", "4.0")
        cfg.main_station_ids = parse_int_csv(env("MAIN_STATION_IDS", ""))
        cfg.ramp_y_threshold = env_float("RAMP_Y_THRESHOLD", "-1.0")
        cfg.ramp_bbox = parse_bbox(env("RAMP_BBOX", ""))
        cfg.role_detection_distance = env_float("ROLE_DETECTION_DISTANCE", "180.0")
        cfg.desired_speed = env("DESIRED_SPEED", "")

        cfg.host_clear_lane_index = env_int("HOST_CLEAR_LANE_INDEX", "1")
        cfg.host_cooperative_lane_change = env_bool("HOST_COOPERATIVE_LANE_CHANGE", "false")
        cfg.host_clear_lane_hold_s = env_float("HOST_CLEAR_LANE_HOLD_S", "6.0")
        cfg.host_return_lock_distance_m = env_float("HOST_RETURN_LOCK_DISTANCE_M", "90.0")
        cfg.slot_lock_s = env_float("SLOT_LOCK_S", "4.0")
        cfg.host_reservation_s = env_float("HOST_RESERVATION_S", "4.0")
        cfg.cooperative_host_lookback_s = env_float("COOPERATIVE_HOST_LOOKBACK_S", "5.0")
        cfg.slot_neighbor_grace_s = env_float("SLOT_NEIGHBOR_GRACE_S", "2.5")
        cfg.allow_hostless_merge = env_bool("ALLOW_HOSTLESS_MERGE", "false")
        cfg.host_min_yield_delta = env_float("HOST_MIN_YIELD_DELTA", "1.0")
        cfg.mcm_request_distance_m = env_float("MCM_REQUEST_DISTANCE_M", "95.0")
        cfg.mcm_timeout_cooldown_s = env_float("MCM_TIMEOUT_COOLDOWN_S", "1.0")
        cfg.mcm_stale_host_grace_s = env_float("MCM_STALE_HOST_GRACE_S", "0.75")
        cfg.mcm_late_host_lock_grace_s = env_float("MCM_LATE_HOST_LOCK_GRACE_S", "1.5")
        cfg.mcm_late_host_lock_distance_m = env_float("MCM_LATE_HOST_LOCK_DISTANCE_M", "70.0")
        cfg.merge_deadlock_timeout_s = env_float("MERGE_DEADLOCK_TIMEOUT_S", "4.0")
        cfg.merge_authorized_timeout_s = env_float("MERGE_AUTHORIZED_TIMEOUT_S", "10.0")
        cfg.merge_safety_hold_timeout_s = env_float("MERGE_SAFETY_HOLD_TIMEOUT_S", "2.5")
        cfg.enable_mcm = env_bool("ENABLE_MCM", "true")
        cfg.enable_denm = env_bool("ENABLE_DENM", "false")
        cfg.publish_idle_actuators = env_bool("PUBLISH_IDLE_ACTUATORS", "true")
        cfg.merge_neighbor_warmup_s = env_float("MERGE_NEIGHBOR_WARMUP_S", "1.0")
        cfg.merge_min_neighbors_before_merge = env_int("MERGE_MIN_NEIGHBORS_BEFORE_MERGE", "0")
        cfg.merge_conflict_follow_distance_m = env_float("MERGE_CONFLICT_FOLLOW_DISTANCE_M", "55.0")
        cfg.merge_conflict_floor_ratio = env_float("MERGE_CONFLICT_FLOOR_RATIO", "0.55")

        return cfg
