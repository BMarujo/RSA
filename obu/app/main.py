import json
import logging
import math
import os
import time

import paho.mqtt.client as mqtt

try:
    from .config import OBUConfig, load_json
    from .geometry import (
        edge_id_from_lane,
        normalize_heading_deg,
        parse_lane_index,
    )
    from .fsm import FsmMixin
    from .merge_support import MergeSupportMixin
    from .messaging import MessagingMixin
    from .protocol import (
        STATE_CRUISE,
        STATE_MERGING,
    )
except ImportError:
    from config import OBUConfig, load_json
    from geometry import (
        edge_id_from_lane,
        normalize_heading_deg,
        parse_lane_index,
    )
    from fsm import FsmMixin
    from merge_support import MergeSupportMixin
    from messaging import MessagingMixin
    from protocol import (
        STATE_CRUISE,
        STATE_MERGING,
    )

log = logging.getLogger("obu")
logging.basicConfig(level=logging.DEBUG, format="%(message)s")

class OBUApp(MessagingMixin, MergeSupportMixin, FsmMixin):
    def __init__(self):
        self.config = OBUConfig.from_env()
        self.__dict__.update(self.config.__dict__)
        self.remote_vehicle_status = {}
        self.merge_completed, self.merge_completed_since, self.past_merge_point, self.missed_merge_logged = False, 0.0, False, False
        self.min_distance_to_merge_seen = float("inf")
        self.post_clear_rear_guard_started_at, self.post_clear_rear_guard_last_log = 0.0, 0.0
        self.apply_rear_guard_last_log = 0.0
        self.merge_committed, self.merge_committed_since = False, 0.0
        self.merge_physical_started_once = False
        self.host_clear_lane_until, self.host_clear_for_station = 0.0, None
        self.locked_slot, self.locked_slot_until, self.slot_blocked_since = None, 0.0, 0.0
        self.active_merge_request, self.active_merge_request_until = None, 0.0
        self.mcm_retry_blocked_until = 0.0
        self.merge_deadlock_since = 0.0
        self.pending_host_lost_since = 0.0
        self.merge_authorized = False
        self.merge_authorized_since = 0.0
        self.merge_accepted = False
        self.merge_accepted_since = 0.0
        self.accepted_slot_invalid_since = 0.0
        self.accepted_slot_invalid_timeout_s = 1.5
        self.last_accepted_wait_log = 0.0
        self.last_lclear_block_log = 0.0
        self.merge_safety_hold_since = 0.0
        self.had_merge_timeout_this_attempt = False
        self.rejected_hosts_until = {}
        self.skip_car_following_this_step = False
        self.committed_lead_id, self.committed_host_id, self.committed_manoeuvre_id = None, None, None
        self.count_late_merge_recovery, self.count_merge_failed_no_gap, self.count_merge_completed, self.count_merge_completed_clean, self.recovery_triggered_this_merge = 0, 0, 0, 0, False
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
        self.last_merge_prepare_wait_log_key, self.last_merge_prepare_wait_log_time = None, 0.0
        self.last_wait_edge_floor_log = 0.0
        self.last_lost_auth_after_point_floor_log = 0.0
        self.merge_lane_apply_seen_since = 0.0
        self.last_lane_clear_time = 0.0
        self.last_cam_sent, self.last_mcm_sent, self.last_fsm_step, self.last_actuator_sent, self.last_status_sent = 0.0, 0.0, 0.0, 0.0, 0.0
        self.neighbors, self.neighbor_memory, self.mcm_messages, self.pending_request, self.last_mcm_response = {}, {}, {}, None, {}
        self.mcm_seq, self.denm_seq, self.target_speed, self.target_lane_index, self.target_speed_mode = 0, 0, None, None, 0
        self.fsm_state, self.fsm_state_since, self.effective_role = STATE_CRUISE, 0.0, self.role
        self.following_active, self.following_station_id, self.following_gap_m, self.following_reason = False, None, None, ""
        self.first_sensor_time = None

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
                if state == "CLEAR":
                    try:
                        target_lane = int(p.get("target_lane"))
                    except (TypeError, ValueError):
                        target_lane = None
                    cspd = self._current_speed() or 0.0
                    if (
                        self.is_ramp_vehicle
                        and not self.merge_completed
                        and target_lane == self.merge_lane_index
                        and cspd < self.min_merge_entry_speed
                    ):
                        self._set_target_speed(max(self.merge_stalled_recovery_speed, self.min_merge_entry_speed), force=True)
                    getattr(self, "_log_timeline_event", lambda *x,**y: None)("CLEAR")
                self.last_lane_command_status_key = key
                if state == "WAIT_EDGE":
                    log.debug("[%.1f] %s MERGE_LANE_WAIT_EDGE_CONFIRMED: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s", self._sim_time(), self.vehicle_id, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"), p.get("executable"))
                elif state == "APPLY":
                    try:
                        if int(p.get("target_lane")) == self.merge_lane_index:
                            self.merge_lane_apply_seen_since = self._sim_time()
                    except Exception:
                        pass
                    log.debug("[%.1f] %s MERGE_LANE_APPLY_CONFIRMED: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s", self._sim_time(), self.vehicle_id, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"), p.get("executable"))
                elif state == "CLEAR":
                    self.last_lane_clear_time = self._sim_time()
                    log.debug("[%.1f] %s MERGE_LANE_CLEAR_CONFIRMED: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s", self._sim_time(), self.vehicle_id, p.get("edge_id"), p.get("current_lane"), p.get("target_lane"), p.get("lane_count"), p.get("executable"))
        elif msg.topic == self.cam_out_topic: self._handle_cam(p)
        elif msg.topic == self.mcm_out_topic: self._handle_mcm(p)

    def _set_state(self, s):
        if self.fsm_state != s:
            self.fsm_state, self.fsm_state_since = s, self._sim_time()
            if s == STATE_MERGING: self.merge_merging_started_since = self._sim_time()

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

    def _lane_change_executable_now(self):
        lcs = self.lane_command_status or {}
        try:
            target_lane = int(lcs.get("target_lane"))
        except Exception:
            target_lane = self.target_lane_index if self.target_lane_index is not None else self.merge_lane_index
        if target_lane != self.merge_lane_index:
            return False
        if self.merge_lane_apply_seen_since > 0.0 and self._sim_time() - self.merge_lane_apply_seen_since <= 2.0:
            return True
        if not self._lane_command_status_fresh(): return False
        if lcs.get("state") in ("APPLY", "CLEAR"):
            return True
        if lcs.get("state") == "FAILED":
            return False
        try:
            lane_count = int(lcs.get("lane_count"))
        except Exception:
            lane_count = None
        return lcs.get("executable") is True or (lane_count is not None and lane_count > target_lane)

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
            if self._is_main_traffic(s) and self._distance_to_merge(float(d["x"]), float(d["y"])) <= self.role_detection_distance: return True
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
            lid = self.committed_lead_id or (int(self.pending_request["lead_id"]) if self.pending_request and self.pending_request.get("lead_id") else None)
            hid = self.committed_host_id or (int(self.pending_request["host_id"]) if self.pending_request and self.pending_request.get("host_id") else None)
            if not self.merge_physical_started_once:
                le, he = self._neighbor_eta(lid) if lid else None, self._neighbor_eta(hid) if hid else None
                e, dtm, cspd = self._merge_eta(), self._self_distance_to_merge(), self._current_speed() or 0.0
                self._mark_merge_physical_start(curt, lid, hid, le, he, e, dtm, cspd, lidx, eid, reason="implicit_before_completed")
            if self._post_clear_rear_guard_hold(curt, lid, hid, eid, lidx):
                return True
            self._log_timeline_event("COMPLETED")
            self.merge_completed_since, self.count_merge_completed = curt, self.count_merge_completed + 1
            clean_merge = not getattr(self, 'recovery_triggered_this_merge', False) and not self.had_merge_timeout_this_attempt
            if clean_merge: self.count_merge_completed_clean += 1
            if self.had_merge_timeout_this_attempt: log.debug("[%.1f] %s MERGE_COMPLETED_AFTER_TIMEOUT: eid=%s lane=%d", curt, self.vehicle_id, eid, lidx)
            else: log.debug("[%.1f] %s MERGE_COMPLETED: eid=%s lane=%d clean=%s", curt, self.vehicle_id, eid, lidx, clean_merge)
            if self.had_merge_timeout_this_attempt:
                dur = curt - getattr(self, 'merge_physical_started_since', -1.0)
                lcs = self.lane_command_status.get("state", "NONE") if self.lane_command_status else "NONE"
                tl = self.lane_command_status.get("target_lane", "None") if self.lane_command_status else "None"
                tspd = self.target_speed if getattr(self, 'target_speed', None) is not None else 0.0
                cspd = self._current_speed() or 0.0
                log.info("MERGE_COMPLETION_LATENCY_DIAG: vehicle=%s duration_start_to_complete=%.1f start_time=%.1f merging_time=%.1f clear_time=%.1f completed_time=%.1f lane_cmd_state=%s edge=%s lane=%s target_lane=%s speed=%.2f target_speed=%.2f", self.vehicle_id, dur, getattr(self, 'merge_physical_started_since', -1.0), getattr(self, 'merge_merging_started_since', -1.0), getattr(self, 'last_lane_clear_time', -1.0), curt, lcs, eid, lidx, tl, cspd, tspd)
            self.merge_completed, self.merge_committed, self.merge_authorized, self.merge_authorized_since, self.had_merge_timeout_this_attempt, self.merge_deadlock_since, self.merge_safety_hold_since, self.merge_accepted, self.merge_accepted_since, self.accepted_slot_invalid_since = True, False, False, 0.0, False, 0.0, 0.0, False, 0.0, 0.0
            self.merge_physical_started_once = False
            self.merge_lane_apply_seen_since = 0.0
            self.post_clear_rear_guard_started_at, self.post_clear_rear_guard_last_log = 0.0, 0.0
            self.committed_lead_id, self.committed_host_id, self.committed_manoeuvre_id, self.recovery_triggered_this_merge = None, None, None, False
            self._set_state(STATE_CRUISE); self.target_lane_index, self.pending_request = None, None
            es = max(self.cruise_speed, self.min_merge_entry_speed); self._set_target_speed(es, force=True); self.target_speed_mode, self.skip_car_following_this_step = self.priority_speed_mode, False; return True
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

    def _is_main_traffic(self, s):
        if s in self.main_station_ids: return True
        if s in self.ramp_station_ids and self.remote_vehicle_status.get(s, {}).get("merge_completed", False): return True
        return False

    def _neighbor_is_merge_candidate(self, s):
        d = self.neighbors.get(s)
        if not d: return False
        dist = self._distance_to_merge(float(d["x"]), float(d["y"]))
        if dist > self.role_detection_distance: return False
        if d.get("distance_delta") is not None and float(d["distance_delta"]) > 0.5: return False
        app = self._neighbor_is_approaching_merge(d)
        if s in self.ramp_station_ids:
            if self.remote_vehicle_status.get(s, {}).get("merge_completed", False): return False
            if not app: return False
            if self.ramp_bbox:
                x1, y1, x2, y2 = self.ramp_bbox
                return (x1 <= float(d["x"]) <= x2 and y1 <= float(d["y"]) <= y2) or dist <= self.priority_distance
            return True
        return app and (s == self.merge_station_id if self.merge_station_id else float(d["y"]) <= self.ramp_y_threshold)

    def _neighbor_is_main_candidate(self, s):
        d = self.neighbors.get(s)
        if not d or self._distance_to_merge(float(d["x"]), float(d["y"])) > self.role_detection_distance: return False
        return self._neighbor_is_approaching_merge(d) and self._is_main_traffic(s)

    def run(self):
        self.connect()
        try:
            while True: self.step(); time.sleep(0.01)
        finally: self.client.loop_stop()

def main(): app = OBUApp(); app.run()
if __name__ == "__main__": main()
