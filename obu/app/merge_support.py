import logging
import math

try:
    from .geometry import edge_id_from_lane, parse_lane_index
    from .protocol import STATE_MERGING, STATE_NEGOTIATING, mcm_action_name
except ImportError:
    from geometry import edge_id_from_lane, parse_lane_index
    from protocol import STATE_MERGING, STATE_NEGOTIATING, mcm_action_name


log = logging.getLogger("obu")


class MergeSupportMixin:
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

    def _post_clear_gap_values(self, sid):
        if sid is None or not self.sensor_state or self._current_heading() is None:
            return None
        d = self.neighbors.get(sid) or self.neighbor_memory.get(sid)
        if not d:
            return None
        ox, oy, oh = float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0)), self._current_heading()
        rad = math.radians(90 - oh)
        fx, fy = math.cos(rad), math.sin(rad)
        dx, dy = float(d.get("x", 0.0)) - ox, float(d.get("y", 0.0)) - oy
        lon, lat = dx * fx + dy * fy, abs(-dx * fy + dy * fx)
        own_speed, ns = self._current_speed() or 0.0, float(d.get("speed", 0.0))
        gap = max(0.0, abs(lon) - self.vehicle_length)
        closing = ns - own_speed if lon < 0 else own_speed - ns
        closing_pos = max(closing, 0.0)
        ttc = gap / closing_pos if closing_pos > 0.01 else float("inf")
        return {
            "gap": gap,
            "lat": lat,
            "closing": closing,
            "ttc": ttc,
            "t1": max(0.0, gap - closing_pos),
            "t2": max(0.0, gap - closing_pos * 2.0),
            "t3": max(0.0, gap - closing_pos * 3.0),
            "rear": lon < 0,
            "speed": ns,
        }

    def _target_lane_rear_hazard(self, hid=None):
        if not self.apply_rear_guard_enabled or not self.sensor_state or self._current_heading() is None:
            return None
        lcs = self.lane_command_status or {}
        if lcs.get("state") not in ("APPLY", "CLEAR"):
            return None
        target_lane = lcs.get("target_lane")
        if target_lane is None:
            target_lane = self.target_lane_index if self.target_lane_index is not None else self.merge_lane_index
        try:
            target_lane = int(target_lane)
        except (TypeError, ValueError):
            target_lane = self.merge_lane_index
        if target_lane != self.merge_lane_index:
            return None
        lid_s = str(self.sensor_state.get("lane_id", ""))
        self_edge = edge_id_from_lane(lid_s)
        cmd_edge = lcs.get("edge_id") or self_edge
        ox, oy, oh = float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0)), self._current_heading()
        rad = math.radians(90 - oh)
        fx, fy = math.cos(rad), math.sin(rad)
        own_speed = self._current_speed() or 0.0
        worst = None
        for sid, d in self.neighbors.items():
            rst = self.remote_vehicle_status.get(sid, {})
            lane_ok = sid == hid
            if not lane_ok:
                try:
                    lane_ok = int(rst.get("lane_index")) == target_lane
                except (TypeError, ValueError):
                    lane_ok = False
            if not lane_ok:
                continue
            redge = rst.get("edge_id")
            if sid != hid and redge and redge not in (self_edge, cmd_edge, "1331698336"):
                continue
            dx, dy = float(d.get("x", 0.0)) - ox, float(d.get("y", 0.0)) - oy
            lon, lat = dx * fx + dy * fy, abs(-dx * fy + dy * fx)
            if lon >= 0 or lat > self.cam_follow_lateral_tolerance * 1.8:
                continue
            ns = float(d.get("speed", 0.0))
            gap = max(0.0, abs(lon) - self.vehicle_length)
            closing = ns - own_speed
            closing_pos = max(closing, 0.0)
            if closing_pos < self.apply_rear_min_closing_mps:
                continue
            ttc = gap / closing_pos
            t1, t2, t3 = max(0.0, gap - closing_pos), max(0.0, gap - closing_pos * 2.0), max(0.0, gap - closing_pos * 3.0)
            hazard = t1 <= self.apply_rear_min_gap_m or t2 <= self.apply_rear_min_gap_m or ttc < self.apply_rear_ttc_s
            if not hazard:
                continue
            cand = {"sid": sid, "gap": gap, "t1": t1, "t2": t2, "t3": t3, "ttc": ttc, "speed": ns, "lat": lat, "closing": closing_pos}
            if worst is None or cand["ttc"] < worst["ttc"] or cand["t2"] < worst["t2"]:
                worst = cand
        return worst

    def _apply_rear_guard_flow_protect(self, curt, hid, eid, lidx, cspd, dtm):
        hv = self._target_lane_rear_hazard(hid)
        if not hv:
            return False
        floor = max(self.cruise_speed * self.apply_rear_flow_floor_ratio, self.min_merge_entry_speed)
        if self.target_speed is None or self.target_speed < floor:
            self._set_target_speed(floor, force=True)
        self.target_speed_mode, self.target_lane_index = self.priority_speed_mode, self.merge_lane_index
        if curt - self.apply_rear_guard_last_log >= 0.5:
            lcs = self.lane_command_status or {}
            log.info(
                "MERGE_APPLY_REAR_GUARD_FLOW_PROTECT: vehicle=%s rear=%s host=%s speed=%.2f target=%.2f "
                "rear_speed=%.2f rear_gap=%.2f rear_gap_t1=%.2f rear_gap_t2=%.2f rear_gap_t3=%.2f rear_closing=%.2f closing_ttc=%.2f "
                "lane_cmd_state=%s edge=%s lane=%s dtm=%.1f",
                self.vehicle_id, hv["sid"], hid, cspd, self.target_speed or 0.0, hv["speed"], hv["gap"], hv["t1"], hv["t2"], hv["t3"], hv["closing"], hv["ttc"],
                lcs.get("state"), eid, lidx, dtm
            )
            self.apply_rear_guard_last_log = curt
        return True

    def _post_clear_rear_guard_hold(self, curt, lid, hid, eid, lidx):
        if not self.post_clear_rear_guard_enabled:
            return False
        
        # Comprehensive scan for any rear hazard in the target lane (not just the host)
        lcs = self.lane_command_status or {}
        target_lane = lcs.get("target_lane")
        if target_lane is None:
            target_lane = self.target_lane_index if self.target_lane_index is not None else self.merge_lane_index
        try:
            target_lane = int(target_lane)
        except (TypeError, ValueError):
            target_lane = self.merge_lane_index
            
        lid_s = str(self.sensor_state.get("lane_id", ""))
        self_edge = edge_id_from_lane(lid_s)
        cmd_edge = lcs.get("edge_id") or self_edge
        
        ox, oy, oh = float(self.sensor_state.get("x", 0.0)), float(self.sensor_state.get("y", 0.0)), self._current_heading()
        rad = math.radians(90 - oh)
        fx, fy = math.cos(rad), math.sin(rad)
        own_speed = self._current_speed() or 0.0
        
        worst = None
        for sid, d in self.neighbors.items():
            rst = self.remote_vehicle_status.get(sid, {})
            # Check if neighbor is in the target lane
            lane_ok = sid == hid
            if not lane_ok:
                try:
                    lane_ok = int(rst.get("lane_index")) == target_lane
                except (TypeError, ValueError):
                    lane_ok = False
            if not lane_ok:
                continue
            
            # Check if neighbor is on a relevant edge
            redge = rst.get("edge_id")
            if sid != hid and redge and redge not in (self_edge, cmd_edge, "1331698336"):
                continue
                
            dx, dy = float(d.get("x", 0.0)) - ox, float(d.get("y", 0.0)) - oy
            lon, lat = dx * fx + dy * fy, abs(-dx * fy + dy * fx)
            
            # Must be behind and within lateral tolerance
            if lon >= 0 or lat > self.cam_follow_lateral_tolerance * 1.8:
                continue
                
            ns = float(d.get("speed", 0.0))
            gap = max(0.0, abs(lon) - self.vehicle_length)
            closing = ns - own_speed
            closing_pos = max(closing, 0.0)
            
            ttc = gap / closing_pos if closing_pos > 0.01 else float("inf")
            t1, t2 = max(0.0, gap - closing_pos), max(0.0, gap - closing_pos * 2.0)
            t3 = max(0.0, gap - closing_pos * 3.0)
            
            hazard = (
                t1 <= self.post_clear_min_rear_gap_m
                or t2 <= self.post_clear_min_rear_gap_m
                or ttc < self.post_clear_rear_ttc_s
            )
            
            if not hazard:
                continue
                
            cand = {"sid": sid, "gap": gap, "t1": t1, "t2": t2, "t3": t3, "ttc": ttc, "speed": ns}
            if worst is None or cand["ttc"] < worst["ttc"] or cand["t2"] < worst["t2"]:
                worst = cand

        if not worst:
            if self.post_clear_rear_guard_started_at > 0.0:
                # Need some dummy values for the release log if we don't have a specific neighbor anymore
                log.info(
                    "MERGE_COMPLETION_REAR_GUARD_RELEASE: vehicle=%s reason=safe_gap host=%s lead=%s edge=%s lane=%s",
                    self.vehicle_id, hid, lid, eid, lidx
                )
                self.post_clear_rear_guard_started_at = 0.0
            return False
            
        if self.post_clear_rear_guard_started_at <= 0.0:
            self.post_clear_rear_guard_started_at = curt
            
        held_s = curt - self.post_clear_rear_guard_started_at
        if held_s >= self.post_clear_rear_guard_max_s:
            log.info(
                "MERGE_COMPLETION_REAR_GUARD_RELEASE: vehicle=%s reason=max_time host=%s lead=%s held_s=%.1f "
                "hazard_sid=%s gap=%.2f ttc=%.2f edge=%s lane=%s",
                self.vehicle_id, hid, lid, held_s, worst["sid"], worst["gap"], worst["ttc"], eid, lidx
            )
            self.post_clear_rear_guard_started_at = 0.0
            return False
            
        lv = self._post_clear_gap_values(lid)
        lead_gap = lv["gap"] if lv else None
        if curt - self.post_clear_rear_guard_last_log >= 0.5:
            log.info(
                "MERGE_COMPLETION_REAR_GUARD_HOLD: vehicle=%s host=%s lead=%s held_s=%.1f speed=%.2f target=%.2f "
                "lead_gap=%s hazard_sid=%s gap=%.2f t1=%.2f t2=%.2f t3=%.2f ttc=%.2f lane_cmd_state=%s edge=%s lane=%s",
                self.vehicle_id, hid, lid, held_s, own_speed, self.target_speed or 0.0,
                f"{lead_gap:.2f}" if lead_gap is not None else "None",
                worst["sid"], worst["gap"], worst["t1"], worst["t2"], worst["t3"], worst["ttc"], 
                lcs.get("state", "NONE"), eid, lidx
            )
            self.post_clear_rear_guard_last_log = curt
            
        self.merge_committed = True
        self.merge_completed = False
        self.target_lane_index = self.merge_lane_index
        self.target_speed_mode = self.priority_speed_mode
        self._set_state(STATE_MERGING)
        return True

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

    def _log_merge_prepare_wait_lane_available(self, curt, dtm, cspd, hid=None, lid=None):
        lcs = self.lane_command_status or {}
        key = (lcs.get("state"), lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"), lcs.get("lane_count"), lcs.get("executable"))
        if key == self.last_merge_prepare_wait_log_key and curt - self.last_merge_prepare_wait_log_time < 1.0:
            return
        self.last_merge_prepare_wait_log_key, self.last_merge_prepare_wait_log_time = key, curt
        log.debug(
            "[%.1f] %s MERGE_PREPARE_WAIT_LANE_AVAILABLE: edge=%s lane=%s target_lane=%s lane_count=%s "
            "executable=%s dtm=%.1f speed=%.2f host=%s lead=%s",
            curt, self.vehicle_id, lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"),
            lcs.get("lane_count"), lcs.get("executable"), dtm, cspd, hid, lid
        )

    def _mark_merge_physical_start(self, curt, lid, hid, le, he, e, dtm, cspd, lidx, eid, reason="start", resume=False):
        lcs = self.lane_command_status or {}
        start_reason = reason
        if not resume and reason == "start":
            if lcs.get("state") == "CLEAR":
                start_reason = "already_clear"
            elif lcs.get("state") == "APPLY" or (self.merge_lane_apply_seen_since > 0.0 and curt - self.merge_lane_apply_seen_since <= 2.0):
                start_reason = "apply_seen"
            elif lcs.get("executable") is True:
                start_reason = "executable"
            else:
                try:
                    if int(lcs.get("lane_count")) > int(lcs.get("target_lane")):
                        start_reason = "lane_count_available"
                except Exception:
                    pass
        if self.merge_physical_started_once:
            if resume:
                self._log_merge_start_gap_diag("resume", lid, hid, le, he, e, dtm, cspd, lidx, eid)
                log.debug("[%.1f] %s MERGE_PHYSICAL_RESUME: host=%s manoeuvre=%s", curt, self.vehicle_id, hid, self.pending_request.get("manoeuvre_id") if self.pending_request else 0)
            self._set_state(STATE_MERGING)
            return False
        if not self.merge_committed:
            self.merge_committed, self.merge_committed_since = True, curt
            self.committed_lead_id, self.committed_host_id = lid, hid
            self.committed_manoeuvre_id = self.pending_request.get("manoeuvre_id") if self.pending_request else None
        waited_s = max(0.0, curt - self.merge_committed_since) if self.merge_committed_since else 0.0
        if cspd < self.min_merge_entry_speed and (self.target_speed is None or self.target_speed < self.min_merge_entry_speed):
            self._set_target_speed(max(self.merge_stalled_recovery_speed, self.min_merge_entry_speed), force=True)
        if not resume and (waited_s > 0.0 or start_reason in ("delayed", "implicit_before_completed", "already_clear", "apply_seen")):
            log.debug(
                "[%.1f] %s MERGE_PHYSICAL_START_DELAYED_UNTIL_LANE_APPLY: waited_s=%.1f edge=%s lane=%s "
                "target_lane=%s lane_count=%s lane_cmd_state=%s executable=%s dtm=%.1f speed=%.2f reason=%s",
                curt, self.vehicle_id, waited_s, lcs.get("edge_id"), lcs.get("current_lane"),
                lcs.get("target_lane"), lcs.get("lane_count"), lcs.get("state"), lcs.get("executable"),
                dtm, cspd, start_reason
            )
        self.merge_physical_started_once, self.merge_physical_started_since = True, curt
        self._log_merge_start_gap_diag(start_reason, lid, hid, le, he, e, dtm, cspd, lidx, eid)
        self._log_timeline_event("PHYSICAL_START")
        log.debug("[%.1f] %s MERGE_PHYSICAL_START: host=%s manoeuvre=%s", curt, self.vehicle_id, hid, self.pending_request.get("manoeuvre_id") if self.pending_request else 0)
        log.debug("[%.1f] %s MERGING!", curt, self.vehicle_id)
        self._set_state(STATE_MERGING)
        return True

    def _start_physical_merge(self, curt, lid, hid, le, he, e, dtm, cspd, lidx, eid, resume=False):
        return self._mark_merge_physical_start(curt, lid, hid, le, he, e, dtm, cspd, lidx, eid, reason="resume" if resume else "start", resume=resume)

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

    def _is_host_busy(self, hid):
        if hid is None: return False
        hst = self.remote_vehicle_status.get(hid, {})
        busy = hst.get("active_merge_request") is True
        busy_for = hst.get("active_merge_request_station_id")
        remaining = hst.get("active_merge_request_remaining_s", 0)
        try:
            busy_for_id = int(busy_for) if busy_for is not None else None
        except (ValueError, TypeError):
            busy_for_id = None
        return busy and remaining > 0.25 and busy_for_id != self.station_id

    def _select_merge_slot(self, e):
        mes = self._main_candidate_etas()
        if not mes: return None, None, None, None, None, None, True, e, "no_main_neighbors"
        hi = next((i for i, (me, _) in enumerate(mes) if me >= e), None)
        if hi is not None:
            while hi < len(mes):
                he, hid = mes[hi]
                if self._is_host_busy(hid):
                    if self._sim_time() - getattr(self, "last_skip_busy_log", 0) > 1.0:
                        hst = self.remote_vehicle_status.get(hid, {})
                        busy_for = hst.get("active_merge_request_station_id")
                        rem = hst.get("active_merge_request_remaining_s")
                        next_hid = mes[hi+1][1] if hi + 1 < len(mes) else None
                        _, lid_temp = mes[hi-1] if hi > 0 else (None, None)
                        log.info("MERGE_SLOT_SKIP_BUSY_HOST: skipped_host=%s busy_for=%s remaining=%s next_host=%s lead=%s own_eta=%.2f", 
                                 hid, busy_for, rem, next_hid, lid_temp, e)
                        self.last_skip_busy_log = self._sim_time()
                    hi += 1
                else:
                    break
            
            if hi < len(mes):
                he, hid = mes[hi]; le, lid = mes[hi-1] if hi > 0 else (None, None); mine, maxe, gp = self._merge_slot_window(le, he)
                return lid, hid, le, he, mine, maxe, gp, self._desired_eta_for_window(e, mine, maxe), "selected"
            else:
                le, lid = mes[-1]; mine, maxe, gp = self._merge_slot_window(le, None); return lid, None, le, None, mine, maxe, gp, self._desired_eta_for_window(e, mine, maxe), "true_after_last_main"
        
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

