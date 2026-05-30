import logging
import math

try:
    from .geometry import edge_id_from_lane, parse_lane_index
    from .protocol import STATE_ABORT, STATE_CRUISE, STATE_MERGING, STATE_NEGOTIATING, STATE_YIELDING
except ImportError:
    from geometry import edge_id_from_lane, parse_lane_index
    from protocol import STATE_ABORT, STATE_CRUISE, STATE_MERGING, STATE_NEGOTIATING, STATE_YIELDING


log = logging.getLogger("obu")


class MergeRoleMixin:
    def _fsm_merge(self):
        e, dtm = self._merge_eta(), self._self_distance_to_merge()
        if e is None or dtm is None: return
        curt, lid_s = self._sim_time(), str(self.sensor_state.get("lane_id", "")); lidx, eid, cspd = parse_lane_index(lid_s), edge_id_from_lane(lid_s), self._current_speed() or 0.0
        if self._check_merge_finalized(): return
        if self.merge_completed and curt - self.merge_completed_since < self.post_merge_lock_s:
            es = max(self.cruise_speed, self.min_merge_entry_speed); self._set_target_speed(es, force=True); self.target_speed_mode, self.skip_car_following_this_step = self.priority_speed_mode, False; return
        if self.merge_committed and not self.merge_completed:
            ca, ceid = curt - self.merge_committed_since, edge_id_from_lane(lid_s)
            lid, hid = self.committed_lead_id, self.committed_host_id; le, he = self._neighbor_eta(lid) if lid else None, self._neighbor_eta(hid) if hid else None
            self.target_speed_mode, self.target_lane_index = self.priority_speed_mode, self.merge_lane_index
            if not self.merge_physical_started_once:
                if self._lane_change_executable_now():
                    self._start_physical_merge(curt, lid, hid, le, he, e, dtm, cspd, lidx, ceid)
                else:
                    self._set_state(STATE_NEGOTIATING)
                    self._log_merge_prepare_wait_lane_available(curt, dtm, cspd, hid, lid)
            else:
                self._set_state(STATE_MERGING)
            if self._lane_command_waiting_edge():
                self._log_timeline_event("WAIT_EDGE")
                self._log_merge_prepare_wait_lane_available(curt, dtm, cspd, hid, lid)
                if hid is None and not self.merge_physical_started_once:
                    floor = max(self.cruise_speed * self.merge_wait_edge_floor_ratio, self.min_speed)
                    rolling_target = max(floor, min(cspd, self.cruise_speed * 0.9))
                    self._set_target_speed(rolling_target, emergency=False)
                    self.target_lane_index, self.target_speed_mode = self.merge_lane_index, self.priority_speed_mode
                    if curt - self.last_wait_edge_floor_log >= 1.0:
                        lcs = self.lane_command_status or {}
                        log.info(
                            "MERGE_WAIT_EDGE_ROLLING_FLOOR: vehicle=%s waited_s=%.1f edge=%s lane=%s target_lane=%s "
                            "lane_count=%s speed=%.2f target=%.2f floor=%.2f dtm=%.1f lead=%s",
                            self.vehicle_id, ca, lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"),
                            lcs.get("lane_count"), cspd, self.target_speed or 0.0, floor, dtm, lid
                        )
                        self.last_wait_edge_floor_log = curt
                    if ca < self.merge_wait_edge_hostless_timeout_s:
                        return
            if self._lane_command_apply_active():
                lcs = self.lane_command_status or {}
                key = (lcs.get("state"), lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"), lcs.get("lane_count"))
                if key != self.last_commit_lane_apply_log_key:
                    self._log_timeline_event("APPLY")
                    self.last_commit_lane_apply_log_key = key
                    log.debug("[%.1f] %s MERGE_COMMIT_LANE_APPLY_ACTIVE: edge=%s lane=%s target_lane=%s lane_count=%s executable=%s speed=%.2f dtm=%.1f", curt, self.vehicle_id, lcs.get("edge_id"), lcs.get("current_lane"), lcs.get("target_lane"), lcs.get("lane_count"), lcs.get("executable"), cspd, dtm)
            self._apply_rear_guard_flow_protect(curt, hid, ceid, lidx, cspd, dtm)
            if ceid == "1331698336" and lidx != self.merge_lane_index: self.skip_car_following_this_step = False; rspd = max(self.min_merge_entry_speed, self.cruise_speed * 0.9); self._set_target_speed(rspd, force=True); return
            lgok, hgok, fgok = (e - le) >= self.safe_headway_s if le else True, (he - e) >= self.merge_commit_headway_s if he else True, self._final_merge_lane_clear(lid, hid, dtm)
            if not lgok or not hgok or not fgok: 
                lcs = self.lane_command_status or {}
                if self.merge_safety_hold_since <= 0.0: self.merge_safety_hold_since = curt
                if curt - self.merge_safety_hold_since > self.merge_safety_hold_timeout_s:
                    self._log_timeline_event("ABORT_SAFETY_HOLD")
                    log.debug("[%.1f] %s MERGE_COMMIT_ABORT_SAFETY_HOLD: lgok=%s hgok=%s fgok=%s edge=%s lane=%s target_lane=%s speed=%.2f lane_cmd_state=%s lane_cmd_executable=%s lane_cmd_edge=%s lane_cmd_lane_count=%s", curt, self.vehicle_id, lgok, hgok, fgok, ceid, lidx, self.target_lane_index, cspd, lcs.get("state"), lcs.get("executable"), lcs.get("edge_id"), lcs.get("lane_count")); self.merge_committed, self.merge_authorized, self.pending_request, self.merge_accepted, self.accepted_slot_invalid_since = False, False, None, False, 0.0; self._set_state(STATE_NEGOTIATING); return
                self._set_target_speed(max(self.min_merge_entry_speed * 0.5, self.min_speed), force=True); log.debug("[%.1f] %s MERGE_COMMIT_SAFETY_HOLD: lgok=%s hgok=%s fgok=%s edge=%s lane=%s target_lane=%s speed=%.2f lane_cmd_state=%s lane_cmd_executable=%s lane_cmd_edge=%s lane_cmd_lane_count=%s", curt, self.vehicle_id, lgok, hgok, fgok, ceid, lidx, self.target_lane_index, cspd, lcs.get("state"), lcs.get("executable"), lcs.get("edge_id"), lcs.get("lane_count")); return
            self.merge_safety_hold_since = 0.0; self._set_target_speed(max(self.min_merge_entry_speed, self.cruise_speed * 0.9)); self.skip_car_following_this_step = False
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
        if self.fsm_state == STATE_ABORT and curt - self.fsm_state_since < self.abort_cooldown_s:
            abort_floor = max(self.cruise_speed * 0.4, self.min_speed)
            force_abort_floor = (
                self.merge_lost_auth_after_point_floor_enabled
                and self.is_ramp_vehicle
                and self.past_merge_point
                and (eid == "1331698336" or eid in self.main_edge_ids)
                and lidx != self.merge_lane_index
                and cspd < 1.0
                and lgok
                and hgok
                and cok
            )
            if force_abort_floor:
                abort_floor = max(abort_floor, self.cruise_speed * self.merge_lost_auth_after_point_floor_ratio)
            self._set_target_speed(abort_floor, force=force_abort_floor)
            self.target_lane_index = None
            return
        elif self.fsm_state == STATE_ABORT:
            self._set_state(STATE_CRUISE)
            self.target_lane_index = None
        if ra == 3: return
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
        
        # Surgical patch: Lead-only or hostless merge after last main vehicle
        hlma_lead_only = False
        hlma_stalled_recovery = False
        if hid is None and not self.allow_hostless_merge and not self.merge_committed and not self.merge_completed:
            # We are after the last main vehicle (true_after_last_main) or there are no main neighbors at all.
            if sreas in ("true_after_last_main", "no_main_neighbors") and dtm <= self.merge_commit_distance_m:
                # All safety guards must be satisfied
                if lgok and lclear and fgok and cok and lgok_proj and esok:
                    hlma_lead_only = True
                elif (
                    self.merge_stalled_recovery_enabled
                    and lgok
                    and lclear
                    and fgok
                    and cok
                    and lgok_proj
                    and (not esok or self.past_merge_point)
                    and cspd < self.min_merge_entry_speed
                ):
                    hlma_lead_only = True
                    hlma_stalled_recovery = True

        has_rm = dtm <= self.priority_distance and (len(self._main_candidate_etas()) > 0 or self._has_any_main_neighbor_near_merge())
        hlma = (self.allow_hostless_merge and hid is None and self.pending_request is None and not has_rm) or hlma_lead_only
        amcm = hlma or ra == 2
        if hid is None and not self.allow_hostless_merge and not hlma_lead_only:
            amcm = False
            if self.merge_authorized: log.debug("[%.1f] %s MERGE_AUTH_CLEAR_HOSTLESS_DISABLED", curt, self.vehicle_id); self.merge_authorized, self.merge_accepted, self.accepted_slot_invalid_since = False, False, 0.0

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
                if hlma_lead_only:
                    if cspd < self.min_merge_entry_speed:
                        if not getattr(self, 'recovery_triggered_this_merge', False):
                            self.count_late_merge_recovery += 1
                            self.recovery_triggered_this_merge = True
                        self._set_target_speed(max(self.merge_stalled_recovery_speed, self.min_merge_entry_speed), force=True)
                        if hlma_stalled_recovery:
                            log.info(
                                "MERGE_STALLED_LEAD_ONLY_RECOVERY: vehicle=%s lead=%s dtm=%.1f speed=%.2f target=%.2f "
                                "fgok=%s lgok=%s lclear=%s source=%s",
                                self.vehicle_id, lid, dtm, cspd, self.target_speed or 0.0,
                                fgok, lgok, lclear, sreas
                            )
                        else:
                            log.info(
                                "MERGE_LEAD_ONLY_START_SPEED_FLOOR: vehicle=%s lead=%s dtm=%.1f speed=%.2f target=%.2f source=%s",
                                self.vehicle_id, lid, dtm, cspd, self.target_speed or 0.0, sreas
                            )
                    self._log_timeline_event("AUTHORIZED")
                    log.info("MERGE_AUTHORIZED_LEAD_ONLY_AFTER_LAST_MAIN: vehicle=%s lead=%s dtm=%.1f lead_gap=%s lead_gap_t1=%s lead_gap_t2=%s source=%s",
                             self.vehicle_id, lid, dtm, f"{lg_v:.2f}" if lg_v is not None else "None",
                             f"{lg1_v:.2f}" if lg1_v is not None else "None", f"{lg2_v:.2f}" if lg2_v is not None else "None", sreas)
                else:
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
            if not self.merge_committed:
                self.merge_committed, self.merge_committed_since = True, curt
                self.committed_lead_id, self.committed_host_id = lid, hid
                self.committed_manoeuvre_id = self.pending_request.get("manoeuvre_id") if self.pending_request else None
                self.last_commit_lane_apply_log_key = None
            merge_start_target = max(self.min_merge_entry_speed, self.cruise_speed * 0.9)
            force_merge_start_target = cspd < self.min_merge_entry_speed or (self.target_speed is not None and self.target_speed < self.min_merge_entry_speed)
            self._set_target_speed(merge_start_target, force=force_merge_start_target); self.target_lane_index, self.target_speed_mode = self.merge_lane_index, self.priority_speed_mode
            if self._lane_change_executable_now():
                if not self.merge_physical_started_once:
                    self._start_physical_merge(curt, lid, hid, le, he, e, dtm, cspd, lidx, eid)
                else:
                    self._start_physical_merge(curt, lid, hid, le, he, e, dtm, cspd, lidx, eid, resume=True)
            else:
                self._set_state(STATE_NEGOTIATING)
                self._log_merge_prepare_wait_lane_available(curt, dtm, cspd, hid, lid)
        elif self.merge_authorized and not self.merge_committed: self._set_state(STATE_NEGOTIATING); self.target_lane_index = None; log.debug("[%.1f] %s MERGE_PREPARE_WAIT_PHYSICAL: dtm=%.1f", curt, self.vehicle_id, dtm)
        elif not self.merge_committed: self._set_state(STATE_NEGOTIATING if hid else STATE_CRUISE); self.target_lane_index = None
        stalled_after_point_floor = (
            self.merge_lost_auth_after_point_floor_enabled
            and self.is_ramp_vehicle
            and self.past_merge_point
            and (eid == "1331698336" or eid in self.main_edge_ids)
            and lidx != self.merge_lane_index
            and cspd < 1.0
        )
        if stalled_after_point_floor and not self.merge_committed and not self.merge_completed:
            floor = max(self.cruise_speed * self.merge_lost_auth_after_point_floor_ratio, self.min_speed)
            self._set_state(STATE_NEGOTIATING)
            self.target_lane_index = None
            self.target_speed_mode = self.default_speed_mode
            self._set_target_speed(floor, emergency=False, force=True)
            return
        if not self.merge_committed and dtm <= self.cruise_speed * self.safe_headway_s and not (self.merge_authorized and physical_zone):
            self._set_state(STATE_YIELDING); stopd = max(dtm - self.merge_stop_margin_m, 0.0); slsp = stopd / max(self.merge_blocked_approach_s, 0.1)
            if dtm > self.merge_stop_margin_m + 6.0: slsp = max(slsp, self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed)
            else: slsp = max(slsp, self.emergency_min_speed)
            self._set_target_speed(slsp, emergency=dtm <= self.merge_stop_margin_m + 8.0); return
        if self.is_ramp_vehicle and self.past_merge_point and not self.merge_completed:
            if not self.merge_committed and not self.merge_authorized:
                late_lane_recovery = (
                    self.merge_stalled_recovery_enabled
                    and (eid == "1331698336" or eid in self.main_edge_ids)
                    and lidx != self.merge_lane_index
                    and hid is None
                    and (lid is not None or sreas == "no_main_neighbors")
                    and sreas in ("true_after_last_main", "no_main_neighbors")
                    and fgok
                    and cok
                    and lclear
                    and lgok_proj
                )
                if late_lane_recovery:
                    if not getattr(self, 'recovery_triggered_this_merge', False):
                        self.count_late_merge_recovery += 1
                        self.recovery_triggered_this_merge = True
                    self.merge_authorized, self.merge_authorized_since = True, curt
                    self.merge_committed, self.merge_committed_since = True, curt
                    self.committed_lead_id, self.committed_host_id = lid, None
                    self.committed_manoeuvre_id = None
                    self.target_lane_index, self.target_speed_mode = self.merge_lane_index, self.priority_speed_mode
                    self._set_target_speed(max(self.merge_stalled_recovery_speed, self.min_merge_entry_speed), force=True)
                    log.info(
                        "MERGE_LATE_LANE_RECOVERY_AFTER_POINT: vehicle=%s lead=%s edge=%s lane=%s target_lane=%s "
                        "dtm=%.1f speed=%.2f target=%.2f source=%s",
                        self.vehicle_id, lid, eid, lidx, self.merge_lane_index, dtm, cspd, self.target_speed or 0.0, sreas
                    )
                    if self._lane_change_executable_now():
                        self._start_physical_merge(curt, lid, None, le, None, e, dtm, cspd, lidx, eid)
                    else:
                        self._set_state(STATE_NEGOTIATING)
                        self._log_merge_prepare_wait_lane_available(curt, dtm, cspd, None, lid)
                    return
                lost_auth_roll = (
                    self.merge_lost_auth_after_point_floor_enabled
                    and hid is None
                    and sreas == "no_main_neighbors"
                    and (eid == "1331698336" or eid in self.main_edge_ids)
                    and lidx != self.merge_lane_index
                    and not fgok
                    and lgok
                    and cok
                    and lclear
                    and lgok_proj
                )
                if lost_auth_roll:
                    floor = max(self.cruise_speed * self.merge_lost_auth_after_point_floor_ratio, self.min_speed)
                    rolling_target = max(floor, min(cspd, self.cruise_speed * 0.9))
                    force_floor = cspd < 1.0 or (self.target_speed is not None and self.target_speed < floor)
                    self._set_state(STATE_NEGOTIATING)
                    self.target_lane_index = None
                    self.target_speed_mode = self.default_speed_mode
                    self._set_target_speed(rolling_target, emergency=False, force=force_floor)
                    if curt - self.last_lost_auth_after_point_floor_log >= 1.0:
                        log.info(
                            "MERGE_LOST_AUTH_AFTER_POINT_ROLLING_FLOOR: vehicle=%s edge=%s lane=%s target_lane=%s "
                            "dtm=%.1f speed=%.2f target=%.2f floor=%.2f forced=%s fgok=%s source=%s neighbors=%d",
                            self.vehicle_id, eid, lidx, self.merge_lane_index, dtm, cspd, self.target_speed or 0.0,
                            floor, force_floor, fgok, sreas, len(self.neighbors)
                        )
                        self.last_lost_auth_after_point_floor_log = curt
                    return
                self._log_slot_quality_diag("LOST_AUTH_AFTER_POINT", lid, hid, le, he, e, dtm, sreas, self.pending_request.get("manoeuvre_id") if self.pending_request else None)
                log.debug("[%.1f] %s MERGE_FAILED_LOST_AUTH_AFTER_POINT: dtm=%.1f past=True hid=%s auth=False", curt, self.vehicle_id, dtm, hid)
                abort_target = self.min_speed
                abort_floor = max(self.cruise_speed * self.merge_lost_auth_after_point_floor_ratio, self.min_speed)
                force_abort_floor = (
                    self.merge_lost_auth_after_point_floor_enabled
                    and hid is None
                    and (eid == "1331698336" or eid in self.main_edge_ids)
                    and lidx != self.merge_lane_index
                    and (cspd < 1.0 or (self.target_speed is not None and self.target_speed < abort_floor))
                )
                if force_abort_floor:
                    abort_target = abort_floor
                    log.info(
                        "MERGE_LOST_AUTH_AFTER_POINT_ABORT_FLOOR: vehicle=%s edge=%s lane=%s target_lane=%s "
                        "dtm=%.1f speed=%.2f target=%.2f floor=%.2f source=%s",
                        self.vehicle_id, eid, lidx, self.merge_lane_index, dtm, cspd, abort_target, abort_floor, sreas
                    )
                self._set_state(STATE_ABORT); self._set_target_speed(abort_target, force=True); self.target_lane_index = None; return
            lp = float(self.sensor_state.get("lane_pos", 0.0)); rem = 63.23 - lp
            if rem < 10.0:
                if not getattr(self, 'recovery_triggered_this_merge', False): self.count_late_merge_recovery += 1; self.recovery_triggered_this_merge = True
                self._set_target_speed(max(self.min_speed, rem / 2.0), force=True); self.target_lane_index = self.merge_lane_index; self.target_speed_mode, self.skip_car_following_this_step = self.priority_speed_mode, False
            elif rem < 2.0: self.count_merge_failed_no_gap += 1; self._set_target_speed(0.0, force=True); self._set_state(STATE_ABORT); self.target_lane_index = None

