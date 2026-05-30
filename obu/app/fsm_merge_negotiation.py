import logging

try:
    from .protocol import STATE_NEGOTIATING
except ImportError:
    from protocol import STATE_NEGOTIATING


log = logging.getLogger("obu")


class MergeNegotiationMixin:
    def _negotiate_merge_slot(self, hid, lid=None, le=None, he=None):
        if hid is None:
            return None

        if self.pending_request is not None:
            pending_hid = int(self.pending_request["host_id"])
            if pending_hid != hid:
                hid = pending_hid

        if self.pending_request is None or int(self.pending_request.get("host_id", 0)) != hid:
            if self._sim_time() < self.rejected_hosts_until.get(hid, 0.0):
                return None
            
            self.mcm_messages.pop(hid, None)
            mid = self._next_manoeuvre_id()
            ht = None

            if hid in self.neighbors:
                host = self.neighbors[hid]
                dist = self._distance_to_merge(float(host["x"]), float(host["y"]))
                own_eta = self._merge_eta()
                if own_eta:
                    ht = dist / max(
                        own_eta + self.safe_headway_s + self.merge_occupancy_s,
                        0.1,
                    )

            self.pending_request = {
                "host_id": hid,
                "host_eta": he,
                "host_target_speed": ht,
                "lead_id": lid,
                "lead_eta": le,
                "manoeuvre_id": mid,
                "timestamp": self._sim_time(),
                "retry_count": 0,
            }
            self.accepted_slot_invalid_since = 0.0
            self._log_slot_quality_diag("REQUEST_NEW", lid, hid, le, he, self._merge_eta(), self._self_distance_to_merge(), "new_request", mid)
            self._send_mcm(1, mid, target_station_id=hid)
            self._set_state(STATE_NEGOTIATING)

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
                "host_remote_merge_committed=%s host_remote_merge_completed=%s host_remote_active_merge_request=%s "
                "host_remote_active_station_id=%s host_remote_active_manoeuvre_id=%s host_remote_active_remaining_s=%s",
                self.vehicle_id, hid, self.pending_request.get("lead_id"), self.pending_request["manoeuvre_id"],
                self.pending_request["retry_count"], curt - float(self.pending_request["timestamp"]), dtm,
                e if e else 0.0, self._neighbor_eta(hid) or "None", self.pending_request.get("host_eta"),
                self._neighbor_eta(self.pending_request.get("lead_id")) or "None", self.pending_request.get("lead_eta"),
                self.mcm_messages.get(hid, {}).get("action", "None"), 
                curt - float(self.mcm_messages.get(hid, {}).get("timestamp", curt)) if hid in self.mcm_messages else "None",
                rst.get("fsm_state", "NONE"), rst.get("edge_id", ""), rst.get("lane_index", ""),
                rst.get("merge_committed", False), rst.get("merge_completed", False),
                rst.get("active_merge_request", False), rst.get("active_merge_request_station_id", "None"),
                rst.get("active_merge_request_manoeuvre_id", "None"), rst.get("active_merge_request_remaining_s", 0.0)
            )
            self._log_slot_quality_diag("REQUEST_RETRY", self.pending_request.get("lead_id"), hid, self.pending_request.get("lead_eta"), self._neighbor_eta(hid) or self.pending_request.get("host_eta"), self._merge_eta(), self._self_distance_to_merge(), "retry_request", self.pending_request["manoeuvre_id"])
            self._send_mcm(1, self.pending_request["manoeuvre_id"], target_station_id=hid)

        resp = self.mcm_messages.get(hid)
        ra = 2 if self.pending_request.get("accepted_at") else None

        if self.pending_request:
            for sid, data in list(self.mcm_messages.items()):
                if int(sid) == hid or data.get("action") != 2:
                    continue
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
                elif ra == 3:
                    log.debug(
                        "[%.1f] %s MCM_REJECT: host=%d manoeuvre=%d",
                        self._sim_time(),
                        self.vehicle_id,
                        hid,
                        self.pending_request["manoeuvre_id"],
                    )
                    self.rejected_hosts_until[hid] = self._sim_time() + 1.5
                    self.pending_request = None
                    self.merge_authorized = False
                    self.merge_accepted = False
                    self.accepted_slot_invalid_since = 0.0
                    self._set_state(STATE_NEGOTIATING)
                    self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed))
                    return 3
            elif not fresh:
                if resp.get("action") == 2:
                    log.debug(
                        "[%.1f] %s MCM_ACCEPT_STALE: host=%d manoeuvre=%s",
                        self._sim_time(),
                        self.vehicle_id,
                        hid,
                        resp.get("manoeuvre_id"),
                    )
                self.mcm_messages.pop(hid, None)
            else:
                if not mid_match:
                    log.debug(
                        "[%.1f] %s MCM_ACCEPT_WRONG_MANOEUVRE: host=%d got=%s expected=%s",
                        self._sim_time(),
                        self.vehicle_id,
                        hid,
                        resp.get("manoeuvre_id"),
                        self.pending_request["manoeuvre_id"],
                    )
                if not target_match:
                    log.debug(
                        "[%.1f] %s MCM_ACCEPT_WRONG_TARGET: host=%d target=%s self=%d",
                        self._sim_time(),
                        self.vehicle_id,
                        hid,
                        resp.get("target_station_id"),
                        self.station_id,
                    )

        if self.pending_request:
            p_time = float(self.pending_request["accepted_at" if self.pending_request.get("accepted_at") else "timestamp"])
            age, tout = self._sim_time() - p_time, self.merge_accept_timeout_s if self.pending_request.get("accepted_at") else self.negotiation_timeout_s
            if age > tout:
                oh = self.pending_request.get("host_id")
                self.pending_request = None
                self.merge_authorized = False
                self.merge_accepted = False
                self.accepted_slot_invalid_since = 0.0
                self.mcm_retry_blocked_until = self._sim_time() + self.mcm_timeout_cooldown_s
                if oh:
                    self.mcm_messages.pop(int(oh), None)
                log.debug("[%.1f] %s MCM_TIMEOUT: host=%s giving up", self._sim_time(), self.vehicle_id, oh)
                self._set_state(STATE_NEGOTIATING)
                self._set_target_speed(max(self.cruise_speed * self.merge_yield_floor_ratio, self.min_speed))
                return 3

        return ra
