import logging

try:
    from .protocol import STATE_CRUISE, STATE_MERGING, STATE_YIELDING
except ImportError:
    from protocol import STATE_CRUISE, STATE_MERGING, STATE_YIELDING


log = logging.getLogger("obu")


class HostFsmMixin:
    def _latest_request(self):
        now = self._sim_time()
        candidates = []

        for sid, data in self.mcm_messages.items():
            if data.get("action") != 1:
                continue
            if now - data.get("timestamp", 0) > self.neighbor_timeout_s:
                continue

            eta = self._neighbor_eta(sid)
            if eta is not None:
                candidates.append((eta, sid, data))

        if not candidates:
            return None

        eta, sid, data = sorted(candidates)[0]
        request = data.copy()
        request["station_id"] = sid
        request["eta"] = eta
        return request

    def _fsm_host(self):
        now = self._sim_time()
        request = self._latest_request()

        if self.active_merge_request:
            active_station_id = int(self.active_merge_request["station_id"])
            active_manoeuvre_id = int(self.active_merge_request["manoeuvre_id"])
            active_target_speed = float(
                self.active_merge_request.get(
                    "target_speed",
                    self.cruise_speed * self.host_yield_floor_ratio,
                )
            )
            
            # Sync manoeuvre_id if requester updated it
            latest_msg = self.mcm_messages.get(active_station_id)
            if latest_msg and latest_msg.get("action") == 1:
                latest_mid = int(latest_msg.get("manoeuvre_id", 0))
                if latest_mid > active_manoeuvre_id:
                    log.info(
                        "HOST_MANOEUVRE_ID_UPDATE: host=%s ramp=%d old_mid=%d new_mid=%d",
                        self.vehicle_id,
                        active_station_id,
                        active_manoeuvre_id,
                        latest_mid,
                    )
                    self.active_merge_request["manoeuvre_id"] = latest_mid
                    active_manoeuvre_id = latest_mid
            
            rst = self.remote_vehicle_status.get(active_station_id, {})
            nlcs = rst.get("lane_command_state", "NONE")
            ne = rst.get("edge_id", "")
            nla = str(rst.get("lane_index", ""))
            rcmpl = rst.get("merge_completed", False)
            rcmt = rst.get("merge_committed", False)
            rfsm = rst.get("fsm_state", "")
            osp = self._current_speed() or 0.0

            time_since_start = now - getattr(self, 'active_merge_request_started_at', 0.0)
            max_s = getattr(self, "host_reservation_max_s", 10.0)

            if rcmpl or (nlcs == "CLEAR" and not (rcmt and not rcmpl and rfsm == STATE_MERGING)):
                log.info("HOST_RESERVATION_RELEASE_AFTER_CLEAR: host=%s ramp=%d manoeuvre=%d reason=completed ramp_fsm=%s ramp_lcs=%s ramp_edge=%s ramp_merge_completed=%s", self.vehicle_id, active_station_id, active_manoeuvre_id, rst.get("fsm_state", ""), nlcs, ne, rcmpl)
                self.active_merge_request, self.active_merge_request_until = None, 0.0
            elif time_since_start > max_s:
                log.info("HOST_RESERVATION_RELEASE_MAX_TIMEOUT: host=%s ramp=%d manoeuvre=%d duration=%.1f ramp_fsm=%s ramp_lcs=%s ramp_edge=%s ramp_merge_completed=%s", self.vehicle_id, active_station_id, active_manoeuvre_id, time_since_start, rst.get("fsm_state", ""), nlcs, ne, rcmpl)
                self.active_merge_request, self.active_merge_request_until = None, 0.0
            elif nlcs in ("WAIT_EDGE", "APPLY") or rfsm == STATE_MERGING or (rcmt and not rcmpl):
                new_until = min(
                    getattr(self, "active_merge_request_started_at", now) + max_s,
                    now + self.host_reservation_s,
                )
                if new_until > self.active_merge_request_until:
                    self.active_merge_request_until = new_until
                    log.info("HOST_RESERVATION_EXTEND_UNTIL_CLEAR: host=%s ramp=%d manoeuvre=%d new_remaining=%.1f ramp_lcs=%s ramp_edge=%s", self.vehicle_id, active_station_id, active_manoeuvre_id, self.active_merge_request_until - now, nlcs, ne)
            elif now >= self.active_merge_request_until:
                log.info("HOST_RELEASE_BEFORE_CLEAR: host=%s ramp=%d manoeuvre=%d reason=timeout ramp_fsm=%s ramp_lane_cmd_state=%s ramp_edge=%s ramp_lane=%s ramp_merge_completed=%s ramp_merge_committed=%s", 
                         self.vehicle_id, active_station_id, active_manoeuvre_id, rfsm, nlcs, ne, nla, rcmpl, rcmt)
                self.active_merge_request, self.active_merge_request_until = None, 0.0

            if self.active_merge_request:
                log.info("HOST_RESERVATION_HOLD: host=%s ramp=%d manoeuvre=%d remaining=%.1f own_speed=%.2f target_speed=%.2f ramp_fsm=%s ramp_lane_cmd_state=%s ramp_edge=%s ramp_lane=%s ramp_merge_completed=%s ramp_merge_committed=%s", 
                         self.vehicle_id, active_station_id, active_manoeuvre_id, self.active_merge_request_until - now, osp, active_target_speed, rfsm, nlcs, ne, nla, rcmpl, rcmt)
                log.debug("[%.1f] %s HOST_HOLD_YIELD: for merge=%d spd=%.2f", now, self.vehicle_id, active_station_id, active_target_speed)
                
                if request and int(request["station_id"]) != active_station_id:
                    osid, omid = int(request["station_id"]), int(request.get("manoeuvre_id") or 0)
                    if now - self.last_mcm_response.get(osid, 0) >= self.response_period_s:
                        self._log_host_decision(osid, omid, "REJECT", "BUSY")
                        self._send_mcm(3, omid, target_station_id=osid)
                        self.last_mcm_response[osid] = now
                    else:
                        self._log_host_decision(osid, omid, "IGNORE", "BUSY_PERIOD")
                self._set_state(STATE_YIELDING)
                self._set_target_speed(active_target_speed, force=True)
                if now - self.last_mcm_response.get(active_station_id, 0) >= self.response_period_s:
                    self._send_mcm(2, active_manoeuvre_id, target_station_id=active_station_id)
                    self.last_mcm_response[active_station_id] = now
                return

        if not request:
            self._set_state(STATE_CRUISE)
            return

        rsid = int(request["station_id"])
        rmid = int(request.get("manoeuvre_id") or 0)
        me = self._neighbor_eta(rsid)
        d = self._self_distance_to_merge()
        oe = self._merge_eta()
        if me is None or d is None or oe is None:
            self._set_state(STATE_CRUISE)
            return

        if d <= self.host_reject_distance_m:
            if now - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
                self._log_host_decision(rsid, rmid, "REJECT", "DISTANCE")
                self._send_mcm(3, rmid, target_station_id=rsid)
                self.last_mcm_response[rsid] = now
            else:
                self._log_host_decision(rsid, rmid, "IGNORE", "DISTANCE_PERIOD")
            self._set_state(STATE_CRUISE)
            return

        te = me + self.safe_headway_s + self.merge_occupancy_s
        rqs = d / max(te, 0.1)
        cs = self._current_speed() or self.cruise_speed
        sf = max(self.cruise_speed * self.host_yield_floor_ratio, self.min_speed)
        rqs = max(min(rqs, cs), sf)

        gd = oe - me
        asafe = gd >= self.merge_commit_headway_s
        dy = rqs < cs - 0.15
        ns = gd >= self.merge_commit_headway_s * 0.75
        sa = rqs >= cs - 0.30
        if dy:
            if gd < self.host_min_accept_gap_s:
                self._set_state(STATE_CRUISE)
                if now - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
                    self._log_host_decision(rsid, rmid, "REJECT", "LATE_GAP")
                    self._send_mcm(3, rmid, target_station_id=rsid)
                    self.last_mcm_response[rsid] = now
                    log.info("HOST_LATE_GAP_REJECT: host=%s ramp=%d manoeuvre=%d gap_eta=%.2f min_gap_eta=%.2f own_speed=%.2f required_speed=%.2f", self.vehicle_id, rsid, rmid, gd, self.host_min_accept_gap_s, cs, rqs)
                else:
                    self._log_host_decision(rsid, rmid, "IGNORE", "LATE_GAP_PERIOD")
                return

            yt = max(min(rqs, cs - self.host_min_yield_delta), sf)
            if now - self.last_mcm_response.get(rsid, 0) < self.response_period_s:
                self._log_host_decision(rsid, rmid, "IGNORE", "YIELDING_PERIOD")
                self._set_state(STATE_CRUISE)
                return

            self._log_host_decision(rsid, rmid, "ACCEPT", "YIELDING")
            self._send_mcm(2, rmid, target_station_id=rsid)
            self.last_mcm_response[rsid] = now
            self.active_merge_request = {
                "station_id": rsid,
                "manoeuvre_id": rmid,
                "target_speed": yt,
                "target_eta": te,
            }
            self.active_merge_request_until = now + self.host_reservation_s
            self.active_merge_request_started_at = now
            self._set_state(STATE_YIELDING)
            self._set_target_speed(yt, force=True)
            log.info("HOST_RESERVATION_START: host=%s ramp=%d manoeuvre=%d until=%.1f own_speed=%.2f target_speed=%.2f required_speed=%.2f gap_eta=%.2f", self.vehicle_id, rsid, rmid, self.active_merge_request_until, cs, yt, rqs, oe - me)
            log.debug("[%.1f] %s HOST_RESERVED: merge=%d manoeuvre=%d until=%.1f target_spd=%.2f", now, self.vehicle_id, rsid, rmid, self.active_merge_request_until, yt)
            log.debug("[%.1f] %s HOST_YIELD: for merge=%d req_spd=%.2f", now, self.vehicle_id, rsid, yt)
            return

        if asafe or (ns and sa):
            hs = min(cs, max(rqs, self.cruise_speed * self.host_yield_floor_ratio))
            if not asafe:
                hs = max(min(hs, cs - self.host_min_yield_delta), sf)

            if now - self.last_mcm_response.get(rsid, 0) < self.response_period_s:
                self._log_host_decision(rsid, rmid, "IGNORE", "NOMINAL_PERIOD")
                self._set_state(STATE_CRUISE)
                return

            self._log_host_decision(rsid, rmid, "ACCEPT", "SAFE_OR_NOMINAL")
            self._send_mcm(2, rmid, target_station_id=rsid)
            self.last_mcm_response[rsid] = now
            self.active_merge_request = {
                "station_id": rsid,
                "manoeuvre_id": rmid,
                "target_speed": hs,
                "target_eta": te,
            }
            self.active_merge_request_until = now + self.host_reservation_s
            self.active_merge_request_started_at = now
            self._set_state(STATE_CRUISE if asafe else STATE_YIELDING)
            if not asafe:
                self._set_target_speed(hs, force=True)
            log.info("HOST_RESERVATION_START: host=%s ramp=%d manoeuvre=%d until=%.1f own_speed=%.2f target_speed=%.2f required_speed=%.2f gap_eta=%.2f", self.vehicle_id, rsid, rmid, self.active_merge_request_until, cs, hs, rqs, oe - me)
            log.debug("[%.1f] %s HOST_RESERVED: merge=%d manoeuvre=%d until=%.1f target_spd=%.2f", now, self.vehicle_id, rsid, rmid, self.active_merge_request_until, hs)
            return

        self._set_state(STATE_CRUISE)
        if now - self.last_mcm_response.get(rsid, 0) >= self.response_period_s:
            self._log_host_decision(rsid, rmid, "REJECT", "UNSAFE_GAP")
            self._send_mcm(3, rmid, target_station_id=rsid)
            self.last_mcm_response[rsid] = now
        else:
            self._log_host_decision(rsid, rmid, "IGNORE", "UNSAFE_PERIOD")

    def _fsm_lead(self):
        if self._has_active_host_reservation():
            self._fsm_host()
            return

        mid = self._merge_candidate_id()
        if not mid:
            self._set_state(STATE_CRUISE)
            return

        me = self._neighbor_eta(mid)
        if not me:
            self._set_state(STATE_CRUISE)
            return

        md = self._distance_to_merge(self.neighbors[mid]["x"], self.neighbors[mid]["y"])
        if md <= self.priority_distance:
            cl = max(self.cruise_speed + self.lead_speed_bonus, self._current_speed() or self.cruise_speed)
            self._set_state(STATE_CRUISE)
            self._set_target_speed(cl, force=True)
            self.target_speed_mode = self.priority_speed_mode
        else:
            self._set_state(STATE_CRUISE)
