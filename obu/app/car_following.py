import logging
import math

try:
    from .geometry import edge_id_from_lane, parse_lane_index
    from .protocol import STATE_CRUISE, STATE_YIELDING
except ImportError:
    from geometry import edge_id_from_lane, parse_lane_index
    from protocol import STATE_CRUISE, STATE_YIELDING


log = logging.getLogger("obu")


class CarFollowingMixin:
    def _apply_car_following(self):
        if self.skip_car_following_this_step:
            self.skip_car_following_this_step = False
            return

        self.following_active = False
        self.following_station_id = None
        self.following_gap_m = None
        self.following_reason = ""

        if not self.enable_cam_following or not self.sensor_state:
            return

        post_merge_window = (
            self.merge_completed
            and self._sim_time() - self.merge_completed_since < self.post_merge_lock_s
        )
        own_heading = self._current_heading()
        if own_heading is None:
            return

        ox = float(self.sensor_state["x"])
        oy = float(self.sensor_state["y"])
        own_speed = self._current_speed() or 0.0
        own_dtm = self._self_distance_to_merge() or 0.0

        rad = math.radians(90 - own_heading)
        fx = math.cos(rad)
        fy = math.sin(rad)

        follow_speed = None
        follow_station_id = None
        follow_gap = None
        follow_reason = ""
        emergency = False

        for sid, data in self.neighbors.items():
            nx = float(data.get("x", 0.0))
            ny = float(data.get("y", 0.0))
            neighbor_speed = float(data.get("speed", 0.0))
            dx = nx - ox
            dy = ny - oy
            lon = dx * fx + dy * fy
            lat = abs(-dx * fy + dy * fx)
            gap = None
            reason = ""

            if 0.0 < lon <= self.cam_follow_lookahead and lat <= self.cam_follow_lateral_tolerance:
                gap = max(0.0, lon - self.vehicle_length)
                reason = "same_lane_cam"
                is_ramp_or_merge_candidate = (
                    (sid in self.ramp_station_ids or self._neighbor_is_merge_candidate(sid))
                    and not self.merge_completed
                )
                if is_ramp_or_merge_candidate:
                    if post_merge_window:
                        continue
                    if gap < 4.0:
                        sf = max(self.cruise_speed * 0.4, self.min_speed)
                        if own_speed < 1.0:
                            follow_speed = max(follow_speed or 0, sf)

                should_ignore_loose_main_leader = (
                    not (self._self_is_on_ramp() or is_ramp_or_merge_candidate)
                    and self.effective_role in ("lead", "host", "cruise")
                    and gap > self.cam_follow_critical_gap
                )
                if should_ignore_loose_main_leader:
                    continue
            elif (
                self.effective_role == "merge"
                or (self.is_ramp_vehicle and not self.merge_completed)
            ) and own_dtm <= self.merge_conflict_follow_distance_m:
                if not self._neighbor_is_main_candidate(sid):
                    continue

                ne, oe = self._neighbor_eta(sid), self._merge_eta()
                if ne and oe and abs(ne - oe) <= self.safe_headway_s * 1.2:
                    de = ne + self.safe_headway_s
                    if oe < de:
                        sf = max(self.cruise_speed * self.merge_conflict_floor_ratio, self.min_speed)
                        candidate_speed = max(own_dtm / max(de, 0.1), sf)
                        if follow_speed is None or candidate_speed < follow_speed:
                            follow_speed = candidate_speed
                            follow_station_id = sid
                            follow_gap = abs(oe - ne)
                            follow_reason = "merge_conflict_eta"
                            emergency = False
                continue

            if gap is None:
                continue

            closing_speed = max(own_speed - neighbor_speed, 0.0)
            brake_decel = max(self.cam_follow_brake_decel, 0.1)
            safe_gap = (
                self.cam_follow_min_gap
                + own_speed * self.cam_follow_headway_s
                + closing_speed * closing_speed / (2.0 * brake_decel)
            )
            if gap < safe_gap:
                available_gap = max(gap - self.cam_follow_min_gap, 0.0)
                headway_speed = max(available_gap / max(self.cam_follow_headway_s, 0.1), 0.0)
                brake_speed = math.sqrt(
                    max(0.0, neighbor_speed * neighbor_speed + 2.0 * brake_decel * available_gap)
                )
                speed_floor = max(self.cruise_speed * 0.35, self.min_speed)
                if self.merge_completed and gap < 3.0:
                    speed_floor = max(self.cruise_speed * 0.45, self.min_speed)

                candidate_speed = min(
                    max(neighbor_speed - self.cam_follow_speed_delta, speed_floor),
                    max(headway_speed, speed_floor),
                    max(brake_speed, speed_floor),
                )
                if gap < self.cam_follow_critical_gap:
                    candidate_speed = max(
                        min(candidate_speed, own_speed * 0.65),
                        max(self.cruise_speed * 0.30, self.emergency_min_speed),
                    )

                candidate_emergency = reason == "same_lane_cam" and (
                    (gap < self.cam_follow_critical_gap * 0.75 and closing_speed > 1.4)
                    or (gap < self.cam_follow_min_gap * 0.55 and closing_speed > 1.5)
                )
                if follow_speed is None or candidate_speed < follow_speed:
                    follow_speed = candidate_speed
                    follow_station_id = sid
                    follow_gap = gap
                    follow_reason = reason
                    emergency = candidate_emergency

        if follow_speed is not None:
            now = self._sim_time()
            self_lid = str(self.sensor_state.get("lane_id", ""))
            self_edge, self_lane = edge_id_from_lane(self_lid), parse_lane_index(self_lid)
            lcs = self.lane_command_status or {}
            fd = self.neighbors.get(follow_station_id, {}) if follow_station_id is not None else {}
            leader_lid = str(fd.get("lane_id", ""))
            leader_edge, leader_lane = edge_id_from_lane(leader_lid), parse_lane_index(leader_lid)
            if follow_station_id in self.ramp_station_ids:
                leader_role_hint = "ramp"
            elif follow_station_id in self.main_station_ids:
                leader_role_hint = "main"
            elif follow_station_id is None:
                leader_role_hint = "unknown"
            else:
                leader_role_hint = (
                    "merge_candidate"
                    if self._neighbor_is_merge_candidate(follow_station_id)
                    else "unknown"
                )

            merge_completed_age = now - self.merge_completed_since if self.merge_completed else -1.0
            target_before = self.target_speed
            self.following_active = True
            self.following_station_id = follow_station_id
            self.following_gap_m = follow_gap
            self.following_reason = follow_reason
            if self.fsm_state == STATE_CRUISE:
                self._set_state(STATE_YIELDING)

            if own_speed < 1.0:
                follow_speed = max(follow_speed or 0.0, own_speed + 0.3)

            raw_follow_speed = follow_speed

            post_merge_lock_forced = False
            post_clear_guard_flow = (
                self.merge_committed
                and not self.merge_completed
                and self.post_clear_rear_guard_started_at > 0.0
                and follow_reason == "same_lane_cam"
            )
            apply_rear_guard_flow = (
                self.merge_committed
                and not self.merge_completed
                and follow_reason == "same_lane_cam"
                and lcs.get("state") in ("APPLY", "CLEAR")
                and self._target_lane_rear_hazard(self.committed_host_id) is not None
            )
            if post_merge_window and follow_reason == "same_lane_cam":
                lock_min_speed = self.cruise_speed * 0.8
                if follow_speed < lock_min_speed:
                    log.info("POST_MERGE_LOCK_ACTIVE: vehicle=%s protected speed from %.2f to %.2f (flow)", self.vehicle_id, follow_speed, lock_min_speed)
                    follow_speed = lock_min_speed
                    emergency = False
                    post_merge_lock_forced = True
            elif post_clear_guard_flow:
                lock_min_speed = max(self.cruise_speed * self.apply_rear_flow_floor_ratio, self.min_merge_entry_speed)
                if follow_speed < lock_min_speed:
                    log.info("POST_CLEAR_REAR_GUARD_FLOW_PROTECT: vehicle=%s protected speed from %.2f to %.2f (flow)", self.vehicle_id, follow_speed, lock_min_speed)
                    follow_speed = lock_min_speed
                    emergency = False
                    post_merge_lock_forced = True
            elif apply_rear_guard_flow:
                lock_min_speed = max(self.cruise_speed * self.apply_rear_flow_floor_ratio, self.min_merge_entry_speed)
                if follow_speed < lock_min_speed:
                    log.info("MERGE_APPLY_REAR_GUARD_FLOW_PROTECT: vehicle=%s protected car-follow speed from %.2f to %.2f (flow)", self.vehicle_id, follow_speed, lock_min_speed)
                    follow_speed = lock_min_speed
                    emergency = False
                    post_merge_lock_forced = True

            self._set_target_speed(follow_speed, emergency=emergency, force=post_merge_lock_forced)
            log.debug(
                "[%.1f] %s CAR_FOLLOW: sid=%d edge=%s lane=%s leader_edge=%s leader_lane=%s leader_role_hint=%s "
                "self_merge_completed=%s self_merge_completed_age=%.1f self_merge_committed=%s lane_cmd_state=%s "
                "lane_cmd_edge=%s lane_cmd_target=%s lane_cmd_executable=%s same_lane=%s reason=%s gap=%.1f "
                "follow_spd=%.2f emergency=%s target_before=%s target_after=%s target_after_raw=%.2f",
                now, self.vehicle_id, follow_station_id or 0, self_edge, self_lane, leader_edge, leader_lane, leader_role_hint,
                self.merge_completed, merge_completed_age, self.merge_committed, lcs.get("state"),
                lcs.get("edge_id"), lcs.get("target_lane"), lcs.get("executable"), follow_reason == "same_lane_cam",
                follow_reason, follow_gap or 0, follow_speed, emergency, target_before, self.target_speed, raw_follow_speed or 0.0
            )
            if self.merge_completed:
                self._log_timeline_event("POST_MERGE_CAR_FOLLOW")
                log.debug(
                    "[%.1f] %s POST_MERGE_CAR_FOLLOW: age=%.1f sid=%d gap=%.1f reason=%s follow_spd=%.2f "
                    "self_edge=%s self_lane=%s leader_edge=%s leader_lane=%s lane_cmd_state=%s",
                    now, self.vehicle_id, merge_completed_age, follow_station_id or 0, follow_gap or 0, follow_reason, follow_speed,
                    self_edge, self_lane, leader_edge, leader_lane, lcs.get("state")
                )
            if follow_reason == "same_lane_cam" and self.last_lane_clear_time > 0.0 and now - self.last_lane_clear_time <= 3.0:
                self._log_timeline_event("POST_CLEAR_CAR_FOLLOW")
                log.debug(
                    "[%.1f] %s POST_CLEAR_CAR_FOLLOW: age=%.1f sid=%d gap=%.1f reason=%s follow_spd=%.2f "
                    "self_edge=%s self_lane=%s leader_edge=%s leader_lane=%s lane_cmd_state=%s",
                    now, self.vehicle_id, now - self.last_lane_clear_time, follow_station_id or 0, follow_gap or 0, follow_reason, follow_speed,
                    self_edge, self_lane, leader_edge, leader_lane, lcs.get("state")
                )
