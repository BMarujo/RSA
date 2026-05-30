import logging
import math


log = logging.getLogger("obu")


class MergeGuardMixin:
    def _final_merge_lane_clear(self, lid, hid, dtm):
        if lid is None and hid is None and self._has_any_main_neighbor_near_merge():
            if self.merge_committed:
                return True
            log.debug(
                "[%.1f] %s FINAL_GUARD: REJECTED blind merge with main traffic",
                self._sim_time(),
                self.vehicle_id,
            )
            return False

        if not self.sensor_state:
            return False

        oe = self._merge_eta()
        oh = self._current_heading()
        sx = float(self.sensor_state["x"])
        sy = float(self.sensor_state["y"])
        cspd = self._current_speed() or 0.0

        rad = math.radians(90 - oh) if oh else 0
        fx = math.cos(rad)
        fy = math.sin(rad)

        checked = set()
        guard_ids = []
        if lid:
            guard_ids.append(lid)
        if hid:
            guard_ids.append(hid)
        for s, _ in self._final_guard_neighbor_items():
            if s not in guard_ids:
                guard_ids.append(s)

        for s in guard_ids:
            if s in checked:
                continue
            checked.add(s)

            data = self.neighbors.get(s)
            if data is None and s in self.neighbor_memory:
                data = self._project_neighbor_data(self.neighbor_memory[s])
            if not data:
                continue

            ndist = self._distance_to_merge(float(data["x"]), float(data["y"]))
            if ndist > self.role_detection_distance:
                continue

            is_cand = self._neighbor_is_merge_candidate(s)
            tlt = self._is_main_traffic(s) or (s in self.ramp_station_ids and not is_cand)
            if not tlt:
                continue
            if s in self.ramp_station_ids and is_cand and ndist > dtm + 0.1:
                continue

            dx = float(data["x"]) - sx
            dy = float(data["y"]) - sy
            pg = math.hypot(dx, dy)
            lat = abs(-dx * fy + dy * fx)
            lon = dx * fx + dy * fy
            ns = float(data.get("speed", 0.0))

            if s in (lid, hid):
                if pg < self.final_merge_clearance_m:
                    log.debug(
                        "[%.1f] %s FINAL_GUARD: REJECTED slot-boundary sid=%d gap=%.1f",
                        self._sim_time(),
                        self.vehicle_id,
                        s,
                        pg,
                    )
                    return False

                ne = self._neighbor_eta(s) or self._neighbor_eta_from_data(data)
                if ne and oe and abs(ne - oe) < self.safe_headway_s * 0.75:
                    log.debug(
                        "[%.1f] %s FINAL_GUARD: REJECTED slot-boundary ETA sid=%d",
                        self._sim_time(),
                        self.vehicle_id,
                        s,
                    )
                    return False
                continue

            ne = self._neighbor_eta(s) or self._neighbor_eta_from_data(data)
            eta_conflict = oe and ne and abs(ne - oe) < self.safe_headway_s
            cc = abs(ndist - dtm) < self.final_merge_clearance_m * 1.5 and lat <= self.cam_follow_lateral_tolerance * 2.0
            
            ttc_danger = False
            if lat <= self.cam_follow_lateral_tolerance * self.final_guard_lateral_mult:
                if lon < 0 and ns > cspd:
                    ttc = -lon / (ns - cspd)
                    ttc_danger = ttc < self.final_guard_ttc_s
                elif lon > 0 and cspd > ns:
                    ttc = lon / (cspd - ns)
                    ttc_danger = ttc < self.final_guard_ttc_s
            
            unsafe_gap = pg < self.final_merge_clearance_m
            unsafe_eta = eta_conflict and cc
            unsafe_close_alignment = cc and pg < 15.0
            if unsafe_gap or unsafe_eta or ttc_danger or unsafe_close_alignment:
                log.debug(
                    "[%.1f] %s FINAL_GUARD: REJECTED sid=%d gap=%.1f ttc_danger=%s",
                    self._sim_time(),
                    self.vehicle_id,
                    s,
                    pg,
                    ttc_danger,
                )
                return False

        return True
