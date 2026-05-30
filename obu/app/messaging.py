import copy
import json
import logging
import math

try:
    from .geometry import (
        clamp_int,
        edge_id_from_lane,
        heading_deg_to_etsi,
        latlon_to_xy,
        normalize_heading_deg,
        parse_lane_index,
        xy_to_latlon,
    )
    from .protocol import MAX_MANOEUVRE_ID, mcm_action_name, ms_since_minute
    from .vanetza_codec import unwrap_vanetza_cam, unwrap_vanetza_mcm, vanetza_station_id
except ImportError:
    from geometry import (
        clamp_int,
        edge_id_from_lane,
        heading_deg_to_etsi,
        latlon_to_xy,
        normalize_heading_deg,
        parse_lane_index,
        xy_to_latlon,
    )
    from protocol import MAX_MANOEUVRE_ID, mcm_action_name, ms_since_minute
    from vanetza_codec import unwrap_vanetza_cam, unwrap_vanetza_mcm, vanetza_station_id


log = logging.getLogger("obu")


class MessagingMixin:
    def _estimate_heading(self, x, y):
        if not self.last_position:
            return self.last_heading
        dx = float(x) - float(self.last_position.get("x", x))
        dy = float(y) - float(self.last_position.get("y", y))
        if math.hypot(dx, dy) < 1e-3:
            return self.last_heading
        return (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0

    def _update_neighbor_observation(self, station_id, x, y, speed, heading):
        dist = self._distance_to_merge(x, y)
        prev = self.neighbors.get(station_id)
        delta = dist - float(prev["distance_to_merge"]) if prev and "distance_to_merge" in prev else None
        observation = {
            "x": x,
            "y": y,
            "speed": speed,
            "heading": heading,
            "distance_to_merge": dist,
            "distance_delta": delta,
            "timestamp": self._sim_time(),
        }
        self.neighbors[station_id] = self.neighbor_memory[station_id] = observation

    def _handle_cam(self, payload):
        station_id = vanetza_station_id(payload)
        if station_id is None or station_id == self.station_id:
            return
        try:
            cam_params = unwrap_vanetza_cam(payload)["camParameters"]
            basic = cam_params["basicContainer"]
            high_freq = cam_params["highFrequencyContainer"]["basicVehicleContainerHighFrequency"]
            xy = latlon_to_xy(
                float(basic["referencePosition"]["latitude"]),
                float(basic["referencePosition"]["longitude"]),
                self.origin_lat,
                self.origin_lon,
            )
            self._update_neighbor_observation(
                station_id,
                xy["x"],
                xy["y"],
                high_freq["speed"]["speedValue"],
                normalize_heading_deg(high_freq["heading"]["headingValue"]),
            )
        except (KeyError, TypeError, ValueError):
            pass

    def _handle_mcm(self, payload):
        if not self.sensor_state:
            return
        mcm = unwrap_vanetza_mcm(payload)
        station_id = vanetza_station_id(payload) or mcm.get("basicContainer", {}).get("stationID")
        if station_id is None or int(station_id) == self.station_id:
            return
        station_id = int(station_id)
        basic = mcm.get("basicContainer", {})
        action = self._parse_mcm_action(basic.get("rational", {}).get("manoeuvreCooperationCost"))
        manoeuvre_id = int(basic.get("manoeuvreId", 0))
        target = self._mcm_target_station_id(mcm)
        if action in (1, 2, 3) and (target is None or target != self.station_id):
            return
        if action is None:
            return
        log.debug(
            "[%.1f] %s MCM_RX_%s: from=%d manoeuvre=%d target=%s",
            self._sim_time(),
            self.vehicle_id,
            mcm_action_name(action),
            station_id,
            manoeuvre_id,
            target,
        )
        try:
            xy = latlon_to_xy(
                float(basic["position"]["latitude"]),
                float(basic["position"]["longitude"]),
                self.origin_lat,
                self.origin_lon,
            )
            state = mcm["mcmContainer"]["vehicleManoeuvreContainer"]["vehicleCurrentStateContainer"]
            self._update_neighbor_observation(
                station_id,
                xy["x"],
                xy["y"],
                state["vehicleSpeed"]["speedValue"],
                heading_deg_to_etsi(state["vehicleHeading"].get("value")),
            )
        except (KeyError, TypeError, ValueError):
            pass
        self.mcm_messages[station_id] = {
            "action": action,
            "manoeuvre_id": manoeuvre_id,
            "target_station_id": target,
            "timestamp": self._sim_time(),
        }

    def _parse_mcm_action(self, value):
        try:
            action = int(value)
        except (TypeError, ValueError):
            return None
        return action if action in (1, 2, 3) else None

    def _mcm_target_station_id(self, payload):
        advice = payload.get("mcmContainer", {}).get("vehicleManoeuvreContainer", {}).get("manoeuvreAdvice", [])
        return int(advice[0]["executantID"]) if advice and advice[0].get("executantID") is not None else None

    def _build_cam(self):
        cam = copy.deepcopy(self.cam_template)
        if not self.sensor_state:
            return cam
        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        speed = float(self.sensor_state.get("speed", 0.0))
        latlon = xy_to_latlon(x, y, self.origin_lat, self.origin_lon)
        cam["stationId"] = self.station_id
        ref = cam.setdefault("camParameters", {}).setdefault("basicContainer", {}).setdefault("referencePosition", {})
        ref["latitude"], ref["longitude"] = latlon["latitude"], latlon["longitude"]
        cam["camParameters"]["basicContainer"]["stationType"] = self.station_type
        high_freq = cam["camParameters"].setdefault("highFrequencyContainer", {}).setdefault("basicVehicleContainerHighFrequency", {})
        high_freq["speed"]["speedValue"] = speed
        heading = self._current_heading() or self._estimate_heading(x, y)
        if heading is not None:
            high_freq.setdefault("heading", {})["headingValue"] = heading
            self.last_heading = heading
        high_freq.setdefault("vehicleLength", {})["vehicleLengthValue"] = self.vehicle_length
        high_freq["vehicleWidth"] = self.vehicle_width
        cam["generationDeltaTime"] = ms_since_minute()
        self.last_position = {"x": x, "y": y}
        return cam

    def _build_mcm(self, action, manoeuvre_id, target_station_id=None):
        mcm = copy.deepcopy(self.mcm_template)
        if not self.sensor_state:
            return mcm
        x = float(self.sensor_state.get("x", 0.0))
        y = float(self.sensor_state.get("y", 0.0))
        speed = float(self.sensor_state.get("speed", 0.0))
        latlon = xy_to_latlon(x, y, self.origin_lat, self.origin_lon)
        manoeuvre_id = self._normalize_manoeuvre_id(manoeuvre_id)
        mcm["stationId"] = self.station_id
        basic = mcm.setdefault("basicContainer", {})
        basic["generationDeltaTime"] = clamp_int(ms_since_minute(), 0, 65535)
        basic["stationID"] = self.station_id
        basic["stationType"] = self.mcm_station_type
        basic["itssRole"] = self.itss_role
        basic["mcmType"] = 8
        basic["manoeuvreId"] = manoeuvre_id
        basic.setdefault("rational", {})["manoeuvreCooperationCost"] = action
        basic.setdefault("position", {})["latitude"] = latlon["latitude"]
        basic["position"]["longitude"] = latlon["longitude"]
        if target_station_id:
            mcm.setdefault("mcmContainer", {}).setdefault("vehicleManoeuvreContainer", {}).setdefault("manoeuvreAdvice", [{}])[0]["executantID"] = int(target_station_id)
        state = mcm.setdefault("mcmContainer", {}).setdefault("vehicleManoeuvreContainer", {}).setdefault("vehicleCurrentStateContainer", {})
        state.setdefault("vehicleSpeed", {})["speedValue"] = clamp_int(speed, 0)
        state.setdefault("vehicleHeading", {})["value"] = heading_deg_to_etsi(self.last_heading)
        size = state.setdefault("vehicleSize", {})
        size["vehicleWidth"] = clamp_int(self.vehicle_width, 1)
        size.setdefault("vehicleLenth", {})["vehicleLengthValue"] = clamp_int(self.vehicle_length, 1)
        return mcm

    def _build_denm(self):
        denm = copy.deepcopy(self.denm_template)
        if not self.sensor_state:
            return denm
        latlon = xy_to_latlon(float(self.sensor_state["x"]), float(self.sensor_state["y"]), self.origin_lat, self.origin_lon)
        self.denm_seq += 1
        management = denm.setdefault("management", {})
        action_id = management.setdefault("actionId", {})
        action_id["originatingStationId"], action_id["sequenceNumber"] = self.station_id, self.denm_seq
        management["referenceTime"] = management["detectionTime"] = self._sim_time()
        management["stationType"] = self.station_type
        event_position = management.setdefault("eventPosition", {})
        event_position["latitude"], event_position["longitude"] = latlon["latitude"], latlon["longitude"]
        return denm

    def _publish_json(self, topic, payload):
        self.client.publish(topic, json.dumps(payload))

    def _publish_actuators(self):
        if self.target_speed is None:
            if not self.publish_idle_actuators or not self.sensor_state:
                return
            self.target_speed = float(self.sensor_state.get("speed", 0.0))
        self._publish_json(self.actuator_speed_topic, {"target_speed": float(self.target_speed), "timestamp": self._sim_time()})
        if self.target_lane_index is not None:
            self._publish_json(self.actuator_lane_topic, {"target_lane_index": int(self.target_lane_index), "timestamp": self._sim_time()})
        if self.target_speed_mode is not None:
            self._publish_json(self.actuator_speed_mode_topic, {"speed_mode": int(self.target_speed_mode), "timestamp": self._sim_time()})

    def _publish_status(self):
        distance = self._self_distance_to_merge()
        eta = self._merge_eta()
        lane_command = self.lane_command_status or {}
        lane_id = str(self.sensor_state.get("lane_id", "")) if self.sensor_state else ""
        payload = {
            "vehicle_id": self.vehicle_id,
            "station_id": self.station_id,
            "role": self.role,
            "merge_completed": getattr(self, "merge_completed", False),
            "merge_committed": getattr(self, "merge_committed", False),
            "lane_command_state": lane_command.get("state", "NONE"),
            "edge_id": edge_id_from_lane(lane_id),
            "lane_index": parse_lane_index(lane_id),
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
            "count_late_merge_recovery": self.count_late_merge_recovery,
            "count_merge_failed_no_gap": self.count_merge_failed_no_gap,
            "count_merge_completed": self.count_merge_completed,
            "count_merge_completed_clean": self.count_merge_completed_clean,
            "timestamp": self._sim_time(),
        }
        active_req = self.active_merge_request if self.active_merge_request else {}
        active_remaining = max(0.0, self.active_merge_request_until - self._sim_time()) if self.active_merge_request else 0.0
        payload["active_merge_request"] = self.active_merge_request is not None and active_remaining > 0.0
        payload["active_merge_request_station_id"] = active_req.get("station_id")
        payload["active_merge_request_manoeuvre_id"] = active_req.get("manoeuvre_id")
        payload["active_merge_request_remaining_s"] = active_remaining
        payload["active_merge_request_target_speed"] = active_req.get("target_speed")
        if self.sensor_state:
            payload["lane_id"], payload["speed"] = self.sensor_state.get("lane_id"), self.sensor_state.get("speed")
        self._publish_json(self.status_topic, payload)

    def _next_manoeuvre_id(self):
        self.mcm_seq = (self.mcm_seq + 1) % (MAX_MANOEUVRE_ID + 1)
        return ((self.station_id * 31 + self.mcm_seq) % MAX_MANOEUVRE_ID) or 1

    def _normalize_manoeuvre_id(self, value):
        try:
            parsed = int(value or self._next_manoeuvre_id())
        except (TypeError, ValueError):
            parsed = self._next_manoeuvre_id()
        return max(0, min(MAX_MANOEUVRE_ID, parsed))
