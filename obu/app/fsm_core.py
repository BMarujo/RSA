import logging

try:
    from .protocol import STATE_CRUISE
except ImportError:
    from protocol import STATE_CRUISE


log = logging.getLogger("obu")


class FsmCoreMixin:
    def step(self):
        now = self._sim_time()

        if now - self.last_cam_sent >= self.cam_period_s:
            self._publish_json(self.cam_in_topic, self._build_cam())
            self.last_cam_sent = now

        if now - self.last_fsm_step >= self.fsm_period_s:
            self._step_fsm()
            self.last_fsm_step = now

        if now - self.last_actuator_sent >= self.actuator_period_s:
            self._publish_actuators()
            self.last_actuator_sent = now

        if now - self.last_status_sent >= self.status_period_s:
            self._publish_status()
            self.last_status_sent = now

    def _step_fsm(self):
        if not self.sensor_state:
            return

        now = self._sim_time()

        self._update_self_merge_progress()
        self._prune_neighbors()
        self._prune_mcm_messages()

        self.effective_role = self._resolve_role()
        self.skip_car_following_this_step = False

        desired_speed = self._base_cruise_speed()
        if self.desired_speed:
            try:
                desired_speed = float(self.desired_speed)
            except (TypeError, ValueError):
                pass

        previous_state = self.fsm_state
        self.target_speed = max(desired_speed, self.min_speed)
        self.target_lane_index = None
        self.target_speed_mode = self.default_speed_mode

        request = self._latest_request() if not self.is_ramp_vehicle else None
        should_host = (
            not self.is_ramp_vehicle
            and (self._has_active_host_reservation() or request)
        )

        if should_host:
            self.effective_role = "host"
            self._fsm_host()
        elif self.effective_role == "cruise":
            self._set_state(STATE_CRUISE)
        elif self.effective_role == "merge":
            self._fsm_merge()
        elif self.effective_role == "host":
            self._fsm_host()
        elif self.effective_role == "lead":
            self._fsm_lead()

        host_should_clear_lane = (
            self.host_cooperative_lane_change
            and self.host_clear_lane_until > self._sim_time()
            and self.effective_role in ("host", "lead")
        )
        if host_should_clear_lane:
            self.target_lane_index = self.host_clear_lane_index

        self._lock_left_lane_near_merge()
        self._apply_car_following()
        self._log_fsm_debug(now, previous_state)

    def _log_fsm_debug(self, now, previous_state):
        if not hasattr(self, "_last_debug_log"):
            self._last_debug_log = 0.0

        if now - self._last_debug_log < 1.0:
            return

        self._last_debug_log = now
        log.debug(
            "[%.1f] %s role=%s state=%s->%s speed=%.2f target=%.2f neighbors=%d",
            now,
            self.vehicle_id,
            self.effective_role,
            previous_state,
            self.fsm_state,
            self._current_speed() or 0,
            self.target_speed or 0,
            len(self.neighbors),
        )
