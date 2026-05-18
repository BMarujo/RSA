import json
import math
import os
import shlex
import sys
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt


MARKER_LAYER = 250
BADGE_LAYER = 260
MERGE_POINT_LAYER = 240
SKIN_BODY_LAYER = 270
SKIN_DETAIL_LAYER = 272
BRAKE_LIGHT_LAYER = 275

ROLE_COLORS = {
    "merge": (255, 128, 32, 120),
    "host": (30, 170, 220, 120),
    "lead": (85, 135, 255, 120),
}
STATE_COLORS = {
    "CRUISE": (130, 135, 140, 240),
    "NEGOTIATING": (255, 195, 45, 245),
    "YIELDING": (45, 205, 215, 245),
    "MERGING": (145, 95, 230, 245),
    "ABORT": (230, 55, 55, 245),
}
DEFAULT_ROLE_COLOR = (245, 245, 245, 105)
DEFAULT_STATE_COLOR = (130, 135, 140, 240)
MERGE_ZONE_COLOR = (255, 205, 45, 165)
ACTIVE_STATES = {"NEGOTIATING", "YIELDING", "MERGING", "ABORT"}


def env(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value is not None and value != "" else default


def env_bool(name: str, default: str) -> bool:
    return env(name, default).lower() in ("1", "true", "yes", "on")


def parse_lane_index(lane_id: str) -> Optional[int]:
    if not lane_id:
        return None
    try:
        return int(lane_id.rsplit("_", 1)[-1])
    except ValueError:
        return None


def ensure_traci() -> None:
    sumo_home = os.getenv("SUMO_HOME")
    if sumo_home:
        tools = os.path.join(sumo_home, "tools")
        if tools not in sys.path:
            sys.path.append(tools)
    fallback = "/usr/share/sumo/tools"
    if fallback not in sys.path:
        sys.path.append(fallback)


ensure_traci()
try:
    import traci  # type: ignore
except Exception as exc:
    raise RuntimeError("traci is not available. Set SUMO_HOME or install sumo.") from exc


class TraciBridge:
    def __init__(self) -> None:
        self.sumo_cfg = env("SUMO_CFG", "/data/sim.sumocfg")
        self.sumo_gui = env("SUMO_GUI", "false").lower() == "true"
        self.step_length = float(env("STEP_LENGTH", "0.1"))
        self.step_delay_s = float(env("STEP_DELAY_S", "0"))
        self.speed_command_duration_s = float(env("SPEED_COMMAND_DURATION_S", "1.0"))
        self.collision_guard = env_bool("COLLISION_GUARD", "false")
        self.collision_guard_lookahead = float(env("COLLISION_GUARD_LOOKAHEAD", "35.0"))
        self.collision_guard_min_gap = float(env("COLLISION_GUARD_MIN_GAP", "6.5"))
        self.collision_guard_headway_s = float(env("COLLISION_GUARD_HEADWAY_S", "0.65"))
        self.collision_guard_speed_delta = float(env("COLLISION_GUARD_SPEED_DELTA", "0.8"))
        self.collision_guard_min_speed = float(env("COLLISION_GUARD_MIN_SPEED", "1.5"))
        self.collision_guard_duration_s = float(env("COLLISION_GUARD_DURATION_S", "0.8"))
        self.collision_guard_max_decel = float(env("COLLISION_GUARD_MAX_DECEL", "3.5"))
        self.vehicle_decel = float(env("TRACI_VEHICLE_DECEL", "50.0"))
        self.vehicle_emergency_decel = float(env("TRACI_VEHICLE_EMERGENCY_DECEL", "50.0"))
        self.lane_change_duration_s = float(env("LANE_CHANGE_DURATION_S", "3.0"))
        self.lane_change_cooldown_s = float(env("LANE_CHANGE_COOLDOWN_S", "2.0"))
        self.command_lane_change_mode = int(env("TRACI_COMMAND_LANE_CHANGE_MODE", "256"))
        self.sumo_end = env("SUMO_END", "")
        self.sumo_end_s = float(self.sumo_end) if self.sumo_end else None
        self.sumo_extra_args = env("SUMO_EXTRA_ARGS", "")
        self.loop_sim = env("LOOP_SIM", "false").lower() == "true"
        self.loop_pause_s = float(env("LOOP_PAUSE_S", "0"))

        self.mqtt_host = env("MQTT_HOST", "mqtt-broker")
        self.mqtt_port = int(env("MQTT_PORT", "1883"))

        self.vehicle_ids = [v for v in env("VEHICLE_IDS", "").split(",") if v]

        self.sensor_topic_fmt = env("SENSOR_TOPIC_FMT", "car/{vehicle_id}/sensors/gps")
        self.actuator_speed_fmt = env("ACT_SPEED_TOPIC_FMT", "car/{vehicle_id}/actuators/speed")
        self.actuator_lane_fmt = env("ACT_LANE_TOPIC_FMT", "car/{vehicle_id}/actuators/lane")
        self.actuator_speed_mode_fmt = env("ACT_SPEED_MODE_TOPIC_FMT", "car/{vehicle_id}/actuators/speed_mode")

        self.gui_markers = env_bool("GUI_MARKERS", "true")
        self.gui_track_vehicle = os.getenv("GUI_TRACK_VEHICLE", "Merge_Car").strip()
        self.gui_fixed_merge_view = env_bool("GUI_FIXED_MERGE_VIEW", "false")
        self.gui_merge_view_radius = float(env("GUI_MERGE_VIEW_RADIUS", "95"))
        self.gui_fit_network = env_bool("GUI_FIT_NETWORK", "false")
        self.gui_boundary_padding = float(env("GUI_BOUNDARY_PADDING", "80"))
        self.gui_zoom = float(env("GUI_ZOOM", "1800"))
        self.gui_marker_radius = float(env("GUI_MARKER_RADIUS", "9"))
        self.gui_badge_size = float(env("GUI_BADGE_SIZE", "5"))
        self.gui_merge_point = env_bool("GUI_MERGE_POINT", "true")
        self.gui_merge_zone_length = float(env("GUI_MERGE_ZONE_LENGTH", "13"))
        self.gui_merge_zone_width = float(env("GUI_MERGE_ZONE_WIDTH", "1.1"))
        self.gui_merge_zone_gap = float(env("GUI_MERGE_ZONE_GAP", "5.5"))
        self.gui_merge_zone_angle_deg = float(env("GUI_MERGE_ZONE_ANGLE_DEG", "0"))
        self.gui_vehicle_skins = env_bool("GUI_VEHICLE_SKINS", "true")
        self.gui_vehicle_skin_scale = float(env("GUI_VEHICLE_SKIN_SCALE", "1.18"))
        self.gui_vehicle_skin_detail = env_bool("GUI_VEHICLE_SKIN_DETAIL", "false")
        self.gui_dim_sumo_vehicles = env_bool("GUI_DIM_SUMO_VEHICLES", "true")
        self.gui_role_markers = env_bool("GUI_ROLE_MARKERS", "false")
        self.gui_state_badges = env_bool("GUI_STATE_BADGES", "false")
        self.gui_state_body_tint = env_bool("GUI_STATE_BODY_TINT", "false")
        self.gui_state_body_tint_amount = float(env("GUI_STATE_BODY_TINT_AMOUNT", "0.34"))
        self.gui_state_indicator_width = float(env("GUI_STATE_INDICATOR_WIDTH", "0.22"))
        self.gui_state_roof = env_bool("GUI_STATE_ROOF", "false")
        self.gui_show_cruise_state = env_bool("GUI_SHOW_CRUISE_STATE", "false")
        self.gui_color_vehicles_by_state = env_bool("GUI_COLOR_VEHICLES_BY_STATE", "false")
        self.gui_brake_lights = env_bool("GUI_BRAKE_LIGHTS", "true")
        self.gui_brake_light_size = float(env("GUI_BRAKE_LIGHT_SIZE", "1.15"))
        self.gui_brake_decel_threshold = float(env("GUI_BRAKE_DECEL_THRESHOLD", "0.25"))
        self.merge_point_x = float(env("MERGE_POINT_X", "0"))
        self.merge_point_y = float(env("MERGE_POINT_Y", "0"))
        self.gui_view_id: Optional[str] = None
        self.gui_tracked_vehicle: Optional[str] = None

        self.speed_commands: Dict[str, float] = {}
        self.lane_commands: Dict[str, int] = {}
        self.speed_mode_commands: Dict[str, int] = {}
        self.initial_speed_mode = int(env("TRACI_DEFAULT_SPEED_MODE", "-1"))
        self.initial_lane_change_mode = int(env("TRACI_DEFAULT_LANE_CHANGE_MODE", "-1"))
        self.initial_speed_mode_applied: set[str] = set()
        self.initial_lane_change_mode_applied: set[str] = set()
        self.lane_command_state: Dict[str, Dict[str, float]] = {}
        self.fsm_status: Dict[str, Dict[str, Any]] = {}
        self.polygon_ids: set[str] = set()
        self.skin_ids: set[str] = set()
        self.marker_ids: set[str] = set()
        self.badge_ids: set[str] = set()
        self.brake_light_ids: set[str] = set()
        self.previous_speeds: Dict[str, float] = {}

        self.client = mqtt.Client(client_id="traci-bridge")
        self.client.on_message = self.on_message

    def connect(self) -> None:
        self.client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.client.subscribe("car/+/actuators/speed")
        self.client.subscribe("car/+/actuators/lane")
        self.client.subscribe("car/+/actuators/speed_mode")
        self.client.subscribe("car/+/status/#")
        self.client.loop_start()

    def on_message(self, _client: mqtt.Client, _userdata, msg: mqtt.MQTTMessage) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except json.JSONDecodeError:
            return

        parts = msg.topic.split("/")
        if len(parts) < 4:
            return
        vehicle_id = parts[1]
        if len(parts) >= 3 and parts[2] == "status":
            self.fsm_status[vehicle_id] = payload
        elif parts[-1] == "speed":
            target = payload.get("target_speed")
            if target is not None:
                self.speed_commands[vehicle_id] = float(target)
        elif parts[-1] == "lane":
            target = payload.get("target_lane_index")
            if target is not None:
                self.lane_commands[vehicle_id] = int(target)
        elif parts[-1] == "speed_mode":
            target = payload.get("speed_mode")
            if target is not None:
                self.speed_mode_commands[vehicle_id] = int(target)

    def start_sumo(self) -> None:
        binary = "sumo-gui" if self.sumo_gui else "sumo"
        cmd = [binary, "-c", self.sumo_cfg, "--step-length", str(self.step_length)]
        if self.sumo_end:
            cmd.extend(["--end", self.sumo_end])
        if self.sumo_extra_args:
            cmd.extend(shlex.split(self.sumo_extra_args))
        traci.start(cmd)
        self._reset_gui_cache()
        self._configure_gui()

    def _reset_gui_cache(self) -> None:
        self.polygon_ids.clear()
        self.skin_ids.clear()
        self.marker_ids.clear()
        self.badge_ids.clear()
        self.brake_light_ids.clear()
        self.previous_speeds.clear()
        self.initial_speed_mode_applied.clear()
        self.initial_lane_change_mode_applied.clear()

    def _configure_gui(self) -> None:
        if not self.sumo_gui:
            return
        try:
            view_ids = traci.gui.getIDList()
        except traci.TraCIException:
            return
        if not view_ids:
            return
        self.gui_view_id = view_ids[0]
        self._set_initial_gui_view()
        if self.gui_merge_point:
            self._draw_merge_zone_marker()

    def _track_vehicle_enabled(self) -> bool:
        return self.gui_track_vehicle.lower() not in ("", "none", "false", "off", "0")

    def _set_initial_gui_view(self) -> None:
        if not self.gui_view_id:
            return
        if self.gui_fixed_merge_view:
            try:
                radius = self.gui_merge_view_radius
                traci.gui.setBoundary(
                    self.gui_view_id,
                    self.merge_point_x - radius,
                    self.merge_point_y - radius,
                    self.merge_point_x + radius,
                    self.merge_point_y + radius,
                )
                return
            except traci.TraCIException:
                pass
        if self.gui_fit_network:
            try:
                (min_x, min_y), (max_x, max_y) = traci.simulation.getNetBoundary()
                padding = self.gui_boundary_padding
                traci.gui.setBoundary(
                    self.gui_view_id,
                    min_x - padding,
                    min_y - padding,
                    max_x + padding,
                    max_y + padding,
                )
                return
            except traci.TraCIException:
                pass
        try:
            traci.gui.setZoom(self.gui_view_id, self.gui_zoom)
        except traci.TraCIException:
            pass

    def _publish_sensor(self, vehicle_id: str, x: float, y: float, speed: float, lane_id: str, heading: float) -> None:
        payload = {
            "x": x,
            "y": y,
            "speed": speed,
            "heading": heading,
            "lane_id": lane_id,
            "time": traci.simulation.getTime(),
            "timestamp": time.time(),
        }
        topic = self.sensor_topic_fmt.format(vehicle_id=vehicle_id)
        self.client.publish(topic, json.dumps(payload))

    def _apply_actuators(self, vehicle_id: str) -> None:
        if self.initial_speed_mode >= 0 and vehicle_id not in self.initial_speed_mode_applied:
            traci.vehicle.setSpeedMode(vehicle_id, self.initial_speed_mode)
            traci.vehicle.setDecel(vehicle_id, self.vehicle_decel)
            traci.vehicle.setEmergencyDecel(vehicle_id, self.vehicle_emergency_decel)
            self.initial_speed_mode_applied.add(vehicle_id)
        if self.initial_lane_change_mode >= 0 and vehicle_id not in self.initial_lane_change_mode_applied:
            traci.vehicle.setLaneChangeMode(vehicle_id, self.initial_lane_change_mode)
            self.initial_lane_change_mode_applied.add(vehicle_id)
        if vehicle_id in self.speed_commands:
            target_speed = self.speed_commands[vehicle_id]
            # Determine the effective speed_mode for this vehicle.
            effective_sm = self.speed_mode_commands.get(vehicle_id, self.initial_speed_mode)
            if effective_sm == 0 or self.speed_command_duration_s <= 0:
                # Full TraCI control — set speed directly so SUMO's
                # car-following model cannot interfere.
                traci.vehicle.setSpeed(vehicle_id, target_speed)
            else:
                traci.vehicle.slowDown(vehicle_id, target_speed, self.speed_command_duration_s)
        if vehicle_id in self.lane_commands:
            target_lane = self.lane_commands[vehicle_id]
            lane_id = traci.vehicle.getLaneID(vehicle_id)
            edge_id = lane_id.rsplit("_", 1)[0]
            current_lane = parse_lane_index(lane_id)
            on_internal_edge = edge_id.startswith(":")
            now = traci.simulation.getTime()
            last = self.lane_command_state.get(vehicle_id)
            
            recently_requested = (
                last is not None
                and int(last.get("target_lane", -1)) == target_lane
                and str(last.get("edge_id", "")) == edge_id
                and now - last.get("timestamp", 0.0) < self.lane_change_cooldown_s
            )
            
            print(
                f"LANE_CMD_STATE veh={vehicle_id} edge={edge_id} lane={current_lane} "
                f"target={target_lane} lane_count={traci.edge.getLaneNumber(edge_id)} "
                f"recently={recently_requested} speed={traci.vehicle.getSpeed(vehicle_id):.2f}"
            )
            
            if current_lane is not None and current_lane == target_lane and not on_internal_edge:
                # Vehicle has reached the target lane. 
                # Only clear if it's not a temporary transition lane or if requested to stop.
                print(f"LANE_CMD_CLEAR veh={vehicle_id} edge={edge_id} target={target_lane}")
                self.lane_commands.pop(vehicle_id, None)
                self.lane_command_state.pop(vehicle_id, None)
                if self.initial_lane_change_mode >= 0:
                    traci.vehicle.setLaneChangeMode(vehicle_id, self.initial_lane_change_mode)
            elif current_lane is not None and current_lane == target_lane:
                print(f"LANE_CMD_HOLD_INTERNAL veh={vehicle_id} edge={edge_id} target={target_lane}")
            elif (
                current_lane is not None
                and current_lane != target_lane
            ):
                if target_lane < traci.edge.getLaneNumber(edge_id):
                    if not recently_requested:
                        print(
                            f"LANE_CMD_APPLY veh={vehicle_id} edge={edge_id} from_lane={current_lane} "
                            f"target={target_lane} duration={self.lane_change_duration_s:.2f} "
                            f"mode={self.command_lane_change_mode} speed={traci.vehicle.getSpeed(vehicle_id):.2f}"
                        )
                        if self.command_lane_change_mode >= 0:
                            traci.vehicle.setLaneChangeMode(vehicle_id, self.command_lane_change_mode)
                        try:
                            traci.vehicle.changeLane(vehicle_id, target_lane, self.lane_change_duration_s)
                            self.lane_command_state[vehicle_id] = {
                                "edge_id": edge_id,
                                "target_lane": float(target_lane),
                                "timestamp": now,
                            }
                        except traci.TraCIException as exc:
                            print(f"LANE_CMD_FAILED veh={vehicle_id} edge={edge_id} error={exc}")
                else:
                    print(
                        f"LANE_CMD_WAIT_EDGE veh={vehicle_id} edge={edge_id} lane={current_lane} "
                        f"target={target_lane} lane_count={traci.edge.getLaneNumber(edge_id)}"
                    )
        if vehicle_id in self.speed_mode_commands:
            traci.vehicle.setSpeedMode(vehicle_id, self.speed_mode_commands[vehicle_id])
        self._apply_collision_guard(vehicle_id)

    def _apply_collision_guard(self, vehicle_id: str) -> None:
        if not self.collision_guard:
            return
        try:
            leader = traci.vehicle.getLeader(vehicle_id, self.collision_guard_lookahead)
        except traci.TraCIException:
            return
        if leader is None:
            return

        leader_id, gap = leader
        try:
            own_speed = traci.vehicle.getSpeed(vehicle_id)
            leader_speed = traci.vehicle.getSpeed(leader_id)
        except traci.TraCIException:
            return

        desired_gap = self.collision_guard_min_gap + own_speed * self.collision_guard_headway_s
        if gap >= desired_gap:
            return

        ratio = max(gap / max(desired_gap, 0.1), 0.0)
        target_speed = min(
            own_speed,
            max(leader_speed - self.collision_guard_speed_delta, self.collision_guard_min_speed),
            max(own_speed * ratio, self.collision_guard_min_speed),
        )
        duration = max(self.step_length, self.collision_guard_duration_s)
        target_speed = max(target_speed, own_speed - self.collision_guard_max_decel * duration)
        traci.vehicle.slowDown(vehicle_id, target_speed, duration)

    def _circle_shape(self, x: float, y: float, radius: float) -> list[tuple[float, float]]:
        return [
            (
                x + radius * math.cos((2.0 * math.pi * idx) / 18.0),
                y + radius * math.sin((2.0 * math.pi * idx) / 18.0),
            )
            for idx in range(18)
        ]

    def _badge_shape(self, x: float, y: float) -> list[tuple[float, float]]:
        size = self.gui_badge_size
        offset = self.gui_marker_radius + (size * 1.6)
        cx = x + offset
        cy = y + offset
        return [
            (cx - size, cy - size),
            (cx + size, cy - size),
            (cx + size, cy + size),
            (cx - size, cy + size),
        ]

    def _rectangle_at(
        self,
        cx: float,
        cy: float,
        forward: tuple[float, float],
        right: tuple[float, float],
        length: float,
        width: float,
    ) -> list[tuple[float, float]]:
        fx, fy = forward
        rx, ry = right
        half_length = length / 2.0
        half_width = width / 2.0
        return [
            (cx - fx * half_length - rx * half_width, cy - fy * half_length - ry * half_width),
            (cx + fx * half_length - rx * half_width, cy + fy * half_length - ry * half_width),
            (cx + fx * half_length + rx * half_width, cy + fy * half_length + ry * half_width),
            (cx - fx * half_length + rx * half_width, cy - fy * half_length + ry * half_width),
        ]

    def _merge_zone_axes(self) -> tuple[tuple[float, float], tuple[float, float]]:
        angle = math.radians(self.gui_merge_zone_angle_deg)
        return (math.sin(angle), math.cos(angle)), (math.cos(angle), -math.sin(angle))

    def _draw_merge_zone_marker(self) -> None:
        forward, right = self._merge_zone_axes()
        for side, sign in (("left", -1.0), ("right", 1.0)):
            cx = self.merge_point_x + right[0] * self.gui_merge_zone_gap * sign
            cy = self.merge_point_y + right[1] * self.gui_merge_zone_gap * sign
            self._set_polygon(
                f"merge-zone-{side}",
                self._rectangle_at(
                    cx,
                    cy,
                    forward,
                    right,
                    self.gui_merge_zone_length,
                    self.gui_merge_zone_width,
                ),
                MERGE_ZONE_COLOR,
                MERGE_POINT_LAYER,
                "merge-zone",
            )

    def _vehicle_axes(self, vehicle_id: str) -> tuple[tuple[float, float], tuple[float, float]]:
        angle = math.radians(traci.vehicle.getAngle(vehicle_id))
        return (math.sin(angle), math.cos(angle)), (math.cos(angle), -math.sin(angle))

    def _vehicle_body_shape(
        self,
        x: float,
        y: float,
        forward: tuple[float, float],
        right: tuple[float, float],
        length: float,
        width: float,
    ) -> list[tuple[float, float]]:
        fx, fy = forward
        rx, ry = right
        return [
            (x - fx * length * 0.50 + rx * width * 0.26, y - fy * length * 0.50 + ry * width * 0.26),
            (x - fx * length * 0.40 + rx * width * 0.50, y - fy * length * 0.40 + ry * width * 0.50),
            (x + fx * length * 0.20 + rx * width * 0.50, y + fy * length * 0.20 + ry * width * 0.50),
            (x + fx * length * 0.50 + rx * width * 0.30, y + fy * length * 0.50 + ry * width * 0.30),
            (x + fx * length * 0.50 - rx * width * 0.30, y + fy * length * 0.50 - ry * width * 0.30),
            (x + fx * length * 0.20 - rx * width * 0.50, y + fy * length * 0.20 - ry * width * 0.50),
            (x - fx * length * 0.40 - rx * width * 0.50, y - fy * length * 0.40 - ry * width * 0.50),
            (x - fx * length * 0.50 - rx * width * 0.26, y - fy * length * 0.50 - ry * width * 0.26),
        ]

    def _safe_vehicle_color(self, vehicle_id: str) -> tuple[int, int, int, int]:
        try:
            color = traci.vehicle.getColor(vehicle_id)
            if len(color) == 4:
                return color
            return (color[0], color[1], color[2], 255)
        except traci.TraCIException:
            return (75, 115, 245, 255)

    def _with_alpha(self, color: tuple[int, ...], alpha: int) -> tuple[int, int, int, int]:
        return (int(color[0]), int(color[1]), int(color[2]), alpha)

    def _mix_color(self, color: tuple[int, ...], target: tuple[int, int, int], amount: float, alpha: int) -> tuple[int, int, int, int]:
        keep = 1.0 - amount
        return (
            int(color[0] * keep + target[0] * amount),
            int(color[1] * keep + target[1] * amount),
            int(color[2] * keep + target[2] * amount),
            alpha,
        )

    def _vehicle_accent_color(self, vehicle_id: str, base_color: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
        state = self._vehicle_highlight_state(vehicle_id)
        if state:
            return self._with_alpha(STATE_COLORS[state], 245)
        role_color = self._role_color(vehicle_id)
        if role_color != DEFAULT_ROLE_COLOR:
            return self._with_alpha(role_color, 245)
        return self._mix_color(base_color, (255, 255, 255), 0.35, 245)

    def _vehicle_state(self, vehicle_id: str) -> str:
        status = self.fsm_status.get(vehicle_id, {})
        return str(status.get("fsm_state") or "").upper()

    def _vehicle_highlight_state(self, vehicle_id: str) -> str:
        state = self._vehicle_state(vehicle_id)
        if state == "CRUISE" and self.gui_show_cruise_state:
            return state
        if state in ACTIVE_STATES:
            return state
        return ""

    def _skin_part_id(self, vehicle_id: str, part: str) -> str:
        return f"vehicle-skin-{vehicle_id}-{part}"

    def _draw_vehicle_skin(self, vehicle_id: str, x: float, y: float) -> None:
        forward, right = self._vehicle_axes(vehicle_id)
        length = max(traci.vehicle.getLength(vehicle_id), 3.8) * self.gui_vehicle_skin_scale
        width = max(traci.vehicle.getWidth(vehicle_id), 1.7) * self.gui_vehicle_skin_scale
        base_color = self._safe_vehicle_color(vehicle_id)
        state = self._vehicle_highlight_state(vehicle_id)
        tint_target = STATE_COLORS.get(state) if self.gui_state_body_tint else None
        body_base = self._mix_color(base_color, tint_target, self.gui_state_body_tint_amount, 255) if tint_target else base_color
        body_color = self._mix_color(body_base, (255, 255, 255), 0.08, 242)
        glass_color = (32, 62, 88, 230)
        accent_color = self._vehicle_accent_color(vehicle_id, base_color)

        if self.gui_dim_sumo_vehicles:
            try:
                traci.vehicle.setColor(vehicle_id, self._with_alpha(base_color, 35))
            except traci.TraCIException:
                pass

        fx, fy = forward
        rx, ry = right
        parts = {
            "body": (
                self._vehicle_body_shape(x, y, forward, right, length, width),
                body_color,
                SKIN_BODY_LAYER,
                "vehicle-body",
            ),
            "glass": (
                self._rectangle_at(x + fx * length * 0.08, y + fy * length * 0.08, forward, right, length * 0.34, width * 0.48),
                glass_color,
                SKIN_DETAIL_LAYER,
                "vehicle-glass",
            ),
            "stripe": (
                self._rectangle_at(
                    x - rx * width * 0.34,
                    y - ry * width * 0.34,
                    forward,
                    right,
                    length * 0.72,
                    width * self.gui_state_indicator_width,
                ),
                accent_color,
                SKIN_DETAIL_LAYER + 1,
                "vehicle-stripe",
            ),
        }
        if self.gui_vehicle_skin_detail:
            parts.update(
                {
                    "head-left": (
                        self._rectangle_at(
                            x + fx * length * 0.52 + rx * width * 0.22,
                            y + fy * length * 0.52 + ry * width * 0.22,
                            forward,
                            right,
                            length * 0.08,
                            width * 0.15,
                        ),
                        (215, 245, 255, 245),
                        SKIN_DETAIL_LAYER + 3,
                        "vehicle-headlight",
                    ),
                    "head-right": (
                        self._rectangle_at(
                            x + fx * length * 0.52 - rx * width * 0.22,
                            y + fy * length * 0.52 - ry * width * 0.22,
                            forward,
                            right,
                            length * 0.08,
                            width * 0.15,
                        ),
                        (215, 245, 255, 245),
                        SKIN_DETAIL_LAYER + 3,
                        "vehicle-headlight",
                    ),
                    "antenna": (
                        self._circle_shape(x - fx * length * 0.22, y - fy * length * 0.22, max(width * 0.11, 0.18)),
                        accent_color,
                        SKIN_DETAIL_LAYER + 4,
                        "vehicle-antenna",
                    ),
                }
            )
        if self.gui_state_roof and state:
            parts["state-roof"] = (
                self._rectangle_at(x - fx * length * 0.23, y - fy * length * 0.23, forward, right, length * 0.22, width * 0.50),
                self._with_alpha(accent_color, 230),
                SKIN_DETAIL_LAYER + 2,
                "vehicle-state",
            )

        for part, (shape, color, layer, polygon_type) in parts.items():
            polygon_id = self._skin_part_id(vehicle_id, part)
            self._set_polygon(polygon_id, shape, color, layer, polygon_type)
            self.skin_ids.add(polygon_id)

    def _brake_light_shapes(self, vehicle_id: str, x: float, y: float) -> list[list[tuple[float, float]]]:
        forward, right = self._vehicle_axes(vehicle_id)
        length = max(traci.vehicle.getLength(vehicle_id), 3.5)
        width = max(traci.vehicle.getWidth(vehicle_id), 1.6)
        rear_x = x - forward[0] * (length / 2.0 + 0.12)
        rear_y = y - forward[1] * (length / 2.0 + 0.12)
        side_offset = width * 0.33
        light_length = self.gui_brake_light_size * 0.55
        light_width = self.gui_brake_light_size * 0.42
        return [
            self._rectangle_at(
                rear_x + right[0] * side_offset,
                rear_y + right[1] * side_offset,
                forward,
                right,
                light_length,
                light_width,
            ),
            self._rectangle_at(
                rear_x - right[0] * side_offset,
                rear_y - right[1] * side_offset,
                forward,
                right,
                light_length,
                light_width,
            ),
        ]

    def _set_polygon(
        self,
        polygon_id: str,
        shape: list[tuple[float, float]],
        color: tuple[int, int, int, int],
        layer: int,
        polygon_type: str,
    ) -> None:
        try:
            if polygon_id in self.polygon_ids:
                traci.polygon.setShape(polygon_id, shape)
                traci.polygon.setColor(polygon_id, color)
            else:
                traci.polygon.add(polygon_id, shape, color, True, polygonType=polygon_type, layer=layer)
                self.polygon_ids.add(polygon_id)
        except traci.TraCIException:
            try:
                traci.polygon.add(polygon_id, shape, color, True, polygonType=polygon_type, layer=layer)
                self.polygon_ids.add(polygon_id)
            except traci.TraCIException:
                pass

    def _remove_polygon(self, polygon_id: str) -> None:
        for layer in (
            MARKER_LAYER,
            BADGE_LAYER,
            SKIN_BODY_LAYER,
            SKIN_BODY_LAYER + 1,
            SKIN_DETAIL_LAYER,
            SKIN_DETAIL_LAYER + 1,
            SKIN_DETAIL_LAYER + 2,
            SKIN_DETAIL_LAYER + 3,
            SKIN_DETAIL_LAYER + 4,
            BRAKE_LIGHT_LAYER,
            MERGE_POINT_LAYER,
            0,
        ):
            try:
                traci.polygon.remove(polygon_id, layer=layer)
                self.polygon_ids.discard(polygon_id)
                return
            except traci.TraCIException:
                continue
        self.polygon_ids.discard(polygon_id)

    def _role_color(self, vehicle_id: str) -> tuple[int, int, int, int]:
        status = self.fsm_status.get(vehicle_id, {})
        role = str(status.get("effective_role") or status.get("role") or "").lower()
        return ROLE_COLORS.get(role, DEFAULT_ROLE_COLOR)

    def _state_color(self, vehicle_id: str) -> tuple[int, int, int, int]:
        status = self.fsm_status.get(vehicle_id, {})
        state = str(status.get("fsm_state") or "").upper()
        return STATE_COLORS.get(state, DEFAULT_STATE_COLOR)

    def _brake_lights_active(self, vehicle_id: str, speed: float) -> bool:
        status = self.fsm_status.get(vehicle_id, {})
        state = str(status.get("fsm_state") or "").upper()
        previous_speed = self.previous_speeds.get(vehicle_id)
        decelerating = previous_speed is not None and previous_speed - speed > self.gui_brake_decel_threshold
        return decelerating or state in ("YIELDING", "ABORT")

    def _update_gui(self, vehicle_ids: list[str]) -> None:
        if not self.sumo_gui:
            return
        active = set(vehicle_ids)
        for vehicle_id in list(self.previous_speeds):
            if vehicle_id not in active:
                self.previous_speeds.pop(vehicle_id, None)
        for polygon_id in list(self.skin_ids):
            if not self.gui_vehicle_skins or not any(polygon_id.startswith(f"vehicle-skin-{vehicle_id}-") for vehicle_id in active):
                self._remove_polygon(polygon_id)
                self.skin_ids.discard(polygon_id)
        for polygon_id in list(self.marker_ids):
            vehicle_id = polygon_id.removeprefix("vehicle-marker-")
            if not self.gui_markers or not self.gui_role_markers or vehicle_id not in active:
                self._remove_polygon(polygon_id)
                self.marker_ids.discard(polygon_id)
        for polygon_id in list(self.badge_ids):
            vehicle_id = polygon_id.removeprefix("fsm-badge-")
            if not self.gui_markers or not self.gui_state_badges or vehicle_id not in active:
                self._remove_polygon(polygon_id)
                self.badge_ids.discard(polygon_id)
        for polygon_id in list(self.brake_light_ids):
            vehicle_id = polygon_id.removeprefix("brake-light-").rsplit("-", 1)[0]
            if not self.gui_brake_lights or vehicle_id not in active:
                self._remove_polygon(polygon_id)
                self.brake_light_ids.discard(polygon_id)

        tracked_vehicle = self.gui_track_vehicle if self._track_vehicle_enabled() and self.gui_track_vehicle in active else None
        if self.gui_view_id and tracked_vehicle and tracked_vehicle != self.gui_tracked_vehicle:
            try:
                traci.gui.trackVehicle(self.gui_view_id, tracked_vehicle)
                traci.gui.setZoom(self.gui_view_id, self.gui_zoom)
                self.gui_tracked_vehicle = tracked_vehicle
            except traci.TraCIException:
                pass

        for vehicle_id in vehicle_ids:
            try:
                x, y = traci.vehicle.getPosition(vehicle_id)
                speed = traci.vehicle.getSpeed(vehicle_id)
                state_color = self._state_color(vehicle_id)
                if self.gui_vehicle_skins:
                    self._draw_vehicle_skin(vehicle_id, x, y)
                elif self.gui_color_vehicles_by_state:
                    traci.vehicle.setColor(vehicle_id, state_color)
            except traci.TraCIException:
                continue
            marker_id = f"vehicle-marker-{vehicle_id}"
            badge_id = f"fsm-badge-{vehicle_id}"
            if self.gui_markers and self.gui_role_markers:
                self._set_polygon(
                    marker_id,
                    self._circle_shape(x, y, self.gui_marker_radius),
                    self._role_color(vehicle_id),
                    MARKER_LAYER,
                    "vehicle-marker",
                )
                self.marker_ids.add(marker_id)
            if self.gui_markers and self.gui_state_badges:
                self._set_polygon(
                    badge_id,
                    self._badge_shape(x, y),
                    state_color,
                    BADGE_LAYER,
                    "fsm-badge",
                )
                self.badge_ids.add(badge_id)
            brake_light_left = f"brake-light-{vehicle_id}-left"
            brake_light_right = f"brake-light-{vehicle_id}-right"
            if self.gui_brake_lights and self._brake_lights_active(vehicle_id, speed):
                try:
                    left_shape, right_shape = self._brake_light_shapes(vehicle_id, x, y)
                except traci.TraCIException:
                    self.previous_speeds[vehicle_id] = speed
                    continue
                self._set_polygon(brake_light_left, left_shape, (255, 24, 24, 245), BRAKE_LIGHT_LAYER, "brake-light")
                self._set_polygon(brake_light_right, right_shape, (255, 24, 24, 245), BRAKE_LIGHT_LAYER, "brake-light")
                self.brake_light_ids.update({brake_light_left, brake_light_right})
            else:
                for brake_light_id in (brake_light_left, brake_light_right):
                    if brake_light_id in self.brake_light_ids:
                        self._remove_polygon(brake_light_id)
                        self.brake_light_ids.discard(brake_light_id)
            self.previous_speeds[vehicle_id] = speed

    def step(self) -> None:
        traci.simulationStep()
        if self.vehicle_ids:
            available = set(traci.vehicle.getIDList())
            vehicle_ids = [vehicle_id for vehicle_id in self.vehicle_ids if vehicle_id in available]
        else:
            vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            try:
                x, y = traci.vehicle.getPosition(vehicle_id)
                speed = traci.vehicle.getSpeed(vehicle_id)
                lane_id = traci.vehicle.getLaneID(vehicle_id)
                heading = traci.vehicle.getAngle(vehicle_id)
            except traci.TraCIException:
                continue
            self._publish_sensor(vehicle_id, x, y, speed, lane_id, heading)
            self._apply_actuators(vehicle_id)
        self._update_gui(list(vehicle_ids))

    def run(self) -> None:
        self.connect()
        try:
            self.start_sumo()
            while True:
                try:
                    self.step()
                except traci.exceptions.FatalTraCIError as exc:
                    print(f"SUMO closed TraCI connection: {exc}")
                    break
                except traci.TraCIException:
                    if self.loop_sim:
                        traci.close()
                        if self.loop_pause_s > 0:
                            time.sleep(self.loop_pause_s)
                        self.start_sumo()
                        continue
                    break
                if traci.simulation.getMinExpectedNumber() == 0:
                    if self.loop_sim:
                        traci.close()
                        if self.loop_pause_s > 0:
                            time.sleep(self.loop_pause_s)
                        self.start_sumo()
                        continue
                    break
                if self.sumo_end_s is not None and traci.simulation.getTime() >= self.sumo_end_s:
                    if self.loop_sim:
                        traci.close()
                        if self.loop_pause_s > 0:
                            time.sleep(self.loop_pause_s)
                        self.start_sumo()
                        continue
                    break
                if self.step_delay_s > 0:
                    time.sleep(self.step_delay_s)
        finally:
            traci.close()
            self.client.loop_stop()


def main() -> None:
    bridge = TraciBridge()
    bridge.run()


if __name__ == "__main__":
    main()
