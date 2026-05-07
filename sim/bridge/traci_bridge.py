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
MERGE_POINT_COLOR = (255, 200, 40, 180)


def env(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value is not None and value != "" else default


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
        self.sumo_end = env("SUMO_END", "")
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

        self.gui_markers = env("GUI_MARKERS", "true").lower() == "true"
        self.gui_track_vehicle = env("GUI_TRACK_VEHICLE", "Merge_Car")
        self.gui_zoom = float(env("GUI_ZOOM", "1800"))
        self.gui_marker_radius = float(env("GUI_MARKER_RADIUS", "9"))
        self.gui_badge_size = float(env("GUI_BADGE_SIZE", "5"))
        self.gui_merge_point = env("GUI_MERGE_POINT", "true").lower() == "true"
        self.merge_point_x = float(env("MERGE_POINT_X", "0"))
        self.merge_point_y = float(env("MERGE_POINT_Y", "0"))
        self.gui_view_id: Optional[str] = None
        self.gui_tracked_vehicle: Optional[str] = None

        self.speed_commands: Dict[str, float] = {}
        self.lane_commands: Dict[str, int] = {}
        self.speed_mode_commands: Dict[str, int] = {}
        self.fsm_status: Dict[str, Dict[str, Any]] = {}
        self.marker_ids: set[str] = set()
        self.badge_ids: set[str] = set()

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
        self._configure_gui()

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
        try:
            traci.gui.setZoom(self.gui_view_id, self.gui_zoom)
        except traci.TraCIException:
            pass
        if self.gui_merge_point:
            self._set_polygon(
                "merge-point-marker",
                self._circle_shape(self.merge_point_x, self.merge_point_y, self.gui_marker_radius * 1.35),
                MERGE_POINT_COLOR,
                MERGE_POINT_LAYER,
                "merge-point",
            )

    def _publish_sensor(self, vehicle_id: str, x: float, y: float, speed: float, lane_id: str) -> None:
        payload = {
            "x": x,
            "y": y,
            "speed": speed,
            "lane_id": lane_id,
            "timestamp": time.time(),
        }
        topic = self.sensor_topic_fmt.format(vehicle_id=vehicle_id)
        self.client.publish(topic, json.dumps(payload))

    def _apply_actuators(self, vehicle_id: str) -> None:
        if vehicle_id in self.speed_commands:
            target_speed = self.speed_commands[vehicle_id]
            if self.speed_command_duration_s > 0:
                traci.vehicle.slowDown(vehicle_id, target_speed, self.speed_command_duration_s)
            else:
                traci.vehicle.setSpeed(vehicle_id, target_speed)
        if vehicle_id in self.lane_commands:
            target_lane = self.lane_commands[vehicle_id]
            lane_id = traci.vehicle.getLaneID(vehicle_id)
            edge_id = lane_id.rsplit("_", 1)[0]
            if target_lane < traci.edge.getLaneNumber(edge_id):
                traci.vehicle.changeLane(vehicle_id, target_lane, 1.0)
        if vehicle_id in self.speed_mode_commands:
            traci.vehicle.setSpeedMode(vehicle_id, self.speed_mode_commands[vehicle_id])

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

    def _set_polygon(
        self,
        polygon_id: str,
        shape: list[tuple[float, float]],
        color: tuple[int, int, int, int],
        layer: int,
        polygon_type: str,
    ) -> None:
        try:
            if polygon_id in traci.polygon.getIDList():
                traci.polygon.setShape(polygon_id, shape)
                traci.polygon.setColor(polygon_id, color)
            else:
                traci.polygon.add(polygon_id, shape, color, True, polygonType=polygon_type, layer=layer)
        except traci.TraCIException:
            pass

    def _remove_polygon(self, polygon_id: str) -> None:
        for layer in (MARKER_LAYER, BADGE_LAYER, MERGE_POINT_LAYER, 0):
            try:
                traci.polygon.remove(polygon_id, layer=layer)
                return
            except traci.TraCIException:
                continue

    def _role_color(self, vehicle_id: str) -> tuple[int, int, int, int]:
        status = self.fsm_status.get(vehicle_id, {})
        role = str(status.get("effective_role") or status.get("role") or "").lower()
        return ROLE_COLORS.get(role, DEFAULT_ROLE_COLOR)

    def _state_color(self, vehicle_id: str) -> tuple[int, int, int, int]:
        status = self.fsm_status.get(vehicle_id, {})
        state = str(status.get("fsm_state") or "").upper()
        return STATE_COLORS.get(state, DEFAULT_STATE_COLOR)

    def _update_gui(self, vehicle_ids: list[str]) -> None:
        if not self.sumo_gui or not self.gui_markers:
            return
        active = set(vehicle_ids)
        for polygon_id in list(self.marker_ids):
            vehicle_id = polygon_id.removeprefix("vehicle-marker-")
            if vehicle_id not in active:
                self._remove_polygon(polygon_id)
                self.marker_ids.discard(polygon_id)
        for polygon_id in list(self.badge_ids):
            vehicle_id = polygon_id.removeprefix("fsm-badge-")
            if vehicle_id not in active:
                self._remove_polygon(polygon_id)
                self.badge_ids.discard(polygon_id)

        tracked_vehicle = self.gui_track_vehicle if self.gui_track_vehicle in active else (vehicle_ids[0] if vehicle_ids else None)
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
                state_color = self._state_color(vehicle_id)
                traci.vehicle.setColor(vehicle_id, state_color)
            except traci.TraCIException:
                continue
            marker_id = f"vehicle-marker-{vehicle_id}"
            badge_id = f"fsm-badge-{vehicle_id}"
            self._set_polygon(
                marker_id,
                self._circle_shape(x, y, self.gui_marker_radius),
                self._role_color(vehicle_id),
                MARKER_LAYER,
                "vehicle-marker",
            )
            self.marker_ids.add(marker_id)
            self._set_polygon(
                badge_id,
                self._badge_shape(x, y),
                state_color,
                BADGE_LAYER,
                "fsm-badge",
            )
            self.badge_ids.add(badge_id)

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
            except traci.TraCIException:
                continue
            self._publish_sensor(vehicle_id, x, y, speed, lane_id)
            self._apply_actuators(vehicle_id)
        self._update_gui(list(vehicle_ids))

    def run(self) -> None:
        self.connect()
        try:
            self.start_sumo()
            while True:
                try:
                    self.step()
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
