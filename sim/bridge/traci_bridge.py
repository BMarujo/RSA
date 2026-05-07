import json
import os
import sys
import time
from typing import Dict, Optional

import paho.mqtt.client as mqtt


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
        self.sumo_end = env("SUMO_END", "")
        self.loop_sim = env("LOOP_SIM", "false").lower() == "true"
        self.loop_pause_s = float(env("LOOP_PAUSE_S", "0"))

        self.mqtt_host = env("MQTT_HOST", "mqtt-broker")
        self.mqtt_port = int(env("MQTT_PORT", "1883"))

        self.vehicle_ids = [v for v in env("VEHICLE_IDS", "").split(",") if v]

        self.sensor_topic_fmt = env("SENSOR_TOPIC_FMT", "car/{vehicle_id}/sensors/gps")
        self.actuator_speed_fmt = env("ACT_SPEED_TOPIC_FMT", "car/{vehicle_id}/actuators/speed")
        self.actuator_lane_fmt = env("ACT_LANE_TOPIC_FMT", "car/{vehicle_id}/actuators/lane")
        self.actuator_speed_mode_fmt = env("ACT_SPEED_MODE_TOPIC_FMT", "car/{vehicle_id}/actuators/speed_mode")

        self.speed_commands: Dict[str, float] = {}
        self.lane_commands: Dict[str, int] = {}
        self.speed_mode_commands: Dict[str, int] = {}

        self.client = mqtt.Client(client_id="traci-bridge")
        self.client.on_message = self.on_message

    def connect(self) -> None:
        self.client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.client.subscribe("car/+/actuators/speed")
        self.client.subscribe("car/+/actuators/lane")
        self.client.subscribe("car/+/actuators/speed_mode")
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
        if parts[-1] == "speed":
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
        traci.start(cmd)

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
            traci.vehicle.setSpeed(vehicle_id, self.speed_commands[vehicle_id])
        if vehicle_id in self.lane_commands:
            traci.vehicle.changeLane(vehicle_id, self.lane_commands[vehicle_id], 1.0)
        if vehicle_id in self.speed_mode_commands:
            traci.vehicle.setSpeedMode(vehicle_id, self.speed_mode_commands[vehicle_id])

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
