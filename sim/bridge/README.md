# TraCI MQTT Bridge

This service runs SUMO via TraCI and publishes each vehicle's state to the shared MQTT broker. It also consumes actuation commands and applies them to SUMO.

## Environment variables

- `SUMO_CFG`: Path to the SUMO config file inside the container. Default: `/data/sim.sumocfg`
- `SUMO_GUI`: `true` to use `sumo-gui`, `false` for headless. Default: `false`
- `STEP_LENGTH`: SUMO step length in seconds. Default: `0.1`
- `STEP_DELAY_S`: Optional sleep after each step. Default: `0`
- `SUMO_END`: Optional override for SUMO end time in seconds.
- `LOOP_SIM`: `true` to restart SUMO when all vehicles are done.
- `LOOP_PAUSE_S`: Optional pause between loop restarts.
- `MQTT_HOST` / `MQTT_PORT`: MQTT broker address. Default: `mqtt-broker:1883`
- `VEHICLE_IDS`: Optional comma list of vehicle IDs to publish. Default: all vehicles
- `SENSOR_TOPIC_FMT`: Topic format for sensors. Default: `car/{vehicle_id}/sensors/gps`
- `ACT_SPEED_TOPIC_FMT`: Topic format for speed commands. Default: `car/{vehicle_id}/actuators/speed`
- `ACT_LANE_TOPIC_FMT`: Topic format for lane commands. Default: `car/{vehicle_id}/actuators/lane`
