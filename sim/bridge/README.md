# TraCI MQTT Bridge

This service runs SUMO via TraCI and publishes each vehicle's state to the shared MQTT broker. It also consumes actuation commands and applies them to SUMO.

## Environment variables

- `SUMO_CFG`: Path to the SUMO config file inside the container. Default: `/data/sim.sumocfg`
- `SUMO_GUI`: `true` to use `sumo-gui`, `false` for headless. Default: `false`
- `STEP_LENGTH`: SUMO step length in seconds. Default: `0.1`
- `STEP_DELAY_S`: Optional sleep after each step. Default: `0`
- `SUMO_END`: Optional override for SUMO end time in seconds.
- `SUMO_EXTRA_ARGS`: Extra arguments appended to the SUMO command. Useful for result files such as `--tripinfo-output /results/tripinfo.xml --summary-output /results/summary.xml`.
- `SPEED_COMMAND_DURATION_S`: Duration used by TraCI `slowDown` when applying speed targets. Default: `1.0`.
- `LOOP_SIM`: `true` to restart SUMO when all vehicles are done.
- `LOOP_PAUSE_S`: Optional pause between loop restarts.
- `GUI_MARKERS`: `true` to draw role/state overlays in `sumo-gui`. Default: `true`
- `GUI_TRACK_VEHICLE`: Vehicle ID followed by the SUMO-GUI camera. Default: `Merge_Car`
- `GUI_ZOOM`: SUMO-GUI zoom level while tracking. Default: `1800`
- `GUI_MARKER_RADIUS`: Radius in meters of the role circle around each vehicle. Default: `9`
- `GUI_BADGE_SIZE`: Size in meters of the FSM state badge next to each vehicle. Default: `5`
- `GUI_MERGE_POINT`: `true` to draw the merge-point marker. Default: `true`
- `MERGE_POINT_X` / `MERGE_POINT_Y`: SUMO coordinates for the merge-point marker.
- `MQTT_HOST` / `MQTT_PORT`: MQTT broker address. Default: `mqtt-broker:1883`
- `VEHICLE_IDS`: Optional comma list of vehicle IDs to publish. Default: all vehicles
- `SENSOR_TOPIC_FMT`: Topic format for sensors. Default: `car/{vehicle_id}/sensors/gps`
- `ACT_SPEED_TOPIC_FMT`: Topic format for speed commands. Default: `car/{vehicle_id}/actuators/speed`
- `ACT_LANE_TOPIC_FMT`: Topic format for lane commands. Default: `car/{vehicle_id}/actuators/lane`

## SUMO-GUI overlays

The bridge subscribes to `car/<vehicle_id>/status/fsm`, published by each OBU, and uses it to color the simulation:

- Large circle: detected vehicle role (`merge`, `host`, `lead`).
- Small square next to the vehicle: FSM state (`CRUISE`, `NEGOTIATING`, `YIELDING`, `MERGING`, `ABORT`).
- Vehicle body color: same color as the FSM state.
- Yellow circle: configured merge point.
