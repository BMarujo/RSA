# TraCI MQTT Bridge

This service runs SUMO via TraCI and publishes each vehicle's state to the shared MQTT broker. It also consumes actuation commands and applies them to SUMO.

## Environment variables

- `SUMO_CFG`: Path to the SUMO config file inside the container. Default: `/data/sim.sumocfg`
- `SUMO_GUI`: `true` to use `sumo-gui`, `false` for headless. Default: `false`
- `STEP_LENGTH`: SUMO step length in seconds. Default: `0.1`
- `STEP_DELAY_S`: Optional sleep after each step. Docker default: `0.02`
- `SUMO_END`: Optional override for SUMO end time in seconds.
- `SUMO_EXTRA_ARGS`: Extra arguments appended to the SUMO command. Useful for result files such as `--tripinfo-output /results/tripinfo.xml --summary-output /results/summary.xml`.
- `SPEED_COMMAND_DURATION_S`: Duration used by TraCI `slowDown` when applying speed targets. Docker default: `2.0`.
- `LANE_CHANGE_DURATION_S`: Duration passed to TraCI `changeLane` for merge commands. Docker default: `3.0`.
- `LANE_CHANGE_COOLDOWN_S`: Minimum time before repeating the same lane-change command. Docker default: `2.0`.
- `LOOP_SIM`: `true` to restart SUMO when all vehicles are done.
- `LOOP_PAUSE_S`: Optional pause between loop restarts.
- `GUI_MARKERS`: `true` to draw role/state overlays in `sumo-gui`. Default: `true`
- `GUI_TRACK_VEHICLE`: Optional vehicle ID followed by the SUMO-GUI camera. Empty disables auto-follow. Default: `Merge_Car`
- `GUI_FIT_NETWORK`: `true` to fit the full network in SUMO-GUI at startup. Default: `false`
- `GUI_BOUNDARY_PADDING`: Padding in meters around the fitted network view. Default: `80`
- `GUI_ZOOM`: SUMO-GUI zoom level used when `GUI_FIT_NETWORK=false` or tracking a vehicle. Default: `1800`
- `GUI_MARKER_RADIUS`: Radius in meters of the role circle around each vehicle. Default: `9`
- `GUI_BADGE_SIZE`: Size in meters of the FSM state badge next to each vehicle. Default: `5`
- `GUI_MERGE_POINT`: `true` to draw the merge-zone marker. Default: `true`
- `GUI_MERGE_ZONE_LENGTH`: Length in meters for each merge-zone gate bar. Default: `13`
- `GUI_MERGE_ZONE_WIDTH`: Width in meters for each merge-zone gate bar. Default: `1.1`
- `GUI_MERGE_ZONE_GAP`: Distance in meters from the merge point to each gate bar. Default: `5.5`
- `GUI_MERGE_ZONE_ANGLE_DEG`: Orientation of the merge-zone gate bars. Default: `0`
- `GUI_VEHICLE_SKINS`: `true` to draw custom top-down vehicle skins over SUMO vehicles. Default: `true`
- `GUI_VEHICLE_SKIN_SCALE`: Scale factor for custom vehicle skins. Default: `1.18`
- `GUI_VEHICLE_SKIN_DETAIL`: `true` to add extra headlights and antenna polygons. Default: `false`
- `GUI_DIM_SUMO_VEHICLES`: `true` to fade the native SUMO vehicle bodies underneath the custom skins. Default: `true`
- `GUI_ROLE_MARKERS`: `true` to draw the large legacy role circle around each vehicle. Default: `false`
- `GUI_STATE_BADGES`: `true` to draw the legacy FSM badge next to each vehicle. Default: `false`
- `GUI_STATE_BODY_TINT`: `true` to subtly tint the custom vehicle body by active FSM state. Default: `false`
- `GUI_STATE_BODY_TINT_AMOUNT`: Amount of state tint mixed into the body color. Default: `0.34`
- `GUI_STATE_INDICATOR_WIDTH`: Width multiplier for the state stripe on the custom skin. Default: `0.22`
- `GUI_STATE_ROOF`: `true` to draw an extra roof state patch on the custom skin. Default: `false`
- `GUI_SHOW_CRUISE_STATE`: `true` to show `CRUISE` as a state color. Default: `false`
- `GUI_COLOR_VEHICLES_BY_STATE`: `true` to overwrite scenario vehicle colors with FSM-state colors. Default: `false`
- `GUI_BRAKE_LIGHTS`: `true` to draw red brake-light overlays while a car decelerates/yields. Default: `true`
- `GUI_BRAKE_LIGHT_SIZE`: Brake-light overlay size in meters. Default: `1.15`
- `GUI_BRAKE_DECEL_THRESHOLD`: Speed drop per bridge step that activates brake lights. Default: `0.25`
- `MERGE_POINT_X` / `MERGE_POINT_Y`: SUMO coordinates for the merge-point marker.
- `MQTT_HOST` / `MQTT_PORT`: MQTT broker address. Default: `mqtt-broker:1883`
- `VEHICLE_IDS`: Optional comma list of vehicle IDs to publish. Default: all vehicles
- `SENSOR_TOPIC_FMT`: Topic format for sensors. Default: `car/{vehicle_id}/sensors/gps`
- `ACT_SPEED_TOPIC_FMT`: Topic format for speed commands. Default: `car/{vehicle_id}/actuators/speed`
- `ACT_LANE_TOPIC_FMT`: Topic format for lane commands. Default: `car/{vehicle_id}/actuators/lane`

## SUMO-GUI overlays

The bridge subscribes to `car/<vehicle_id>/status/fsm`, published by each OBU, and uses it to draw a cleaner showcase layer in SUMO-GUI:

- Custom vehicle skin: top-down body, glass, and clean role/active-state stripe.
- Brake lights: red rear lights while a car decelerates, yields, or aborts.
- Optional large circle: detected vehicle role (`merge`, `host`, `lead`) when `GUI_ROLE_MARKERS=true`.
- Optional small square next to the vehicle: FSM state when `GUI_STATE_BADGES=true`.
- Native SUMO body color: scenario color unless `GUI_COLOR_VEHICLES_BY_STATE=true`.
- Subtle yellow gate bars: configured merge zone.
