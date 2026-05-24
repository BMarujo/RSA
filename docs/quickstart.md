# Quickstart (SITL)

1. Build and run the dynamic SITL stack:

```
scripts/run_vanetza_scenario.sh up
```

The launcher reads the SUMO route file referenced by `SUMO_CFG` and generates one
OBU container per explicit SUMO `<vehicle>`.
The default Aveiro Vanetza scenario is `dense`: 12 vehicles at `t=0`, all focused
on the same merge area. Other focused scenarios are available with
`VANETZA_SCENARIO=base|gap|dense|ramp-platoon|blocked|single-lane|host-acceptance`.

List scenarios:

```
scripts/run_vanetza_scenario.sh scenarios
```

Run a specific scenario:

```
VANETZA_SCENARIO=gap scripts/run_vanetza_scenario.sh up
```

Run the clean GUI acceptance demo:

```
VANETZA_SCENARIO=host-acceptance SUMO_GUI=true LOOP_SIM=false scripts/run_vanetza_scenario.sh up
```

The generator refuses more than 40 OBUs by default; override with
`MAX_OBU_SERVICES` if needed.

2. Observe CAM messages on the shared broker:

```
mosquitto_sub -h 127.0.0.1 -t "+/vanetza/out/cam" -v
```

3. Inspect SUMO actuator topics:

```
mosquitto_sub -h 127.0.0.1 -t "car/+/actuators/#" -v
```

Notes:

- The OBU app publishes CAMs via the local broker to Vanetza.
- The SUMO bridge publishes sensors to the remote broker.
- The embedded mosquitto bridges only the per-vehicle sensor/actuator topics.
- SUMO-GUI uses a fixed merge-area view for the dense scenarios; the
  `host-acceptance` demo tracks `Merge_Car` to keep the presentation focused.
  Set `GUI_FIT_NETWORK=true GUI_TRACK_VEHICLE=` to see the whole map.
- SUMO-GUI draws custom top-down vehicle skins by default. Set
  `GUI_VEHICLE_SKINS=false GUI_ROLE_MARKERS=true GUI_STATE_BADGES=true` to go
  back to the older overlay-heavy view.
- Active FSM states are highlighted by a clean stripe on the custom skin.
  `CRUISE` is not colored by default, keeping normal cars on their scenario
  colors.
- The merge zone is indicated by subtle gate bars instead of a yellow circle.
- Docker GUI runs use `STEP_DELAY_S=0.02` by default, which is faster than
  realtime. Use `STEP_DELAY_S=0.1` for realtime-ish playback or `STEP_DELAY_S=0`
  for fastest headless runs.
- To inspect the generated Compose file without starting containers, run
  `scripts/run_vanetza_scenario.sh generate`.
