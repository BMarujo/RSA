# Project Handoff: Mind the Gap - Autonomous V2X Lane Merging

This file is a working memory snapshot for starting a new conversation about this project.
It summarizes the architecture, current implementation, debugging history, known issues,
useful commands, and next steps.

## High-Level Goal

The project is a Software-in-the-Loop simulation for decentralized autonomous lane merging
using ETSI C-ITS/V2X messages. The core idea is that each vehicle has an isolated OBU
container with:

- Vanetza socktap for ETSI message encode/decode and V2X network I/O.
- A Python OBU brain for local sensing, traffic-map building, FSM logic, and actuation.
- An embedded local Mosquitto broker acting like the vehicle internal bus.

SUMO is the ground-truth physical world. A TraCI bridge publishes each vehicle's own
sensor data to MQTT and consumes actuator commands from each OBU.

The design rule is important: **an OBU must not read another vehicle's SUMO sensor topic**.
Other vehicles must be known through Vanetza-decoded CAM/MCM messages.

## Repository Shape

Important paths:

- `docker-compose.yml`
  - Starts shared MQTT and the SUMO TraCI bridge.
- `scripts/run_vanetza_scenario.sh`
  - Generates a Compose override with one OBU container per explicit SUMO vehicle
    in the route file and starts the full stack.
- `scripts/generate_obu_compose.py`
  - Parses `SUMO_CFG` / route XML and writes `.generated/vanetza-obus.compose.yml`.
- `sim/bridge/traci_bridge.py`
  - Starts SUMO/SUMO-GUI.
  - Publishes sensors to `car/<vehicle_id>/sensors/gps`.
  - Subscribes to `car/+/actuators/*`.
  - Subscribes to `car/+/status/#` for visualization.
  - Applies speed, lane, and speed-mode commands to SUMO.
- `obu/app/main.py`
  - Main OBU brain.
  - Subscribes to own sensors and Vanetza decoded output.
  - Publishes CAM/MCM/DENM requests to Vanetza.
  - Publishes actuator commands and status.
- `obu/config/vanetza-obu.ini`
  - Vanetza socktap config.
  - CAM/MCM/DENM MQTT topics are enabled.
- `obu/entrypoint.sh`
  - Starts embedded Mosquitto.
  - Bridges only own sensor/actuator/status topics.
  - Starts socktap and the Python OBU app.
- `sumo-lane-merge/aveiro_map`
  - Aveiro SUMO map.
  - `vanetza.sumocfg` and `vanetza.rou.xml` are used by the Docker/Vanetza flow.
  - `vanetza.rou.xml` is now the default focused local dense merge demo.
  - `vanetza_scenarios/` contains focused variants for the same merge area:
    `base`, `gap`, `dense`, `ramp-platoon`, `blocked`, and a separate
    `single-lane` one-lane merge.
- `sumo-lane-merge/scripts/run_traci.py`
  - Standalone TraCI demo with scenario logic, useful as a reference/demo path.

## Current Architecture

Shared broker:

- Container: `mqtt-broker`
- Port: `1883`
- Main global topics:
  - `car/<vehicle_id>/sensors/gps`
  - `car/<vehicle_id>/actuators/speed`
  - `car/<vehicle_id>/actuators/lane`
  - `car/<vehicle_id>/actuators/speed_mode`
  - `car/<vehicle_id>/status/fsm`

Per-OBU local brokers:

- Each generated OBU has its own embedded Mosquitto broker inside the container.
- Local OBU brokers are no longer mapped to fixed host ports by default, because
  the number of OBUs follows the SUMO vehicle count.

Local Vanetza topics inside each OBU:

- OBU app -> Vanetza:
  - `vanetza/in/cam`
  - `vanetza/in/mcm`
  - `vanetza/in/denm`
- Vanetza -> OBU app:
  - `vanetza/out/cam`
  - `vanetza/out/mcm`
  - `vanetza/out/denm`

Mosquitto bridge rules in each OBU:

- Remote -> local:
  - `car/<vehicle_id>/sensors/#`
- Local -> remote:
  - `car/<vehicle_id>/actuators/#`
  - `car/<vehicle_id>/status/#`

This preserves isolation: each OBU receives only its own SUMO sensor data.

## Aveiro Map Setup

The Docker stack currently uses:

```text
/data/sumo-lane-merge/aveiro_map/vanetza.sumocfg
```

Routes:

- Main route: `560761994 1331698336 135424828`
- Ramp route: `34126779 1331698336 135424828`

Merge-point values currently used in Docker env:

```text
MERGE_POINT_X=194.89
MERGE_POINT_Y=2212.42
MERGE_LANE_INDEX=1
RAMP_EDGE_IDS=34126779
MAIN_EDGE_IDS=560761994,1331698336,135424828
RAMP_BBOX=205,2120,340,2225
ROLE_DETECTION_DISTANCE=260.0
```

`MERGE_LANE_INDEX=1` matters because the Aveiro lane connection does not work with
the earlier synthetic map's lane assumptions.

## OBU Brain Logic

The OBU app has three main information sources:

1. Own sensor state from `car/<vehicle_id>/sensors/gps`.
2. Neighbor CAMs from `vanetza/out/cam`.
3. Maneuver messages from `vanetza/out/mcm`.

It publishes:

1. CAMs to `vanetza/in/cam`.
2. MCMs to `vanetza/in/mcm`.
3. DENMs to `vanetza/in/denm` if enabled.
4. Actuator commands to `car/<vehicle_id>/actuators/*`.
5. Status to `car/<vehicle_id>/status/fsm`.

### Traffic Map

Received Vanetza CAMs are decoded from the Vanetza-NAP envelope:

- CAM payload is under `fields.cam`.
- Header/station ID is under `fields.header.stationId`.

The app converts CAM latitude/longitude back to SUMO XY using the configured origin:

```text
ORIGIN_LAT=40.0
ORIGIN_LON=-8.0
```

Neighbors are stored by ETSI station ID and expire after `NEIGHBOR_TIMEOUT_S`.

### ETA Calculation

The main coordination variable is ETA to the merge point:

```text
distance_to_merge = hypot(merge_x - x, merge_y - y)
eta = distance_to_merge / max(speed, 0.1)
```

The project intentionally avoids pure absolute-distance rules and uses temporal
headway at the merge point.

### Role Detection

When `ROLE_MODE=auto`:

- A vehicle on the ramp edge or ramp bounding box becomes `merge`.
- Main-lane vehicles detect a ramp/merge candidate from Vanetza CAMs.
- Main-lane roles are assigned by ETA relative to the merge candidate:
  - lower ETA than merge candidate -> `lead`
  - higher ETA than merge candidate -> `host`
  - exact tie -> lower station ID wins

### FSM States

Current states:

- `CRUISE`
- `NEGOTIATING`
- `YIELDING`
- `MERGING`
- `ABORT`

### Merge Vehicle Behavior

The merge vehicle:

1. Computes own ETA to the merge point.
2. Finds a lead candidate arriving before it.
3. Finds a host candidate arriving after it.
4. Computes the safe temporal window:

```text
min_eta = lead_eta + SAFE_HEADWAY_S
max_eta = host_eta - SAFE_HEADWAY_S
```

5. Adjusts speed to fit inside the ETA window.
6. Sends MCM request when near the merge zone.
7. If gap is safe AND host sent MCM ACCEPT, sends lane/speed commands to merge.
8. If REJECT received or negotiation times out, enters ABORT with a cooldown before
   retrying. The merge car slows down and waits for conditions to change.

Docker config now uses `MERGE_PRIORITY=false` (strict negotiation). The merge car
will NOT force its way in without an ACCEPT.

### Host Behavior

The host vehicle:

1. Detects the merge candidate via CAMs.
2. Computes the merge candidate ETA.
3. Checks if yielding is safe (distance to merge > `HOST_REJECT_DISTANCE_M`).
4. If too close to merge point, sends MCM REJECT.
5. Otherwise, reduces speed to open a safe gap.
6. Sends MCM ACCEPT periodically.

### Lead Behavior

The lead vehicle:

1. Detects a merge candidate behind/near it.
2. If the merge is close, accelerates enough to clear the merge zone.
3. Uses priority speed mode so SUMO does not block the maneuver too much.

## Vanetza/MQTT Message Notes

Vanetza-NAP decoded envelopes look like this:

- CAM:
  - `fields.header`
  - `fields.cam`
- MCM:
  - `fields.header`
  - `fields.payload`

The OBU app supports both plain payloads and Vanetza envelopes.

Important recent MCM debugging:

- We saw repeated:

```text
Can't determine size for unaligned PER encoding of type MCM because of ManoeuvreId sub-type
--- Vanetza UPER Encoding Error ---
```

- The current project-side fix is in `obu/app/main.py`:

```python
MAX_MANOEUVRE_ID = 255
```

Reason: MCM `ManoeuvreId` is based on `Identifier1B`, so it should stay in `0..255`.

In the latest smoke test:

- `vanetza/in/mcm` was observed on the merge OBU.
- `vanetza/out/mcm` was observed on the merge OBU with decoded ACCEPTs from stations 101/102.
- Logs did not show `UPER`, `Encoding Error`, `Can't determine`, or `Invalid payload`.

If the UPER spam returns, first rebuild the OBU images:

```bash
scripts/run_vanetza_scenario.sh bridge
```

Then run headless and inspect:

```bash
SUMO_GUI=false LOOP_SIM=false SUMO_END=80 STEP_DELAY_S=0 \
scripts/run_vanetza_scenario.sh bridge

docker compose -f docker-compose.yml -f .generated/vanetza-obus.compose.yml logs --no-color | \
  rg "UPER|Encoding Error|Can't determine|Invalid payload"
```

## SUMO-GUI Visualization

The TraCI bridge can visualize vehicle state in SUMO-GUI:

- Tracks `Merge_Car` by default to keep the presentation focused on the merge area.
- Set `GUI_FIT_NETWORK=true GUI_TRACK_VEHICLE=` to see the full map.
- Draws custom top-down vehicle skins over the native SUMO vehicles.
- The default custom skins are lightweight: body, glass, and a clean
  role/active-state stripe. Extra headlights and antenna polygons are optional
  with `GUI_VEHICLE_SKIN_DETAIL=true`.
- Legacy role circles and FSM badges are optional and disabled by default for a
  cleaner demo view.
- Keeps scenario vehicle body colors by default, so the presentation stays
  visually readable.
- Set `GUI_COLOR_VEHICLES_BY_STATE=true` if you want vehicle bodies overwritten
  by FSM-state colors.
- Draws brake-light overlays while cars decelerate, yield, or abort.
- Draws a subtle merge-zone gate instead of a circular merge-point marker.

Relevant env vars:

```text
GUI_MARKERS=true
GUI_TRACK_VEHICLE=Merge_Car
GUI_FIT_NETWORK=false
GUI_BOUNDARY_PADDING=80
GUI_MERGE_ZONE_LENGTH=13
GUI_MERGE_ZONE_WIDTH=1.1
GUI_MERGE_ZONE_GAP=5.5
GUI_MERGE_ZONE_ANGLE_DEG=0
GUI_VEHICLE_SKINS=true
GUI_DIM_SUMO_VEHICLES=true
GUI_VEHICLE_SKIN_DETAIL=false
GUI_ROLE_MARKERS=false
GUI_STATE_BADGES=false
GUI_STATE_BODY_TINT=false
GUI_STATE_BODY_TINT_AMOUNT=0.34
GUI_STATE_INDICATOR_WIDTH=0.22
GUI_STATE_ROOF=false
GUI_SHOW_CRUISE_STATE=false
GUI_COLOR_VEHICLES_BY_STATE=false
GUI_BRAKE_LIGHTS=true
GUI_ZOOM=1800
GUI_MARKER_RADIUS=9
GUI_BADGE_SIZE=5
GUI_MERGE_POINT=true
```

For GUI runs, Docker needs X11 authorization:

```bash
xhost +local:docker
SUMO_GUI=true scripts/run_vanetza_scenario.sh up
```

For reliable tests, prefer headless:

```bash
SUMO_GUI=false LOOP_SIM=false SUMO_END=80 STEP_DELAY_S=0 \
scripts/run_vanetza_scenario.sh bridge
```

## Compose Important Detail

`docker-compose.yml` must allow runtime env overrides. Current desired form:

```yaml
SUMO_CFG=${SUMO_CFG:-/data/sumo-lane-merge/aveiro_map/vanetza.sumocfg}
SUMO_GUI=${SUMO_GUI:-true}
DISPLAY=${DISPLAY:-:0}
STEP_DELAY_S=${STEP_DELAY_S:-0.02}
LOOP_SIM=${LOOP_SIM:-true}
SUMO_END=${SUMO_END:-120}
SUMO_EXTRA_ARGS=${SUMO_EXTRA_ARGS:-}
```

If these are hardcoded again, headless runs may accidentally start `sumo-gui`
and fail with:

```text
Authorization required, but no authorization protocol specified
FXApp::openDisplay: unable to open display :0
```

Merge-control tuning now avoids hard stops and abrupt lane jumps:

- `PRIORITY_SPEED_MODE` defaults to `31` instead of unsafe `0`.
- `MAX_SPEED_STEP_UP` / `MAX_SPEED_STEP_DOWN` smooth OBU speed targets.
- `MERGE_COMMIT_DISTANCE` lets the ramp vehicle commit once close enough and
  clear enough, avoiding two-car yield deadlocks.
- `HOST_YIELD_FLOOR_RATIO` / `MERGE_YIELD_FLOOR_RATIO` keep vehicles rolling.
- `LANE_CHANGE_DURATION_S` / `LANE_CHANGE_COOLDOWN_S` prevent repeated abrupt
  TraCI lane-change commands.
- `RAMP_PLATOON_HEADWAY_S` / `RAMP_PLATOON_MIN_GAP` keep ramp vehicles spaced
  before the merge, allowing a larger ramp queue without all cars committing at
  once.

## Useful Debug Commands

Shared broker:

```bash
docker run --rm --network host eclipse-mosquitto:2 \
  mosquitto_sub -h 127.0.0.1 -p 1883 -t 'car/+/status/#' -v

docker run --rm --network host eclipse-mosquitto:2 \
  mosquitto_sub -h 127.0.0.1 -p 1883 -t 'car/+/actuators/#' -v
```

Generated OBU local Vanetza topics are not mapped to fixed host ports by default.
For most debugging, inspect all generated OBU logs:

```bash
docker compose -f docker-compose.yml -f .generated/vanetza-obus.compose.yml logs --no-color | \
  rg "vanetza/(in|out)|MCM|CAM|DENM|UPER|Encoding Error"
```

Run the main dynamic stack:

```bash
scripts/run_vanetza_scenario.sh up
```

Choose a focused scenario:

```bash
scripts/run_vanetza_scenario.sh scenarios
VANETZA_SCENARIO=gap scripts/run_vanetza_scenario.sh up
```

Generate only the dynamic OBU override:

```bash
scripts/run_vanetza_scenario.sh generate
```

Run the generated OBUs in the background and the TraCI bridge as a one-shot
headless process:

```bash
SUMO_GUI=false LOOP_SIM=false SUMO_END=80 STEP_DELAY_S=0 \
scripts/run_vanetza_scenario.sh bridge
```

Stop everything:

```bash
scripts/run_vanetza_scenario.sh down
```

## Results and Metrics

The TraCI bridge supports `SUMO_EXTRA_ARGS`, so result XMLs can be extracted:

```bash
SUMO_GUI=false LOOP_SIM=false SUMO_END=120 STEP_DELAY_S=0 \
SUMO_EXTRA_ARGS='--summary-output /results/summary.xml --tripinfo-output /results/tripinfo.xml --collision-output /results/collisions.xml --fcd-output /results/fcd.xml' \
docker compose run --rm traci-bridge
```

Useful result files:

- `summary.xml`
  - collisions per step
  - teleports
  - mean speed
  - halting vehicles
- `tripinfo.xml`
  - travel time
  - route length
  - time loss
- `collisions.xml`
  - safety proof / collision details
- `fcd.xml`
  - per-step vehicle positions and speeds
  - useful for plotting speed curves and trajectories

MQTT logs are also useful for presentation:

- `car/+/status/fsm`
  - FSM state, role, ETA, distance to merge, neighbor count, target speed/lane.
- `car/+/actuators/#`
  - commands requested by each OBU.
- `vanetza/in/#`
  - what the OBU asks Vanetza to transmit.
- `vanetza/out/#`
  - what Vanetza decoded from V2X.

## Scenario Work

The standalone `sumo-lane-merge/scripts/run_traci.py` has mature demo scenarios:

- `base`
- `gap`
- `adaptive`
- `loss`

There was also experimental work to create Vanetza/Aveiro scenario files:

- `sumo-lane-merge/scripts/generate_vanetza_scenarios.py`
- possible generated folder:
  - `sumo-lane-merge/aveiro_map/vanetza_scenarios/`

If continuing this, check whether those files exist and whether they are tracked.
The root repo currently has only `docker-compose.yml` and `obu/app/main.py` modified
at the time this handoff file was written.

## Known Current Caveats

1. The Aveiro role detection can become odd after vehicles leave the ramp and reach
   common downstream lanes. Status may show `Merge_Car` as `host` later in the run.
   This is not necessarily a V2X failure; it is the auto role logic re-evaluating
   after the maneuver zone.

2. MCM currently uses a large Vanetza example/template payload. It works in the latest
   smoke test, but it is semantically heavy. A future cleanup should make request and
   response payloads clearer and smaller if Vanetza accepts them.

3. `MERGE_PRIORITY=false` enforces strict negotiation. To test the older permissive
   mode, set `MERGE_PRIORITY=true`.

4. `COLLISION_GUARD=false` by default so vehicle behavior is solely V2X-driven.
   Re-enable with `COLLISION_GUARD=true` if you need a safety net during development.

4. Do not deeply edit `vanetza-nap` unless needed. Most bugs so far were in our payload
   values, Docker environment, or compose/runtime setup.

5. Some generated result files under `results/` may be root-owned because Docker wrote
   them. If cleanup fails, use `sudo rm -rf results/...`.

## Recent Verified State

Last meaningful checks:

```bash
python3 -m py_compile scripts/generate_obu_compose.py obu/app/main.py sim/bridge/traci_bridge.py
scripts/run_vanetza_scenario.sh config
```

Smoke run:

```bash
SUMO_GUI=false LOOP_SIM=false SUMO_END=80 STEP_DELAY_S=0 \
scripts/run_vanetza_scenario.sh bridge
```

Observed:

- SUMO ran headless.
- MCM appeared on `vanetza/in/mcm`.
- Decoded MCM ACCEPTs appeared on `vanetza/out/mcm`.
- No UPER/encoding spam in OBU logs after rebuild.

## Recommended Next Steps

1. Commit or stash the current clean fix:
   - `docker-compose.yml` env overrides.
   - `MAX_MANOEUVRE_ID = 255`.
   - Dynamic OBU generation through `scripts/run_vanetza_scenario.sh`.

2. Run one controlled result extraction:

```bash
SUMO_GUI=false LOOP_SIM=false SUMO_END=120 STEP_DELAY_S=0 \
SUMO_EXTRA_ARGS='--summary-output /results/summary.xml --tripinfo-output /results/tripinfo.xml --collision-output /results/collisions.xml --fcd-output /results/fcd.xml' \
scripts/run_vanetza_scenario.sh bridge
```

3. Inspect:

```bash
docker compose -f docker-compose.yml -f .generated/vanetza-obus.compose.yml logs --no-color | \
  rg "UPER|Encoding Error|Can't determine|Invalid payload"
```

4. Build a small plotting/analyzer script for:
   - FSM state over time.
   - target speed vs actual speed.
   - MCM request/accept timestamps.
   - collision count.
   - min time headway.

5. Improve MCM semantics:
   - Separate request vs accept payload intent.
   - Include ETA/headway/gap data in a structured application-level field if possible.
   - Keep ETSI fields valid and bounded.

6. Expand scenarios after the MCM path is stable.
