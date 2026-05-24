# Architecture Notes

This project uses a local MQTT broker per OBU container to simulate an internal vehicle bus. The SUMO TraCI bridge publishes sensor data on a shared broker, and each OBU broker bridges only the topics for its vehicle. Vanetza socktap encodes and decodes ETSI CAM/MCM/DENM messages on the local broker.

The Docker stack uses the Aveiro OSM SUMO network in `sumo-lane-merge/aveiro_map` through focused Vanetza scenario configs under `sumo-lane-merge/aveiro_map/vanetza_scenarios`. The synthetic `sumo-lane-merge/map` network remains useful for smaller standalone tests. For the containerized V2X/SITL flow, `scripts/run_vanetza_scenario.sh` picks a focused scenario with `VANETZA_SCENARIO`, reads the referenced route file, and generates a Compose override with one OBU service for each explicit SUMO vehicle. The `host-acceptance` scenario is the clean GUI demonstration: one ramp vehicle on the Aveiro acceleration lane, one named accepting host, and one lead vehicle.

When `ROLE_MODE=auto`, the OBU app treats roles as maneuver-relative instead of fixed identities. A vehicle on the configured ramp edge acts as the merge requester. Main-lane vehicles compare their ETA to the detected merge candidate's ETA: vehicles arriving earlier become lead vehicles, and vehicles arriving later become host/yield candidates. ETA ties are resolved by lower station ID.

The merge FSM separates authorization from physical lane-change start. A vehicle can be authorized and committed while the bridge reports `LANE_CMD_WAIT_EDGE`; it is not counted as physically merging until the target lane is executable or the lane command reaches `APPLY`/`CLEAR`.

See [mqtt-topic-contract.md](mqtt-topic-contract.md) for full topic mappings.
