# Architecture Notes

This project uses a local MQTT broker per OBU container to simulate an internal vehicle bus. The SUMO TraCI bridge publishes sensor data on a shared broker, and each OBU broker bridges only the topics for its vehicle. Vanetza socktap encodes and decodes ETSI CAM/MCM/DENM messages on the local broker.

The Docker stack uses the Aveiro OSM SUMO network in `sumo-lane-merge/aveiro_map` through `vanetza.sumocfg`. The synthetic `sumo-lane-merge/map` network remains useful for smaller standalone tests; the Aveiro `vanetza.sumocfg` adds fixed `Lead_Car`, `Host_Car`, and `Merge_Car` vehicles for the containerized V2X/SITL flow.

When `ROLE_MODE=auto`, the OBU app treats roles as maneuver-relative instead of fixed identities. A vehicle on the configured ramp edge acts as the merge requester. Main-lane vehicles compare their ETA to the detected merge candidate's ETA: vehicles arriving earlier become lead vehicles, and vehicles arriving later become host/yield candidates. ETA ties are resolved by lower station ID.

See [docs/mqtt-topic-contract.md](docs/mqtt-topic-contract.md) for full topic mappings.
