# MQTT Topic Contract

This document defines the MQTT topics used across the SITL stack, including the local OBU broker and the shared remote broker. It aligns with the default Vanetza socktap configuration in [vanetza-nap/tools/socktap/config.ini](../vanetza-nap/tools/socktap/config.ini) and the remote MQTT prefixing logic in [vanetza-nap/tools/socktap/config.cpp](../vanetza-nap/tools/socktap/config.cpp).

## Brokers

- Local broker (per OBU container): embedded mosquitto started inside the OBU container.
- Remote broker (shared): a single mosquitto instance used by SUMO/TraCI bridge and external observers.

## Local broker topics (inside each OBU)

### Sensor ingress (from SUMO bridge)

- `car/<vehicle_id>/sensors/gps`
  - JSON: `{ "x": <float>, "y": <float>, "speed": <float>, "lane_id": <string>, "timestamp": <float> }`
  - Units: meters, meters, m/s, SUMO lane id, seconds.

### Actuator egress (to SUMO bridge)

- `car/<vehicle_id>/actuators/speed`
  - JSON: `{ "target_speed": <float>, "timestamp": <float> }` (m/s)
- `car/<vehicle_id>/actuators/lane`
  - JSON: `{ "target_lane_index": <int>, "timestamp": <float> }`
- `car/<vehicle_id>/actuators/speed_mode`
  - JSON: `{ "speed_mode": <int>, "timestamp": <float> }`

### Status egress (to SUMO bridge and observers)

- `car/<vehicle_id>/status/fsm`
  - JSON includes `vehicle_id`, `station_id`, `role`, `effective_role`, `fsm_state`, `distance_to_merge_m`, `merge_eta_s`, `neighbor_count`, `merge_committed`, `merge_completed`, `lane_command_state`, target actuator values, and `timestamp`.
  - The TraCI bridge uses this topic to color the SUMO-GUI overlays for role and FSM state.
  - Host vehicles also use this status to keep or release reservations while a ramp vehicle is committed but still waiting for the lane command to become executable.

### Vanetza ingress (OBU -> Vanetza)

- `vanetza/in/cam`
- `vanetza/in/mcm`
- `vanetza/in/denm`

### Vanetza egress (Vanetza -> OBU)

- `vanetza/out/cam`
- `vanetza/out/mcm`
- `vanetza/out/denm`

Vanetza-NAP wraps decoded ETSI messages in a metadata envelope. The decoded PDU lives under `fields`:

- CAM: `fields.header` and `fields.cam`
- MCM: `fields.header` and `fields.payload`
- DENM: `fields.header` and `fields.denm`

The top-level metadata also includes fields such as `stationID`, `stationAddr`, `receiverID`, `timestamp`, `rssi`, and `packet_size`.

Optional timing topics (useful for latency analysis):

- `vanetza/time/cam`
- `vanetza/time/mcm`
- `vanetza/time/denm`

## Remote broker topics (shared)

### SUMO bridge topics (global bus)

- `car/<vehicle_id>/sensors/#` (published by the SUMO bridge)
- `car/<vehicle_id>/actuators/#` (subscribed by the SUMO bridge)
- `car/<vehicle_id>/status/#` (published by each OBU, subscribed by the SUMO bridge and observers)

### Vanetza remote topics (per-station prefix)

When `general.remote_mqtt_broker` is enabled, socktap publishes and subscribes to a prefixed topic per station:

- Non-RSU stations (station_type != 15): `obu<station_id>/vanetza/...`
- RSU stations (station_type == 15): `p<station_id>/vanetza/...`

Example:

- `obu2/vanetza/out/cam`
- `obu2/vanetza/time/mcm`

These remote topics are intended for external monitoring or control and are not required for the OBU local control loop.

## Mosquitto bridge rules (per OBU)

Each OBU local broker bridges only the vehicle-specific sensor and actuator topics:

- Inbound (remote -> local): `car/<vehicle_id>/sensors/#`
- Outbound (local -> remote): `car/<vehicle_id>/actuators/#`
- Outbound (local -> remote): `car/<vehicle_id>/status/#`

This keeps the Vanetza and OBU internal topics isolated per vehicle while still receiving SUMO state and emitting actuation commands.

## Notes

- JSON payloads for CAM/MCM/DENM must follow ETSI specifications. See [vanetza-nap/README.md](../vanetza-nap/README.md) for examples and required fields.
- The ITS PDU header is not included when publishing to `vanetza/in/*`; Vanetza fills it automatically.
- The station ID can be included in message fields; it overrides the header value if present.
