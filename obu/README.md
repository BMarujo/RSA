# OBU App

This container runs two processes:

1. Vanetza socktap (ETSI CAM/MCM/DENM encode/decode)
2. Python OBU logic (FSM placeholder and MQTT wiring)

The embedded mosquitto broker bridges SUMO sensor topics from the remote broker into the local broker and forwards actuator commands back to the remote broker.

## Environment variables

- `VEHICLE_ID`: SUMO vehicle ID (e.g., `Lead_Car`)
- `VEHICLE_ROLE`: `lead`, `host`, or `merge`
- `LOCAL_MQTT_HOST` / `LOCAL_MQTT_PORT`: Local broker address
- `REMOTE_MQTT_HOST` / `REMOTE_MQTT_PORT`: Remote broker address for local bridge
- `STATION_ID` / `STATION_TYPE`: ETSI station identifiers used by the OBU app
- `VANETZA_*`: Socktap configuration overrides (see [vanetza-nap/README.md](vanetza-nap/README.md))
- `VANETZA_LINK_LAYER`: `udp` or `ethernet` (default uses socktap's default)
- `ORIGIN_LAT` / `ORIGIN_LON`: Reference origin for converting SUMO x/y to lat/lon
- `ENABLE_MCM` / `ENABLE_DENM`: Enable sending MCM or DENM templates
- `MERGE_POINT_X` / `MERGE_POINT_Y`: Merge point in SUMO coordinates
- `ETA_THRESHOLD_S`: ETA threshold to trigger negotiation
