# OBU Python App

This is a minimal MQTT-based OBU loop that:

- Subscribes to `car/<vehicle_id>/sensors/gps` (local broker)
- Publishes CAM JSON to `vanetza/in/cam`
- Listens to `vanetza/out/cam` to build a local neighbor map

MCM/DENM templates are loaded but only sent when enabled.
