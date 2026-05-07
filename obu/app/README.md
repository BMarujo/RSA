# OBU Python App

This is a minimal MQTT-based OBU loop that:

- Subscribes to `car/<vehicle_id>/sensors/gps` (local broker)
- Publishes CAM JSON to `vanetza/in/cam`
- Listens to `vanetza/out/cam` to build a local neighbor map
- Optionally infers the current maneuver role (`merge`, `host`, `lead`) from the SUMO lane/position and neighbor ETAs

MCM/DENM templates are loaded but only sent when enabled.
