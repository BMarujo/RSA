# OBU Python App

This is a minimal MQTT-based OBU loop that:

- Subscribes to `car/<vehicle_id>/sensors/gps` (local broker)
- Publishes CAM JSON to `vanetza/in/cam`
- Listens to `vanetza/out/cam` to build a local neighbor map
- Optionally infers the current maneuver role (`merge`, `host`, `lead`) from the SUMO lane/position and neighbor ETAs

MCM/DENM templates are loaded but only sent when enabled.

## Scenario Commands

Basic focused merge with the smallest vehicle set.

```bash
VANETZA_SCENARIO=base scripts/run_vanetza_scenario.sh up
```

Merge with a clearer gap on the main road, useful to show a smooth acceptance.

```bash
VANETZA_SCENARIO=gap scripts/run_vanetza_scenario.sh up
```

Default busy demo around the Aveiro merge, with more vehicles in the same zone.

```bash
VANETZA_SCENARIO=dense scripts/run_vanetza_scenario.sh up
```

Ramp queue/platoon demo with several vehicles arriving from the acceleration lane.

```bash
VANETZA_SCENARIO=ramp-platoon scripts/run_vanetza_scenario.sh up
```

Blocked/tight merge demo, useful for showing yielding and recovery behavior.   (tá manhoso este)

```bash
VANETZA_SCENARIO=blocked scripts/run_vanetza_scenario.sh up
```

One-lane merge stress test: multiple vehicles coordinate by speed only, with no left-lane escape.

```bash
VANETZA_SCENARIO=single-lane scripts/run_vanetza_scenario.sh up
```
