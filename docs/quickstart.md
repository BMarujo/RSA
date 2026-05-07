# Quickstart (SITL)

1. Build and run the stack:

```
docker compose up --build
```

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
