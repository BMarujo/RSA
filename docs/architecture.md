# Architecture Notes

This project uses a local MQTT broker per OBU container to simulate an internal vehicle bus. The SUMO TraCI bridge publishes sensor data on a shared broker, and each OBU broker bridges only the topics for its vehicle. Vanetza socktap encodes and decodes ETSI CAM/MCM/DENM messages on the local broker.

See [docs/mqtt-topic-contract.md](docs/mqtt-topic-contract.md) for full topic mappings.
