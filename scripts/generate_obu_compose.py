#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SUMO_CFG = "/data/sumo-lane-merge/aveiro_map/vanetza.sumocfg"
DEFAULT_OUTPUT = REPO_ROOT / ".generated" / "vanetza-obus.compose.yml"


@dataclass(frozen=True)
class Vehicle:
    vehicle_id: str
    route_id: str | None


def env(name: str, default: str) -> str:
    value = os.getenv(name)
    return value if value is not None and value != "" else default


def local_path(path_value: str, base: Path | None = None) -> Path:
    if path_value.startswith("/data/"):
        return REPO_ROOT / path_value.removeprefix("/data/")
    path = Path(path_value)
    if path.is_absolute():
        return path
    return (base or REPO_ROOT) / path


def container_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return "/data/" + resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(resolved)


def parse_sumo_cfg(sumo_cfg: Path) -> list[Path]:
    root = ET.parse(sumo_cfg).getroot()
    route_files: list[Path] = []
    for element in root.findall(".//route-files"):
        value = element.get("value", "")
        for item in value.split(","):
            item = item.strip()
            if item:
                route_files.append(local_path(item, sumo_cfg.parent))
    if not route_files:
        raise SystemExit(f"No <route-files> entry found in {sumo_cfg}")
    return route_files


def parse_route_files(route_files: list[Path]) -> tuple[list[Vehicle], dict[str, tuple[str, ...]]]:
    vehicles: list[Vehicle] = []
    routes: dict[str, tuple[str, ...]] = {}
    for route_file in route_files:
        root = ET.parse(route_file).getroot()
        for route in root.findall("route"):
            route_id = route.get("id")
            edges = tuple(edge for edge in route.get("edges", "").split() if edge)
            if route_id:
                routes[route_id] = edges
        for vehicle in root.findall("vehicle"):
            vehicle_id = vehicle.get("id")
            if vehicle_id:
                vehicles.append(Vehicle(vehicle_id=vehicle_id, route_id=vehicle.get("route")))
    if not vehicles:
        raise SystemExit(
            "No explicit <vehicle id=...> entries were found. "
            "Dynamic OBU generation currently needs explicit SUMO vehicle IDs."
        )
    return vehicles, routes


def enforce_obu_limit(vehicles: list[Vehicle]) -> None:
    limit = int(env("MAX_OBU_SERVICES", "40"))
    if limit > 0 and len(vehicles) > limit:
        raise SystemExit(
            f"Refusing to generate {len(vehicles)} OBU services because MAX_OBU_SERVICES={limit}. "
            "Increase MAX_OBU_SERVICES, or set it to 0, if this machine can handle that many containers."
        )


def service_name(vehicle_id: str) -> str:
    name = re.sub(r"[^a-zA-Z0-9]+", "-", vehicle_id).strip("-").lower()
    return f"obu-{name or 'vehicle'}"


def station_id(index: int) -> int:
    return int(env("STATION_ID_BASE", "101")) + index


def mac_address(index: int) -> str:
    suffix = int(env("VANETZA_MAC_SUFFIX_BASE", "17")) + index
    return f"6e:06:e0:03:{(suffix // 256) & 0xff:02x}:{suffix & 0xff:02x}"


def is_ramp_vehicle(vehicle: Vehicle, routes: dict[str, tuple[str, ...]]) -> bool:
    ramp_edges = {item.strip() for item in env("RAMP_EDGE_IDS", "34126779").split(",") if item.strip()}
    route_edges = routes.get(vehicle.route_id or "", ())
    if route_edges:
        return route_edges[0] in ramp_edges
    return bool(re.search(r"(merge|ramp)", vehicle.vehicle_id, re.IGNORECASE))


def quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def common_obu_env(vehicle: Vehicle, index: int, ramp_station_ids: str, main_station_ids: str) -> list[tuple[str, str]]:
    sid = str(station_id(index))
    values = {
        "VEHICLE_ID": vehicle.vehicle_id,
        "VEHICLE_ROLE": "auto",
        "ROLE_MODE": "auto",
        "STATION_ID": sid,
        "STATION_TYPE": env("STATION_TYPE", "5"),
        "MCM_STATION_TYPE": env("MCM_STATION_TYPE", "1"),
        "START_EMBEDDED_MOSQUITTO": "true",
        "SUPPORT_MAC_BLOCKING": env("SUPPORT_MAC_BLOCKING", "true"),
        "REMOTE_MQTT_HOST": "mqtt-broker",
        "REMOTE_MQTT_PORT": "1883",
        "LOCAL_MQTT_HOST": "127.0.0.1",
        "LOCAL_MQTT_PORT": "1883",
        "VANETZA_LOCAL_MQTT_BROKER": "127.0.0.1",
        "VANETZA_LOCAL_MQTT_PORT": "1883",
        "VANETZA_REMOTE_MQTT_BROKER": "mqtt-broker",
        "VANETZA_REMOTE_MQTT_PORT": "1883",
        "VANETZA_STATION_ID": sid,
        "VANETZA_STATION_TYPE": env("VANETZA_STATION_TYPE", "5"),
        "VANETZA_MAC_ADDRESS": mac_address(index),
        "VANETZA_INTERFACE": env("VANETZA_INTERFACE", "br0"),
        "ORIGIN_LAT": env("ORIGIN_LAT", "40.0"),
        "ORIGIN_LON": env("ORIGIN_LON", "-8.0"),
        "MERGE_POINT_X": env("MERGE_POINT_X", "194.89"),
        "MERGE_POINT_Y": env("MERGE_POINT_Y", "2212.42"),
        "CRUISE_SPEED": env("CRUISE_SPEED", "11.0"),
        "MERGE_SPEED_BONUS": env("MERGE_SPEED_BONUS", "0.75"),
        "LEAD_SPEED_BONUS": env("LEAD_SPEED_BONUS", "1.0"),
        "MERGE_PRIORITY": env("MERGE_PRIORITY", "false"),
        "DEFAULT_SPEED_MODE": env("DEFAULT_SPEED_MODE", "0"),
        "PRIORITY_SPEED_MODE": env("PRIORITY_SPEED_MODE", "0"),
        "PRIORITY_DISTANCE_M": env("PRIORITY_DISTANCE_M", "60.0"),
        "MERGE_STATION_ID": env("MERGE_STATION_ID", "0"),
        "RAMP_EDGE_IDS": env("RAMP_EDGE_IDS", "34126779"),
        "MAIN_EDGE_IDS": env("MAIN_EDGE_IDS", "560761994,1331698336,135424828"),
        "RAMP_STATION_IDS": env("RAMP_STATION_IDS", ramp_station_ids),
        "MAIN_STATION_IDS": env("MAIN_STATION_IDS", main_station_ids),
        "RAMP_Y_THRESHOLD": env("RAMP_Y_THRESHOLD", "-1.0"),
        "RAMP_BBOX": env("RAMP_BBOX", "205,2120,340,2225"),
        "ROLE_DETECTION_DISTANCE": env("ROLE_DETECTION_DISTANCE", "260.0"),
        "SAFE_HEADWAY_S": env("SAFE_HEADWAY_S", "1.5"),
        "NEGOTIATION_TIMEOUT_S": env("NEGOTIATION_TIMEOUT_S", "2.0"),
        "REQUEST_RETRY_S": env("REQUEST_RETRY_S", "0.5"),
        "RESPONSE_PERIOD_S": env("RESPONSE_PERIOD_S", "0.5"),
        "YIELD_SPEED_DELTA": env("YIELD_SPEED_DELTA", "6.0"),
        "ABORT_SPEED": env("ABORT_SPEED", "2.0"),
        "ABORT_COOLDOWN_S": env("ABORT_COOLDOWN_S", "3.0"),
        "MIN_CLEARANCE_M": env("MIN_CLEARANCE_M", "8.0"),
        "MAX_SPEED_STEP_UP": env("MAX_SPEED_STEP_UP", "0.25"),
        "MAX_SPEED_STEP_DOWN": env("MAX_SPEED_STEP_DOWN", "0.45"),
        "MAX_SPEED_STEP_EMERGENCY": env("MAX_SPEED_STEP_EMERGENCY", "0.9"),
        "MERGE_YIELD_FLOOR_RATIO": env("MERGE_YIELD_FLOOR_RATIO", "0.2"),
        "HOST_YIELD_FLOOR_RATIO": env("HOST_YIELD_FLOOR_RATIO", "0.2"),
        "HOST_REJECT_DISTANCE_M": env("HOST_REJECT_DISTANCE_M", "20.0"),
        "RAMP_PLATOON_HEADWAY_S": env("RAMP_PLATOON_HEADWAY_S", "1.4"),
        "RAMP_PLATOON_MIN_GAP": env("RAMP_PLATOON_MIN_GAP", "14.0"),
        "RAMP_PLATOON_SPEED_DELTA": env("RAMP_PLATOON_SPEED_DELTA", "0.8"),
        "CAM_FOLLOW_HEADWAY_S": env("CAM_FOLLOW_HEADWAY_S", "1.2"),
        "CAM_FOLLOW_MIN_GAP": env("CAM_FOLLOW_MIN_GAP", "10.0"),
        "CAM_FOLLOW_LOOKAHEAD": env("CAM_FOLLOW_LOOKAHEAD", "50.0"),
        "MERGE_LANE_INDEX": env("MERGE_LANE_INDEX", "1"),
        "ENABLE_MCM": env("ENABLE_MCM", "true"),
        "ENABLE_DENM": env("ENABLE_DENM", "false"),
        "ETA_THRESHOLD_S": env("ETA_THRESHOLD_S", "12.0"),
        "STATUS_PERIOD_MS": env("STATUS_PERIOD_MS", "250"),
        "PUBLISH_IDLE_ACTUATORS": env("PUBLISH_IDLE_ACTUATORS", "true"),
    }
    return list(values.items())


def write_compose(output: Path, sumo_cfg: Path, vehicles: list[Vehicle], routes: dict[str, tuple[str, ...]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    vehicle_ids = ",".join(vehicle.vehicle_id for vehicle in vehicles)
    ramp_station_ids = ",".join(
        str(station_id(index))
        for index, vehicle in enumerate(vehicles)
        if is_ramp_vehicle(vehicle, routes)
    )
    main_station_ids = ",".join(
        str(station_id(index))
        for index, vehicle in enumerate(vehicles)
        if not is_ramp_vehicle(vehicle, routes)
    )
    track_vehicle = os.getenv("GUI_TRACK_VEHICLE", "none")
    fit_network = os.getenv("GUI_FIT_NETWORK", "false")
    fixed_merge_view = os.getenv("GUI_FIXED_MERGE_VIEW", "true")
    merge_view_radius = os.getenv("GUI_MERGE_VIEW_RADIUS", "")

    lines = [
        "# Generated by scripts/generate_obu_compose.py; do not edit by hand.",
        "services:",
        "  traci-bridge:",
        "    environment:",
        f"      - SUMO_CFG={container_path(sumo_cfg)}",
        f"      - VEHICLE_IDS={vehicle_ids}",
        f"      - COLLISION_GUARD={env('COLLISION_GUARD', 'false')}",
        f"      - GUI_TRACK_VEHICLE={track_vehicle}",
        f"      - GUI_FIT_NETWORK={fit_network}",
        f"      - GUI_FIXED_MERGE_VIEW={fixed_merge_view}",
    ]
    if merge_view_radius:
        lines.append(f"      - GUI_MERGE_VIEW_RADIUS={merge_view_radius}")

    used_services: set[str] = set()
    for index, vehicle in enumerate(vehicles):
        base_service = service_name(vehicle.vehicle_id)
        name = base_service
        suffix = 2
        while name in used_services:
            name = f"{base_service}-{suffix}"
            suffix += 1
        used_services.add(name)

        lines.extend(
            [
                "",
                f"  {name}:",
                "    image: rsa-obu:latest",
                "    build:",
                "      context: .",
                "      dockerfile: obu/Dockerfile",
                "    restart: unless-stopped",
                "    depends_on:",
                "      - mqtt-broker",
                "    cap_add:",
                "      - NET_ADMIN",
                "      - NET_RAW",
                "    environment:",
            ]
        )
        for key, value in common_obu_env(vehicle, index, ramp_station_ids, main_station_ids):
            lines.append(f"      - {quote(f'{key}={value}')}")
        lines.extend(
            [
                "    networks:",
                "      - v2x_net",
            ]
        )

    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a Compose override with one OBU per SUMO vehicle.")
    parser.add_argument("--sumo-cfg", default=env("SUMO_CFG", DEFAULT_SUMO_CFG))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    sumo_cfg = local_path(args.sumo_cfg).resolve()
    if not sumo_cfg.exists():
        raise SystemExit(f"SUMO config not found: {sumo_cfg}")
    route_files = parse_sumo_cfg(sumo_cfg)
    for route_file in route_files:
        if not route_file.exists():
            raise SystemExit(f"SUMO route file not found: {route_file}")
    vehicles, routes = parse_route_files(route_files)
    enforce_obu_limit(vehicles)
    write_compose(args.output, sumo_cfg, vehicles, routes)
    print(f"Generated {args.output} with {len(vehicles)} OBU service(s).")
    print("Vehicles: " + ", ".join(vehicle.vehicle_id for vehicle in vehicles))


if __name__ == "__main__":
    try:
        main()
    except ET.ParseError as exc:
        sys.exit(f"Invalid SUMO XML: {exc}")
