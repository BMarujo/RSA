#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO_DIR="${ROOT_DIR}/sumo-lane-merge/aveiro_map/vanetza_scenarios"
VANETZA_SCENARIO="${VANETZA_SCENARIO:-dense}"
if [ -n "${SUMO_CFG:-}" ]; then
    SUMO_CFG="$SUMO_CFG"
else
    SUMO_CFG="/data/sumo-lane-merge/aveiro_map/vanetza_scenarios/${VANETZA_SCENARIO}.sumocfg"
fi
OVERRIDE_FILE="${OBU_COMPOSE_FILE:-${ROOT_DIR}/.generated/vanetza-obus.compose.yml}"
COMMAND="${1:-up}"

if [ "$#" -gt 0 ]; then
    shift
fi

cd "$ROOT_DIR"

default_env() {
    local name="$1"
    local value="$2"
    if [ -z "${!name:-}" ]; then
        export "$name=$value"
    else
        export "$name"
    fi
}

case "$VANETZA_SCENARIO" in
    dense)
        default_env MERGE_POINT_X "194.89"
        default_env MERGE_POINT_Y "2212.42"
        default_env RAMP_EDGE_IDS "34126779"
        default_env MAIN_EDGE_IDS "560761994,1331698336,135424828"
        default_env RAMP_BBOX "205,2120,340,2225"
        default_env MERGE_STATION_ID "0"
        default_env MERGE_LANE_INDEX "0"
        default_env HOST_CLEAR_LANE_INDEX "1"
        default_env HOST_COOPERATIVE_LANE_CHANGE "true"
        default_env LANE_CHANGE_DURATION_S "5.0"
        default_env LANE_CHANGE_COOLDOWN_S "5.0"
        default_env HOST_CLEAR_LANE_HOLD_S "12.0"
        default_env HOST_RETURN_LOCK_DISTANCE_M "90.0"

        default_env PRIORITY_DISTANCE "180.0"
        default_env ROLE_DETECTION_DISTANCE "320.0"
        default_env ETA_THRESHOLD_S "18.0"
        default_env NEIGHBOR_TIMEOUT_S "2.5"
        default_env REQUEST_RETRY_S "0.25"
        default_env RESPONSE_PERIOD_S "0.25"
        default_env NEGOTIATION_TIMEOUT_S "3.5"

        default_env CRUISE_SPEED "7.5"
        default_env MERGE_SPEED_BONUS "0.0"
        default_env LEAD_SPEED_BONUS "0.0"

        default_env DEFAULT_SPEED_MODE "31"
        default_env PRIORITY_SPEED_MODE "31"
        default_env TRACI_DEFAULT_SPEED_MODE "31"
        default_env TRACI_DEFAULT_LANE_CHANGE_MODE "1621"
        default_env COLLISION_GUARD "true"

        default_env TRACI_VEHICLE_DECEL "4.5"
        default_env TRACI_VEHICLE_EMERGENCY_DECEL "9.0"
        default_env SUMO_EXTRA_ARGS "--collision.action warn --collision.check-junctions true --emergencydecel.warning-threshold 5.0"

        default_env MIN_SPEED "0.0"
        default_env EMERGENCY_MIN_SPEED "0.0"
        default_env ABORT_SPEED "0.0"
        default_env HOST_YIELD_FLOOR_RATIO "0.45"
        default_env MERGE_YIELD_FLOOR_RATIO "0.35"

        default_env SAFE_HEADWAY_S "1.5"
        default_env MIN_CLEARANCE_M "8.0"
        default_env HOST_REJECT_DISTANCE_M "18.0"
        default_env HOST_SAME_LANE_GUARD_GAP "14.0"
        default_env MERGE_ZONE_CLEARANCE_M "8.0"
        default_env MERGE_STOP_MARGIN_M "8.0"
        default_env MERGE_BLOCKED_APPROACH_S "3.5"

        default_env MERGE_NEIGHBOR_WARMUP_S "1.2"
        default_env MERGE_MIN_NEIGHBORS_BEFORE_MERGE "5"
        
        default_env FINAL_MERGE_GUARD_M "28.0"
        default_env FINAL_MERGE_CLEARANCE_M "10.0"
        default_env MERGE_OCCUPANCY_S "3.2"
        default_env MIN_MERGE_ENTRY_SPEED "4.5"
        default_env MERGE_ENTRY_SPEED_GUARD_M "35.0"
        default_env MERGE_COMMIT_TIMEOUT_S "8.0"

        default_env RAMP_PLATOON_HEADWAY_S "1.2"
        default_env RAMP_PLATOON_MIN_GAP "10.0"
        default_env RAMP_PLATOON_SPEED_DELTA "1.0"
        default_env MERGE_QUEUE_RELEASE_GAP "14.0"

        default_env ENABLE_CAM_FOLLOWING "true"
        default_env CAM_FOLLOW_HEADWAY_S "1.5"
        default_env CAM_FOLLOW_MIN_GAP "12.0"
        default_env CAM_FOLLOW_LOOKAHEAD "100.0"
        default_env CAM_FOLLOW_LATERAL_TOLERANCE_M "2.0"
        default_env CAM_FOLLOW_SPEED_DELTA "0.8"
        default_env CAM_FOLLOW_CRITICAL_GAP_M "5.0"
        default_env CAM_FOLLOW_BRAKE_DECEL "4.5"
        default_env CAM_FOLLOW_EMERGENCY_DECEL "9.0"

        default_env MAX_SPEED_STEP_UP "0.18"
        default_env MAX_SPEED_STEP_DOWN "0.25"
        default_env MAX_SPEED_STEP_EMERGENCY "0.45"

        default_env MERGE_CONFLICT_FOLLOW_DISTANCE_M "55.0"
        default_env MERGE_CONFLICT_FLOOR_RATIO "0.55"

        default_env GUI_TRACK_VEHICLE "none"
        default_env GUI_FIXED_MERGE_VIEW "true"
        default_env GUI_MERGE_VIEW_RADIUS "130"
        default_env GUI_ZOOM "1200"
        default_env GUI_BOUNDARY_PADDING "70"
        default_env GUI_MERGE_ZONE_LENGTH "13"
        ;;
    single-lane)
        default_env MERGE_POINT_X "1647.94"
        default_env MERGE_POINT_Y "758.48"
        default_env RAMP_EDGE_IDS "1042215851"
        default_env MAIN_EDGE_IDS "-251663459#1,111762619#0"
        default_env RAMP_BBOX "0,0,0,0"
        default_env MERGE_STATION_ID "0"
        default_env MERGE_LANE_INDEX "0"
        default_env MERGE_ZONE_CLEARANCE_M "260.0"
        default_env MERGE_STOP_MARGIN_M "24.0"
        default_env MERGE_BLOCKED_APPROACH_S "6.0"
        default_env PRIORITY_DISTANCE "220.0"
        default_env ROLE_DETECTION_DISTANCE "340.0"
        default_env ETA_THRESHOLD_S "20.0"
        default_env NEIGHBOR_TIMEOUT_S "2.5"
        default_env REQUEST_RETRY_S "0.25"
        default_env RESPONSE_PERIOD_S "0.25"
        default_env CRUISE_SPEED "8.5"
        default_env MERGE_SPEED_BONUS "0.0"
        default_env LEAD_SPEED_BONUS "0.0"
        default_env SUMO_EXTRA_ARGS "--collision.action warn --collision.check-junctions true --emergencydecel.warning-threshold 100.0"
        default_env DEFAULT_SPEED_MODE "0"
        default_env PRIORITY_SPEED_MODE "0"
        default_env TRACI_DEFAULT_SPEED_MODE "0"
        default_env TRACI_DEFAULT_LANE_CHANGE_MODE "0"
        default_env TRACI_VEHICLE_DECEL "50.0"
        default_env TRACI_VEHICLE_EMERGENCY_DECEL "50.0"
        default_env COLLISION_GUARD "false"
        default_env COLLISION_GUARD_LOOKAHEAD "60.0"
        default_env COLLISION_GUARD_MIN_GAP "9.0"
        default_env COLLISION_GUARD_HEADWAY_S "1.0"
        default_env COLLISION_GUARD_MIN_SPEED "1.8"
        default_env COLLISION_GUARD_DURATION_S "1.6"
        default_env COLLISION_GUARD_MAX_DECEL "3.5"
        default_env HOST_YIELD_FLOOR_RATIO "0.0"
        default_env MERGE_YIELD_FLOOR_RATIO "0.0"
        default_env MIN_CLEARANCE_M "22.0"
        default_env HOST_REJECT_DISTANCE_M "35.0"
        default_env ABORT_SPEED "0.0"
        default_env MIN_SPEED "0.0"
        default_env EMERGENCY_MIN_SPEED "0.0"
        default_env RAMP_PLATOON_HEADWAY_S "3.0"
        default_env RAMP_PLATOON_MIN_GAP "45.0"
        default_env MERGE_QUEUE_RELEASE_GAP "70.0"
        default_env CAM_FOLLOW_HEADWAY_S "3.0"
        default_env CAM_FOLLOW_MIN_GAP "24.0"
        default_env CAM_FOLLOW_LOOKAHEAD "160.0"
        default_env CAM_FOLLOW_LATERAL_TOLERANCE_M "5.0"
        default_env CAM_FOLLOW_SPEED_DELTA "2.0"
        default_env CAM_FOLLOW_CRITICAL_GAP_M "14.0"
        default_env CAM_FOLLOW_BRAKE_DECEL "4.5"
        default_env CAM_FOLLOW_EMERGENCY_DECEL "9.0"
        default_env MAX_SPEED_STEP_UP "0.15"
        default_env MAX_SPEED_STEP_EMERGENCY "0.9"
        default_env GUI_TRACK_VEHICLE "none"
        default_env GUI_FIXED_MERGE_VIEW "true"
        default_env GUI_MERGE_VIEW_RADIUS "115"
        default_env GUI_ZOOM "1200"
        default_env GUI_BOUNDARY_PADDING "70"
        default_env GUI_MERGE_ZONE_LENGTH "12"
        ;;
esac

generate() {
    python3 scripts/generate_obu_compose.py \
        --sumo-cfg "$SUMO_CFG" \
        --output "$OVERRIDE_FILE"
}

compose() {
    docker compose -f docker-compose.yml -f "$OVERRIDE_FILE" "$@"
}

case "$COMMAND" in
    scenarios)
        find "$SCENARIO_DIR" -maxdepth 1 -name '*.sumocfg' -printf '%f\n' \
            | sed 's/\.sumocfg$//' \
            | sort
        ;;
    generate)
        generate
        ;;
    config)
        generate
        compose config "$@"
        ;;
    up)
        generate
        compose up --build "$@"
        ;;
    bridge)
        generate
        compose build
        compose up -d --scale traci-bridge=0
        compose run --rm traci-bridge "$@"
        ;;
    down)
        if [ -f "$OVERRIDE_FILE" ]; then
            compose down --remove-orphans "$@"
        else
            docker compose down --remove-orphans "$@"
        fi
        ;;
    *)
        echo "Usage: $0 [up|bridge|down|config|generate|scenarios] [docker compose args...]" >&2
        echo "Select a focused scenario with VANETZA_SCENARIO=base|gap|dense|ramp-platoon|blocked|single-lane" >&2
        exit 2
        ;;
esac
