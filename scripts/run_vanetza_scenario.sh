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
    dense|blocked|single-lane)
        default_env MERGE_POINT_X "194.89"
        default_env MERGE_POINT_Y "2212.42"
        default_env RAMP_EDGE_IDS "34126779"
        default_env MAIN_EDGE_IDS "560761994,1331698336,135424828"
        default_env RAMP_BBOX "205,2120,340,2225"
        default_env MERGE_STATION_ID "0"
        default_env MERGE_LANE_INDEX "1"
        default_env HOST_CLEAR_LANE_INDEX "1"
        default_env HOST_COOPERATIVE_LANE_CHANGE "false"
        default_env LANE_CHANGE_DURATION_S "2.0"
        default_env LANE_CHANGE_COOLDOWN_S "5.0"
        default_env HOST_CLEAR_LANE_HOLD_S "12.0"
        default_env HOST_RETURN_LOCK_DISTANCE_M "90.0"

        default_env PRIORITY_DISTANCE "120.0"
        default_env ROLE_DETECTION_DISTANCE "220.0"
        default_env ETA_THRESHOLD_S "18.0"
        default_env NEIGHBOR_TIMEOUT_S "2.5"
        default_env REQUEST_RETRY_S "0.25"
        default_env RESPONSE_PERIOD_S "0.25"
        default_env NEGOTIATION_TIMEOUT_S "3.5"
        default_env HOST_RESERVATION_S "4.0"
        default_env HOST_RESERVATION_MAX_S "10.0"
        default_env MCM_LATE_HOST_LOCK_GRACE_S "1.5"
        default_env MCM_LATE_HOST_LOCK_DISTANCE_M "70.0"

        default_env CRUISE_SPEED "7.5"
        default_env MERGE_SPEED_BONUS "0.0"
        default_env LEAD_SPEED_BONUS "0.0"

        default_env DEFAULT_SPEED_MODE "31"
        default_env PRIORITY_SPEED_MODE "31"
        default_env TRACI_DEFAULT_SPEED_MODE "31"
        default_env TRACI_DEFAULT_LANE_CHANGE_MODE "512"
        default_env TRACI_COMMAND_LANE_CHANGE_MODE "256"
        default_env COLLISION_GUARD "true"

        default_env TRACI_VEHICLE_DECEL "4.5"
        default_env TRACI_VEHICLE_EMERGENCY_DECEL "9.0"
        default_env SUMO_EXTRA_ARGS "--collision.action warn --collision.check-junctions true --emergencydecel.warning-threshold 5.0"

        default_env MIN_SPEED "0.0"
        default_env EMERGENCY_MIN_SPEED "0.0"
        default_env ABORT_SPEED "0.0"
        default_env HOST_YIELD_FLOOR_RATIO "0.65"
        default_env MERGE_YIELD_FLOOR_RATIO "0.50"

        default_env SAFE_HEADWAY_S "1.5"
        default_env MIN_CLEARANCE_M "8.0"
        default_env HOST_REJECT_DISTANCE_M "18.0"
        default_env HOST_SAME_LANE_GUARD_GAP "14.0"
        default_env MERGE_ZONE_CLEARANCE_M "8.0"
        default_env MERGE_STOP_MARGIN_M "8.0"
        default_env MERGE_BLOCKED_APPROACH_S "3.5"

        default_env MERGE_NEIGHBOR_WARMUP_S "1.2"
        default_env MERGE_MIN_NEIGHBORS_BEFORE_MERGE "0"
        
        default_env FINAL_MERGE_GUARD_M "28.0"
        default_env FINAL_MERGE_CLEARANCE_M "10.0"
        default_env MERGE_OCCUPANCY_S "3.2"
        default_env MERGE_COMMIT_HEADWAY_S "1.5"
        default_env MIN_MERGE_ENTRY_SPEED "4.5"
        default_env MERGE_ENTRY_SPEED_GUARD_M "35.0"
        default_env MERGE_COMMIT_TIMEOUT_S "12.0"
        default_env MERGE_COMMIT_DISTANCE_M "70.0"

        default_env RAMP_PLATOON_HEADWAY_S "1.2"
        default_env RAMP_PLATOON_MIN_GAP "7.0"
        default_env RAMP_PLATOON_SPEED_DELTA "1.0"
        default_env MERGE_QUEUE_RELEASE_GAP "9.0"

        default_env ENABLE_CAM_FOLLOWING "true"
        default_env CAM_FOLLOW_HEADWAY_S "0.8"
        default_env CAM_FOLLOW_MIN_GAP "7.0"
        default_env CAM_FOLLOW_LOOKAHEAD "55.0"
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
        if [ "$VANETZA_SCENARIO" = "single-lane" ]; then
            default_env MCM_REQUEST_DISTANCE_M "80.0"
            default_env HOST_RESERVATION_S "5.0"
            default_env HOST_RESERVATION_MAX_S "14.0"
            default_env GUI_TRACK_VEHICLE "Merge_Car"
            default_env GUI_MERGE_VIEW_RADIUS "95"
        fi
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
    log)
        mkdir -p logs
        LOG_FILE="${LOG_FILE:-logs/${VANETZA_SCENARIO}_$(date +%Y%m%d_%H%M%S).log}"
        echo "Writing run log to ${LOG_FILE}"
        generate
        set +e
        compose up --build --abort-on-container-exit --exit-code-from traci-bridge "$@" 2>&1 | tee "$LOG_FILE"
        status=${PIPESTATUS[0]}
        compose down --remove-orphans >/dev/null 2>&1 || true
        exit "$status"
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
        echo "Usage: $0 [up|log|bridge|down|config|generate|scenarios] [docker compose args...]" >&2
        echo "Set LOG_FILE=logs/my_run.log to choose the output file for the log command." >&2
        echo "Select a focused scenario with VANETZA_SCENARIO=dense|blocked|single-lane" >&2
        exit 2
        ;;
esac
