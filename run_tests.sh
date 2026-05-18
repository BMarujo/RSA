#!/bin/bash
mkdir -p logs
for i in $(seq 1 10); do
  echo "=== dense intensive $i ==="
  LOG_FILE=logs/dense_intensive_v3_${i}.log \
  SUMO_GUI=false LOOP_SIM=false SUMO_END=120 STEP_DELAY_S=0 \
  ./scripts/run_vanetza_scenario.sh log

  docker compose down --remove-orphans >/dev/null 2>&1 || true
done
