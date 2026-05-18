#!/bin/bash
set -e

mkdir -p logs
rm -f logs/dense_final_*.log

for i in $(seq 1 10); do
  echo "=== Validation Run $i/10 ==="
  LOG_FILE=logs/dense_final_${i}.log \
  SUMO_GUI=false LOOP_SIM=false SUMO_END=120 STEP_DELAY_S=0 \
  ./scripts/run_vanetza_scenario.sh log
  
  docker compose down --remove-orphans >/dev/null 2>&1 || true
done

echo "Validating runs..."
FAIL=0

if grep -q "Traceback" logs/dense_final_*.log; then echo "FAILED: Traceback found"; FAIL=1; fi
if grep -q "Warning" logs/dense_final_*.log; then echo "FAILED: Warning found"; FAIL=1; fi
if grep -q "collision" logs/dense_final_*.log; then echo "FAILED: collision found"; FAIL=1; fi
if grep -q "LANE_CMD_FAILED" logs/dense_final_*.log; then echo "FAILED: LANE_CMD_FAILED found"; FAIL=1; fi
if grep -q "MERGE_ALLOWED_HOSTLESS" logs/dense_final_*.log; then echo "FAILED: MERGE_ALLOWED_HOSTLESS found"; FAIL=1; fi

START_COUNT=$(grep -c 'MERGE_PHYSICAL_START:' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$START_COUNT" ]; then START_COUNT=$(grep 'MERGE_PHYSICAL_START:' logs/dense_final_*.log | wc -l); fi
if [ "$START_COUNT" -ne 60 ]; then echo "FAILED: MERGE_PHYSICAL_START count is $START_COUNT, expected 60"; FAIL=1; fi

MERGING_COUNT=$(grep -c 'MERGING!' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$MERGING_COUNT" ]; then MERGING_COUNT=$(grep 'MERGING!' logs/dense_final_*.log | wc -l); fi
if [ "$MERGING_COUNT" -ne 60 ]; then echo "FAILED: MERGING! count is $MERGING_COUNT, expected 60"; FAIL=1; fi

COMP_CLEAN=$(grep -c 'MERGE_COMPLETED:' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$COMP_CLEAN" ]; then COMP_CLEAN=$(grep 'MERGE_COMPLETED:' logs/dense_final_*.log | wc -l); fi

COMP_TO=$(grep -c 'MERGE_COMPLETED_AFTER_TIMEOUT' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$COMP_TO" ]; then COMP_TO=$(grep 'MERGE_COMPLETED_AFTER_TIMEOUT' logs/dense_final_*.log | wc -l); fi

TOTAL_COMP=$((COMP_CLEAN + COMP_TO))
if [ "$TOTAL_COMP" -ne 60 ]; then echo "FAILED: Total completions count is $TOTAL_COMP, expected 60"; FAIL=1; fi

if [ "$FAIL" -eq 1 ]; then
  echo "Validation FAILED."
  exit 1
else
  echo "Validation PASSED."
  exit 0
fi
