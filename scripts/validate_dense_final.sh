#!/bin/bash
set -e

mkdir -p logs
rm -f logs/dense_final_*.log

RUN_COUNT=${RUNS:-10}
EXPECTED_TOTAL=$((RUN_COUNT * 6))

for i in $(seq 1 $RUN_COUNT); do
  echo "=== Validation Run $i/$RUN_COUNT ==="
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
if [ "$START_COUNT" -ne $EXPECTED_TOTAL ]; then echo "FAILED: MERGE_PHYSICAL_START count is $START_COUNT, expected $EXPECTED_TOTAL"; FAIL=1; fi

MERGING_COUNT=$(grep -c 'MERGING!' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$MERGING_COUNT" ]; then MERGING_COUNT=$(grep 'MERGING!' logs/dense_final_*.log | wc -l); fi
if [ "$MERGING_COUNT" -ne $EXPECTED_TOTAL ]; then echo "FAILED: MERGING! count is $MERGING_COUNT, expected $EXPECTED_TOTAL"; FAIL=1; fi

COMP_CLEAN=$(grep -c 'MERGE_COMPLETED:' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$COMP_CLEAN" ]; then COMP_CLEAN=$(grep 'MERGE_COMPLETED:' logs/dense_final_*.log | wc -l); fi

COMP_TO=$(grep -c 'MERGE_COMPLETED_AFTER_TIMEOUT' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
if [ -z "$COMP_TO" ]; then COMP_TO=$(grep 'MERGE_COMPLETED_AFTER_TIMEOUT' logs/dense_final_*.log | wc -l); fi

TOTAL_COMP=$((COMP_CLEAN + COMP_TO))

STRICT_COMPLETION=${STRICT_COMPLETION:-false}
MIN_COMPLETION_RATIO=${MIN_COMPLETION_RATIO:-0.98}

if [ "$STRICT_COMPLETION" = "true" ]; then
  if [ "$TOTAL_COMP" -ne $EXPECTED_TOTAL ]; then echo "FAILED: Total completions count is $TOTAL_COMP, expected $EXPECTED_TOTAL (STRICT)"; FAIL=1; fi
else
  RATIO=$(echo "$TOTAL_COMP / $EXPECTED_TOTAL" | bc -l)
  if (( $(echo "$RATIO < $MIN_COMPLETION_RATIO" | bc -l) )); then
    echo "FAILED: Total completions ratio is $RATIO, expected at least $MIN_COMPLETION_RATIO"
    FAIL=1
  fi
fi

# Warnings for performance indicators
MAX_SPD0=${MAX_SPD0:-120}
MAX_T018=${MAX_T018:-80}
SPD0_TOTAL=$(grep -c 'speed=0.00' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')
T018_TOTAL=$(grep -c 'target=0.18' logs/dense_final_*.log | awk -F: '{s+=$2} END {print s}')

if [ "$SPD0_TOTAL" -gt "$MAX_SPD0" ]; then echo "WARNING: High SPD0 count ($SPD0_TOTAL > $MAX_SPD0)"; fi
if [ "$T018_TOTAL" -gt "$MAX_T018" ]; then echo "WARNING: High T018 count ($T018_TOTAL > $MAX_T018)"; fi

if [ "$FAIL" -eq 1 ]; then
  echo "Validation FAILED."
  exit 1
else
  echo "Validation PASSED."
  exit 0
fi
