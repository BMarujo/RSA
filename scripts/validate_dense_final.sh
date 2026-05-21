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

START_COUNT=$( (grep -h 'MERGE_PHYSICAL_START:' logs/dense_final_*.log || true) | wc -l )

MERGING_COUNT=$( (grep -h 'MERGING!' logs/dense_final_*.log || true) | wc -l )

COMP_CLEAN=$( (grep -h 'MERGE_COMPLETED:' logs/dense_final_*.log || true) | wc -l )

COMP_TO=$( (grep -h 'MERGE_COMPLETED_AFTER_TIMEOUT' logs/dense_final_*.log || true) | wc -l )

TOTAL_COMP=$((COMP_CLEAN + COMP_TO))

if [ "$START_COUNT" -ne "$TOTAL_COMP" ]; then echo "FAILED: MERGE_PHYSICAL_START count is $START_COUNT, expected completed total $TOTAL_COMP"; FAIL=1; fi
if [ "$MERGING_COUNT" -ne "$TOTAL_COMP" ]; then echo "FAILED: MERGING! count is $MERGING_COUNT, expected completed total $TOTAL_COMP"; FAIL=1; fi

for f in logs/dense_final_*.log; do
  START=$(grep -c 'MERGE_PHYSICAL_START:' "$f" || true)
  MERGING=$(grep -c 'MERGING!' "$f" || true)
  CLEAN=$(grep -c 'MERGE_COMPLETED:' "$f" || true)
  AFTER=$(grep -c 'MERGE_COMPLETED_AFTER_TIMEOUT' "$f" || true)
  TOTAL=$((CLEAN + AFTER))
  RUN_NAME=$(basename "$f")
  if [ "$START" -ne "$TOTAL" ]; then echo "FAILED: $RUN_NAME START=$START TOTAL=$TOTAL"; FAIL=1; fi
  if [ "$MERGING" -ne "$TOTAL" ]; then echo "FAILED: $RUN_NAME MERGING=$MERGING TOTAL=$TOTAL"; FAIL=1; fi
done

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
SPD0_TOTAL=$( (grep -h 'speed=0.00' logs/dense_final_*.log || true) | wc -l )
T018_TOTAL=$( (grep -h 'target=0.18' logs/dense_final_*.log || true) | wc -l )

if [ "$SPD0_TOTAL" -gt "$MAX_SPD0" ]; then echo "WARNING: High SPD0 count ($SPD0_TOTAL > $MAX_SPD0)"; fi
if [ "$T018_TOTAL" -gt "$MAX_T018" ]; then echo "WARNING: High T018 count ($T018_TOTAL > $MAX_T018)"; fi

if [ "$FAIL" -eq 1 ]; then
  echo "Validation FAILED."
  exit 1
else
  echo "Validation PASSED."
  exit 0
fi
