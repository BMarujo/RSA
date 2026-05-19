#!/bin/bash

echo "Run Trace Warn Coll LaneF Hostless Start Merging Clean After Total LeadOnly TtcReject Spd0 T018"

for i in $(seq 1 10); do
  f="logs/dense_final_${i}.log"
  if [ ! -f "$f" ]; then continue; fi
  
  TRC=$(grep -c 'Traceback' "$f" || true)
  WRN=$(grep -c 'Warning' "$f" || true)
  COL=$(grep -c 'collision' "$f" || true)
  LANEF=$(grep -c 'LANE_CMD_FAILED' "$f" || true)
  HOSTLESS=$(grep -c 'MERGE_ALLOWED_HOSTLESS' "$f" || true)
  START=$(grep -c 'MERGE_PHYSICAL_START:' "$f" || true)
  MERGING=$(grep -c 'MERGING!' "$f" || true)
  CLEAN=$(grep -c 'MERGE_COMPLETED:' "$f" || true)
  AFTER=$(grep -c 'MERGE_COMPLETED_AFTER_TIMEOUT' "$f" || true)
  TOTAL=$((CLEAN + AFTER))
  LEADONLY=$(grep -c 'MERGE_AUTHORIZED_LEAD_ONLY_AFTER_LAST_MAIN' "$f" || true)
  TTCREJECT=$(grep -c 'ttc_danger=True' "$f" || true)
  SPD0=$(grep -c 'speed=0.00' "$f" || true)
  T018=$(grep -c 'target=0.18' "$f" || true)
  
  printf "%-3s %-5s %-4s %-4s %-5s %-8s %-5s %-7s %-5s %-5s %-5s %-8s %-9s %-4s %-4s\n" "$i" "$TRC" "$WRN" "$COL" "$LANEF" "$HOSTLESS" "$START" "$MERGING" "$CLEAN" "$AFTER" "$TOTAL" "$LEADONLY" "$TTCREJECT" "$SPD0" "$T018"
done
