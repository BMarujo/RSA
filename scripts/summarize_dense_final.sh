#!/bin/bash

echo "Run Trace Warn Coll LaneF Hostless Start Merging Clean After Total LeadOnly TtcReject"

for i in $(seq 1 10); do
  f="logs/dense_final_${i}.log"
  if [ ! -f "$f" ]; then continue; fi
  
  TRC=$(grep -c 'Traceback' "$f" || echo 0)
  WRN=$(grep -c 'Warning' "$f" || echo 0)
  COL=$(grep -c 'collision' "$f" || echo 0)
  LANEF=$(grep -c 'LANE_CMD_FAILED' "$f" || echo 0)
  HOSTLESS=$(grep -c 'MERGE_ALLOWED_HOSTLESS' "$f" || echo 0)
  START=$(grep -c 'MERGE_PHYSICAL_START:' "$f" || echo 0)
  MERGING=$(grep -c 'MERGING!' "$f" || echo 0)
  CLEAN=$(grep -c 'MERGE_COMPLETED:' "$f" || echo 0)
  AFTER=$(grep -c 'MERGE_COMPLETED_AFTER_TIMEOUT' "$f" || echo 0)
  TOTAL=$((CLEAN + AFTER))
  LEADONLY=$(grep -c 'MERGE_AUTHORIZED_LEAD_ONLY_AFTER_LAST_MAIN' "$f" || echo 0)
  TTCREJECT=$(grep -c 'ttc_danger=True' "$f" || echo 0)
  
  printf "%-3s %-5s %-4s %-4s %-5s %-8s %-5s %-7s %-5s %-5s %-5s %-8s %-9s\n" "$i" "$TRC" "$WRN" "$COL" "$LANEF" "$HOSTLESS" "$START" "$MERGING" "$CLEAN" "$AFTER" "$TOTAL" "$LEADONLY" "$TTCREJECT"
done
