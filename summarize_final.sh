#!/bin/bash
for f in logs/dense_final_*.log; do
  printf "%-32s " "$f"
  printf "Trc=%s "   "$(grep -c 'Traceback' "$f" || true)"
  printf "Warn=%s "  "$(grep -c 'Warning' "$f" || true)"
  printf "Coll=%s "  "$(grep -c 'collision' "$f" || true)"
  printf "Skip=%s "  "$(grep -c 'MCM_SKIP_BUSY_HOST' "$f" || true)"
  printf "Start=%s " "$(grep -c 'MERGE_PHYSICAL_START' "$f" || true)"
  printf "Merg=%s "  "$(grep -c 'MERGING!' "$f" || true)"
  printf "Comp=%s "  "$(grep -c 'MERGE_COMPLETED:' "$f" || true)"
  printf "Lost=%s "  "$(grep -c 'MERGE_FAILED_LOST_AUTH_AFTER_POINT' "$f" || true)"
  printf "Spd0=%s\n"  "$(grep -c 'speed=0.00' "$f" || true)"
done
