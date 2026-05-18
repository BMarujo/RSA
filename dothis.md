Sim, agora parece mesmo pronto para testes intensivos.

Confirmei no repo:

* O bug do `fmt_gap` está corrigido no `MERGE_ACCEPTED_WAIT_SLOT_VALID`. Já não há formatação perigosa com `None`. 
* `accepted_slot_invalid_since` já é resetado em criação de novo `pending_request`, `ACCEPT_MATCHED`, `MCM_REJECT`, `MCM_TIMEOUT`, `MCM_PENDING_ABANDON`, `maxe <= 0`, `STOPPED_TOO_LONG` e `MERGE_ACCEPTED_SLOT_EXPIRED`.
* A lógica `ACCEPT_MATCHED` → `merge_accepted` → só depois `MERGE_AUTHORIZED_BY_MCM` se `accepted_ready` estiver válido está no sítio certo. 
* A telemetria do host/OBUs e a reserva até `CLEAR` continuam no código.

Eu arrancava com **10 runs**, não 5, para stressar melhor:

```bash
mkdir -p logs

python3 -m py_compile obu/app/main.py sim/bridge/traci_bridge.py scripts/generate_obu_compose.py

for i in $(seq 1 10); do
  echo "=== dense intensive $i ==="
  LOG_FILE=logs/dense_intensive_${i}.log \
  SUMO_GUI=false LOOP_SIM=false SUMO_END=120 STEP_DELAY_S=0 \
  ./scripts/run_vanetza_scenario.sh log

  docker compose down --remove-orphans >/dev/null 2>&1 || true
done
```

Resumo automático:

```bash
for f in logs/dense_intensive_*.log; do
  printf "%-32s " "$f"
  printf "Trc=%s "   "$(grep -c 'Traceback' "$f" || true)"
  printf "Warn=%s "  "$(grep -c 'Warning' "$f" || true)"
  printf "Coll=%s "  "$(grep -c 'collision' "$f" || true)"
  printf "LaneF=%s " "$(grep -c 'LANE_CMD_FAILED' "$f" || true)"
  printf "Hostl=%s " "$(grep -c 'MERGE_ALLOWED_HOSTLESS' "$f" || true)"
  printf "Acc=%s "   "$(grep -c 'MCM_ACCEPT_MATCHED' "$f" || true)"
  printf "Wait=%s "  "$(grep -c 'MERGE_ACCEPTED_WAIT_SLOT_VALID' "$f" || true)"
  printf "Exp=%s "   "$(grep -c 'MERGE_ACCEPTED_SLOT_EXPIRED' "$f" || true)"
  printf "Auth=%s "  "$(grep -c 'MERGE_AUTHORIZED_BY_MCM' "$f" || true)"
  printf "Start=%s " "$(grep -c 'MERGE_PHYSICAL_START' "$f" || true)"
  printf "Merg=%s "  "$(grep -c 'MERGING!' "$f" || true)"
  printf "Comp=%s "  "$(grep -c 'MERGE_COMPLETED:' "$f" || true)"
  printf "After=%s " "$(grep -c 'MERGE_COMPLETED_AFTER_TIMEOUT' "$f" || true)"
  printf "Lost=%s "  "$(grep -c 'MERGE_FAILED_LOST_AUTH_AFTER_POINT' "$f" || true)"
  printf "Spd0=%s "  "$(grep -c 'speed=0.00' "$f" || true)"
  printf "T018=%s\n" "$(grep -c 'target=0.18' "$f" || true)"
done
```

Critério para fechar:

```text
0 Traceback
0 Warning/collision
0 LANE_CMD_FAILED
0 MERGE_ALLOWED_HOSTLESS
10/10 runs com 6 MERGE_PHYSICAL_START
10/10 runs com 6 MERGING!
10/10 runs com 6 MERGE_COMPLETED limpo
MERGE_FAILED_LOST_AUTH_AFTER_POINT = 0
speed=0.00 / target=0.18 baixos ou zero
```

Se alguma run falhar, manda logo o resumo e o log dessa run. Agora os sinais mais importantes são `MERGE_ACCEPTED_SLOT_EXPIRED`, `MERGE_FAILED_LOST_AUTH_AFTER_POINT`, `MCM_ACCEPT_STALE/WRONG_MANOEUVRE` e se os `MERGE_AUTHORIZED_BY_MCM` já não aparecem com slot morto.
