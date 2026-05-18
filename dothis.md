Vi o repositório. Está bem melhor: as alterações principais já estão no `main.py`, `entrypoint.sh`, `run_vanetza_scenario.sh` e generator.

O que está certo:

* A OBU agora subscreve `car/+/status/fsm` e guarda `remote_vehicle_status`, portanto o host já consegue ver estado real dos ramp vehicles. 
* O `status/fsm`/telemetria remota está a ser usado pelo host para decidir/logar `HOST_RESERVATION_HOLD`, `HOST_RESERVATION_EXTEND_UNTIL_CLEAR`, `HOST_RESERVATION_RELEASE_AFTER_CLEAR`, `HOST_RESERVATION_RELEASE_MAX_TIMEOUT` e `HOST_RELEASE_BEFORE_CLEAR`. 
* O `entrypoint.sh` já tem bridge inbound de `car/+/status/fsm`, portanto as OBUs conseguem receber o estado umas das outras. 
* O launcher `dense` já passa `HOST_RESERVATION_MAX_S=10.0`, `MCM_LATE_HOST_LOCK_GRACE_S=1.5` e `MCM_LATE_HOST_LOCK_DISTANCE_M=70.0`. 
* A separação entre `ACCEPT_MATCHED` e `MERGE_AUTHORIZED_BY_MCM` já foi implementada: agora existe `merge_accepted`, `accepted_ready`, `MERGE_ACCEPTED_WAIT_SLOT_VALID` e `MERGE_ACCEPTED_SLOT_EXPIRED`. Isto é mesmo o caminho certo. 

Mas encontrei **dois pontos perigosos** no patch novo.

### 1. Possível crash no log de `MERGE_ACCEPTED_WAIT_SLOT_VALID`

Tens isto:

```python
if not lgok:
    reason.append(f"lgok=False(gap={lg_v:.2f} if lg_v else 'None')")
if not hgok:
    reason.append(f"hgok=False(gap={hg_v:.2f} if hg_v else 'None')")
```

Isto não faz o ternário que parece. O Python tenta formatar `lg_v` com `:.2f` antes de avaliar a parte textual. Se `lg_v` ou `hg_v` forem `None`, dá `TypeError`.

Trocar por helper:

```python
def fmt_gap(v):
    return f"{v:.2f}" if v is not None else "None"
```

E depois:

```python
if not lgok:
    reason.append(f"lgok=False(gap={fmt_gap(lg_v)})")
if not hgok:
    reason.append(f"hgok=False(gap={fmt_gap(hg_v)})")
if not lgok_proj:
    reason.append(f"lgok_proj=False(t1={fmt_gap(lg1_v)}, t2={fmt_gap(lg2_v)})")
```

Isto é pequeno, mas importante.

### 2. `accepted_slot_invalid_since` precisa de reset mais agressivo

Agora, se um ACCEPT expira por slot inválido, fazes:

```python
self.pending_request, self.merge_accepted, self.merge_authorized = None, False, False
```

mas não limpas `accepted_slot_invalid_since`. Se vier uma nova tentativa/novo ACCEPT depois, pode herdar a idade antiga e expirar quase instantaneamente.

Eu fazia reset em todos estes pontos:

```python
self.accepted_slot_invalid_since = 0.0
```

Quando:

* crias novo `pending_request`;
* recebes novo `ACCEPT_MATCHED`;
* expira com `MERGE_ACCEPTED_SLOT_EXPIRED`;
* recebes `REJECT`;
* fazes `MCM_TIMEOUT`;
* fazes `MCM_PENDING_ABANDON`;
* completas merge.

### O que mandava ao Codex agora

```text
Vi o estado atual do repo. A direção está certa.

Manter:
- remote_vehicle_status via car/+/status/fsm
- HOST_RESERVATION_EXTEND_UNTIL_CLEAR / RELEASE_AFTER_CLEAR / RELEASE_MAX_TIMEOUT
- MCM_PENDING_HOLD_LATE_HOST
- merge_accepted
- MERGE_ACCEPTED_WAIT_SLOT_VALID
- MERGE_ACCEPTED_SLOT_EXPIRED
- MERGE_ATTEMPT_TIMELINE
- MERGE_START_GAP_DIAG

Corrigir antes de mais testes:

1. Corrigir formatting bug no log MERGE_ACCEPTED_WAIT_SLOT_VALID.

Código atual é perigoso:
if not lgok:
    reason.append(f"lgok=False(gap={lg_v:.2f} if lg_v else 'None')")

Trocar por helper:
def fmt_gap(v):
    return f"{v:.2f}" if v is not None else "None"

E usar:
reason.append(f"lgok=False(gap={fmt_gap(lg_v)})")
reason.append(f"hgok=False(gap={fmt_gap(hg_v)})")
reason.append(f"lgok_proj=False(t1={fmt_gap(lg1_v)}, t2={fmt_gap(lg2_v)})")

2. Resetar accepted_slot_invalid_since sempre que a tentativa muda.

Adicionar:
self.accepted_slot_invalid_since = 0.0

Em:
- novo pending_request
- ACCEPT_MATCHED
- MERGE_ACCEPTED_SLOT_EXPIRED
- MCM_REJECT
- MCM_TIMEOUT
- MCM_PENDING_ABANDON
- MERGE_COMPLETED

3. Depois correr 5 runs dense outra vez.

Critérios:
- 0 Traceback
- 0 Warning/collision
- 0 LANE_CMD_FAILED
- 0 hostless
- ver se MERGE_ACCEPTED_SLOT_EXPIRED aparece em vez de MERGE_AUTHORIZED_BY_MCM com gap morto
- tentar chegar a 6/6 starts/completions de forma consistente
```

Resumo: **o patch atual está no alvo certo**, mas eu não corria mais baterias antes de corrigir esses dois detalhes. O bug de formatação pode rebentar a simulação, e o `accepted_slot_invalid_since` stale pode criar expiração falsa em tentativas novas.
