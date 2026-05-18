# Merge Algorithm Improvements - Final

## Initial Problem
The merge algorithm initially struggled in dense traffic scenarios, leading to several issues:
- **Deadlocks/Timeouts:** Ramp vehicles would time out waiting for hosts that were permanently blocked by preceding vehicles (busy hosts).
- **Incomplete Merges (Hostless):** Vehicles arriving at the end of a platoon on the main road would fail to merge because they had no host vehicle to authorize the merge, leaving them stranded.
- **Rear-End Collisions:** Vehicles merging early onto the acceleration lane could collide with traffic approaching from behind in the main lane due to inadequate safety checks.

## Improvements Implemented

### Busy Host Skip
The FSM was updated to detect if a candidate host vehicle is already busy accommodating another merging vehicle (`active_merge_request = True`). If a host is busy, it is skipped during the host selection phase, allowing the ramp vehicle to negotiate with a different host further upstream. This significantly reduced deadlock and waiting times.

### Lead-Only Authorization After Last Main
A surgical patch was added to authorize merges for vehicles that find themselves at the end of the main road platoon. When a vehicle has no main neighbors behind it (`true_after_last_main` or `no_main_neighbors`), it can authorize its own merge in a "hostless" or "lead-only" state. The merge is only allowed if all other safety guards (lead gap, final guard, clearance, entry speed) are rigorously satisfied.

### Dynamic TTC Final Guard
To resolve rear-end collisions during the acceleration lane phase, `FINAL_GUARD` was upgraded. The static ETA-based check was inadequate for vehicles moving side-by-side at drastically different speeds. A dynamic Time-To-Collision (TTC) metric was added:
- Evaluates the closing speed (`lon / (speed_diff)`) for vehicles in adjacent lanes.
- If an approaching vehicle from behind has a TTC of less than `FINAL_GUARD_TTC_S` (default: 3.0s), the merge is rejected.
- The lateral tolerance for this check is configured via `FINAL_GUARD_LATERAL_MULT` (default: 2.0).

## Validation Criteria
The algorithm was validated against 10 continuous runs of the `dense` scenario. The passing criteria for "perfect safety" are:
- **0 Tracebacks**
- **0 Warnings**
- **0 Collisions**
- **0 LANE_CMD_FAILED**
- **0 MERGE_ALLOWED_HOSTLESS** (Global hostless bypass flag disabled)
- **Exactly 60 MERGE_PHYSICAL_START**
- **Exactly 60 MERGING!**
- **Exactly 60 total completions** (MERGE_COMPLETED + MERGE_COMPLETED_AFTER_TIMEOUT)

## Reproduction Commands
To validate the current state and run the 10 dense simulations:

```bash
# Run the 10 dense scenarios and automatically check for regressions
./scripts/validate_dense_final.sh

# Summarize the output of the 10 runs
./scripts/summarize_dense_final.sh
```