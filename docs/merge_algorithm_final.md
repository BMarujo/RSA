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

## Final Post-Merge Safety Fixes
The dense scenario revealed a critical failure mode where vehicles would drop their speed to near-zero (0.18 m/s) immediately after merging or when failing a merge attempt, leading to rear-end collisions. The following fixes were implemented:

- **Post-Merge Lock (`POST_MERGE_LOCK_S`):** Protects the vehicle for the first 3 seconds after completing a merge. During this window, CAM following logic is prevented from dropping the target speed below 80% of the cruise speed, ensuring safe flow.
- **Ramp-to-Main Car Following Hardening:** All ramp vehicles approaching the merge point now actively check for conflicts with main road vehicles, regardless of their current state. This prevents vehicles in `CRUISE` state on the ramp from colliding with slow-moving traffic in the target lane during the merge transition.
- **Ramp Car-Following Restoration:** Fixed a bug where ramp vehicles would skip following leaders further than 6 meters away. Vehicles on the ramp now correctly follow their leaders regardless of gap, ensuring safe queuing in dense traffic.
- **Dynamic Abort Speed:** Replaced the hardcoded `0.18` target speed in `STATE_ABORT` (used when a vehicle is past the merge point without authorization) with the vehicle's dynamic `min_speed`, preventing abrupt and dangerous stops.
- **Abort State Lane Cancellation:** Entering `STATE_ABORT` now explicitly clears the target lane index. This ensures that if a merge attempt fails, the vehicle immediately stops trying to change lanes, preventing unauthorized and dangerous entries into the main flow.
- **Speed Recovery Hardening:** Vehicles that are virtually stopped (speed < 1.0 m/s) are now guaranteed a minimum target speed step-up of 0.3 m/s. This prevents a "speed reflection" deadlock where vehicles would get stuck at dangerously low speeds (like 0.18 m/s), ensuring they can decisively accelerate away from the merge zone.

## Reproduction Commands
To validate the current state and run the 30 dense simulations:

```bash
# Run the 30 dense scenarios and automatically check for regressions
RUNS=30 ./scripts/validate_dense_final.sh

# Summarize the output of the 30 runs
./scripts/summarize_dense_final.sh
```