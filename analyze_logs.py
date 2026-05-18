
import re
import os

logs_dir = 'logs'
log_files = sorted([f for f in os.listdir(logs_dir) if f.startswith('dense_final_') and f.endswith('.log')], key=lambda x: int(re.search(r'\d+', x).group()))
vehicles = ['Merge_Car', 'Ramp_Merge_2', 'Ramp_Merge_3', 'Ramp_Merge_4', 'Ramp_Merge_5', 'Ramp_Back']

events = {
    'PHYSICAL_START': 'MERGE_PHYSICAL_START',
    'MERGING': 'MERGING!',
    'COMPLETED': 'MERGE_COMPLETED:',
    'TIMEOUT': 'MERGE_COMPLETED_AFTER_TIMEOUT',
    'ABORT': 'STATE_ABORT|MCM_REJECT',
    'SAFETY_HOLD': 'MERGE_SAFETY_HOLD',
    'WAIT_EDGE': 'LANE_CMD_WAIT_EDGE|WAIT_EDGE',
    'LANE_CMD_APPLY': 'LANE_CMD_APPLY',
    'LANE_CMD_CLEAR': 'LANE_CMD_CLEAR',
    'POST_MERGE_CF': 'POST_MERGE_CAR_FOLLOW',
    'POST_CLEAR_CF': 'POST_CLEAR_CAR_FOLLOW'
}

results = {}

for log_file in log_files:
    results[log_file] = {}
    path = os.path.join(logs_dir, log_file)
    with open(path, 'r') as f:
        content = f.readlines()
        
    for vehicle in vehicles:
        results[log_file][vehicle] = {event: 0 for event in events}
        vehicle_lines = [line for line in content if vehicle in line]
        
        for event_name, pattern in events.items():
            count = sum(1 for line in vehicle_lines if re.search(pattern, line))
            results[log_file][vehicle][event_name] = count

# Print the table
header = "Log File | Vehicle | " + " | ".join(events.keys())
print(header)
print("-" * len(header))

for log_file in log_files:
    for vehicle in vehicles:
        counts = [str(results[log_file][vehicle][event]) for event in events]
        print(f"{log_file} | {vehicle} | " + " | ".join(counts))

# Identify problematic instances
print("\n--- Problematic Instances ---")
for log_file in log_files:
    for vehicle in vehicles:
        res = results[log_file][vehicle]
        if res['PHYSICAL_START'] > 0 and (res['COMPLETED'] == 0 and res['TIMEOUT'] == 0):
            print(f"FAILED (Incomplete): {vehicle} in {log_file}")
        if res['TIMEOUT'] > 0:
            print(f"TIMEOUT: {vehicle} in {log_file}")
