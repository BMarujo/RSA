
import re
import os

logs_dir = 'logs'
timeouts = [
    ('dense_final_1.log', 'Ramp_Merge_5'),
    ('dense_final_6.log', 'Ramp_Merge_2'),
    ('dense_final_6.log', 'Ramp_Merge_3'),
    ('dense_final_6.log', 'Ramp_Merge_5'),
    ('dense_final_7.log', 'Ramp_Merge_5'),
    ('dense_final_8.log', 'Ramp_Merge_3'),
    ('dense_final_8.log', 'Ramp_Merge_5'),
    ('dense_final_8.log', 'Ramp_Back'),
    ('dense_final_10.log', 'Merge_Car'),
    ('dense_final_10.log', 'Ramp_Back')
]

for log_file, vehicle in timeouts:
    print(f"\n=== Investigating {vehicle} in {log_file} ===")
    path = os.path.join(logs_dir, log_file)
    with open(path, 'r') as f:
        lines = f.readlines()
    
    # Find the time of PHYSICAL_START and TIMEOUT
    start_time = None
    timeout_time = None
    for line in lines:
        if vehicle in line and 'MERGE_PHYSICAL_START' in line:
            match = re.search(r'\[(\d+\.?\d*)\]', line)
            if match:
                start_time = float(match.group(1))
        if vehicle in line and 'MERGE_COMPLETED_AFTER_TIMEOUT' in line:
            match = re.search(r'\[(\d+\.?\d*)\]', line)
            if match:
                timeout_time = float(match.group(1))
    
    if start_time and timeout_time:
        print(f"Physical Start at: {start_time}, Timeout at: {timeout_time}")
        # Look for relevant logs between start and timeout
        relevant_lines = []
        for line in lines:
            if vehicle in line:
                match = re.search(r'\[(\d+\.?\d*)\]', line)
                if match:
                    t = float(match.group(1))
                    if start_time <= t <= timeout_time:
                        relevant_lines.append(line)
        
        # Check for WAIT_EDGE
        wait_edge_count = sum(1 for line in relevant_lines if 'LANE_CMD_WAIT_EDGE' in line or 'WAIT_EDGE' in line)
        print(f"WAIT_EDGE occurrences during merge: {wait_edge_count}")
        
        # Check for SAFETY_HOLD
        safety_hold_count = sum(1 for line in relevant_lines if 'MERGE_SAFETY_HOLD' in line)
        print(f"SAFETY_HOLD occurrences during merge: {safety_hold_count}")
        
        # Last position and state before timeout
        last_decision = None
        for line in reversed(relevant_lines):
            if 'MERGE_DECISION' in line or 'MERGE_SLOT_QUALITY_DIAG' in line:
                last_decision = line
                break
        if last_decision:
            print(f"Last status before timeout: {last_decision.strip()}")
            
        # Check for proximity/conflicts
        # Look for MCM_REJECT or other hints
        rejects = [line.strip() for line in relevant_lines if 'MCM_REJECT' in line or 'STATE_ABORT' in line]
        if rejects:
            print(f"Aborts/Rejects found: {len(rejects)}")
            for r in rejects[-3:]: # Show last 3
                print(f"  {r}")
    else:
        print(f"Could not find start/timeout times for {vehicle} (Start: {start_time}, Timeout: {timeout_time})")

