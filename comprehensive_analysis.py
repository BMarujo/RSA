
import os
import glob
import re

vehicles = ['Merge_Car', 'Ramp_Merge_2', 'Ramp_Merge_3', 'Ramp_Merge_4', 'Ramp_Merge_5', 'Ramp_Back']
log_files = sorted(glob.glob('logs/dense_final_*.log'))

events = {
    'MERGE_PHYSICAL_START': 'MERGE_PHYSICAL_START',
    'MERGING!': 'MERGING!',
    'MERGE_COMPLETED': 'MERGE_COMPLETED:',
    'MERGE_TIMEOUT': 'MERGE_COMPLETED_AFTER_TIMEOUT',
    'ABORT': ['STATE_ABORT', 'MCM_REJECT'],
    'SAFETY_HOLD': 'SAFETY_HOLD',
    'WAIT_EDGE': 'WAIT_EDGE',
    'LANE_CMD_APPLY': 'LANE_CMD_APPLY',
    'LANE_CMD_CLEAR': 'LANE_CMD_CLEAR',
    'POST_MERGE_CF': 'POST_MERGE_CAR_FOLLOW',
    'POST_CLEAR_CF': 'POST_CLEAR_CAR_FOLLOW'
}

def analyze():
    report = []
    
    # Global counts: {log_file: {vehicle: {event: count}}}
    all_counts = {}

    timeout_cases = []
    no_completion_cases = []

    for log_file in log_files:
        counts = {v: {e: 0 for e in events} for v in vehicles}
        all_counts[log_file] = counts
        
        # Track state for diagnostics
        vehicle_data = {v: {
            'last_edge': None,
            'last_lane': None,
            'wait_edge_count': 0,
            'has_physical_start': False,
            'has_completion': False,
            'in_safety_hold': False,
            'last_host': None,
            'last_lead': None,
            'last_neighbors': None
        } for v in vehicles}

        with open(log_file, 'r') as f:
            for line in f:
                for v in vehicles:
                    if v in line:
                        # Count events
                        for e_name, e_pattern in events.items():
                            if isinstance(e_pattern, list):
                                if any(p in line for p in e_pattern):
                                    counts[v][e_name] += 1
                            else:
                                if e_pattern in line:
                                    counts[v][e_name] += 1
                        
                        # Diagnostics collection
                        if 'edge=' in line:
                            m = re.search(r'edge=([^\s,]+)', line)
                            if m: vehicle_data[v]['last_edge'] = m.group(1)
                        if 'lane=' in line:
                            m = re.search(r'lane=([^\s,]+)', line)
                            if m: vehicle_data[v]['last_lane'] = m.group(1)
                        
                        if 'LANE_CMD_WAIT_EDGE' in line:
                            vehicle_data[v]['wait_edge_count'] += 1
                        
                        if 'MERGE_SLOT_QUALITY_DIAG' in line:
                            m_host = re.search(r'host=([^\s,]+)', line)
                            m_lead = re.search(r'lead=([^\s,]+)', line)
                            if m_host: vehicle_data[v]['last_host'] = m_host.group(1)
                            if m_lead: vehicle_data[v]['last_lead'] = m_lead.group(1)
                        
                        if 'neighbors=' in line:
                            m_neigh = re.search(r'neighbors=(\d+)', line)
                            if m_neigh: vehicle_data[v]['last_neighbors'] = m_neigh.group(1)

                        if 'MERGE_PHYSICAL_START' in line:
                            vehicle_data[v]['has_physical_start'] = True
                        
                        if 'MERGE_COMPLETED:' in line or 'MERGE_COMPLETED_AFTER_TIMEOUT' in line:
                            vehicle_data[v]['has_completion'] = True
                        
                        if 'SAFETY_HOLD' in line:
                            vehicle_data[v]['in_safety_hold'] = True
                        elif 'MERGE_COMPLETED' in line or 'ABORT' in line or 'STATE_ABORT' in line:
                             vehicle_data[v]['in_safety_hold'] = False

                        if 'MERGE_COMPLETED_AFTER_TIMEOUT' in line:
                            timeout_cases.append({
                                'log': log_file,
                                'vehicle': v,
                                'wait_edge_count': vehicle_data[v]['wait_edge_count'],
                                'in_safety_hold': vehicle_data[v]['in_safety_hold'],
                                'last_edge': vehicle_data[v]['last_edge'],
                                'last_lane': vehicle_data[v]['last_lane'],
                                'host': vehicle_data[v]['last_host'],
                                'lead': vehicle_data[v]['last_lead'],
                                'neighbors': vehicle_data[v]['last_neighbors']
                            })

        # Check for MERGE_PHYSICAL_START without completion
        for v in vehicles:
            if vehicle_data[v]['has_physical_start'] and not vehicle_data[v]['has_completion']:
                no_completion_cases.append({
                    'log': log_file,
                    'vehicle': v,
                    'last_edge': vehicle_data[v]['last_edge'],
                    'last_lane': vehicle_data[v]['last_lane']
                })

    # Generate Report
    with open('analysis_report.txt', 'w') as out:
        out.write("PER-VEHICLE EVENT COUNTS BY LOG FILE\n")
        out.write("="*80 + "\n\n")
        
        headers = ['Log File', 'Vehicle'] + list(events.keys())
        header_row = "{:<20} {:<15} " + " ".join(["{:<10}"] * len(events))
        out.write(header_row.format(*headers) + "\n")
        out.write("-" * (35 + 11 * len(events)) + "\n")
        
        for log_file in log_files:
            for v in vehicles:
                row = [os.path.basename(log_file), v]
                for e in events:
                    row.append(all_counts[log_file][v][e])
                out.write(header_row.format(*row) + "\n")
            out.write("-" * (35 + 11 * len(events)) + "\n")

        out.write("\n\nDIAGNOSTICS\n")
        out.write("="*80 + "\n")
        
        out.write("\nTIMEOUT CASES (MERGE_COMPLETED_AFTER_TIMEOUT):\n")
        if not timeout_cases:
            out.write("None found.\n")
        for case in timeout_cases:
            out.write(f"- Log: {case['log']}, Vehicle: {case['vehicle']}\n")
            out.write(f"  * Stuck in WAIT_EDGE? {'Yes' if case['wait_edge_count'] > 10 else 'No'} ({case['wait_edge_count']} logs)\n")
            out.write(f"  * In SAFETY_HOLD at timeout? {'Yes' if case['in_safety_hold'] else 'No'}\n")
            out.write(f"  * Last reported Edge/Lane: {case['last_edge']} / {case['last_lane']}\n")
            out.write(f"  * Nearby Vehicles: Host={case['host']}, Lead={case['lead']}, Neighbors={case['neighbors']}\n")
            out.write("\n")

        out.write("\nMERGE_PHYSICAL_START WITHOUT COMPLETION:\n")
        if not no_completion_cases:
            out.write("None found.\n")
        for case in no_completion_cases:
            out.write(f"- Log: {case['log']}, Vehicle: {case['vehicle']}\n")
            out.write(f"  * Last reported Edge/Lane: {case['last_edge']} / {case['last_lane']}\n")
            out.write("\n")

if __name__ == "__main__":
    analyze()
