import os
import subprocess

patterns = [
    "Traceback", "Warning", "collision", "LANE_CMD_FAILED",
    "MERGE_ALLOWED_HOSTLESS", "MCM_ACCEPT_MATCHED",
    "MERGE_ACCEPTED_WAIT_SLOT_VALID", "MERGE_ACCEPTED_SLOT_EXPIRED",
    "MERGE_AUTHORIZED_BY_MCM", "MERGE_PHYSICAL_START", "MERGING!",
    "MERGE_COMPLETED:", "MERGE_COMPLETED_AFTER_TIMEOUT",
    "MERGE_FAILED_LOST_AUTH_AFTER_POINT", "speed=0.00", "target=0.18"
]

log_files = sorted([f for f in os.listdir('logs') if f.startswith('dense_intensive_') and f.endswith('.log')], key=lambda x: int(x.split('_')[-1].split('.')[0]))

header = f"{'File':<32} " + " ".join([f"{p[:5]:<5}" for p in patterns])
print(header)

for f in log_files:
    path = os.path.join('logs', f)
    counts = []
    for p in patterns:
        try:
            output = subprocess.check_output(['grep', '-c', p, path])
            counts.append(output.decode().strip())
        except subprocess.CalledProcessError:
            counts.append("0")
    
    row = f"{f:<32} " + " ".join([f"{c:<5}" for c in counts])
    print(row)
