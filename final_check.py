import os
import re

def final_summary():
    log_dir = "logs"
    if not os.path.exists(log_dir):
        print("No logs directory found.")
        return
    files = sorted([f for f in os.listdir(log_dir) if f.startswith("dense_final_") and f.endswith(".log")], 
                   key=lambda x: int(re.search(r'(\d+)', x).group(1)) if re.search(r'(\d+)', x) else 0)
    
    if not files:
        print("No logs found.")
        return

    print(f"{'File':<20} | {'Start':<5} | {'Merg!':<5} | {'Comp':<5} | {'Clean':<5} | {'TO':<3} | {'Special':<7} | {'Collisions':<10}")
    print("-" * 80)
    
    total_start = 0
    total_merg = 0
    total_comp = 0
    total_clean = 0
    total_to = 0
    total_special = 0
    total_collisions = 0
    
    for f in files:
        path = os.path.join(log_dir, f)
        with open(path, 'r') as log:
            content = log.read()
            start = content.count('MERGE_PHYSICAL_START')
            merg = content.count('MERGING!')
            comp = content.count('MERGE_COMPLETED:')
            clean = content.count('clean=True')
            to = content.count('AFTER_TIMEOUT')
            special = content.count('MERGE_AUTHORIZED_LEAD_ONLY_AFTER_LAST_MAIN')
            cols = content.count('collision')
            
            print(f"{f:<20} | {start:<5} | {merg:<5} | {comp:<5} | {clean:<5} | {to:<3} | {special:<7} | {cols:<10}")
            
            total_start += start
            total_merg += merg
            total_comp += comp
            total_clean += clean
            total_to += to
            total_special += special
            total_collisions += cols
            
    print("-" * 80)
    print(f"{'TOTAL':<20} | {total_start:<5} | {total_merg:<5} | {total_comp:<5} | {total_clean:<5} | {total_to:<3} | {total_special:<7} | {total_collisions:<10}")

if __name__ == "__main__":
    final_summary()