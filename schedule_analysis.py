# coding: utf-8

def rms_utilization_test(tasks):
    """RMS Schedulability Test using Liu & Layland bound"""
    print("\n" + "="*70)
    print("RATE MONOTONIC SCHEDULING (RMS) ANALYSIS")
    print("="*70)
    
    n = len(tasks)
    U_bound = n * (2**(1/n) - 1)
    
    U_total = 0
    print(f"\n{'Task':<18} | {'WCET':<8} | {'Period':<8} | {'Utilization':<12} | {'Priority':<8}")
    print("-" * 70)
    
    # Sort by period (RMS assigns priority by period)
    sorted_tasks = sorted(tasks, key=lambda t: t.period)
    
    for i, task in enumerate(sorted_tasks):
        U_i = task.wcet / task.period
        U_total += U_i
        priority = i + 1  # 1 = highest
        print(f"{task.name:<18} | {task.wcet:<8.2f} | {task.period:<8} | {U_i:<12.4f} | {priority:<8}")
    
    print("-" * 70)
    print(f"Total Utilization: U = {U_total:.4f} ({U_total*100:.2f}%)")
    print(f"RMS Bound: U_bound = {U_bound:.4f} ({U_bound*100:.2f}%)")
    print(f"Safety Margin: {(U_bound - U_total):.4f} ({(U_bound - U_total)*100:.2f}%)")
    
    if U_total <= U_bound:
        print(f"✓ SCHEDULABLE by RMS (U ≤ U_bound)")
    elif U_total <= 1.0:
        print(f"⚠ POSSIBLY SCHEDULABLE (U_bound < U ≤ 1.0, need response time analysis)")
    else:
        print(f"✗ NOT SCHEDULABLE (U > 1.0)")
    
    return U_total <= 1.0

def edf_utilization_test(tasks):
    """EDF Schedulability Test using Utilization Bound"""
    print("\n" + "="*70)
    print("EARLIEST DEADLINE FIRST (EDF) ANALYSIS")
    print("="*70)
    
    U_total = 0
    print(f"\n{'Task':<18} | {'WCET':<8} | {'Period':<8} | {'Deadline':<10} | {'Utilization':<12}")
    print("-" * 70)
    
    for task in tasks:
        U_i = task.wcet / task.period
        U_total += U_i
        deadline = task.period  # Assuming implicit deadline (D = T)
        print(f"{task.name:<18} | {task.wcet:<8.2f} | {task.period:<8} | {deadline:<10} | {U_i:<12.4f}")
    
    print("-" * 70)
    print(f"Total Utilization: U = {U_total:.4f} ({U_total*100:.2f}%)")
    print(f"EDF Bound: U_bound = 1.0000 (100.00%)")
    print(f"Safety Margin: {(1.0 - U_total):.4f} ({(1.0 - U_total)*100:.2f}%)")
    
    if U_total <= 1.0:
        print(f"✓ SCHEDULABLE by EDF (U ≤ 1.0)")
    else:
        print(f"✗ NOT SCHEDULABLE (U > 1.0)")
    return U_total <= 1.0

def main():
    # Task definitions with WCET from measurements (in ms)
    class Task:
        def __init__(self, name, wcet, period):
            self.name = name
            self.wcet = wcet
            self.period = period
    
    tasks = [
        Task("runThread", 0.32, 100),           # 316 μs = 0.32 ms
        Task("sendStatusThread", 9.23, 100),    # 9228 μs = 9.23 ms
        Task("displayThread", 1.27, 300),       # 1272 μs = 1.27 ms
        Task("heartbeatThread", 0.01, 1000)     # 10 μs = 0.01 ms
    ]

    print("="*70)
    print("PLATOONING SYSTEM - SCHEDULING ANALYSIS")
    print("="*70)
    print("\nTask Set Configuration:")
    print(f"{'Task':<18} | {'WCET (ms)':<10} | {'Period (ms)':<12}")
    print("-" * 70)
    for task in tasks:
        print(f"{task.name:<18} | {task.wcet:<10.2f} | {task.period:<12}")

    # RMS Schedulability Analysis
    rms_schedulable = rms_utilization_test(tasks)
    
    # EDF Schedulability Analysis
    edf_schedulable = edf_utilization_test(tasks)
    
if __name__ == "__main__":
    main()
