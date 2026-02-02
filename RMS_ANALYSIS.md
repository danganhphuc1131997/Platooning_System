# RMS Utilization Calculation

## Task Set Configuration

| Task               | WCET (ms) | Period (ms) | Priority       |
|--------------------|-----------|-------------|----------------|
| runThread          | 2.25      | 10          | P1 (highest)   |
| sendStatusThread   | 25.86     | 50          | P2             |
| displayThread      | 0.15      | 50          | P3             |
| heartbeatThread    | 0.08      | 1000        | P4 (lowest)    |

## Utilization Formula

### Per-Task Utilization:
```
U_i = C_i / T_i
```
Where:
- `C_i` = WCET (Worst-Case Execution Time) of task i
- `T_i` = Period of task i

### Detailed Calculation:

**1. runThread:**
```
U₁ = 2.25 / 10 = 0.2250 (22.50%)
```

**2. sendStatusThread:**
```
U₂ = 25.86 / 50 = 0.5172 (51.72%)
```

**3. displayThread:**
```
U₃ = 0.15 / 50 = 0.0030 (0.30%)
```

**4. heartbeatThread:**
```
U₄ = 0.08 / 1000 = 0.0001 (0.01%)
```

### Total System Utilization:
```
U_total = U₁ + U₂ + U₃ + U₄
U_total = 0.2250 + 0.5172 + 0.0030 + 0.0001
U_total = 0.7453 (74.53%)
```

## RMS Schedulability Condition (Liu & Layland Bound)

### Formula:
```
U_bound = n × (2^(1/n) - 1)
```
Where:
- `n` = number of tasks = 4

### Calculation:
```
U_bound = 4 × (2^(1/4) - 1)
U_bound = 4 × (1.18921 - 1)
U_bound = 4 × 0.18921
U_bound = 0.7568 (75.68%)
```

## Schedulability Conclusion

```
U_total = 0.7453 < U_bound = 0.7568
```

**✅ SYSTEM IS SCHEDULABLE UNDER RMS**

Since total utilization (74.53%) is less than the RMS bound (75.68%), the system guarantees:
- All tasks will complete before their deadlines
- No task will miss its deadline
- Real-time requirements are satisfied

## Safety Margin:
```
Margin = U_bound - U_total = 0.7568 - 0.7453 = 0.0115 (1.15%)
```

The system has 1.15% remaining utilization that can accommodate additional tasks or increased WCET.
