# MPC Prediction Horizon Benchmark

Recommended `horizon_steps`: **6**

Selection rule: among safe runs that reached the goal with no solver failures, choose the fastest
average MPC solve time within `0.03 m` of the best final distance.

| horizon | mean solve ms | total runtime s | final distance m | min safety margin m | steps | failures | reached |
|---:|---:|---:|---:|---:|---:|---:|:---:|
| 4 | 3.16 | 0.137 | 0.213 | 0.000000 | 39 | 0 | yes |
| 6 **best** | 3.13 | 0.128 | 0.142 | 0.000000 | 39 | 0 | yes |
| 8 | 3.45 | 0.138 | 0.169 | 0.000000 | 38 | 0 | yes |
| 10 | 4.85 | 0.192 | 0.249 | 0.000000 | 38 | 0 | yes |
| 12 | 5.23 | 0.208 | 0.249 | 0.000000 | 38 | 0 | yes |
| 16 | 6.76 | 0.269 | 0.248 | 0.000000 | 38 | 0 | yes |
| 20 | 6.46 | 0.254 | 0.243 | 0.000000 | 37 | 0 | yes |
| 24 | 5.87 | 0.235 | 0.243 | 0.000000 | 37 | 0 | yes |
| 28 | 6.09 | 0.245 | 0.243 | 0.000000 | 37 | 0 | yes |
| 32 | 6.75 | 0.273 | 0.244 | 0.000000 | 37 | 0 | yes |
| 36 | 7.13 | 0.290 | 0.244 | 0.000000 | 37 | 0 | yes |
| 40 | 7.81 | 0.317 | 0.244 | 0.000000 | 37 | 0 | yes |

Interpretation:
- Smaller `mean solve ms` means faster online MPC replanning.
- Smaller `final distance m` means better terminal accuracy.
- Non-negative `min safety margin m` means the closed-loop path stayed outside safety circles.