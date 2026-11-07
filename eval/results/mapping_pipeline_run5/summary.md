# Mapping evaluation

| trajectory | poses | APE RMSE [m] | APE max [m] | RPE 1 m RMSE [m] | RPE 100 m RMSE [m] |
|---|---|---|---|---|---|
| trajectory_optimized_tum.txt | 3328 | 0.044 | 0.176 | 0.014 | 0.117 |

| map | direction | mean [m] | median [m] | p95 [m] | > 0.5 m [%] |
|---|---|---|---|---|---|
| town03.pcd | map_to_reference | 0.083 | 0.055 | 0.253 | 1.05 |
| town03.pcd | reference_to_map | 0.122 | 0.077 | 0.364 | 2.23 |
| town03_static.pcd | map_to_reference | 0.083 | 0.055 | 0.253 | 1.06 |
| town03_static.pcd | reference_to_map | 0.123 | 0.077 | 0.368 | 2.30 |
