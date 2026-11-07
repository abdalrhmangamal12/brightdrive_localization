# Mapping evaluation

| trajectory | poses | APE RMSE [m] | APE max [m] | RPE 1 m RMSE [m] | RPE 100 m RMSE [m] |
|---|---|---|---|---|---|
| trajectory_optimized_tum.txt | 3329 | 0.047 | 0.184 | 0.014 | 0.126 |
| trajectory_fastlio_in_map_tum.txt | 3329 | 0.391 | 0.819 | 0.014 | 0.106 |

| map | direction | mean [m] | median [m] | p95 [m] | > 0.5 m [%] |
|---|---|---|---|---|---|
| town03.pcd | map_to_reference | 0.084 | 0.056 | 0.251 | 1.03 |
| town03.pcd | reference_to_map | 0.123 | 0.078 | 0.364 | 2.23 |
| town03_static.pcd | map_to_reference | 0.084 | 0.056 | 0.252 | 1.04 |
| town03_static.pcd | reference_to_map | 0.124 | 0.078 | 0.368 | 2.31 |
