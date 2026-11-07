| run | scenario | APE [m] | lat p95 [m] | lon p95 [m] | head p95 [deg] | max [m] | conv [s] | latency p99 [ms] (1) | age p99 [ms] | NIS mean | rejected / matches | outage drift [m] | outage recovery [s] | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dense_traffic | dense_traffic | 0.074 | 0.048 | 0.055 | 0.15 | 1.620 | 3.2 | 0.16 | 20 | 0.24 | 26 / 7705 | - | - | FAIL: horizontal max [m] |
| gnss_outage | gnss_outage | 0.086 | 0.111 | 0.073 | 0.22 | 0.394 | 3.2 | 0.19 | 20 | 0.27 | 2 / 2940 | 0.095 | 0.2 | pass |
| high_speed_turns | high_speed_turns | 0.085 | 0.096 | 0.088 | 0.19 | 0.454 | 3.1 | 0.22 | 20 | 0.28 | 3 / 4403 | - | - | pass |
| live1x_dense_traffic | dense_traffic | 0.088 | 0.064 | 0.054 | 0.17 | 1.467 | 3.1 | 0.25 | 70 | 0.31 | 17 / 5373 | - | - | FAIL: horizontal max [m] |
| live1x_gnss_outage | gnss_outage | 0.091 | 0.119 | 0.076 | 0.22 | 0.423 | 3.2 | 0.23 | 60 | 0.30 | 1 / 2942 | 0.097 | 0.2 | pass |
| live1x_high_speed_turns | high_speed_turns | 0.085 | 0.101 | 0.091 | 0.19 | 0.446 | 3.1 | 0.23 | 60 | 0.26 | 2 / 4405 | - | - | pass |
| live1x_nominal | nominal | 0.079 | 0.121 | 0.074 | 0.23 | 0.370 | 5.3 | 0.22 | 60 | 0.22 | 6 / 4983 | - | - | pass |
| live1x_relocalization | relocalization | 0.068 | 0.099 | 0.067 | 0.22 | 0.253 | 5.9 | 0.24 | 60 | 0.18 | 2 / 2291 | - | - | pass |
| live1x_reverse_reuse | reverse_reuse | 0.060 | 0.037 | 0.118 | 0.13 | 0.139 | 7.2 | 0.25 | 60 | 0.09 | 3 / 732 | - | - | pass |
| nominal | nominal | 0.079 | 0.104 | 0.077 | 0.22 | 0.395 | 3.1 | 0.21 | 237 | 0.23 | 62 / 4931 | - | - | pass |
| relocalization | relocalization | 0.073 | 0.112 | 0.064 | 0.21 | 0.305 | 3.9 | 0.22 | 20 | 0.23 | 3 / 2331 | - | - | pass |
| reverse_reuse | reverse_reuse | 0.060 | 0.034 | 0.126 | 0.18 | 0.142 | 4.3 | 0.19 | 20 | 0.08 | 3 / 785 | - | - | pass |
| step6_relocalization | relocalization | 0.089 | 0.134 | 0.072 | 0.21 | 0.425 | 3.8 | 0.23 | 20 | 0.35 | 3 / 2328 | - | - | pass |
| step7_gnss_outage | gnss_outage | 0.091 | 0.115 | 0.076 | 0.22 | 0.429 | 3.1 | 0.22 | 20 | 0.31 | 2 / 2941 | 0.093 | 0.1 | pass |
| unseen_gnss_outage_45s | gnss_outage | 0.088 | 0.116 | 0.072 | 0.22 | 0.407 | 3.1 | 0.24 | 60 | 0.27 | 1 / 2942 | 0.097 | 0.2 | pass |
| unseen_init_8m_90deg | relocalization | 0.074 | 0.106 | 0.068 | 0.22 | 0.306 | 3.4 | 0.24 | 60 | 0.22 | 1 / 2330 | - | - | pass |
| unseen_speed_plus20 | nominal | 0.150 | 0.158 | 0.137 | 0.29 | 0.734 | 5.2 | 0.23 | 80 | 0.93 | 4 / 4165 | - | - | FAIL: horizontal max [m] |
| unseen_traffic_x2 | dense_traffic | 0.049 | 0.048 | 0.054 | 0.15 | 0.147 | 3.3 | 0.26 | 50 | 0.05 | 22 / 9597 | - | - | pass |

(1) end to end on a wall-clock recorder; the node-measured processing latency when the run was recorded on sim time (10 ms steps).
