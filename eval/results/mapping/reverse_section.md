# Reserved reverse section against the mapping drive

Ground truth of `/bags/mapping_20261105_070252` and `/bags/reverse_reuse_20261107_153511`; map `/maps/town03_static.yaml`. Written by `eval/check_reverse_section.py`.

## Mapping drive

| | value |
|---|---|
| path, time | 3914.4 m in 568.6 s (after 5.2 s standing at the start) |
| speed | median 7.88 m/s, max 8.32 m/s; stopped 52.2 s while driving |
| laps | 2: 1957.5 m, 1956.5 m |

Positions of the first lap by the number of same-direction passes within 3 m over the whole drive:

| passes | share of the first lap |
|---|---|
| 2 | 98.5 % |
| 3 | 0.2 % |
| 4 | 1.3 % |

Stretches of the first lap with extra passes on another lane of the same road (within 6 m), at least 10 m long:

| lap metres | from | to | extra passes |
|---|---|---|---|
| 446 to 558 | (7, -120) | (16, -10) | same direction, another lane: 4 same, 0 opposite |
| 1759 to 1873 | (1, -120) | (17, -11) | same direction, another lane: 4 same, 0 opposite |
| 705 to 830 | (16, 133) | (140, 131) | both directions: 2 same, 2 opposite |
| 1180 to 1307 | (141, 137) | (14, 138) | both directions: 2 same, 2 opposite |

## Reserved section

`main` waypoints 248 to 299: 323.6 m planned, (-88.5, 58.9) to (1.7, -80.3).

## Reverse drive

(-6.4, -75.2) to (-78.0, 48.5): 288.6 m in 37.7 s, max 8.11 m/s.

| leg | length [m] | nearest mapping point min / median / max [m] | heading difference, median [deg] |
|---|---|---|---|
| southbound, x ≈ -10 to -7 | 44 | 8.0 / 10.9 / 11.2 | 177 |
| westbound, y ≈ -132 to -129 | 46 | 3.1 / 3.6 / 5.2 | 178 |
| northbound, x ≈ -78 to -73 | 171 | 10.5 / 10.8 / 14.0 | 179 |
| corners | 28 | at least 3.5 | at least 126 |

## Proof

| check | value | required |
|---|---|---|
| same direction: a mapping sample within 3 m heading within 30 deg | 0.0 % | 0 % |
| map coverage: at least 25 road points within 1 m | 100.0 % (median 90, min 68; mapped lanes median 111) | 100 % |
| for reference: nearest mapping point within 3 m | 0.0 % | (not a criterion) |

**PASS**
