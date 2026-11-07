# brightdrive_mapping

Mapping backend for BrightDrive: keyframes from a LiDAR-inertial front-end, a GTSAM pose graph with odometry,
GNSS and loop-closure factors, and the georeferenced map. FAST-LIO (OmerMersin/FAST_LIO_OUSTER @ 8ffac51) is the
front-end; it is cloned unmodified in the Docker image and is not part of this package. Everything here is mine.

## Purpose and input contract

The backend subscribes to three topics and needs nothing else:

| Topic | Type | Content |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | body (IMU) pose in the front-end's fixed frame (`origin_frame`), one per scan |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | the deskewed scan in the body frame, same stamp as the pose |
| `/gnss/fix` | `sensor_msgs/NavSatFix` | raw fixes, with or without covariance |

A scan and a pose pair by stamp (1 ms). Any front-end that publishes these works unchanged: the frame name comes
from `backend.yaml`, and no class, function or member refers to FAST-LIO. Outputs: TF `map → origin_frame` on
every odometry message once GNSS alignment exists, `/mapping/path` after every keyframe and `/mapping/global_map`
every 5 s (both only while subscribed), the `/mapping/save_map` service, and the files in `/maps`.

`mapping_core` (`types.hpp`, `params.hpp`, `geo`, `PoseGraph`, `Backend`) is a static library without ROS;
`backend_node.cpp` is the only ROS code. Frames: `map` → `origin_frame` (this backend: georeference and loop
correction) → the front-end's own frames. GNSS and loop closure are each a YAML switch.

## Class diagram

```mermaid
%%{init: {"theme": "neutral"}}%%
classDiagram
    direction LR

    class Backend {
        -cfg_ : BackendConfig
        -graph_ : PoseGraph
        -keyframes_ : vector~Keyframe~
        -loops_ : vector~LoopEdge~
        -odom_hist_ : map~double, Isometry3d~
        -gnss_buf_ : vector~GnssFix~
        -gnss_attached_ : vector~pair~int, Vector3d~~
        -T_corr_ : Isometry3d
        -path_len_ : double
        -aligned_ : bool
        -align_time_ : double
        -n_gnss_dropped_, n_loops_rejected_ : int
        -loop_checked_ : set~int~
        -mtx_ : mutex
        +Backend(cfg : const BackendConfig ref)
        +addOdometry(t : double, T_odom : Isometry3d) void
        +processScan(t : double, T_odom : Isometry3d, cloud : CloudPtr) bool
        +addGnssFix(fix : const GnssFix ref) void
        +tryLoopClosure() bool
        +mapToOdomTransform() optional~Isometry3d~
        +keyframePathInMap() vector~Isometry3d~
        +buildGlobalMap(leaf : double) CloudPtr
        +saveMap(dir : string, source : MapSource) bool
        +aligned() bool
        +numKeyframes() int
        +numLoops() int
        +numLoopsRejected() int
        -isKeyframe(T_odom) bool
        -preprocess(cloud) CloudPtr
        -interpolate(t) Isometry3d
        -tryAlign() void
        -attachGnss(fix) void
        -attachBufferedUpTo(t_latest) void
        -findLoopCandidate(snapshot) optional~int~
        -matchScans(snapshot, prev, T_rel, fitness) bool
    }

    class PoseGraph {
        -impl_ : unique_ptr~Impl~
        +PoseGraph(relin_threshold : double, relin_skip : int)
        +addKeyframeNode(k, guess : Isometry3d) void
        +addPriorFactor(k, T : Isometry3d, sigmas : Vector6d) void
        +addOdomFactor(from, to, T_rel, sigmas) void
        +addLoopFactor(prev, cur, T_rel, sigmas) void
        +addMapOriginNode(T_mo0, sigmas) void
        +addGnssFactor(k, p_map : Vector3d, cov : Matrix3d, lever_arm, huber_k) void
        +update(extra_iterations) void
        +batchOptimize() void
        +keyframePose(k) Isometry3d
        +keyframePoses() vector~Isometry3d~
        +mapOrigin() optional~Isometry3d~
        +numKeyframes() int
    }

    class Impl {
        <<struct, pose_graph.cpp only>>
        +isam : ISAM2
        +new_factors, all_factors : NonlinearFactorGraph
        +new_values, estimate : Values
        +n_keyframes : int
        +has_origin : bool
    }

    class Keyframe {
        <<struct>>
        +id : int
        +t : double
        +T_odom : Isometry3d
        +cloud_body : CloudConstPtr
    }

    class GnssFix {
        <<struct>>
        +t : double
        +p_map : Vector3d
        +cov : Matrix3d
    }

    class LoopEdge {
        <<struct>>
        +cur : int
        +prev : int
        +T_rel : Isometry3d
        +fitness : double
    }

    class MapSource {
        <<struct>>
        +bag : string
        +frontend_commit : string
        +backend_commit : string
    }

    class BackendConfig {
        <<struct, every backend.yaml value>>
        +lever_arm : Vector3d
        +kf_dist, kf_angle, kf_leaf, min_range, max_range : double
        +prior_sigma, odom_sigma, origin_sigma : Vector6d
        +gnss_enable : bool
        +align_distance, max_time_shift, gnss_huber_k, gnss_default_sigma : double
        +geo_projection : string
        +geo_origin_lat_deg, geo_origin_lon_deg, geo_origin_alt_m : double
        +loop_enable : bool
        +loop_radius, loop_time_diff, loop_icp_leaf, loop_fitness : double
        +loop_search_num : int
        +loop_rot_scale, loop_trans_scale : double
        +map_frame, origin_frame, map_name : string
        +save_resolution : double
        +save_keyframes : bool
    }

    class geo {
        <<namespace, Eigen only>>
        +Projection : enum SphericalMercator
        +parseProjection(name)$ Projection
        +align4Dof(p_ant_odom : vector~Vector3d~, p_map : vector~Vector3d~)$ Isometry3d
    }

    class Projector {
        <<geo, built once from geo_*>>
        -type_ : Projection
        -lat0_deg_, lon0_deg_, alt0_m_ : double
        +llaToMap(lat_deg, lon_deg, alt_m) Vector3d
        +mapToLla(p_map) Vector3d
    }

    class BackendNode {
        <<ROS 2, backend_node.cpp>>
        -backend_ : Backend
        -projector_ : Projector
        -odom_buf_ : stamp to Isometry3d
        -pending_ : clouds waiting for odometry
        -loop_thread_ : thread
        -map_timer_, status_timer_ : TimerBase
        -tf_broadcaster_ : TransformBroadcaster
        -pub_path_, pub_map_ : Publisher
        +odomCallback(msg) void
        +cloudCallback(msg) void
        +gnssCallback(msg) void
        +loopThread() void
        +mapTimer() void
        +saveMapCallback(req, res) void
        +stop() void
    }

    Backend "1" *-- "1" BackendConfig : owns
    Backend "1" *-- "1" PoseGraph : owns
    Backend "1" *-- "0..*" Keyframe : owns
    Backend "1" *-- "0..*" LoopEdge : owns
    Backend "1" o-- "0..*" GnssFix : buffers until attached
    Backend ..> geo : align4Dof
    Backend ..> MapSource : writes into map.yaml
    PoseGraph "1" *-- "1" Impl : pimpl, GTSAM lives here
    BackendNode "1" *-- "1" Backend : owns
    BackendNode "1" *-- "1" Projector : owns
    BackendNode ..> Projector : llaToMap
    BackendNode ..> GnssFix : creates
    BackendNode ..> MapSource : fills from parameters
    BackendNode ..> BackendConfig : fills from parameters
```

`PoseGraph` is the only place GTSAM appears: Eigen types in, Eigen types out, keys `X(k)` for keyframes and `G(0)`
for `T_mo`. Prior and odometry are `PriorFactor`/`BetweenFactor<Pose3>` with diagonal sigmas, the loop factor a
`BetweenFactor<Pose3>` under Cauchy(1.0), the GNSS factor an `ExpressionFactor<Point3>` on
`T_mo · X_k · lever_arm` under Huber; no custom factor class. ICP, the candidate search, the voxel filter and the
file writers are file-local functions in `backend.cpp`. `geo` depends on Eigen alone, so a localizer can copy
`geo.hpp`/`geo.cpp` and rebuild the map's `Projector` from `map.yaml`.

## How it works

**Keyframe.** A paired scan becomes a keyframe after 1 m or 0.2 rad since the last one. Its cloud is range-cropped
(3 to 100 m), voxelled (0.2 m) and kept in the body frame together with the raw pose. The graph gets one node
`X(k)` and one odometry factor `X(k-1) → X(k)` with the front-end's relative pose (sigma 1 cm, 0.06°); `X(0)`
carries a tight prior, so the graph frame is the odometry frame. One iSAM2 update per keyframe, then
`T_corr = X_k · T_odom,k⁻¹` records how far the optimized pose has moved from the raw one. Optimized poses live
only in the graph and are read back when needed, so nothing goes stale after a loop closure.

**GNSS.** The node projects every fix to map metres with one `geo::Projector`, built at startup from the
`geo_*` keys (`SphericalMercator` at lat0 = lon0 = 0 for CARLA, the formula `eval/` uses; "map.yaml contract"
below), and the backend buffers it. After 30 m of keyframe path, `geo::align4Dof` fits x, y, z and yaw between the
antenna positions along the odometry (`interpolate(t_fix) · lever_arm`) and the fixes; the result becomes the
node `G(0)` = `T_mo` (`map ← origin_frame`) with a weak prior (2° roll and pitch, loose yaw and xyz), estimated
with the rest of the graph from then on. Each fix becomes a factor `T_mo · X_k · lever_arm = z_k` on the keyframe
nearest in time, within 0.2 s, else it is dropped; the fix is moved to the keyframe's time with
`z_k = z_g + R_mo (p_ant(t_k) − p_ant(t_g))` from the interpolated odometry (under 1 cm at 15 m/s per degree of
`R_mo` error). Attach rule: at keyframe k every buffered fix not newer than t_k attaches to k or k−1, whichever
is nearer; newer fixes wait for the next keyframe, which may be nearer still. GNSS factors enter iSAM2 with the
next keyframe's update. A message without covariance gets diag(0.3 m)²; every GNSS factor is under Huber
(k = 1.345).

**Loop.** A thread at 1 Hz copies the poses and cloud pointers under the lock and works on the copy. Candidate:
the nearest keyframe within 10 m that is at least 30 s older; one ICP attempt per keyframe. Source: the current
scan at its estimate; target: the candidate ± 15 keyframes at theirs, voxel 0.4 m. Accepted if ICP converges
with fitness (mean squared point distance) below 0.3 m², otherwise counted as rejected. The factor
`X_prev → X_cur` gets sigma = √(fitness × scale), i.e. variance = fitness × scale with scales 0.0025 (rot) and
0.25 (trans): fitness 0.04 m² gives 0.57° and 0.10 m. Cauchy(1.0) on top, then five extra iSAM2 iterations.

**Save.** From the `save_map` service or at shutdown (`main()` after `spin()`, never a destructor): a
Levenberg-Marquardt batch solve over all factors from the current estimate, then the five files below, with the
keyframe clouds placed at `T_mo · X_k` and voxelled at 0.2 m. Before alignment the files are written in
`origin_frame` and `map.yaml` says `georeferenced: false`. The lock is held for the whole write.

**One lock.** `Backend` owns all state and one mutex. The node runs the three subscriptions in one mutually
exclusive callback group; the loop thread and the map timer lock only to copy and, for a loop, to add the factor.

## Parameters (`config/backend.yaml`)

Every noise value is a sigma in GTSAM Pose3 order: rot x, y, z [rad] then trans x, y, z [m]. Backend values
have no defaults: a missing or non-positive value stops the node at startup with the key's name. The `geo_*`
keys are the exception to "positive": required and finite, latitude in [-90, 90], longitude in [-180, 180], and
an unknown projection name stops the node too. The node logs every value it loaded.

| Name | Unit | Value | Meaning |
|---|---|---|---|
| `odom_topic` | | `/Odometry` | front-end pose, body in `origin_frame` |
| `cloud_topic` | | `/cloud_registered_body` | deskewed scan in the body frame, same stamp |
| `gnss_topic` | | `/gnss/fix` | raw NavSatFix, never gated |
| `path_topic` | | `/mapping/path` | keyframes in `map_frame`, after every keyframe |
| `map_topic` | | `/mapping/global_map` | the map in `map_frame`, built only while subscribed |
| `save_service` | | `/mapping/save_map` | `std_srvs/Trigger`; the same writer runs at shutdown |
| `publish_tf` | | true | `map_frame → origin_frame` on every odometry message, once aligned |
| `loop_frequency` | Hz | 1.0 | how often the loop thread tries the latest keyframe |
| `map_period` | s | 5.0 | between two `/mapping/global_map` builds |
| `save_dir` | | `/maps` | where the files go (`MAPS_DIR` in compose) |
| `source_bag`, `frontend_commit`, `backend_commit` | | set by the launch | provenance written into `map.yaml`; empty when not given |
| `gnss_lever_arm` | m | [0, 0, 1.8] | antenna in the IMU frame (`sim/config/sensors.yaml`) |
| `kf_dist` | m | 1.0 | translation since the last keyframe that makes a new one |
| `kf_angle` | rad | 0.2 | rotation since the last keyframe that makes a new one |
| `kf_leaf` | m | 0.2 | voxel size of the cloud stored per keyframe |
| `min_range` | m | 3.0 | drops the car's own returns |
| `max_range` | m | 100.0 | sensor range |
| `prior_sigma` | rad, m | 1e-3 ×6 | prior on `X(0)`, pins the graph frame onto the odometry frame |
| `odom_sigma` | rad, m | [1e-3 ×3, 0.01 ×3] | odometry factor, 0.06° and 1 cm |
| `origin_sigma` | rad, m | [0.035, 0.035, 1.0, 10, 10, 10] | prior on `T_mo`: 2° roll and pitch, loose yaw and position |
| `gnss_enable` | | true | GNSS factors and alignment; false means `T_mo = I` from the start |
| `align_distance` | m | 30.0 | keyframe path before the 4-DoF fit of `T_mo` |
| `max_time_shift` | s | 0.2 | a fix farther than this from every keyframe is dropped |
| `gnss_huber_k` | sigma | 1.345 | Huber threshold on GNSS residuals |
| `gnss_default_sigma` | m | 0.3 | used when a NavSatFix carries no covariance |
| `geo_projection` | | `SphericalMercator` | how lat/lon/alt becomes map xyz; written into `map.yaml` (`map.projection.type`) |
| `geo_origin_lat_deg`, `geo_origin_lon_deg` | deg | 0.0, 0.0 | projection origin, Town03's OpenDRIVE `<geoReference>` (`+lat_0=0 +lon_0=0`) |
| `geo_origin_alt_m` | m | 0.0 | altitude that maps to z = 0 |
| `loop_enable` | | true | loop-closure thread |
| `loop_radius` | m | 10.0 | candidate must be this close |
| `loop_time_diff` | s | 30.0 | candidate must be this much older |
| `loop_search_num` | | 15 | keyframes on each side of the candidate in the ICP target |
| `loop_icp_leaf` | m | 0.4 | voxel size of source and target for ICP |
| `loop_fitness` | m² | 0.3 | accept below this ICP mean squared distance |
| `loop_rot_scale` | rad²/m² | 0.0025 | loop sigma (rot) = √(fitness × scale) |
| `loop_trans_scale` | m²/m² | 0.25 | loop sigma (trans) = √(fitness × scale) |
| `map_frame` | | `map` | georeferenced frame of the map |
| `origin_frame` | | `fastlio_origin` | the front-end's fixed frame (`fastlio_carla.yaml` `frames.map_frame`) |
| `map_name` | | `town03` | `<save_dir>/<map_name>.pcd` |
| `save_resolution` | m | 0.2 | voxel size of the saved and published map |
| `save_keyframes` | | false | also write `<save_dir>/keyframes/NNNNNN.pcd` per keyframe for the dynamic-object filter; a launch argument too |

## Outputs in `/maps`

| File | Content |
|---|---|
| `town03.pcd` | the map: keyframe clouds at optimized poses in `map`, voxel 0.2 m, binary XYZI |
| `trajectory_optimized_tum.txt` | keyframes in `map`, `t x y z qx qy qz qw`; the evo input |
| `trajectory_fastlio_tum.txt` | the same keyframes at their raw front-end poses in `origin_frame` |
| `loops.txt` | accepted loop closures, `cur prev fitness` |
| `map.yaml` | the `map` block a localizer reads (pcd, frame, projection, resolution), then provenance and quality: `source`, `mapping` (`T_map_origin`, counts, loops), `gnss` (lever arm, factors, dropped, residual RMS, alignment time); "map.yaml contract" below |
| `keyframes/NNNNNN.pcd` | with `save_keyframes`: every keyframe cloud in `map`, its pose in the PCD `VIEWPOINT` line (DUFOMap unified format) |
| `town03_static.pcd`, `town03_dynamic.pcd`, `town03_dynamic_bev.png` | written by `scripts/dynamic_filter.py`: the map without dynamic objects, what it removed, and the picture |

## map.yaml contract

`map.yaml` has two parts. The `map` block is what a localizer reads: the file to load, its frame, how a GNSS
fix becomes map metres, and the voxel size. Everything after it (`source`, `mapping`, `gnss`) is provenance and
quality, useful for a report and never needed to use the map. Run c:

```yaml
map:
  pcd: town03.pcd                  # file in the same folder as this yaml
  frame: map
  georeferenced: true              # false -> projection.type: Local, no origin keys
  projection:                      # from backend.yaml geo_*
    type: SphericalMercator
    origin_lat_deg: 0.0
    origin_lon_deg: 0.0
    origin_alt_m: 0.0
  resolution_m: 0.2
```

`SphericalMercator` is how CARLA generates lat/lon (the inverse of its `GeoLocation::Transform`). With
R = 6 378 137 m, k = cos(lat0) and angles in radians:

    x = k R (lon − lon0),   y = k R (ln tan(π/4 + lat/2) − ln tan(π/4 + lat0/2)),   z = alt − alt0

Town03's OpenDRIVE `<geoReference>` is `+lat_0=0 +lon_0=0`, so k = 1 and the shifts vanish: x = R lon,
y = R ln tan(π/4 + lat/2), z = alt. A map saved before GNSS alignment has `georeferenced: false`,
`frame: <origin_frame>` and `projection: {type: Local}`.

Same idea as Autoware's `map_projector_info.yaml`; real data = LocalCartesian (GeographicLib) at an origin inside
the area, set in `backend.yaml`; localizer code unchanged.

The rest of the file: `source` (bag, front-end and backend commits at launch time), `mapping` (`origin_frame`,
`T_map_origin` as xyz, quaternion and roll/pitch/yaw in degrees, keyframes, path length, map points, loops
accepted and rejected) and `gnss` (lever arm, factors, dropped, per-axis residual RMS of the final solve,
alignment time). `T_map_origin` keeps full precision; the other numbers are rounded for reading.

## How to run

Inside the `mapping` container (`docker compose up -d mapping && docker compose exec mapping bash`, repo mounted
at `/ws/src/brightdrive_localization`, bags at `/bags`, maps at `/maps`):

```
colcon build                              # mapping/ only (docker/colcon_defaults.yaml); then source install/setup.bash
```

Replay of a recorded FAST-LIO run (the results below):

```
ros2 launch brightdrive_mapping backend_replay.launch.py [bag:=/bags/fastlio_run_20261105_133549] [rate:=2] [save_dir:=/maps]
                                                         [save_keyframes:=true] [loop_enable:=false]
```

The launch starts the node, plays the bag 3 s later at `rate` (the run bags were recorded at half speed and have
no `/clock`, so rate 2 is real time), and shuts down 5 s after the bag ends, which saves the map. It also passes
the bag path and the two git commits for `map.yaml`; `save_keyframes` and `loop_enable` override the YAML for one
run (the per-keyframe PCDs for the dynamic-object filter, and the no-loop row of the table below). `ros2 service call /mapping/save_map std_srvs/srv/Trigger`
saves mid-run without stopping.

Full pipeline, FAST-LIO live on a raw CARLA bag (`/points_raw`, `/imu/data`, `/gnss/fix`, `/clock`), three
terminals:

```
ros2 bag play /bags/mapping_20261105_070252 --rate 0.5 --start-offset 2   # offset skips the spawn drop, so the IMU init sees a still car
ros2 launch fast_lio mapping.launch.py rviz:=false use_sim_time:=true \
    config_path:=/ws/src/brightdrive_localization/mapping/brightdrive_mapping/config config_file:=fastlio_carla.yaml
ros2 run brightdrive_mapping backend_node --ros-args \
    --params-file /ws/src/brightdrive_localization/mapping/brightdrive_mapping/config/backend.yaml \
    -p save_dir:=/maps -p source_bag:=/bags/mapping_20261105_070252
```

Ctrl-C on the backend saves the map. The backend never reads the clock, only header stamps, so `use_sim_time`
does not matter to it. Whether FAST-LIO keeps up at rate 1 on this machine was not measured; the recorded run
used 0.5 and lost no scan. Run this way on the same bag: 11 441 scans paired, 3 329 keyframes, 547 loops accepted
(twice the replay's: at half speed the 1 Hz loop thread sees every keyframe), APE 0.044 m RMSE without alignment.
`ros2 run` knows no commits; pass `-p frontend_commit:=... -p backend_commit:=...` if `map.yaml` should carry them.

Evaluation (host, evo): `evo_ape tum ground_truth_imu_tum.txt trajectory_optimized_tum.txt -r trans_part --t_max_diff 0.005`
without `--align`: the map is georeferenced, an alignment would hide a `T_mo` error. Trajectories are the IMU
frame; GT is converted with `T_base_imu` before evaluation (`eval/gt_to_tum.py`, run inside the container).

## Results

`/bags/fastlio_run_20261105_133549`: two laps of Town03, 572 s, 3 918 m of path, 11 442 scans at 20 Hz, 5 724 GNSS
fixes at 10 Hz with 0.3 m noise, replayed with the launch above.

| | Value |
|---|---|
| scans / paired / unsynced | 11 442 / 11 442 / 0 |
| keyframes | 3 329 |
| GNSS alignment | after 27 keyframes; final `T_mo` xyz (100.052, -4.467, 0.009) m, yaw -0.91° |
| GNSS factors / dropped / residual RMS | 5 118 / 604 (no keyframe within 0.2 s, slow stretches) / (0.303, 0.301, 0.299) m |
| loops accepted / rejected | 273 / 5 (ICP fitness median 0.04 m²) |
| map | 4 968 367 points, 79.5 MB, saved in 2.3 s |

APE of `trajectory_optimized_tum.txt` against the CARLA ground truth, 3 329 of 3 329 keyframes matched at 1 ms,
without `--align`:

| APE | RMSE [m] | mean [m] | max [m] |
|---|---|---|---|
| 3-D translation | 0.047 | 0.042 | 0.184 |
| horizontal | 0.037 | 0.031 | 0.184 |
| z | 0.029 | 0.024 | 0.098 |

Rotation RMSE 0.125° (yaw 0.056°); the per-axis mean error is under 1 cm, so `T_mo` carries no bias.

What each stage contributes, same bag, all against the IMU-frame ground truth (`eval/gt_to_tum.py`), 3 329
keyframes matched at 1 ms; APE without `--align` except the labelled row; RPE translation RMSE at `--delta 1
--delta_unit m`:

| Trajectory | APE 3-D RMSE / mean / max [m] | horizontal RMSE / mean / max [m] | z RMSE / mean / max [m] | RPE 1 m, translation RMSE [m] |
|---|---|---|---|---|
| (a) FAST-LIO only, placed in `map` with the backend's `T_mo`, no align | 0.391 / 0.349 / 0.819 | 0.169 / 0.150 / 0.391 | 0.353 / 0.297 / 0.794 | 0.014 |
| (a) FAST-LIO only, raw, SE(3)-aligned | 0.126 / 0.105 / 0.333 | 0.122 / 0.099 / 0.333 | 0.033 / 0.023 / 0.103 | 0.014 |
| (b) backend, `loop_enable: false`, no align | 0.059 / 0.054 / 0.196 | 0.044 / 0.039 / 0.196 | 0.039 / 0.031 / 0.129 | 0.014 |
| (c) full backend, no align | 0.047 / 0.042 / 0.184 | 0.037 / 0.031 / 0.184 | 0.029 / 0.024 / 0.098 | 0.014 |

The raw front-end drifts to 0.33 m over the two laps and, placed in the map with one rigid `T_mo`, is off by
0.8 m at worst, mostly in z: its frame tilts and sinks with time, which one transform cannot follow. GNSS alone
(b) pins it to 0.059 m; the loop closures (c) take a further 20 % off, most of it in z. RPE at 1 m is the same
0.014 m for all four, as it must be: the backend moves keyframes, not the odometry between neighbours.

## Dynamic objects

Other road users leave streaks in a map built from a moving sensor: a car seen in three keyframes is in the map
three times. DUFOMap ("DUFOMap: Efficient Dynamic Awareness Mapping", Duberg, Zhang, Jia, Jensfelt, RA-L 2024)
removes them per scan: the rays of every keyframe are cast from its viewpoint into a voxel grid, and a voxel that
rays passed through at one time but holds points at another was empty once, so those points are dynamic. Static
structure is hit from many viewpoints and never seen through. The backend writes the per-keyframe input with
`save_keyframes` (every cloud in `map`, its pose in the PCD `VIEWPOINT` line: the DynamicMap_Benchmark unified
format); `scripts/dynamic_filter.py` runs DUFOMap with the published settings of its own `main.py` (resolution
0.1 m, d_s 0.2 m, d_p 2), untuned, and writes `town03_static.pcd`, the removed points as `town03_dynamic.pcd`, and
a top-down picture.

```
ros2 launch brightdrive_mapping backend_replay.launch.py save_keyframes:=true                  # container, /maps/keyframes
pip install dufomap && mapping/brightdrive_mapping/scripts/dynamic_filter.py /var/tmp/brightdrive_maps   # host
```

On the run above (`sim.yaml` `mapping: {laps: 2, traffic: 10}`, ten Traffic-Manager cars near the route), 3 329
keyframes and 26.3 M points, 78 s to integrate and 10 s to label on 20 threads:

| | points | map → GT reference mean / p95 [m] | GT reference → map mean / p95 [m] |
|---|---|---|---|
| `town03.pcd`, every keyframe point | 4 968 367 | 0.084 / 0.251 | 0.123 / 0.364 |
| `town03_static.pcd` | 4 953 789 (0.29 % fewer) | 0.084 / 0.252 | 0.124 / 0.368 |
| removed, `town03_dynamic.pcd` | 26 654 cells, 215 216 raw points (0.82 %) | 0.063 / 0.145 | |

Two things the numbers say. First, cloud-to-cloud distance cannot measure the removal: the GT reference
(`eval/gt_map_builder.py`, the raw scans at ground-truth poses) contains the same ten cars at every sweep time, so
a removed car point has a reference neighbour as well; the unchanged 0.084 m says the filter did no harm, nothing
more. Second, the failure mode is road surface under and behind the moving cars: of the removed cells, 18 % are
the cars themselves (hit in three keyframes or fewer, 87 % of them 0.3 to 2.5 m above the road) and 82 % are
persistent ground within 0.3 m of the local surface, 8 to 14 m from the sensor and 72 % within 3 m of a car
cell, where the space a car freed, inflated by d_p, reaches the road. Nothing above 2.5 m (walls, poles, trees) was
removed and far ground is 2.5 % of it, so grazing-angle ground is not the issue here. `town03_dynamic_bev.png`
shows the removals as streaks along the lanes the ten cars drove.

## Known limitations / future work

- **Ground protection before trusting the filter.** 82 % of what DUFOMap removed here is road. Label ground with
  Patchwork++ on each keyframe cloud and keep ground-labelled points whatever DUFOMap says before merging its
  result into the static map.
- **The loop-detection rate depends on wall time.** The thread checks the latest keyframe at 1 Hz, so at speed
  about one keyframe in six gets an ICP attempt and a half-speed replay finds twice as many loops (547 against 273).
  Fix: push every new keyframe onto a queue the loop thread drains, instead of the timer.
- **A mid-run `save_map` pauses ingest for a few seconds**: the lock is held for the batch solve and the write
  (2 to 3 s here). Scans queue and are processed afterwards; nothing was lost at 20 Hz with the 50-deep queue.
- Rejected loop candidates are counted, not logged with their fitness.
- GNSS at 0.3 m pins the whole trajectory, so the loop closures' share is small (a fifth, table above); a
  `gnss_enable: false` run, evaluated with alignment, would show the loops alone.
- Everything is kept for the run: the odometry history and every keyframe cloud, no marginalization. Resident
  memory was 0.5 GB at 1 000 keyframes and 0.8 GB at 2 400.
- At 15 m/s and 20 Hz a scan advances 0.75 m, so `kf_dist` 1.0 m makes a keyframe every 1.5 m, not every metre.
- `/mapping/path` stamps every pose with the path header's stamp; the keyframe times are in the TUM files.
- The commits in `map.yaml` are those of the checkouts at launch time, not of the binary; rebuild after
  checking out.
- Parameters are typed strictly: an integer where a double is expected (`kf_dist: 1`) stops the node.
