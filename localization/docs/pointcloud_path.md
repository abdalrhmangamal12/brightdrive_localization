# Point cloud path

One LiDAR sweep from arrival to the pose update. Two threads: the ROS executor thread only hands the
message over; the matcher thread does all the work, one sweep at a time.

```mermaid
flowchart TD
    subgraph EX["ROS executor thread"]
        A["/points_raw arrives<br/>QoS best effort, depth 1"] --> B["cloudCallback<br/>put msg in one-deep slot<br/>slot full → replace, sweeps_dropped_busy++"]
    end
    B -- "notify" --> C

    subgraph MT["matcher thread (matcherLoop)"]
        C["take msg from slot"] --> D["scanFromMsg<br/>points + per-point time"]
        D --> E{"processScan:<br/>filter_active_?"}
        E -- "no" --> INIT["initSweep<br/>GNSS seed → 36-yaw search ~1 s<br/>(see Initialization)"]
        E -- "yes" --> P1

        subgraph TS["trackSweep"]
            P1["Phase 1 · under lock<br/>t_ref = sweep end<br/>wait ≤ imu_wait_s for the IMU pose at t_ref<br/>poses over the sweep, T_pred = pose at t_ref"]
            P1 --> P2["Phase 2 · no lock, align p95 5.7–7.9 ms<br/>deskew with the filter's poses<br/>preprocess: range crop, 0.5 m voxel, covariances<br/>align on fine map from T_pred · T_imu_lidar<br/>(only if ≥ 1000 points)"]
            P2 --> P3["Phase 3 · under lock<br/>onMatchLocked: checks → forward to now<br/>→ correctPose (NIS gate) → patch history<br/>confirm counter if confirming"]
        end

        P1 -. "skip" .-> SK["no IMU pose in time → sweeps_skipped_no_imu<br/>no poses → sweeps_skipped_no_pose"]
        INIT --> DBG
        P3 --> DBG["publishDebug<br/>/debug/match metrics (small)<br/>hand aligned scan to cloud slot"]
        DBG --> C
    end
    DBG -. "one-deep slot,<br/>drop if busy" .-> CL

    subgraph CT["cloud thread (cloudLoop, only with debug_clouds)"]
        CL["convert + publish /debug/aligned_scan<br/>best effort, depth 1, ≤ 5 Hz, only with a subscriber"]
    end
```

### ROS executor thread

`cloudCallback` puts the message into a one-deep slot and wakes the matcher thread. If the matcher is
still busy, the new sweep replaces the waiting one (counted in `sweeps_dropped_busy`): the newest sweep
always wins and no queue can build up behind a slow match.

### Matcher thread

1. Take the message from the slot and convert it to a `Scan` (points plus per-point time).
2. `processScan`: no filter → `initSweep` (the initializer, about 1 s); filter active → `trackSweep`.
3. `trackSweep` holds the lock only in the short phases:
   - **Phase 1 (lock).** `t_ref` = sweep end. Wait up to `imu_wait_s` for the IMU sample at `t_ref`
     (the sweep is published the instant its last slice is measured). Then the filter poses covering the
     sweep (for deskew) and `T_pred` = the pose at `t_ref` (the align warm start). Missing IMU or poses:
     the sweep is skipped and counted.
   - **Phase 2 (no lock; VGICP align p95 5.7–7.9 ms per sweep in the final runs).** Deskew with the filter's poses, preprocess (range crop, 0.5 m
     voxel, per-point covariances), align on the fine map from `T_pred · T_imu_lidar`, only with
     ≥ 1000 points. The IMU thread keeps predicting at 100 Hz meanwhile.
   - **Phase 3 (lock).** `onMatchLocked`: acceptance checks → forward the match to "now" with the
     filter's own motion → `correctPose` with the NIS gate → patch the stored history → health record;
     during confirmation, the confirm counter.
4. `publishDebug`: the small `/debug/match` array is published here; the aligned scan is only handed to the
   cloud thread. Then back to step 1.

### Debug outputs

A reliable publisher can wait for a slow viewer, and the next sweep would then wait too. So the matcher thread
publishes only the small message:

- `/debug/match` (10 numbers per sweep, `debug_metrics: true`) is published on the matcher thread; it is cheap.
- `aligned_scan` (about 10 600 points) is converted and published by its own thread, fed through a one-deep slot
  (dropped if busy), best effort, depth 1, at most `debug_cloud_rate_hz` (5 Hz), only with `debug_clouds: true`
  (default false) and only while someone subscribes.

The matcher loop is therefore only take → deskew → align → update, and no subscriber can slow it down. A
remaining live delay with a viewer attached is upstream of the node
([README, Known limitations](../README.md#known-limitations)).
