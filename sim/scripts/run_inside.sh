#!/usr/bin/env bash
# Runs INSIDE the sim container; sim/scripts/run.sh starts it. One scenario, start to end:
#   bridge (sensors -> ROS 2) + GNSS gate + ground truth (eval only) + initial pose (eval, only if the scenario
#   has one) + foxglove_bridge (watch live) + optional recording + ScenarioRunner (drives the run, ticks the world).
# Usage: run_inside.sh <scenario> <speed> <record 0|1> [--tick 0.05] [--param Name:value ...]
SCENARIO=$1; SPEED=$2; RECORD=$3; shift 3

# Work from the repo root (its mount point in the container).
cd /ws/src/brightdrive_localization
# Stop everything we started when this script ends (normal end, error, or Ctrl-C).
PIDS=(); SR=
cleanup() {
  # ScenarioRunner first, and wait for it: it removes the cars it spawned (and our car going tells the bridge).
  [ -n "$SR" ] && kill -INT "$SR" 2>/dev/null && wait "$SR" 2>/dev/null
  # Then the rest. SIGINT lets ros2 bag close the bag file properly: 20 s for that, then whatever is still there is
  # killed (a run must end even when a process ignores the signal, e.g. after a server crash).
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
  for _ in $(seq 200); do
    alive=0; for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.1
  done
  kill -KILL "${PIDS[@]}" 2>/dev/null
  wait 2>/dev/null
}
# Ctrl-C or a stop request: end the script, which runs cleanup.
trap exit INT TERM
trap cleanup EXIT

# The scenario must exist (also catches a name that is only in the YAML but not generated yet).
python3 sim/config/settings.py "$SCENARIO" > /dev/null || exit 1
# 0. Empty world: a run that was killed may have left its car behind, and the bridge would attach to that one.
python3 -u sim/scripts/clean_world.py || exit 1
# 1. The bridge: waits for our car, then publishes /points_raw, /imu/data, /gnss/fix, /clock, /tf_static.
python3 -u sim/carla_bridge/carla_bridge.py --scenario "$SCENARIO" &
PIDS+=($!)
# 2. The GNSS gate: /gnss/fix -> /gnss/fix_gated, withheld during the scenario's outage window (none by default).
#    It runs in every scenario, so the estimator always subscribes /gnss/fix_gated.
python3 -u sim/gnss_gate/gnss_gate.py --scenario "$SCENARIO" &
PIDS+=($!)
# 3. Ground truth for evaluation only: waits for our car, then publishes /eval/ground_truth at every tick.
python3 -u eval/gt_publisher.py &
PIDS+=($!)
# 4. The initial pose on /initialpose, once, if the scenario asks for one: the scenario's start waypoint + its
#    offsets (nothing from the live simulation), stamped with the first /clock.
if [ -n "$(python3 sim/config/settings.py "$SCENARIO" init_offset_m)" ]; then
  python3 -u eval/initial_pose_publisher.py --scenario "$SCENARIO" &
  PIDS+=($!)
fi
# 5. foxglove_bridge on port 8765 (host network): Foxglove -> Open connection -> ws://localhost:8765 shows the
#    run live. Its log goes to a file: it prints one line per subscription. The binary itself, not `ros2 run`: that
#    wrapper swallows the stop signal and the node would outlive the run.
"$(ros2 pkg prefix foxglove_bridge)/lib/foxglove_bridge/foxglove_bridge" --ros-args -p port:=8765 > /tmp/foxglove.log 2>&1 &
PIDS+=($!)
# 6. Optional recording of every topic into /bags (BAGS_DIR on the host), mcap. Not /rosout and /parameter_events:
#    logging only. --use-sim-time: the bag's own timestamps are simulation time, so `ros2 bag info` shows the
#    scenario's simulated duration and `ros2 bag play` paces at the simulation's rate whatever --speed was.
if [ "$RECORD" = 1 ]; then
  BAG=/bags/${SCENARIO}_$(date +%Y%m%d_%H%M%S)
  echo "recording to $BAG"
  ros2 bag record -s mcap --use-sim-time -a -x '/(rosout|parameter_events)' -o "$BAG" > /tmp/record.log 2>&1 &
  PIDS+=($!)
fi
# Give them a moment to start, then let ScenarioRunner load the town, spawn the cars and drive.
sleep 3
# In the background + wait: bash runs the Ctrl-C trap only after a foreground program ends, which is too late.
python3 -u sim/scenarios/run_scenario.py "$SCENARIO" --speed "$SPEED" "$@" &
SR=$!
wait "$SR"
