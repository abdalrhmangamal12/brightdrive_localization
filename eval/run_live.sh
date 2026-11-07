#!/usr/bin/env bash
# One live CARLA scenario with the localizer running beside it, then its metrics.
#
#   eval/run_live.sh <scenario> <name> [--outputs-only] [--node-arg key:=value ...] [run.sh options ...]
#
#   default         sim/scripts/run.sh <scenario> --record ...: the scenario's own recorder (sim time, every topic,
#                   so the bag holds the sensors, /localization/*, TF and /eval/*)
#   --outputs-only  no scenario recording; our recorder on the wall clock records the outputs, status, TF, the IMU
#                   and /eval/* but not /points_raw (latency, input age, processing time)
#   --node-arg      a ros2 launch argument for localization.launch.py (e.g. params_file:=/tmp/x.yaml)
#   env EXTRA_TOPICS (more topics to record with --outputs-only), METRICS_ARGS (extra loc_metrics.py options)
#
# Writes /bags/loc_runs/<name>.node.log (+ <name>/ with --outputs-only) and eval/results/<name>/ (metrics, plots).
# Needs the loc image built; starts the loc container and carla-server (through run.sh) when they are not up.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
SCENARIO=$1; NAME=$2; shift 2
OUTPUTS_ONLY=0; NODE_ARGS=(); RUN_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --outputs-only) OUTPUTS_ONLY=1 ;;
    --node-arg) NODE_ARGS+=("$2"); shift ;;
    *) RUN_ARGS+=("$1") ;;
  esac
  shift
done
RUNS=/bags/loc_runs
HOST_RUNS="$(grep -E '^BAGS_DIR=' .env | cut -d= -f2)/loc_runs"
mkdir -p "$HOST_RUNS"
TOPICS="/localization/pose /localization/velocity /localization/odometry /localization/status /localization/path \
/localization/debug/match /tf /tf_static /clock /imu/data /eval/ground_truth /eval/gnss_outage /eval/init_offset /initialpose"
TOPICS="$TOPICS ${EXTRA_TOPICS:-}"   # more topics for the outputs-only recorder (variants)

docker compose up -d loc > /dev/null
in_loc() { docker compose exec -T loc bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; $1"; }

echo "[run_live] starting localization_node (${NODE_ARGS[*]:-defaults})"
docker compose exec -d loc bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; \
  exec ros2 launch brightdrive_localization localization.launch.py ${NODE_ARGS[*]:-} > $RUNS/$NAME.node.log 2>&1"
for _ in $(seq 120); do grep -q "map ready" "$HOST_RUNS/$NAME.node.log" 2>/dev/null && break; sleep 0.5; done
grep -m1 "map ready" "$HOST_RUNS/$NAME.node.log" || { echo "[run_live] node did not come up"; exit 1; }

if [ $OUTPUTS_ONLY = 1 ]; then
  echo "[run_live] recording outputs (wall clock) to $RUNS/$NAME"
  docker compose exec -d loc bash -c "source /opt/ros/humble/setup.bash; \
    exec ros2 bag record -s mcap -o $RUNS/$NAME $TOPICS > $RUNS/$NAME.record.log 2>&1"
  sleep 2
  sim/scripts/run.sh "$SCENARIO" "${RUN_ARGS[@]}" 2>&1 | tee "$HOST_RUNS/$NAME.run.log"
  BAG=$RUNS/$NAME
else
  sim/scripts/run.sh "$SCENARIO" --record "${RUN_ARGS[@]}" 2>&1 | tee "$HOST_RUNS/$NAME.run.log"
  BAG=$(grep -m1 -o 'recording to /bags/[^ ]*' "$HOST_RUNS/$NAME.run.log" | sed 's/recording to //')
fi

echo "[run_live] stopping the recorder and the node"
docker compose exec -T loc pkill -INT -f "ros2 bag record" > /dev/null 2>&1
docker compose exec -T loc pkill -INT -f "brightdrive_localization localization.launch.py" > /dev/null 2>&1
for _ in $(seq 60); do
  docker compose exec -T loc pgrep -f "localization_node|ros2 bag record" > /dev/null || break; sleep 0.5
done
in_loc "ros2 bag info $BAG 2>/dev/null | grep -E 'Duration|Messages'" || true
echo "[run_live] metrics for $BAG"
in_loc "MPLCONFIGDIR=/tmp/mpl python3 eval/loc_metrics.py $BAG --scenario $SCENARIO --out eval/results/$NAME ${METRICS_ARGS:-}"
echo "[run_live] bag: $BAG"
