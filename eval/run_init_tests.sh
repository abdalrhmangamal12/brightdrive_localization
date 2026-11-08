#!/usr/bin/env bash
# Initialization tests on replays of one nominal bag (no CARLA needed). Each run records the outputs and the bag's
# ground truth to /bags/loc_runs/<prefix>_<run>; eval/init_report.py turns them into one table.
#
#   eval/run_init_tests.sh <prefix> [run ...]        (default: every run below)
#
# moving_<s>     the localizer starts on data from s seconds into the bag, the car driving at ~7.9 m/s: seed from
#                GNSS, v0 from the GNSS velocity, heading hint from the course over ground; 40 s of bag.
#                12 straight, 58 the entry of a turn, 100 the elevated section, 21 inside a 1.4 rad/s turn
# gtinit_1m      GNSS off from t = 0 (the node's gnss_topic points at a topic nobody publishes) and an /initialpose
#                from eval/init_from_gt.py: ground truth at 2.5 s (car standing) + 1 m at 45 deg, heading +10 deg;
#                60 s of bag
# gtinit_<N>m    same with N m: 5 and 10 converge, 15 is rejected as ambiguous (the failure status)
# gtinit_1m_dz-8 same as gtinit_1m with the /initialpose z 8 m below the truth: a 2-D click (z = 0) on the 8 m
#                elevated section of the route
# nominal        the whole bag, defaults: the regression run against the previous numbers
#
# BAG (default /bags/nominal_20261107_144548) picks the bag. Needs the loc container up and the package built.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
PREFIX=$1; shift
RUNS=("$@"); [ ${#RUNS[@]} -eq 0 ] && RUNS=(nominal moving_12 moving_58 moving_100 moving_21 gtinit_1m gtinit_5m \
  gtinit_10m gtinit_15m gtinit_1m_dz-8)
BAG=${BAG:-/bags/nominal_20261107_144548}
HOST_RUNS="$(grep -E '^BAGS_DIR=' .env | cut -d= -f2)/loc_runs"
mkdir -p "$HOST_RUNS"
ros() { docker compose exec -T loc bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; $1"; }

# localization.yaml with the GNSS input pointed at a topic nobody publishes.
cp localization/config/localization.yaml "$HOST_RUNS/${PREFIX}_gnss_off.yaml"
sed -i "s|^    gnss_topic: .*|    gnss_topic: /gnss/off|" "$HOST_RUNS/${PREFIX}_gnss_off.yaml"

for run in "${RUNS[@]}"; do
  name=${PREFIX}_$run
  args="bag:=$BAG record:=/bags/loc_runs/$name"
  helper=""
  case $run in
    nominal) ;;
    moving_*) args="$args start_offset:=${run#moving_} play_duration:=40" ;;
    gtinit_*)
      args="$args play_duration:=60 params_file:=/bags/loc_runs/${PREFIX}_gnss_off.yaml"
      off=${run#gtinit_}; off=${off%%m*}; dz=0
      [[ $run == *_dz* ]] && dz=${run##*_dz}
      helper="python3 -u eval/init_from_gt.py --at 2.5 --offset-m $off --dir-deg 45 --heading-deg 10 --dz $dz" ;;
    *) echo "[init_tests] unknown run $run"; continue ;;
  esac
  rm -rf "$HOST_RUNS/$name"
  echo "[init_tests] $name: $args ${helper:+(helper: $helper)}"
  if [ -n "$helper" ]; then
    docker compose exec -d loc bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; \
      exec $helper > /bags/loc_runs/$name.helper.log 2>&1"
  fi
  ros "ros2 launch brightdrive_localization localization_replay.launch.py $args > /bags/loc_runs/$name.launch.log 2>&1"
  if [ -n "$helper" ]; then
    docker compose exec -T loc pkill -INT -f "[i]nit_from_gt.py" > /dev/null 2>&1
    sleep 1
    grep -m1 "initialpose at" "$HOST_RUNS/$name.helper.log"
  fi
  ros "ros2 bag info /bags/loc_runs/$name 2>/dev/null | grep -E 'Duration|Messages'"
done
ros "python3 eval/init_report.py $(printf "/bags/loc_runs/${PREFIX}_%s " "${RUNS[@]}")"
