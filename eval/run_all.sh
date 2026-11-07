#!/usr/bin/env bash
# Every scenario through the localizer, then one summary table.
#
#   eval/run_all.sh live [run.sh options]   the task's six scenarios live in CARLA, one at a time, each recorded by the
#                                           scenario's own recorder with the localizer running (eval/run_live.sh);
#                                           default options: --speed 0.5
#   eval/run_all.sh replay <bag> ...        recorded scenario bags replayed at --rate 1 (scenario = the bag's name
#                                           up to the date), outputs recorded into /bags/loc_runs/replay_<bag>
#
# Results: eval/results/<name>/ per run (metrics.json, summary.md, PNGs), eval/results/summary.md for all of them.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
SCENARIOS="nominal gnss_outage relocalization dense_traffic high_speed_turns reverse_reuse"
MODE=${1:-}; shift || true
in_loc() { docker compose exec -T loc bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; $1"; }

case "$MODE" in
  live)
    OPTS=("$@"); [ ${#OPTS[@]} -eq 0 ] && OPTS=(--speed 0.5)
    for s in $SCENARIOS; do
      echo "=== $s (live)"
      eval/run_live.sh "$s" "$s" "${OPTS[@]}"
    done
    ;;
  replay)
    docker compose up -d loc > /dev/null
    for bag in "$@"; do
      name=$(basename "$bag")
      scenario=$(echo "$name" | sed -E 's/_[0-9]{8}_[0-9]{6}$//')
      echo "=== $scenario (replay of $name)"
      in_loc "ros2 launch brightdrive_localization localization_replay.launch.py bag:=$bag rate:=1.0 \
              record:=/bags/loc_runs/replay_$name > /bags/loc_runs/replay_$name.log 2>&1"
      in_loc "MPLCONFIGDIR=/tmp/mpl python3 eval/loc_metrics.py /bags/loc_runs/replay_$name --scenario $scenario \
              --out eval/results/replay_$name"
    done
    ;;
  *)
    sed -n '2,12p' "$0"; exit 1 ;;
esac
in_loc "python3 eval/summary.py eval/results > eval/results/summary.md" && cat eval/results/summary.md
