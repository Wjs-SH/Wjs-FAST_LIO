#!/usr/bin/env bash
# Run the modified FAST-LIO on a conditioned bag (README steps 1-2) and record /Odometry + /cloud_registered.
# Usage: scripts/run_fastlio.sh <input_bag_dir> <output_bag_dir> [options]
#   --duration S    seconds of sensor time to play (0 = whole bag, default 0)
#   --rate R        bag playback rate (default 0.5; slower playback = no dropped scans)
#   --config FILE   FAST-LIO parameter file (default config/fastlio_go2_hesai_xt16.yaml)
#   --leg-gain K    FASTLIO_LEG_K, leg-odometry speed fusion (default 0.5, needs /leg_odom in the bag)
#   --attk K        FASTLIO_ATTK, attitude leveling toward gravity (default 0.05)
#   --z-gain K      FASTLIO_ZK, vertical velocity damping (default 0)
# Other FASTLIO_* variables (GNDK, YAWK, MANH_WARM, ODOMYAW, MAXRANGE, SPK_*) are taken from the environment.
# Logs go to <output_bag_dir>_logs. Existing outputs are never overwritten.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
usage() { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ $# -ge 2 ]] || usage
BAG="$1"; OUT="$2"; shift 2
DURATION=0; RATE=0.5; CFG="$REPO/config/fastlio_go2_hesai_xt16.yaml"; LEG_GAIN=0.5; ATTK=0.05; Z_GAIN=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --rate)     RATE="$2"; shift 2 ;;
        --config)   CFG="$2"; shift 2 ;;
        --leg-gain) LEG_GAIN="$2"; shift 2 ;;
        --attk)     ATTK="$2"; shift 2 ;;
        --z-gain)   Z_GAIN="$2"; shift 2 ;;
        -h|--help)  usage ;;
        *) echo "unknown option: $1" >&2; usage ;;
    esac
done

BAG="$(realpath "$BAG")"; CFG="$(realpath "$CFG")"; OUT="$(realpath -m "$OUT")"
QOS="$REPO/config/qos_reliable.yaml"
LOG="${OUT%/}_logs"
[[ -d "$BAG" ]] || { echo "ERROR: input bag not found: $BAG" >&2; exit 1; }
[[ -f "$CFG" ]] || { echo "ERROR: config not found: $CFG" >&2; exit 1; }
if [[ -e "$OUT" || -e "$LOG" ]]; then echo "ERROR: output already exists: $OUT or $LOG" >&2; exit 1; fi
[[ -f "$REPO/install/setup.bash" ]] || { echo "ERROR: build first: cd $REPO && colcon build" >&2; exit 1; }
# A leftover fastlio_mapping from an earlier run also subscribes and doubles every output (frames 50 ms apart).
if pgrep -x fastlio_mapping >/dev/null; then
    echo "ERROR: fastlio_mapping is already running (pgrep -x fastlio_mapping). Stop it first." >&2; exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

PLAY_TOPICS=(/lidar_points /utlidar/imu)
if awk -v g="$LEG_GAIN" 'BEGIN {exit !(g > 0)}'; then
    if ! ros2 bag info "$BAG" | grep -q '/leg_odom'; then
        echo "ERROR: --leg-gain $LEG_GAIN needs /leg_odom in the bag (scripts/add_leg_odom.py), or use --leg-gain 0" >&2
        exit 1
    fi
    PLAY_TOPICS+=(/leg_odom)
fi

mkdir -p "$LOG"
export ROS_LOG_DIR="$LOG/roslog"
export FASTLIO_SPK_KG="${FASTLIO_SPK_KG:-100}" FASTLIO_SPK_KA="${FASTLIO_SPK_KA:-1}" FASTLIO_SPK_ALPHA="${FASTLIO_SPK_ALPHA:-0.9}"
export FASTLIO_LEG_K="$LEG_GAIN" FASTLIO_ATTK="$ATTK" FASTLIO_ZK="$Z_GAIN"

mapping_pid=""; record_pid=""
stop_one() {
    local pid="$1"
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null || return 0
    kill -INT "$pid" 2>/dev/null || true
    for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
        sleep 0.25
    done
    kill -TERM "$pid" 2>/dev/null || true; sleep 1
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}
stop_processes() { stop_one "$record_pid"; record_pid=""; stop_one "$mapping_pid"; mapping_pid=""; }
trap stop_processes EXIT

cd "$LOG"
# Start the node binary directly, not through `ros2 run`: stopping the `ros2 run` wrapper can leave
# fastlio_mapping alive, and a leftover node doubles the output of the next run.
"$REPO/install/fast_lio/lib/fast_lio/fastlio_mapping" --ros-args --params-file "$CFG" >"$LOG/fastlio.log" 2>&1 &
mapping_pid=$!
sleep 8

ros2 bag record -o "$OUT" /Odometry /cloud_registered >"$LOG/record.log" 2>&1 &
record_pid=$!
sleep 2

play=(ros2 bag play "$BAG" --rate "$RATE" --topics "${PLAY_TOPICS[@]}"
      --qos-profile-overrides-path "$QOS" --disable-keyboard-controls)
set +e
if [[ "$DURATION" == "0" ]]; then
    "${play[@]}" >"$LOG/play.log" 2>&1
    play_status=$?
else
    wall_seconds=$(awk -v d="$DURATION" -v r="$RATE" 'BEGIN {print d / r}')
    timeout --signal=INT --kill-after=5 "${wall_seconds}s" "${play[@]}" >"$LOG/play.log" 2>&1
    play_status=$?
fi
set -e
sleep 5
stop_processes
trap - EXIT
if pgrep -x fastlio_mapping >/dev/null; then
    echo "WARNING: a fastlio_mapping process is still running (pgrep -x fastlio_mapping); stop it before the next run" >&2
fi

ros2 bag info "$OUT" | tee "$LOG/bag_info.txt"
echo "input=$BAG config=$CFG duration=$DURATION rate=$RATE leg_gain=$LEG_GAIN attk=$ATTK z_gain=$Z_GAIN play_status=$play_status" \
    | tee "$LOG/run_summary.txt"
