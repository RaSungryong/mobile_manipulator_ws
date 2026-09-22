#!/usr/bin/env bash
# run_stack.sh — ExecStart of the mobile-manipulator systemd service: bring
# up `roslaunch apriltag_nav mobile_manipulator.launch` at robot boot, after
# the navifra driver (which runs roscore) is up.
#
# Modelled on ~/navifra/run_robot.sh. Runs as user abc with no tty, so:
#   - the workspace's devel/setup.bash is sourced here (MM_WS, ROS_LOG_DIR ->
#     <ws>/log/ros, the catkin env hook);
#   - PYTHONUNBUFFERED=1 + stdbuf so node output reaches the journal live
#     (a non-tty stdout is block-buffered otherwise, python and C alike);
#   - it WAITS, bounded, for what the launch needs and cannot wait for itself:
#     the ROS master (navifra-robot), the Fairino controller's RPC port
#     (arm_node connects once at start and dies if the arm is still booting),
#     the Keyence. A timed-out wait launches anyway with a warning — cameras,
#     base, web UI are useful without the arm; `systemctl restart
#     mobile-manipulator` once the missing device is on.
#
# Overrides: tools/systemd/mobile-manipulator.env (EnvironmentFile of the
# unit) or the environment, e.g.  LAUNCH_ARGS="use_hand_cam:=false".
#   DRY_RUN=1 ./run_stack.sh   prints the roslaunch command and exits.
set -u

WS="${MM_WS:-$(cd "$(dirname "$0")/../../../.." && pwd)}"
LAUNCH_ARGS="${LAUNCH_ARGS:-}"
ARM_IP="${ARM_IP:-192.168.58.2}";         ARM_PORT="${ARM_PORT:-20003}"
KEYENCE_IP="${KEYENCE_IP:-192.168.100.105}"; KEYENCE_PORT="${KEYENCE_PORT:-64000}"
WAIT_MASTER_S="${WAIT_MASTER_S:-120}"     # navifra-robot brings roscore up
WAIT_ARM_S="${WAIT_ARM_S:-180}"           # the Fairino controller boots slower than this PC
WAIT_KEYENCE_S="${WAIT_KEYENCE_S:-30}"
DRY_RUN="${DRY_RUN:-0}"

log() { echo "[run_stack] $*"; }

# --- environment -------------------------------------------------------------
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    log "ERROR: /opt/ros/noetic/setup.bash missing" >&2; exit 1
fi
source /opt/ros/noetic/setup.bash
if [ ! -f "$WS/devel/setup.bash" ]; then
    log "ERROR: $WS/devel/setup.bash missing — run catkin_make in $WS" >&2; exit 1
fi
source "$WS/devel/setup.bash"           # exports MM_WS and ROS_LOG_DIR
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export PYTHONUNBUFFERED=1
export HOME="${HOME:-/home/abc}"
log "WS=$MM_WS ROS_LOG_DIR=$ROS_LOG_DIR ROS_MASTER_URI=$ROS_MASTER_URI args='$LAUNCH_ARGS'"

# --- waits ----------------------------------------------------------------------
tcp_open() { timeout 2 bash -c "</dev/tcp/$1/$2" 2>/dev/null; }
master_up() {
    timeout 3 python3 - "$ROS_MASTER_URI" <<'PY' 2>/dev/null
import sys, xmlrpc.client
code, _, _ = xmlrpc.client.ServerProxy(sys.argv[1]).getSystemState('/run_stack')
sys.exit(0 if code == 1 else 1)
PY
}
wait_for() {  # wait_for <label> <seconds> <check-fn> [args...]
    local label="$1" limit="$2"; shift 2
    local t=0
    while ! "$@"; do
        if [ "$t" -ge "$limit" ]; then
            log "WARN: $label not reachable after ${limit}s — launching anyway"
            return 1
        fi
        [ $((t % 10)) -eq 0 ] && log "waiting for $label ... (${t}s)"
        sleep 2; t=$((t + 2))
    done
    log "$label ok (${t}s)"
}

if [ "$DRY_RUN" != "1" ]; then
    if ! wait_for "ROS master (navifra-robot)" "$WAIT_MASTER_S" master_up; then
        log "ERROR: no ROS master — the navifra driver runs roscore; is navifra-robot up?" >&2
        exit 1                                   # Restart=on-failure retries
    fi
    [ "$WAIT_ARM_S" -gt 0 ]     && wait_for "Fairino arm $ARM_IP:$ARM_PORT" "$WAIT_ARM_S" tcp_open "$ARM_IP" "$ARM_PORT"
    [ "$WAIT_KEYENCE_S" -gt 0 ] && wait_for "Keyence $KEYENCE_IP:$KEYENCE_PORT" "$WAIT_KEYENCE_S" tcp_open "$KEYENCE_IP" "$KEYENCE_PORT"
fi

# --- launch --------------------------------------------------------------------
# shellcheck disable=SC2086
CMD=(stdbuf -oL -eL roslaunch apriltag_nav mobile_manipulator.launch $LAUNCH_ARGS)
log "exec: ${CMD[*]}"
[ "$DRY_RUN" = "1" ] && exit 0
exec "${CMD[@]}"
