#!/usr/bin/env bash
# stop_stack.sh — bring a roslaunch of THIS workspace down cleanly, and
# finish the job when the graceful path stalls.
#
#   tools/stop_stack.sh                 # mobile_manipulator.launch (default)
#   tools/stop_stack.sh path_tag_locator.launch
#   tools/stop_stack.sh --force         # skip the y/n before SIGKILL
#   tools/stop_stack.sh --timeout 60    # wait longer for the graceful exit
#
# What it does, in order:
#   0. if the main launch is the systemd service mobile-manipulator
#      (tools/systemd/, boot start since 2026-09-22), says so and exits —
#      `sudo systemctl stop mobile-manipulator` is the stop for that one;
#   1. finds the roslaunch process by launch-file name (never the navifra
#      driver's `motor_driver robot.launch` — that one runs roscore, so
#      killing it takes every node with it; use `systemctl restart
#      navifra-robot` for that);
#   2. sends SIGINT — identical to Ctrl-C in the launch terminal — so
#      roslaunch shuts its nodes down in order and their rospy shutdown
#      hooks run (VISION lamp off, camera closed, arm disconnect, charge
#      relay off);
#   3. waits for roslaunch to exit;
#   4. if it does not: checks whether the launch terminal's pty is blocked
#      (2026-09-21: a frozen VS Code terminal held every node in
#      tty_write_lock at interpreter exit and roslaunch's own
#      SIGTERM->SIGKILL escalation with them — `ProcessMonitor shutdown
#      failed!`). By then the nodes have unregistered and run their hooks,
#      so the leftovers are killed with SIGKILL, children first;
#   5. `rosnode cleanup` for stale registrations, and prints what is left.
#
# Run it from a DIFFERENT terminal than the launch — if the launch
# terminal is the frozen one, this script would freeze in it too.
set -u

LAUNCH="mobile_manipulator.launch"
TIMEOUT=30
FORCE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --force)   FORCE=1 ;;
        --timeout) shift; TIMEOUT="$1" ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *)         LAUNCH="$1" ;;
    esac
    shift
done

if [ "$LAUNCH" = "robot.launch" ] || echo "$LAUNCH" | grep -q navifra; then
    echo "refusing: $LAUNCH is the navifra driver (it runs roscore)." >&2
    echo "use: sudo systemctl restart navifra-robot" >&2
    exit 2
fi

# --- 0. systemd-managed? ---------------------------------------------------
# Since 2026-09-22 the main launch normally runs as the systemd service
# mobile-manipulator (tools/systemd/). A SIGINT from here would take it
# down too, but systemd is the right owner: it sends the same SIGINT to
# the whole cgroup (no tty, so the frozen-terminal failure below cannot
# happen) and Restart=on-failure would otherwise be left guessing.
if [ "$LAUNCH" = "mobile_manipulator.launch" ] && command -v systemctl >/dev/null 2>&1 \
   && systemctl is-active --quiet mobile-manipulator 2>/dev/null; then
    echo "mobile_manipulator.launch is running as the systemd service mobile-manipulator." >&2
    echo "use: sudo systemctl stop mobile-manipulator      (journalctl -u mobile-manipulator -f)" >&2
    exit 2
fi

# --- 1. find it -----------------------------------------------------------
# Only the real roslaunch interpreter process: the pattern is anchored on
# `bin/roslaunch` as the SCRIPT argument, so a shell whose command line
# merely mentions the launch name (this script, an editor, `grep`) never
# matches. The first version of this script matched its own `bash -c`
# and SIGINT'd itself.
find_launch() {
    pgrep -f "^(/usr/bin/)?python3? (/opt/ros/[a-z]+/)?bin/roslaunch .*$LAUNCH" \
        | grep -vx "$$" || true
}
PIDS=$(find_launch)
if [ -z "$PIDS" ]; then
    echo "no roslaunch of $LAUNCH is running."
    exit 0
fi
for P in $PIDS; do
    echo "found: $(ps -o pid=,lstart=,cmd= -p "$P" | cut -c1-110)"
done

# --- 2. SIGINT ------------------------------------------------------------
for P in $PIDS; do kill -INT "$P" 2>/dev/null; done
echo "SIGINT sent; waiting up to ${TIMEOUT}s for the graceful exit ..."

# --- 3. wait ----------------------------------------------------------------
alive() { for P in $PIDS; do kill -0 "$P" 2>/dev/null && return 0; done; return 1; }
for i in $(seq 1 "$TIMEOUT"); do
    alive || { echo "roslaunch exited after ${i}s."; break; }
    sleep 1
done

# --- 4. stalled? --------------------------------------------------------
if alive; then
    echo
    echo "still running after ${TIMEOUT}s."
    for P in $PIDS; do
        TTY=$(ps -o tty= -p "$P" | tr -d ' ')
        if [ -n "$TTY" ] && [ "$TTY" != "?" ]; then
            if ! timeout 2 sh -c "echo '[stop_stack probe]' > /dev/$TTY" 2>/dev/null; then
                echo "  cause: writes to /dev/$TTY (the launch terminal) BLOCK — the"
                echo "  terminal is not draining its pty, so every node hangs on its"
                echo "  final stdout write. Close/replace that terminal tab afterwards."
            fi
        fi
        CH=$(ps -o pid= --ppid "$P" | tr -d ' ' | tr '\n' ' ')
        echo "  roslaunch $P children: ${CH:-none}"
        for C in $CH; do
            echo "    $C $(cat /proc/$C/wchan 2>/dev/null) $(ps -o comm= -p "$C")"
        done
    done
    if [ "$FORCE" -ne 1 ]; then
        read -r -p "SIGKILL the leftovers? [y/N] " ans
        [ "$ans" = "y" ] || { echo "left as is."; exit 1; }
    fi
    for P in $PIDS; do
        CH=$(ps -o pid= --ppid "$P" | tr -d ' ')
        [ -n "$CH" ] && kill -KILL $CH 2>/dev/null
        sleep 1
        kill -0 "$P" 2>/dev/null && kill -KILL "$P" 2>/dev/null
    done
    sleep 1
    echo "SIGKILL sent."
fi

# --- 5. clean up / report ---------------------------------------------------
echo
LEFT=$(find_launch)
[ -n "$LEFT" ] && echo "WARNING: roslaunch still present: $LEFT"
if command -v rosnode >/dev/null 2>&1; then
    echo y | timeout 20 rosnode cleanup 2>/dev/null | grep -i unregister || true
    echo "nodes on the master now:"
    timeout 10 rosnode list 2>/dev/null | sed 's/^/  /'
fi
if command -v ss >/dev/null 2>&1; then
    ss -ltnp 2>/dev/null | grep -E ':9090|:8080' && echo "WARNING: 9090/8080 still bound" || echo "ports 9090 / 8080 free."
fi
echo "done. Relaunch from a freshly sourced shell (source devel/setup.bash)."
