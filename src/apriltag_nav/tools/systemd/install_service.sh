#!/usr/bin/env bash
# install_service.sh — install / remove the mobile-manipulator systemd unit.
#
#   sudo src/apriltag_nav/tools/systemd/install_service.sh            # install + enable (starts at next boot)
#   sudo src/apriltag_nav/tools/systemd/install_service.sh --start    # ... and start it now
#   sudo src/apriltag_nav/tools/systemd/install_service.sh --uninstall
#
# --start refuses while a hand-started mobile_manipulator.launch is running:
# two copies would fight over node names and ports 9090 / 8080. Stop that one
# first (tools/stop_stack.sh), then `sudo systemctl start mobile-manipulator`.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../../.." && pwd)"
UNIT=mobile-manipulator
DST=/etc/systemd/system/$UNIT.service

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

if [ "${1:-}" = "--uninstall" ]; then
    systemctl disable --now $UNIT 2>/dev/null || true
    rm -f "$DST"; systemctl daemon-reload
    echo "$UNIT removed."; exit 0
fi

[ -x "$HERE/run_stack.sh" ] || chmod +x "$HERE/run_stack.sh"
[ -f "$WS/devel/setup.bash" ] || { echo "no $WS/devel/setup.bash — catkin_make first" >&2; exit 1; }
systemctl cat navifra-robot >/dev/null 2>&1 || echo "WARN: navifra-robot.service not found — this unit Requires= it"

sed "s#@WS@#$WS#g" "$HERE/$UNIT.service" > "$DST"
systemctl daemon-reload
systemctl enable $UNIT
echo "installed $DST (WS=$WS), enabled for boot."

if [ "${1:-}" = "--start" ]; then
    if pgrep -f "bin/roslaunch .*mobile_manipulator.launch" >/dev/null; then
        echo "a hand-started mobile_manipulator.launch is running — stop it first" >&2
        echo "(tools/stop_stack.sh), then: sudo systemctl start $UNIT" >&2
        exit 2
    fi
    systemctl start $UNIT
    sleep 3; systemctl --no-pager status $UNIT | head -12
fi
