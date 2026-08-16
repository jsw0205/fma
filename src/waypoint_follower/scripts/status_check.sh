#!/usr/bin/env bash
# One-shot snapshot of everything that's normally spread across 5+
# terminals: RTK fix status, GPS validity, which control source the
# arbiter is currently using, camera lane detection, obstacle-avoidance
# state. Run any time to see "what's actually happening right now"
# without hunting through logs.
# Filters out ros2 CLI's own "WARNING: topic [...] does not appear to be
# published yet" noise (goes to stdout, not stderr, so 2>/dev/null alone
# doesn't catch it) so callers just see a clean "(no data)".
clean() {
    if [[ "$1" == WARNING:* ]] || [ -z "$1" ]; then
        echo "(no data)"
    else
        echo "$1"
    fi
}

echo "===== ros2_ws status check ($(date +%H:%M:%S)) ====="

echo
echo "-- RTK --"
FLAGS=$(timeout 2 ros2 topic echo /navpvt --field flags --once 2>/dev/null | head -1)
if ! [[ "$FLAGS" =~ ^[0-9]+$ ]]; then
    echo "  (no /navpvt - GPS node not running or not fixed yet?)"
else
    CARR=$(( FLAGS & 192 ))
    if [ "$CARR" = "128" ]; then STATUS="FIXED"
    elif [ "$CARR" = "64" ]; then STATUS="FLOAT"
    else STATUS="none/basic"; fi
    echo "  flags=$FLAGS -> $STATUS"
fi

echo
echo "-- Antenna (num_sv=0 investigation, 2026-07-28 - see README) --"
A_STATUS=$(timeout 2 ros2 topic echo /monhw --field a_status --once 2>/dev/null)
A_POWER=$(timeout 2 ros2 topic echo /monhw --field a_power --once 2>/dev/null)
JAM=$(timeout 2 ros2 topic echo /monhw --field flags --once 2>/dev/null)
echo "  a_status=$(clean "$A_STATUS") (0=INIT 1=UNKNOWN 2=OK 3=SHORT 4=OPEN)"
echo "  a_power=$(clean "$A_POWER") (0=OFF 1=ON 2=UNKNOWN)"
echo "  flags=$(clean "$JAM") (jamming bits 12: 0=unknown/disabled 4=ok 8=warning 12=critical)"

echo
echo "-- GPS waypoint follower --"
VALID=$(timeout 2 ros2 topic echo /gps_control/valid --field data --once 2>/dev/null)
IDX=$(timeout 2 ros2 topic echo /gps_control/target_idx --field data --once 2>/dev/null)
echo "  valid=$(clean "$VALID")  target_idx=$(clean "$IDX")"

echo
echo "-- Camera (yolopv2_zed_node) --"
LANE_VALID=$(timeout 2 ros2 topic echo /yolopv2_zed_node/lane_valid --field data --once 2>/dev/null)
STEER=$(timeout 2 ros2 topic echo /yolopv2_zed_node/steering_deg --field data --once 2>/dev/null)
echo "  lane_valid=$(clean "$LANE_VALID")  steering_deg=$(clean "$STEER")"

echo
echo "-- Obstacle avoidance --"
AVOID_STATE=$(timeout 2 ros2 topic echo /avoid/state --field data --once 2>/dev/null)
echo "  state=$(clean "$AVOID_STATE")"

echo
echo "-- Arbiter: what's actually driving right now --"
SOURCE=$(timeout 2 ros2 topic echo /control_arbiter/active_source --field data --once 2>/dev/null)
echo "  active_source=$(clean "$SOURCE")"

echo
echo "===================================================="
