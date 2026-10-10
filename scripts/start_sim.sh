#!/usr/bin/env bash
# Safe entry point for the Scout Mini voice-control simulation.
#
# Running sim.launch twice at once is silently destructive: the second instance
# registers nodes the first already owns, ROS shuts the first's down, and the
# displaced controller_spawner unloads every wheel controller on its way out.
# Because controller_manager lives inside Gazebo and is shared, the surviving
# instance loses its controllers too. The symptom is baffling -- speech is still
# recognized and /cmd_vel still published, but the robot will not move.
#
# A guard node inside the launch file cannot prevent this: roslaunch starts
# nodes concurrently, so Gazebo and the spawner are already up by the time the
# guard could abort. The check has to happen before roslaunch runs at all,
# which is what this script is for.
#
#   start_sim.sh              environment + voice stack (sim.launch)
#   start_sim.sh --env        environment only, then run voice_control.launch
#                             yourself in a second terminal
#   start_sim.sh --restart    tear the running stack down first, then start
#
# Any extra arguments are passed through to roslaunch, e.g.
#   start_sim.sh --restart gui:=false
#
# Only the environment needs this protection. voice_control.launch owns just
# the two voice nodes and can be restarted freely.

set -u

RESTART=0
LAUNCH=sim.launch
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --restart|-r) RESTART=1 ;;
        --env|-e) LAUNCH=environment.launch ;;
        *) ARGS+=("$arg") ;;
    esac
done

stack_running() {
    pgrep -f '[g]zserver' >/dev/null 2>&1 && return 0
    pgrep -f '[r]oslaunch scout' >/dev/null 2>&1 && return 0
    return 1
}

shutdown_stack() {
    echo "Shutting down the running stack..."
    pkill -f '[r]oslaunch scout'      >/dev/null 2>&1
    sleep 2
    pkill -f '[g]zserver'             >/dev/null 2>&1
    pkill -f '[g]zclient'             >/dev/null 2>&1
    pkill -f '[v]oice_recognizer.py'  >/dev/null 2>&1
    pkill -f '[v]oice_teleop.py'      >/dev/null 2>&1
    pkill -f '[s]cout_skid_steer.py'  >/dev/null 2>&1
    for _ in $(seq 1 10); do
        stack_running || break
        sleep 1
    done
    sleep 2
}

if stack_running; then
    if [ "$RESTART" -eq 1 ]; then
        shutdown_stack
    else
        cat >&2 <<'EOF'
A Scout Mini stack is already running.

Starting a second one would unload the running instance's wheel controllers and
leave the robot unable to move even though speech is still recognized.

  - to talk to the stack that is already up, just speak; it is listening
  - to restart it cleanly:  start_sim.sh --restart
EOF
        exit 1
    fi
fi

exec roslaunch scout_mini_voice_control "$LAUNCH" "${ARGS[@]}"
