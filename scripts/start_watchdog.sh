#!/usr/bin/env bash

set -euo pipefail

readonly path_repo="$(dirname "$(dirname "$(realpath "$BASH_SOURCE")")")"
source "$path_repo/libs/nimbro_config/config.sh"

# TODO: Could this be done without hard-coding here? Add it to the config and use when launching node?
readonly name_session="$NAME_CONTAINER_NIMBRO_CAMERA_IDS"
readonly topics=(/camera_ids/image_color /camera_ids/camera_info)
readonly duration_loop=10
readonly duration_timeout=5

is_active() {
    local topic="$1"

    local count_publishers="$(ros2 topic info "$topic" | grep 'Publisher' | awk '{print $3}')"

    [[ $count_publishers -ne "0" ]]
}

are_active() {
    for topic in "${topics[@]}"; do
        if is_active "$topic"; then
            echo "$topic is active."
        else
            sleep "$duration_timeout"
            # Check a second time
            if ! is_active "$topic"; then
                echo "$topic is silent."
                return 1
            fi
        fi
    done

    return 0
}

kill_session() {
    tmux kill-session -t "$name_session" 2>/dev/null
}

run_watchdog() {
    if ! are_active; then
        echo "At least one topic is silent for more than $duration_timeout seconds."

        if kill_session; then
            echo "Tmux session named "$name_session" killed."
        else
            echo "Tmux session named "$name_session" does not exist or could not be killed. Continuing..."
        fi

        "$path_repo/scripts/start_camera_tmux.sh"

        sleep "$duration_loop"
    fi
    sleep "$duration_timeout"
}

main() {
    sleep "$duration_timeout"
    while true; do
        run_watchdog
    done
}

main "$@"
