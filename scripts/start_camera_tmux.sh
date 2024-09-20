#!/usr/bin/env bash

set -euo pipefail

readonly path_repo="$(dirname "$(dirname "$(realpath "$BASH_SOURCE")")")"
source "$path_repo/libs/nimbro_config/config.sh"

readonly name_session="$NAME_CONTAINER_NIMBRO_CAMERA_IDS"

start_tmux() {
    tmux new-session -d -s "$name_session" "$path_repo/scripts/start_camera.sh"
}

main() {
    start_tmux
}

main "$@"
