#!/usr/bin/env bash

set -euo pipefail

size_buffer=""

show_help() {
    echo "Usage:"
    echo "  ./set_usb_buffer_size.sh <buffer_size_MB>"
    echo
    echo "Set the systems USB buffer size to a given value in MB."
    echo
}

parse_args() {
    if [ "$#" -ne 1 ]; then
        show_help
        exit 1
    fi
    size_buffer="$1"
}

set_usb_buffer_size() {
    # The buffer memory's default value of the USB file system is often too low for a multi-camera system/high resolution cameras.
    # Increase the memory value to e.g. 1000 avoid transfer losses.
    # Note: This is reset upon reboot.
    echo "$size_buffer" >/sys/module/usbcore/parameters/usbfs_memory_mb
}

main() {
    parse_args "$@"
    set_usb_buffer_size
}

main "$@"
