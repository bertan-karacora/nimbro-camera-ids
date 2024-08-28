#!/usr/bin/env bash

set -euo pipefail

path_udev=/etc/udev/rules.d/99-ids-usb-access.rules

string_udev="# Global U3V rule: Defined as class=EFh, subclass=02h, protocol=01h, ECN=U3V 
# SUBSYSTEM==\"usb\", ATTR{bDeviceClass}==\"ef\", ATTR{bDeviceProtocol}==\"01\", ATTR{bDeviceSubClass}==\"02\", ATTR{configuration}==\"USB3 Vision\", MODE=\"0777\"

# Also explicitly add current U3V
SUBSYSTEM==\"usb\", ATTR{bDeviceClass}==\"ef\", ATTR{idVendor}==\"1409\", MODE=\"0777\""

show_help() {
    echo "Usage:"
    echo "  ./add_udev_rule.sh"
    echo
    echo "Add the udev rule installed with ids peak to the system."
    echo
}

parse_args() {
    if [ "$#" -ne 0 ]; then
        show_help
        exit 1
    fi
}

add_udev_rule() {
    cat >"$path_udev" <<EOF
$string
EOF
}

main() {
    parse_args "$@"
    add_udev_rule
}

main "$@"
