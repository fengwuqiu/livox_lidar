#!/usr/bin/env bash
set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

PKGS=(
    livox_interfaces
    livox_sdk_vendor
    livox_ros2_driver
)

for pkg in "${PKGS[@]}"; do
    cd ${CURR_DIR}/${pkg}
    bash .make_deb.sh
done