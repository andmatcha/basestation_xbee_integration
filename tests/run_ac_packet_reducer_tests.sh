#!/bin/sh

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_build_dir=$(mktemp -d "${TMPDIR:-/tmp}/ac-packet-reducer-test.XXXXXX")
trap 'rm -rf "$test_build_dir"' EXIT HUP INT TERM

build_and_run() {
    variant=$1

    cc -std=c11 -Wall -Wextra -Werror \
        -I"$repo_dir/$variant/include" \
        "$repo_dir/tests/test_ac_packet_reducer.c" \
        "$repo_dir/$variant/src/modules/ac_packet_reducer.c" \
        -o "$test_build_dir/test-$variant"
    "$test_build_dir/test-$variant"
}

build_and_run uplink
build_and_run integrated
