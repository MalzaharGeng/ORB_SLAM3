#!/usr/bin/env bash
set -euo pipefail

source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${BUILD_DIR:-$source_dir/build}"

if [ ! -f "$source_dir/Vocabulary/ORBvoc.txt" ]; then
    tar -xf "$source_dir/Vocabulary/ORBvoc.txt.tar.gz" -C "$source_dir/Vocabulary"
fi

cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$build_dir" --parallel "${BUILD_JOBS:-2}"
