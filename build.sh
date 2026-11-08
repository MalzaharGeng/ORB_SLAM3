#!/usr/bin/env bash
set -euo pipefail

source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${BUILD_DIR:-$source_dir/build}"
build_jobs="${BUILD_JOBS:-2}"
if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
    echo 'BUILD_JOBS 必须是正整数。' >&2
    exit 2
fi

# Preserve the generator of an existing build directory. A new directory uses
# Ninja when available unless the caller selected another CMake generator.
configure_options=()
if [[ ! -f "$build_dir/CMakeCache.txt" && -z "${CMAKE_GENERATOR:-}" ]] && command -v ninja >/dev/null 2>&1; then
    explicit_generator=0
    for argument in "$@"; do
        [[ "$argument" != -G* ]] || explicit_generator=1
    done
    ((explicit_generator)) || configure_options+=(-G Ninja)
fi
if [[ "${USE_CCACHE:-1}" == 1 ]] && command -v ccache >/dev/null 2>&1; then
    configure_options+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
elif [[ "${USE_CCACHE:-1}" == 0 ]]; then
    configure_options+=(-DCMAKE_C_COMPILER_LAUNCHER= -DCMAKE_CXX_COMPILER_LAUNCHER=)
fi

if [ ! -f "$source_dir/Vocabulary/ORBvoc.txt" ]; then
    tar -xf "$source_dir/Vocabulary/ORBvoc.txt.tar.gz" -C "$source_dir/Vocabulary"
fi

cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release "${configure_options[@]}" "$@"
cmake --build "$build_dir" --parallel "$build_jobs"
