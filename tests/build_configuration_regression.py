#!/usr/bin/env python3
"""Configure-only regression for target selection, CPU flags and source hygiene.

Requires the same core dependencies as a normal configure. Builds no C++ and
uses temporary binary directories; it is deliberately separate from CTest.
"""
import argparse
import json
import pathlib
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    source = args.source.resolve()
    generated_source = source/"Thirdparty/g2o/config.h"
    before = generated_source.read_bytes() if generated_source.exists() else None
    with tempfile.TemporaryDirectory(prefix="orb-build-config-") as temporary:
        def configure(name, options):
            binary = pathlib.Path(temporary)/name
            command = ["cmake", "-S", str(source), "-B", str(binary), "-G", "Ninja",
                       "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON", "-DORB_SLAM3_WITH_PX4=OFF",
                       "-DORB_SLAM3_BUILD_TESTS=OFF", "-DORB_SLAM3_BUILD_CAMERA_EXAMPLES=OFF"]+options
            result = subprocess.run(command, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stdout+result.stderr)
            commands = json.loads((binary/"compile_commands.json").read_text())
            assert (binary/"Thirdparty/g2o/g2o/config.h").is_file(), "missing binary-tree g2o header"
            return commands
        minimal = configure("minimal", ["-DCMAKE_BUILD_TYPE=Release", "-DORB_SLAM3_NATIVE_ARCH=OFF",
                                         "-DORB_SLAM3_BUILD_DATASET_EXAMPLES=OFF"])
        assert not any("/Examples" in c["file"] for c in minimal), "minimal build contains examples"
        assert all("-march=native" not in c["command"] and "-mcpu=native" not in c["command"] for c in minimal)
        debug = configure("debug", ["-DCMAKE_BUILD_TYPE=Debug", "-DORB_SLAM3_NATIVE_ARCH=ON",
                                     "-DORB_SLAM3_BUILD_DATASET_EXAMPLES=OFF"])
        assert all("-O3" not in c["command"] and "-march=native" not in c["command"]
                   and "-mcpu=native" not in c["command"] for c in debug), "Debug has forced optimization"
        datasets = configure("datasets", ["-DORB_SLAM3_NATIVE_ARCH=OFF"])
        dataset_count = sum("/Examples/" in c["file"] for c in datasets)
        assert dataset_count == 12, f"expected 12 dataset examples, got {dataset_count}"
        assert not any("/Examples_old/" in c["file"] for c in datasets), "legacy defaults to enabled"
        legacy = configure("legacy", ["-DORB_SLAM3_BUILD_LEGACY_EXAMPLES=ON", "-DORB_SLAM3_NATIVE_ARCH=OFF"])
        assert sum("/Examples_old/" in c["file"] for c in legacy) == 12, "legacy opt-in not honored"
        after = generated_source.read_bytes() if generated_source.exists() else None
        assert before == after, "configuration changed the source-tree g2o header"
        print(f"BUILD_CONFIGURATION_PASS minimal_units={len(minimal)} dataset_examples={dataset_count} legacy_examples=12 Debug_O3=0 source_writes=0")


if __name__ == "__main__":
    main()
