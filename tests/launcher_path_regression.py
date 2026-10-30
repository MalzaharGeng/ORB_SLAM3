"""Check launcher discovery without a camera or SLAM input (Linux only)."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="orb launcher ") as temporary:
        root = Path(temporary) / "repository with spaces"
        root.mkdir()
        shutil.copy2(source / "run_recovery_test.sh", root)
        shutil.copytree(source / "config", root / "config")
        (root / "Vocabulary").mkdir()
        (root / "Vocabulary/ORBvoc.txt").write_text("launcher-only fixture\n")
        settings = root / "Examples/Stereo-Inertial/RealSense_D435i.yaml"
        settings.parent.mkdir(parents=True)
        settings.write_text("launcher-only fixture\n")
        caller = Path(temporary) / "unrelated working directory"
        caller.mkdir()
        environment = dict(os.environ)
        for key in ["BUILD_DIR", "RECORDING_DIR", "LD_LIBRARY_PATH"]:
            environment.pop(key, None)
        # --check calls ldd but never starts this stand-in for the sensor binary.
        for build in [root / "build", Path(temporary) / "custom build with spaces"]:
            executable = build / "Examples/Stereo-Inertial/stereo_inertial_realsense_D435i"
            executable.parent.mkdir(parents=True)
            shutil.copy2("/bin/true", executable)
            if build != root / "build":
                environment["BUILD_DIR"] = os.path.relpath(build, caller)
            result = subprocess.run(
                ["bash", str(root / "run_recovery_test.sh"), "--headless", "--check"],
                cwd=caller, env=environment, text=True, capture_output=True, timeout=15)
            if result.returncode or str(build).replace(" ", "\\ ") not in result.stdout:
                raise RuntimeError(result.stdout + result.stderr)
            if (root / "recordings").exists():
                raise RuntimeError("Read-only check created an output directory")
        replay = Path(temporary) / "external recording with spaces"
        (replay / "combined").mkdir(parents=True)
        for name in ["stereo.csv", "gyro.csv", "accel.csv", "reset_at.txt"]:
            (replay / "combined" / name).write_text("fixture\n")
        for name in ["export/left/0.png", "second_capture/export/left/0.png", "settings.yaml"]:
            path = replay / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("fixture\n")
        executable = build / "tests/d435i_recovery_replay"
        executable.parent.mkdir(parents=True)
        shutil.copy2("/bin/true", executable)
        environment["RECORDING_DIR"] = str(replay)
        result = subprocess.run(
            ["bash", str(root / "run_recovery_test.sh"), "--headless", "--replay", "--check"],
            cwd=caller, env=environment, text=True, capture_output=True, timeout=15)
        if result.returncode or str(replay).replace(" ", "\\ ") not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        if (root / "recordings").exists():
            raise RuntimeError("Replay check wrote output")
        # The optional bridge must compose with custom paths and read-only checks.
        px4_config = Path(temporary) / "PX4 config with spaces.yaml"
        shutil.copy2(source / "config/px4_vio.yaml", px4_config)
        for mode in [[], ["--replay"]]:
            result = subprocess.run(
                ["bash", str(root / "run_recovery_test.sh"), "--headless", "--check",
                 "--px4-config", str(px4_config)] + mode,
                cwd=caller, env=environment, text=True, capture_output=True, timeout=15)
            if result.returncode or "--px4-config" not in result.stdout or str(px4_config).replace(" ", "\\ ") not in result.stdout:
                raise RuntimeError(result.stdout + result.stderr)
            if (root / "recordings").exists():
                raise RuntimeError("PX4 check wrote output")
    print("PASS: default/custom build paths, spaces, unrelated cwd, external replay directory, PX4 config, read-only checks")


if __name__ == "__main__":
    main()
