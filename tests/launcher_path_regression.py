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
        settings = root / "Examples/Stereo-Inertial/DECXIN_Camera.yaml"
        settings.parent.mkdir(parents=True)
        settings.write_text("launcher-only fixture\nRecording.rolling_seconds: 20.0\n")
        (settings.parent / "DECXIN_rectify.yaml").write_text("launcher-only fixture\n")
        (settings.parent / "DECXIN_Camera_Light.yaml").write_text("launcher-only fixture\n")
        caller = Path(temporary) / "unrelated working directory"
        caller.mkdir()
        environment = dict(os.environ)
        for key in ["BUILD_DIR", "RECORDING_DIR", "LD_LIBRARY_PATH", "VIO_ENTRY", "VIO_IMU_CONFIG", "VIO_PROCESSING_PROFILE", "VIO_IMAGE_RECORDING", "VIO_ROLLING_IMAGE_RECORDING"]:
            environment.pop(key, None)
        # --check calls ldd but never starts this stand-in for the sensor binary.
        for build in [root / "build", Path(temporary) / "custom build with spaces"]:
            for name in ["Examples/Stereo-Inertial/stereo_inertial_realsense_D435i", "Examples/Stereo/stereo_uvc"]:
                executable = build / name
                executable.parent.mkdir(parents=True, exist_ok=True)
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
            for entry in ["stereo_uvc", "stereo_inertial_realsense_D435i"]:
                check_env = dict(environment, VIO_ENTRY=entry)
                result = subprocess.run(
                    ["bash", str(root / "run_recovery_test.sh"), "--headless", "--check"],
                    cwd=caller, env=check_env, text=True, capture_output=True, timeout=15)
                if result.returncode or entry not in result.stdout:
                    raise RuntimeError(result.stdout + result.stderr)
            imu_file = Path(temporary) / "IMU config with spaces.yaml"
            imu_file.write_text("launcher-only fixture\n")
            result = subprocess.run(
                ["bash", str(root / "run_recovery_test.sh"), "--headless", "--check"],
                cwd=caller, env=dict(environment, VIO_ENTRY="stereo_uvc", VIO_IMU_CONFIG=str(imu_file)),
                text=True, capture_output=True, timeout=15)
            if result.returncode or "--imu-config" not in result.stdout or str(imu_file).replace(" ", "\\ ") not in result.stdout:
                raise RuntimeError(result.stdout + result.stderr)

        for profile in ["full", "light"]:
            result = subprocess.run(
                ["bash", str(root / "run_recovery_test.sh"), "--headless", "--check"],
                cwd=caller, env=dict(environment, VIO_ENTRY="stereo_uvc", VIO_PROCESSING_PROFILE=profile),
                text=True, capture_output=True, timeout=15)
            if result.returncode or ("DECXIN_Camera_Light.yaml" in result.stdout) != (profile=="light"):
                raise RuntimeError(result.stdout + result.stderr)
        result = subprocess.run(
            ["bash", str(root / "run_recovery_test.sh"), "--headless", "--check"],
            cwd=caller, env=dict(environment, VIO_PROCESSING_PROFILE="unknown"),
            text=True, capture_output=True, timeout=15)
        if result.returncode==0:
            raise RuntimeError("Unknown processing profile was accepted")
        # Run /bin/echo as a capture stand-in to inspect argument snapshots.
        # It accepts the launcher's arguments and cannot open a camera.
        active_executable=build/"Examples/Stereo/stereo_uvc"
        shutil.copy2("/bin/echo",active_executable)
        launch_out=Path(temporary)/"launch outputs"
        run_env=dict(environment,VIO_ENTRY="stereo_uvc",VIO_IMU_CONFIG=str(imu_file),
                     OUTPUT_ROOT=str(launch_out),MAP_ROOT=str(Path(temporary)/"maps"))
        result=subprocess.run(
            ["bash",str(root/"run_recovery_test.sh"),"--headless","--px4-config",str(source/"config/decxin_px4.yaml")],
            cwd=caller,env=run_env,text=True,capture_output=True,timeout=20)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        # Launcher output root is determined by its OUTPUT_DIR/OUTPUT_ROOT option.
        import re
        match=re.search(r"测试目录：([^\n]+)",result.stdout)
        if not match:raise RuntimeError("missing launch directory")
        run_path=Path(match.group(1).strip())
        log=(run_path/"run.log").read_text()
        if "--imu-config "+str(imu_file) not in log or "--px4-config "+str(run_path/"px4.yaml") not in log:
            raise RuntimeError("PX4 snapshot overwrote the IMU argument: "+log)
        if "Recording.rolling_seconds: 0.0" not in (run_path/"settings.yaml").read_text():
            raise RuntimeError("default image recording is still enabled")
        if "VIO_CAPTURE_FORMAT=" not in (run_path/"effective_launcher.conf").read_text():
            raise RuntimeError("capture pixel format missing from run snapshot")
        shutil.rmtree(run_path)
        (root/"recordings/test_runs/.launcher.lock").unlink(missing_ok=True)
        # Runtime run parent is disposable test data; remove only an empty tree.
        for parent in [root/"recordings/test_runs",root/"recordings"]:
            if parent.exists() and not any(parent.iterdir()):parent.rmdir()
        rolling_env=dict(run_env,VIO_ROLLING_IMAGE_RECORDING="1",VIO_IMAGE_RECORDING="0")
        result=subprocess.run(
            ["bash",str(root/"run_recovery_test.sh"),"--headless","--px4-config",str(source/"config/decxin_px4.yaml")],
            cwd=caller,env=rolling_env,text=True,capture_output=True,timeout=20)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        match=re.search(r"测试目录：([^\n]+)",result.stdout)
        if not match:raise RuntimeError("missing rolling-recording launch directory")
        run_path=Path(match.group(1).strip())
        snapshot=(run_path/"settings.yaml").read_text()
        if "Recording.rolling_seconds: 20.0" not in snapshot:
            raise RuntimeError("explicit rolling recording was not preserved")
        if "--record" in (run_path/"run.log").read_text():
            raise RuntimeError("rolling recording unexpectedly enabled full recording")
        if "VIO_ROLLING_IMAGE_RECORDING=1" not in (run_path/"effective_launcher.conf").read_text():
            raise RuntimeError("rolling recording mode missing from run snapshot")
        shutil.rmtree(run_path)
        (root/"recordings/test_runs/.launcher.lock").unlink(missing_ok=True)
        for parent in [root/"recordings/test_runs",root/"recordings"]:
            if parent.exists() and not any(parent.iterdir()):parent.rmdir()
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
            raise RuntimeError("Replay check wrote output: "+str(list((root/"recordings").rglob("*"))))
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
