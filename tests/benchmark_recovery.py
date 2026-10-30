#!/usr/bin/env python3
"""Sequential D435i replay benchmarks; sensor time and wall time stay separate."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def csv_rows(path):
    if not path.exists():
        return []
    with path.open() as stream:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(stream)]


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * q
    low, high = math.floor(position), math.ceil(position)
    return values[low] + (values[high] - values[low]) * (position - low)


def stats(rows):
    if not rows:
        return {"frames": 0}
    track = [r["track_ms"] for r in rows]
    work = [r["processing_ms"] for r in rows]
    return dict(frames=len(rows), track_mean_ms=statistics.mean(track),
                track_median_ms=statistics.median(track), track_p95_ms=percentile(track, .95),
                track_p99_ms=percentile(track, .99), track_max_ms=max(track),
                reciprocal_mean_track_hz=1000 / statistics.mean(track),
                processing_mean_ms=statistics.mean(work), read_mean_ms=statistics.mean(r["read_ms"] for r in rows),
                cycle_mean_ms=statistics.mean(r["cycle_ms"] for r in rows),
                wait_mean_ms=statistics.mean(r["wait_ms"] for r in rows),
                processing_over_33_33ms_percent=100 * sum(v > 1000 / 30 for v in work) / len(rows))


def analyze(directory):
    rows = csv_rows(directory / "timing.csv")
    init = csv_rows(directory / "initialization.csv")
    log = (directory / "run.log").read_text(errors="replace")
    result = json.loads((directory / "run.json").read_text())
    result["pass"] = "RECOVERY_REPLAY_PASS" in log and result["exit_code"] == 0
    result["first_segment_incomplete"] = "REPLAY_FIRST_INIT_INCOMPLETE" in log
    result["all_frames"] = stats(rows)
    result["tracking_ok_fraction"] = sum(r["state"] == 2 for r in rows) / max(1, len(rows))
    if rows:
        result["wall_processing_span_s"] = rows[-1]["wall_end_s"] - rows[0]["wall_start_s"]
        result["observed_replay_fps"] = len(rows) / result["wall_processing_span_s"]
        result["processed_sensor_span_s"] = rows[-1]["t"] - rows[0]["t"]
    # Exclude save/reset/load/matching and every InitializeIMU interval from the
    # regular tracking sample. This is not total application throughput.
    def normal(r):
        event = r["save_state_before"] in (1, 2) or r["recovery_stage_before"] in (2, 5, 6)
        overlap = any(r["wall_start_s"] <= e["wall_end_s"] and r["wall_end_s"] >= e["wall_start_s"] for e in init)
        return r["state"] == 2 and r["ba2"] == 1 and not event and not overlap
    result["regular_initialized_tracking"] = stats([r for r in rows if normal(r)])
    result["initialization_calls"] = init
    successes = [e for e in init if e["stage"] == 2 and e["completed"] == 1]
    result["map_initializations"] = []
    for last in successes:
        same = [e for e in init if e["map"] == last["map"] and e["anchor"] == last["anchor"] and e["completed"] == 1]
        start = next((r for r in rows if r["map"] == last["map"] and r["anchor"] == last["anchor"]), None)
        phases = []
        for e in same:
            phases.append(dict(stage=int(e["stage"]), compute_ms=e["compute_ms"],
                               mTinit_trigger_s=e["mTinit_before"],
                               sensor_from_anchor_at_trigger_s=e["trigger_sensor_time"] - e["anchor_sensor_time"],
                               wall_from_anchor_observed_s=e["wall_end_s"] - start["wall_start_s"] if start else None))
        observed = next((r for r in rows if r["map"] == last["map"] and r["anchor"] == last["anchor"] and r["ba2"] == 1), None)
        # Tracking receives replay-relative timestamps + 1. A tracking failure
        # can allocate a new map before the intentional segment reset.
        segment = 1 if last["anchor_sensor_time"] - 1 < result["reset_at_s"] else 2
        segment_rows = [r for r in rows if (r["t"] < result["reset_at_s"]) == (segment == 1)]
        result["map_initializations"].append(dict(map=int(last["map"]), anchor=int(last["anchor"]), segment=segment, phases=phases,
            sensor_from_anchor_at_ba2_observed_s=observed["t"] + 1 - last["anchor_sensor_time"] if observed else None,
            wall_from_segment_start_s=last["wall_end_s"] - segment_rows[0]["wall_start_s"] if segment_rows else None,
            ba2_observed_replay_t=observed["t"] if observed else None))
    started_segments = len({r["t"] >= result["reset_at_s"] for r in rows})
    result["early_map_restarts"] = max(0, len(re.findall(r"\[Navigation\] WORLD = initial IMU", log)) - started_segments)
    result["motion_insufficient_resets"] = log.count("Not enough motion for initializing. Reseting...")
    alignment = re.search(r"RECOVERY_REPLAY_PASS inliers=(\d+) pairs=(\d+) residual=([\d.eE+-]+)", log)
    if alignment:
        result["alignment"] = dict(inliers=int(alignment[1]), pairs=int(alignment[2]), rmse_m=float(alignment[3]))
    (directory / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--build-dir", type=Path, default=Path(os.environ.get("BUILD_DIR", ROOT / "build")))
    parser.add_argument("--recording-dir", type=Path, default=Path(os.environ.get("RECORDING_DIR", ROOT / "recordings/d435i_recovery_verified_20260918_1654")))
    parser.add_argument("--profiles", default="5:15,3:10,2:5")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--unpaced", action="store_true")
    parser.add_argument("--timeout", type=int, default=300)
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    profiles = [tuple(map(float, s.split(":"))) for s in args.profiles.split(",")]
    if args.repeats < 1 or args.stride < 1 or any(not (0 < a < b) for a, b in profiles):
        parser.error("Require repeats/stride >= 1 and 0 < first < second")
    base = args.recording_dir.resolve()
    build = args.build_dir.resolve()
    source = (base / "settings.yaml").read_text()
    source = "\n".join(s for s in source.splitlines() if not any(s.startswith(k + ":") for k in ("IMU.fastInit", "IMU.VIBA1Time", "IMU.VIBA2Time")))
    executable = build / "tests/d435i_recovery_replay"
    environment = dict(platform=platform.platform(), cpu_info=Path("/proc/cpuinfo").read_text().split("model name", 1)[-1].splitlines()[0],
        cpu_count=os.cpu_count(), source_dataset=str(base), image_shape="640x480 stereo", imu="all samples retained", opencv_threads=2,
        build_flags=(build / "CMakeFiles/ORB_SLAM3.dir/flags.make").read_text(),
        library_sha256=hashlib.sha256((build / "lib/libORB_SLAM3.so").read_bytes()).hexdigest(),
        replay_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        viewer=args.viewer, unpaced=args.unpaced, stride=args.stride, repeats=args.repeats)
    (output / "environment.json").write_text(json.dumps(environment, indent=2) + "\n")
    results = []
    # Interleave profiles across repetitions; never run SLAM trials concurrently.
    for repeat in range(1, args.repeats + 1):
        for first, second in profiles:
            directory = output / f"viba_{first:g}_{second:g}_run{repeat}"
            directory.mkdir(exist_ok=False)
            settings = directory / "settings.yaml"
            settings.write_text(source + f"\nIMU.fastInit: 0\nIMU.VIBA1Time: {first}\nIMU.VIBA2Time: {second}\n")
            env = os.environ.copy()
            for key in ("ORB_REPLAY_VIEWER", "ORB_REPLAY_UNPACED", "ORB_REPLAY_CONSOLE"):
                env.pop(key, None)
            if args.viewer: env["ORB_REPLAY_VIEWER"] = "1"
            else:
                env.pop("DISPLAY", None);env.pop("WAYLAND_DISPLAY", None)
            if args.unpaced: env["ORB_REPLAY_UNPACED"] = "1"
            env["ORB_REPLAY_STRIDE"] = str(args.stride)
            env["ORB_INIT_METRICS"] = str(directory / "initialization.csv")
            cmd = [str(executable), str(ROOT / "Vocabulary/ORBvoc.txt"), str(settings), str(base / "combined"), str(directory)]
            record = dict(command=cmd, first_threshold=first, second_threshold=second, repeat=repeat, stride=args.stride,
                          unpaced=args.unpaced, viewer=args.viewer, reset_at_s=float((base / "combined/reset_at.txt").read_text()))
            print("START", directory.name, flush=True)
            begin = time.monotonic()
            with (directory / "run.log").open("w") as log:
                try:
                    process = subprocess.run(cmd, env=env, cwd=directory, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
                    record["exit_code"] = process.returncode
                except subprocess.TimeoutExpired:
                    record["exit_code"] = 124
            record["total_wall_s"] = time.monotonic() - begin
            (directory / "run.json").write_text(json.dumps(record, indent=2) + "\n")
            result = analyze(directory);results.append(result)
            (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
            print("DONE", directory.name, "pass=", result["pass"], "fps=", round(result.get("observed_replay_fps", 0), 2),
                  "initialized_maps=", len(result["map_initializations"]), flush=True)
    print("RESULTS", output / "results.json", flush=True)


if __name__ == "__main__":
    main()
