#!/usr/bin/env python3
"""Validate a new capture delay while retaining a verified, unchanged rigid mount.

Pixel-format changes require new temporal calibration, not new spatial extrinsics.
The first capture is the fitting set; every other capture is a complete independent
validation set. This tool writes evidence only, never live configuration or flight
authorization. The full rotation fitter remains the workflow for changed mounts.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def prepare(pairs, imu, mount):
    pairs, imu, mount = (np.asarray(x, dtype=float) for x in (pairs, imu, mount))
    if (pairs.ndim != 2 or pairs.shape[1] < 5 or imu.ndim != 2 or imu.shape[1] != 4
            or len(imu) < 3 or not np.isfinite(pairs).all() or not np.isfinite(imu).all()):
        raise ValueError("Invalid visual/IMU arrays")
    if (mount.shape != (3, 3) or not np.isfinite(mount).all()
            or np.linalg.norm(mount.T @ mount - np.eye(3)) > 1e-5
            or abs(np.linalg.det(mount) - 1) > 1e-5):
        raise ValueError("Mount must be a verified rigid rotation")
    times, gyro = imu[:, 0], imu[:, 1:]
    if np.any(np.diff(times) <= 0) or np.any(pairs[:, 1] <= pairs[:, 0]):
        raise ValueError("Nonmonotonic IMU or nonpositive image intervals")
    if len(pairs) and np.any(np.diff(pairs[:, 0]) <= 0):
        raise ValueError("Visual intervals must be ordered")
    original = len(pairs)
    pairs = pairs[(pairs[:, 0] > times[0] + .25) & (pairs[:, 1] < times[-1] - .25)]
    for i in np.where(np.diff(times) > .035)[0]:
        # Reject every interval whose possible +/-200ms delay touches a gap.
        pairs = pairs[~((pairs[:, 0] - .2 < times[i+1]) & (pairs[:, 1] + .2 > times[i]))]
    if len(pairs) < 80:
        raise ValueError("Too few IMU-covered visual intervals")
    integral = np.vstack([np.zeros(3), np.cumsum(
        (gyro[1:] + gyro[:-1]) * .5 * np.diff(times)[:, None], axis=0)])
    visual = Rotation.from_matrix(mount).apply(pairs[:, 2:5])
    dt = pairs[:, 1] - pairs[:, 0]
    moving = np.linalg.norm(visual / dt[:, None], axis=1) > .05
    if moving.sum() < 20:
        raise ValueError("Insufficient moving intervals for temporal calibration")
    return dict(pairs=pairs, times=times, integral=integral, visual=visual, dt=dt,
                moving=moving, discarded_intervals=original-len(pairs))


def residual(data, x):
    pairs, times, integral = data['pairs'], data['times'], data['integral']
    a, b = pairs[:, 0] + x[0], pairs[:, 1] + x[0]
    if a.min() < times[0] or b.max() > times[-1]:
        raise ValueError("Delay leaves recorded IMU coverage")
    integrated = np.column_stack([
        np.interp(b, times, integral[:, j]) - np.interp(a, times, integral[:, j])
        for j in range(3)])
    return (integrated - data['visual']) / data['dt'][:, None] - x[1:]


def metrics(data, x):
    error = residual(data, x)
    return dict(pairs=len(data['pairs']), moving_pairs=int(data['moving'].sum()),
                moving_rate_rmse_rad_s=float(np.sqrt(np.mean(error[data['moving']]**2))),
                angle_p95_deg=float(np.rad2deg(np.percentile(
                    np.linalg.norm(error * data['dt'][:, None], axis=1), 95))))


def fit_delay(data):
    grid = []
    for delay in np.linspace(-.2, .2, 161):
        error = residual(data, np.r_[delay, np.zeros(3)])
        bias = np.clip(np.median(error, axis=0), -.049, .049)
        grid.append((np.mean((error-bias)**2), delay, bias))
    _, delay, bias = min(grid, key=lambda entry: entry[0])
    fit = least_squares(lambda x: residual(data, x).ravel(), np.r_[delay, bias],
                        bounds=([-.2, -.05, -.05, -.05], [.2, .05, .05, .05]),
                        loss='soft_l1', f_scale=.03, max_nfev=100)
    # Constant-rate motion does not determine delay. Remove the bias direction
    # before assessing how much the residual changes with the candidate delay.
    derivative = (residual(data, fit.x + [1e-4, 0, 0, 0])
                  - residual(data, fit.x - [1e-4, 0, 0, 0])) / 2e-4
    derivative -= derivative.mean(axis=0)
    sensitivity = float(np.sqrt(np.mean(derivative**2)))
    if sensitivity < .05:
        raise ValueError("Delay unobservable under constant-rate or static motion")
    if not fit.success or abs(fit.x[0]) >= .18:
        raise ValueError("Temporal fit failed or reached delay boundary")
    return fit.x, sensitivity


def validate(captures, mount):
    if len(captures) < 2:
        raise ValueError("Need independent fit and validation captures")
    data = [prepare(pairs, imu, mount) for pairs, imu in captures]
    fits = [fit_delay(d) for d in data]
    estimates = [dict(image_time_offset_seconds=float(x[0]), gyro_bias_rad_s=x[1:].tolist(),
                      delay_sensitivity_rad_s2=sensitivity,
                      discarded_intervals=d['discarded_intervals'], **metrics(d, x))
                 for d, (x, sensitivity) in zip(data, fits)]
    checks = [dict(source=i, target=j, **metrics(target, fits[i][0]))
              for i in range(len(data)) for j, target in enumerate(data) if i != j]
    spread = float(np.ptp([x[0] for x, _ in fits]) * 1000)
    # Retain the existing independent-capture timing/gyro/error thresholds.
    passed = (spread < 10 and all(c['moving_rate_rmse_rad_s'] < .03
                                 and c['angle_p95_deg'] < .3 for c in checks))
    return dict(passed=bool(passed), primary_capture=0,
                image_time_offset_seconds=float(fits[0][0][0]),
                offset_spread_ms=spread, fits=estimates, cross_checks=checks,
                thresholds=dict(offset_spread_ms=10, moving_rate_rmse_rad_s=.03, angle_p95_deg=.3),
                scope='Temporal calibration only; retain existing verified spatial transform unchanged',
                flight_ready=False)


def load_capture(root):
    summary = json.loads((root/'summary.json').read_text())
    if (not summary['complete'] or summary.get('errors') or summary['imu_nonpositive']
            or summary['counts']['imu_invalid']):
        raise ValueError('Incomplete capture or invalid IMU samples')
    profile = dict(format=summary.get('capture_format', 'MJPG'),
                   fps=summary.get('capture_fps', 30), buffers=summary.get('capture_buffer_count', 1),
                   timestamp=summary['image_timestamp'])
    raw = np.genfromtxt(root/'imu.csv', delimiter=',', names=True)
    first, last = float(raw['timestamp_us'][0])*1e-6, float(raw['timestamp_us'][-1])*1e-6
    imu = np.column_stack([raw['timestamp_us']*1e-6-first, raw['gx'], raw['gy'], raw['gz']])
    pairs = np.array(json.loads((root/'visual_pairs.json').read_text()), float)
    return profile, (pairs, imu), (first, last)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    parser.add_argument('--mount-config', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--mount-unchanged', required=True, action='store_true',
                        help='Confirm camera and PX4 rigid mounting has not changed')
    args = parser.parse_args()
    if len(set(p.resolve() for p in args.captures)) != len(args.captures):
        raise ValueError('Validation captures must be distinct')
    fs = cv2.FileStorage(str(args.mount_config), cv2.FILE_STORAGE_READ)
    if not fs.isOpened() or int(fs.getNode('calibration_verified').real()) != 1:
        raise ValueError('Existing spatial mount must already be verified')
    transform = fs.getNode('T_b_c1').mat()
    if (transform is None or transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])):
        raise ValueError('Invalid existing camera/body transform')
    loaded = [load_capture(root) for root in args.captures]
    if any(row[0] != loaded[0][0] for row in loaded[1:]):
        raise ValueError('Independent capture profiles differ')
    intervals = sorted(row[2] for row in loaded)
    if any(a[1] >= b[0] for a, b in zip(intervals, intervals[1:])):
        raise ValueError('Validation captures overlap')
    result = validate([row[1] for row in loaded], transform[:3, :3])
    result.update(captures=[str(p) for p in args.captures], capture_profile=loaded[0][0],
                  spatial_reference=str(args.mount_config),
                  spatial_reference_sha256=hashlib.sha256(args.mount_config.read_bytes()).hexdigest(),
                  T_b_c1_unchanged=transform.tolist(), mount_unchanged=True)
    args.out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
