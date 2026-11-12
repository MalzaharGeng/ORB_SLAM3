"""Independent synthetic signals verify temporal-only calibration and rejection."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from decxin_temporal_calibration import prepare, validate


def capture(phase, delay=-.052, fixed_rate=False):
    times = np.arange(0, 35, .005)
    if fixed_rate:
        gyro = np.tile([.2, -.1, .3], (len(times), 1))
    else:
        gyro = np.column_stack([.35*np.sin(times*1.3+phase), .2*np.cos(times*.9+phase),
                                .25*np.sin(times*1.8+phase)])
    integral = np.vstack([np.zeros(3), np.cumsum(
        (gyro[1:]+gyro[:-1])*.5*np.diff(times)[:, None], axis=0)])
    mount = Rotation.from_rotvec([.3, -.2, .1])
    pairs = []
    for a in np.arange(.5, 34, .13):
        b = a+.13
        turn = np.array([np.interp(b+delay, times, integral[:, j])
                         - np.interp(a+delay, times, integral[:, j]) for j in range(3)])
        pairs.append([a, b, *mount.inv().apply(turn)])
    return np.array(pairs), np.column_stack([times, gyro+[.001, -.002, .0015]]), mount.as_matrix()


class TemporalCalibrationTest(unittest.TestCase):
    def test_known_delay_independent_signals(self):
        a, ia, mount = capture(0)
        b, ib, _ = capture(1.2)
        result = validate([(a, ia), (b, ib)], mount)
        self.assertTrue(result['passed'], result)
        self.assertAlmostEqual(result['image_time_offset_seconds'], -.052, places=4)
        self.assertFalse(result['flight_ready'])

    def test_constant_rate_cannot_identify_delay(self):
        pairs, imu, mount = capture(0, fixed_rate=True)
        with self.assertRaisesRegex(ValueError, 'unobservable'):
            validate([(pairs, imu), (pairs, imu)], mount)

    def test_independent_delay_disagreement_fails(self):
        a, ia, mount = capture(0)
        b, ib, _ = capture(1.2, delay=-.085)
        self.assertFalse(validate([(a, ia), (b, ib)], mount)['passed'])

    def test_imu_gaps_are_excluded(self):
        pairs, imu, mount = capture(0)
        imu = imu[(imu[:, 0]<12) | (imu[:, 0]>13)]
        data = prepare(pairs, imu, mount)
        self.assertGreater(data['discarded_intervals'], 0)
        self.assertFalse(np.any((data['pairs'][:, 0]-.2<13) & (data['pairs'][:, 1]+.2>12)))

    def test_invalid_mount_and_timestamps_rejected(self):
        pairs, imu, mount = capture(0)
        with self.assertRaisesRegex(ValueError, 'rigid rotation'):
            prepare(pairs, imu, -mount)
        imu[10, 0] = imu[9, 0]
        with self.assertRaisesRegex(ValueError, 'Nonmonotonic'):
            prepare(pairs, imu, mount)


if __name__ == '__main__':
    unittest.main()
