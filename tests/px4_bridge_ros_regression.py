#!/usr/bin/env python3
"""Real DDS peer for bridge tests. Domain 43 (tests) / 42 (replay) isolates hardware.

--serve LOG keeps a disarmed peer running for recorded VIO replay. Otherwise
launches the supplied fixture and verifies delivery, gating and reset semantics.
"""
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '42' if '--serve' in sys.argv else '43'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (VehicleStatus, VehicleOdometry, TimesyncStatus,
                          VehicleLocalPosition, EstimatorStatusFlags)


def topic(base, cls):
    version = getattr(cls, 'MESSAGE_VERSION', 0)
    return '/vio_test/fmu/' + base + (f'_v{version}' if version else '')


def main():
    rclpy.init()
    node = rclpy.create_node('vio_bridge_test_peer')
    received = []
    state = {'armed': False, 'status': True, 'sync': True}
    pubs = {}
    for name, cls in [('vehicle_status', VehicleStatus), ('timesync_status', TimesyncStatus),
                      ('vehicle_local_position', VehicleLocalPosition), ('estimator_status_flags', EstimatorStatusFlags)]:
        pubs[name] = node.create_publisher(cls, topic('out/' + name, cls), qos_profile_sensor_data)
    node.create_subscription(VehicleOdometry, topic('in/vehicle_visual_odometry', VehicleOdometry),
                             lambda m: received.append((time.monotonic(), m)), qos_profile_sensor_data)

    def emit():
        now = time.time_ns() // 1000
        if state['status']:
            m = VehicleStatus(timestamp=now)
            m.arming_state = VehicleStatus.ARMING_STATE_ARMED if state['armed'] else VehicleStatus.ARMING_STATE_DISARMED
            pubs['vehicle_status'].publish(m)
        if state['sync']:
            m = TimesyncStatus(timestamp=now, source_protocol=TimesyncStatus.SOURCE_PROTOCOL_DDS,
                               observed_offset=123000, estimated_offset=123000, round_trip_time=500)
            pubs['timesync_status'].publish(m)
        pubs['vehicle_local_position'].publish(VehicleLocalPosition(timestamp=now, xy_valid=True, z_valid=True))
        pubs['estimator_status_flags'].publish(EstimatorStatusFlags(timestamp=now))
    node.create_timer(.1, emit)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def until(predicate, seconds=8):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert predicate(), 'DDS observation condition timed out'

    if len(sys.argv) > 1 and sys.argv[1] == '--serve':
        try:
            while True:
                spin(1)
                pathlib.Path(sys.argv[2]).write_text(json.dumps({
                    'received': len(received),
                    'reset_counters': sorted({int(m.reset_counter) for _, m in received}),
                    'invalid': sum(not all(math.isfinite(v) for v in list(m.position)+list(m.velocity)+list(m.q))
                                   for _, m in received),
                }, indent=2))
        except KeyboardInterrupt:
            return
    with tempfile.TemporaryDirectory(prefix='vio-dds-test-') as directory:
        with open(pathlib.Path(directory)/'bridge.log', 'w+') as log:
            proc = subprocess.Popen([sys.argv[1]], stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                                    text=True, cwd=directory)
            def command(text):
                proc.stdin.write(text+'\n')
                proc.stdin.flush()
            def silent_after_grace(grace=.25, duration=.4):
                spin(grace)
                count = len(received)
                spin(duration)
                assert len(received) == count, 'unexpected publication while gate should be closed'
            try:
                until(lambda: len(received) >= 10)
                for _, m in received:
                    assert m.pose_frame == VehicleOdometry.POSE_FRAME_FRD
                    assert m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_FRD
                    assert abs(sum(q*q for q in m.q)-1) < 1e-5
                    assert m.timestamp >= m.timestamp_sample > 0
                    assert m.timestamp-m.timestamp_sample < 200000
                    assert all(math.isnan(v) for v in m.angular_velocity)
                    assert all(v > 0 for v in m.position_variance)
                assert len({m.timestamp_sample for _, m in received}) == len(received), 'duplicate observations'
                state['armed'] = True
                spin(.3)
                command('reset')
                silent_after_grace(duration=1.2)
                state['armed'] = False
                until(lambda: any(m.reset_counter == 1 for _, m in received))
                assert max(abs(v) for v in received[-1][1].position) < 1e-5
                command('lost')
                silent_after_grace()
                count = len(received)
                command('resume')
                spin(.5)
                assert len(received) == count, 'resume bypassed one-second stabilization'
                until(lambda: len(received) > count)
                state['status'] = False
                silent_after_grace(grace=1.3)
                state['status'] = True
                count = len(received)
                until(lambda: len(received) > count)
                assert received[-1][1].reset_counter == 1, 'link reconnect changed coordinate origin'
                state['sync'] = False
                silent_after_grace(grace=2.3)
                state['sync'] = True
                count = len(received)
                until(lambda: len(received) > count)
                command('stale')
                silent_after_grace()
                command('resume')
                count = len(received)
                until(lambda: len(received) > count)
                command('clock')
                until(lambda: received[-1][1].reset_counter >= 2)
                command('quit')
                proc.wait(timeout=5)
                assert proc.returncode == 0
                print(f'PX4_ROS_REGRESSION_PASS observations={len(received)}')
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
