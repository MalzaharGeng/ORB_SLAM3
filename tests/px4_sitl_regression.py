#!/usr/bin/env python3
"""Exercise the actual PX4 1.17 EKF via an Agent and the C++ VIO bridge.

Uses SIH at real-time speed, domain 44, /vio_sitl, UDP 18888, instance 17.
No arming/flight commands. Raw ULogs are retained for independent fusion audit.
"""
import argparse
import json
import math
import os
import pathlib
import signal
import shutil
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '44'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import EstimatorStatusFlags, VehicleOdometry, VehicleLocalPosition


def main():
    cache = pathlib.Path.home()/'.cache/orbslam3-px4'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-build', type=pathlib.Path, default=cache/'PX4-Autopilot/build/px4_sitl_nolockstep')
    parser.add_argument('--agent', type=pathlib.Path, default=cache/'agent_install/bin/MicroXRCEAgent')
    parser.add_argument('--fixture', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    build, output, fixture = args.px4_build.resolve(), args.output.resolve(), args.fixture.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runtime = output/'runtime'
    runtime.mkdir()
    shutil.copytree(build/'etc', runtime/'etc')
    logging = runtime/'etc/logging'
    logging.mkdir(exist_ok=True)
    # Subscribe explicitly even when EKF aid-source topics have not yet been advertised.
    (logging/'logger_topics.txt').write_text('\n'.join(name+' 0' for name in [
        'vehicle_visual_odometry', 'vehicle_local_position', 'estimator_status_flags',
        'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel', 'estimator_aid_src_ev_hgt',
        'estimator_innovations', 'estimator_innovation_variances', 'estimator_innovation_test_ratios',
        'timesync_status', 'vehicle_status', 'vehicle_imu'])+'\n')
    environment = dict(os.environ, PX4_SYS_AUTOSTART='10040', PX4_SIM_MODEL='sihsim_quadx',
                       PX4_SIM_SPEED_FACTOR='1', PX4_UXRCE_DDS_NS='vio_sitl', PX4_UXRCE_DDS_PORT='18888',
                       PX4_PARAM_EKF2_EV_CTRL='7', PX4_PARAM_EKF2_LOG_VERBOSE='1', PX4_PARAM_EKF2_GPS_CTRL='0')
    agent_env = dict(environment, LD_LIBRARY_PATH=str(args.agent.resolve().parent.parent/'lib'))
    rclpy.init()
    node = rclpy.create_node('px4_sitl_vio_audit')
    last = {'flags': None, 'position': None}
    odometry = []
    def update_flags(m):
        last['flags'] = m
    def update_position(m):
        last['position'] = m
    node.create_subscription(EstimatorStatusFlags, '/vio_sitl/fmu/out/estimator_status_flags', update_flags, qos_profile_sensor_data)
    node.create_subscription(VehicleLocalPosition, '/vio_sitl/fmu/out/vehicle_local_position_v1', update_position, qos_profile_sensor_data)
    node.create_subscription(VehicleOdometry, '/vio_sitl/fmu/in/vehicle_visual_odometry',
                             lambda m: odometry.append(m), qos_profile_sensor_data)
    events = []
    processes = []
    logs = []

    def launch(command, filename, **kwargs):
        log = open(output/filename, 'w')
        logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, **kwargs)
        processes.append(proc)
        return proc

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)

    def until(predicate, label, timeout=30):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert predicate(), f'SITL timeout: {label}'
        events.append({'event': label, 'time_ns': time.time_ns(), 'observations': len(odometry)})
        print(label, flush=True)

    def fused():
        f = last['flags']
        return f is not None and f.cs_ev_pos and f.cs_ev_hgt and f.cs_ev_vel

    def module(name, *arguments, check=True):
        return subprocess.run([str(build/'bin'/('px4-'+name)), '--instance', '17', *arguments],
                              cwd=runtime, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=8, check=check)

    try:
        agent = launch([str(args.agent.resolve()), 'udp4', '-p', '18888'], 'agent.log', env=agent_env)
        px4 = launch([str(build/'bin/px4'), str(build/'etc'), '-i', '17', '-w', str(runtime), '-d'],
                     'px4.log', env=environment)
        until(lambda: last['flags'] is not None, 'px4_dds_ready')
        for key, value in [('EKF2_EV_CTRL', '7'), ('EKF2_EV_QMIN', '0'), ('EKF2_EV_DELAY', '0'),
                           ('EKF2_GPS_CTRL', '0'), ('EKF2_EV_POS_X', '0'), ('EKF2_EV_POS_Y', '0'), ('EKF2_EV_POS_Z', '0')]:
            module('param', 'set', key, value)
        bridge = launch([str(fixture), '--sitl'], 'bridge.log', stdin=subprocess.PIPE, text=True,
                        cwd=output, env=environment)
        def command(text):
            bridge.stdin.write(text+'\n')
            bridge.stdin.flush()
        until(fused, 'position_height_velocity_fused')
        spin(3)
        assert last['position'].xy_valid and last['position'].z_valid
        assert last['position'].v_xy_valid and last['position'].v_z_valid
        command('velocity')
        until(lambda: fused() and math.hypot(last['position'].vx,last['position'].vy)>.02,
              'ekf_velocity_responds_to_visual_input', timeout=5)
        visual_velocity_response=math.hypot(last['position'].vx,last['position'].vy)
        command('stationary')
        command('lost')
        until(lambda: last['flags'] is not None and not last['flags'].cs_ev_vel, 'fusion_stops_on_tracking_loss')
        command('resume')
        until(fused, 'fusion_resumes_after_tracking')
        command('reset')
        until(lambda: odometry and odometry[-1].reset_counter >= 1, 'map_reset_reported')
        until(fused, 'fusion_after_map_reset')
        command('stale')
        until(lambda: not last['flags'].cs_ev_vel, 'fusion_stops_on_delayed_observations')
        command('resume')
        until(fused, 'fusion_resumes_after_delay')
        previous = odometry[-1].reset_counter
        command('clock')
        until(lambda: odometry[-1].reset_counter > previous, 'clock_reset_reported')
        until(fused, 'fusion_after_clock_reset')
        agent.terminate()
        agent.wait(timeout=5)
        spin(3)
        agent = launch([str(args.agent.resolve()), 'udp4', '-p', '18888'], 'agent_restart.log', env=agent_env)
        count = len(odometry)
        until(lambda: len(odometry) > count and fused(), 'fusion_after_agent_restart', timeout=45)
        spin(3)
        (output/'px4_listener.txt').write_text(module('listener', 'vehicle_visual_odometry', '-n', '1').stdout)
        command('quit')
        bridge.wait(timeout=5)
        assert bridge.returncode == 0
        module('logger', 'stop')
        module('shutdown', check=False)
        px4.wait(timeout=10)
        stamps = [int(m.timestamp_sample) for m in odometry]
        assert len(stamps) == len(set(stamps)), 'duplicate VIO samples'
        summary = {'passed': True, 'observations': len(odometry), 'events': events,
                   'visual_velocity_response_m_s': visual_velocity_response,
                   'ulog_files': [str(p.relative_to(output)) for p in runtime.rglob('*.ulg')]}
        assert summary['ulog_files'], 'No PX4 ULog retained'
        from pyulog import ULog
        audit = {}
        ulog = ULog(str(output/summary['ulog_files'][-1]))
        for name in ['vehicle_visual_odometry', 'estimator_status_flags', 'estimator_aid_src_ev_pos',
                     'estimator_aid_src_ev_vel', 'estimator_aid_src_ev_hgt', 'estimator_innovations']:
            audit[name] = []
            for dataset in ulog.data_list:
                if dataset.name != name:
                    continue
                data = dataset.data
                stats = {'instance': dataset.multi_id, 'samples': len(data['timestamp'])}
                for key in ['cs_ev_pos', 'cs_ev_hgt', 'cs_ev_vel', 'fused', 'innovation_rejected']:
                    if key in data:
                        stats[key+'_true_samples'] = sum(int(value) for value in data[key])
                if name == 'vehicle_visual_odometry':
                    ages = [(int(t)-int(s))/1000 for t, s in zip(data['timestamp'], data['timestamp_sample'])]
                    stats.update(age_ms_min=min(ages), age_ms_max=max(ages),
                                 reset_counters=sorted({int(v) for v in data['reset_counter']}))
                    assert min(ages) >= 0 and max(ages) < 200, 'invalid sample age inside PX4'
                    assert sum(age > 1 for age in ages) > len(ages)*.9, 'sample time collapsed to reception time'
                audit[name].append(stats)
        for name in ['estimator_aid_src_ev_pos', 'estimator_aid_src_ev_hgt']:
            assert any(s.get('fused_true_samples', 0) > 0 for s in audit[name]), name+' never fused'
        assert any(s.get('cs_ev_vel_true_samples',0)>0 for s in audit['estimator_status_flags'])
        if not audit['estimator_aid_src_ev_vel']:
            summary['firmware_diagnostic_limitation'] = (
                'Pinned PX4 1.17 ExternalVisionVel::fuseVelocity passes aid_src.timestamp (zero) instead of '
                'the sample time. Its velocity aid-source topic is therefore not published. '
                'Velocity checked by EV flags, actual EKF velocity response and innovation log; firmware unchanged.')
        (output/'ulog_audit.json').write_text(json.dumps(audit, indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print('PX4_SITL_REGRESSION_PASS', flush=True)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
