# DECXIN Camera / PX4 IMU / behavior tree integration

All source changes, compilation and tests run on the Nano (aarch64). Windows is an SSH terminal only.
Report date follows client date 2026-11-10; Nano's own wall clock currently reads 2026-11-06. Do not step the clock during a navigation session.

## Hardware and coordinate conventions
User confirmed DECXIN Camera optical axis forward, module top upward.
Left optical center relative to the FC IMU in body FRD: **[+0.150, +0.0205, +0.0495] metres**.
The preliminary T_b_c1 includes left-image rectification R1: R_body_rect = R_body_raw_camera * R1.transpose().
The default optical camera-to-body axis mapping is [camera z, camera x, camera y].
This is a nominal mounting rotation, not a completed camera/IMU calibration.

## Changes
- Retain stereo as default, connect its actual navigation samples to the existing PX4 bridge.
- Pure stereo explicitly disables publication authorization; inertial initialization is never fabricated.
- Opt-in --imu-config consumes PX4 SensorCombined raw body FRD acceleration and angular rate, not fused odometry.
- Require verified spatiotemporal calibration for normal inertial mode. --imu-bench permits provisional calibration while forcing publication disabled.
- Reject clipped/non-finite/delayed/asynchronous IMU measurements; bounded 2048-sample storage.
- Require full image-interval coverage, interpolate endpoints, reject clock/gap/calibration discontinuity after tracking starts.
- Do not resubmit the shared endpoint: ORB retains its previous IMU endpoint, and duplicates can cause zero-dt preintegration.
- Stamp images immediately after acquisition, before JPEG decoding/remap/mailbox. These remain software timestamps; exposure/USB delay needs calibration.
- Validate runtime rectified intrinsics against settings. Keep 1280x960 per eye and 1000 ORB features.
- Restore terminal console, freeze_images requests and SIGINT/SIGTERM cleanup. Preserve previous crash recordings.
- Fix launcher regression fixtures to cover UVC, D435i, paths containing spaces and optional IMU config.
- Read-only observer tools/decxin_readiness.py has no command/setpoint publishers.

## Profiles
- Examples/Stereo-Inertial/DECXIN_Camera.yaml: normal stereo settings.
- Examples/Stereo-Inertial/DECXIN_PX4_Bench.yaml: preliminary camera/body transform; rolling recording off for bench work.
- config/decxin_px4_imu.yaml: preliminary T_b_c1; calibration_verified remains 0.
- config/decxin_px4_bench.yaml: identity R_BI because PX4 raw IMU is already body FRD; extrinsics_valid=0.
- Existing config/px4_vio_local.yaml is preserved for history; UVC stereo overrides its publication authorization in memory.

## Verification
All listed results below were obtained on Nano; see /home/nano/decxin_integration_20261110/.
Behavior tree unit/isolated DDS tests: 44 passed. Navigation gate, sortie, recording gate, launcher, IMU buffer and isolated IMU ROS checks: 6 passed. Mocked installed watchdog test passed (live heartbeat, real stall, correct process target).

## Remaining before flight readiness
- Verify mounting rotation, lever arm accuracy, camera/IMU temporal offset and IMU noise.
- Account for sensor_combined onboard filtering and image software timestamp latency.
- Motion excitation and VIBA2 completion, translation/rotation consistency, actual EKF EV fusion and navigation health.
- Exposure consistency between both eyes, failure/recovery and long-duration tests.
Passing software tests does not establish flight readiness.

## Backup / rollback
Original source/config subset: /home/nano/decxin_integration_20261110/before.tar.gz.
Launcher and launcher-test before-images are alongside it.
To revert: stop px4-vio first, restore only the backed-up modified files, remove the added CMake targets by restoring CMakeLists.txt, reconfigure with ROS sourced, and rebuild stereo_uvc at -j1 or -j2 before restarting.
Original unrelated dirty files were not reverted or committed. No PX4 parameters, arm/disarm, mode or flight commands were sent.


## Actual hardware results
- aarch64 CMake builds succeeded (initial and final build exit 0), with services stopped and -j2.
- C++ OpenCV is 4.8.0; Python cv2 is 4.5.4, and their stereoRectify(alpha=0) focal lengths differ.
  All active settings now use the C++ runtime output: fx=fy=694.006880918, cx=649.014984131, cy=444.565422058.
  The preliminary T_b_c1 also uses runtime R1. Source values verified by --check-config.
- Normal inertial startup rejects calibration_verified=0 with exit 2. Bench mode forces extrinsics_valid=false.
- Actual raw IMU probe: 15 seconds, 1493 measurements, no rejected samples/epoch changes. DDS delivery ~100 Hz.
  Sensor integral is ~5 ms but exported delivery is ~10 ms; bench IMU frequency is set to 100 Hz.
- Ground movement test: 150 seconds, 1263 processed frames, IMU received 14901/rejected 0/epoch 0.
  The user moved the entire airframe. Navigation reached valid=1, imu=1, ba2=1.
  Mean 9.21 fps (8.0..10.8) with PNG recording load, five brief tracking-recently-lost transitions.
  Last 20 seconds stationary pose ranges XYZ were 0.760/0.518/0.360 mm; this is a short static observation, not accuracy certification.
  Bench timeout returned 124, as expected, after orderly signal handling, SLAM shutdown and trajectory save.
- Full PNG recording FAILED after 277 written frames: bounded disk-writer queue exceeded 60.
  error.txt and original files are preserved. It is not a complete replay/calibration dataset.
  InputRecording already raises explicit stderr/error.txt; entry now additionally returns exit 4 for recording-close failure.
  The default service does not enable full --record. Full recording throughput still requires work.
- Bench observer: IMU 100.13 Hz; health 4.95 Hz; no visual-odometry messages; flight controller remained disarmed.
- Restored default stereo service observed >65 seconds with one capture process; console status/pose works.
  Health correctly remains navigation_ready=false and EKF EV fusion is not active.

## Installed watchdog corrections
/home/nano/vio_daemon.sh now uses the selected Linux comm (stereo_uvc vs truncated stereo_inertial)
for shutdown, recovery, CPU activity and locating the actual run directory.
Both [STAT] and [VIO_PERF] prove image-processing progress.
The default bridge profile is config/decxin_px4.yaml (identity body mapping, receive only).
No persistent airborne inertial mode was enabled.

## Bench rerun (ground, disarmed; VIO service stopped, one DDS agent running)
From /home/nano/ORB_SLAM3 with ROS and px4_msgs sourced:
build/Examples/Stereo/stereo_uvc Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/DECXIN_PX4_Bench.yaml Examples/Stereo-Inertial/DECXIN_rectify.yaml decxin_bench --headless --console --imu-config config/decxin_px4_imu.yaml --imu-bench --px4-config config/decxin_px4_bench.yaml

Normal mode must wait for verified rotation/noise/temporal calibration. Do not set calibration_verified merely because VIBA2 ran.
The current behavior tree is running but correctly waiting; this work has not established actual behavior-tree flight readiness.

## USB and frame-rate audit
- lsusb -t: DECXIN Camera is on Bus 02 at 5000M SuperSpeed under the USB3 hub; it is not USB2.
- V4L2 advertises 2720x960 MJPEG/YUYV at 30/60 fps.
- Raw V4L2 probe, 15s: MJPEG 29.62 fps, 7.73 MB/s (61.8 Mbit/s); YUYV 30.41 fps, 158.79 MB/s (1.27 Gbit/s), zero sequence drops.
- Raw V4L2 probe, 15s at 60: MJPEG 60.03 fps, 17.73 MB/s (141.8 Mbit/s), with 26 bad JPEG headers/87 sequence gaps; YUYV 59.90 fps, 312.82 MB/s (2.50 Gbit/s), 9 sequence gaps.
- Pure stereo at Fps=60: 9.5–9.7 fps in a no-daemon benchmark (capture 20ms, rectification 14ms, SLAM ~103ms). With the daemon/bridge active: 8.1–8.3 fps, MJPEG decode warnings and higher load.
- Default live profile remains 30 fps for stability. Raising camera input to 60 does not make SLAM or the behavior-tree loop 60 Hz; it currently worsens end-to-end service throughput.
- USB kernel history contains old UVC control/probe errors from earlier XU experiments, but current link is enumerated SuperSpeed and no bandwidth error appeared in the frame probes. Bad JPEG points to the device MJPEG stream/driver path, not USB2 negotiation.
