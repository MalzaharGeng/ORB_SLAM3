# DECXIN Camera / PX4 IMU milestone — 2026-11-10

Built and tested on Jetson Orin Nano, 1280x960 per eye, 1000 ORB features.
Camera is MJPG 2720x960 @30 fps with one V4L2 buffer and host timestamp after read.

- Two complete motion datasets independently validated camera/body rotation and timing.
  Offset -44.534 ms; inter-dataset disagreement 0.276 ms and 0.309 degrees.
  Translation [+0.150,+0.0205,+0.0495] m in body FRD is user measured, not jointly optimized.
- Actual TrackLocalMap missing-prior fixture: SEGFAULT before fix, passes after keyframe bootstrap.
- 557 recorded frame pairs replay without crash and exercise the bootstrap; rotation-heavy replay does not complete VIBA2.
- New calibrated live run completes VIBA1/VIBA2 and keeps valid=1, imu=1, ba2=1 with a constant map until a later raw IMU wait timeout.
- A 1-second DDS/serial data gap also occurs in a recording; this is a real receive gap, not a host clock jump.
  Long-duration VIO readiness is therefore NOT established. A missing IMU interval stops tracking; no synthetic measurements are substituted.
- Publication remains inhibited (extrinsics_valid=0); no arming/mode/flight/PX4 parameter commands were sent.
- Default service remains stereo. VIO is opt-in with VIO_IMU_CONFIG; calibration profile changes require fresh timing validation.
- 44 behavior-tree tests passed earlier. Final calibration changes pass launcher, VIO gate, sortie, recording gate, IMU buffer and missing-prior tracking regressions.
- Large PNG recording can overflow its bounded disk writer queue; incomplete recordings are reported and preserved.
- Source and raw datasets remain on the Nano. Raw imagery, credentials, builds and runtime image rings are not part of this commit.
