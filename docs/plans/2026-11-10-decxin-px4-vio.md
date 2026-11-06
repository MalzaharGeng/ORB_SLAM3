# DECXIN / PX4 IMU integration plan
Goal: restore the UVC entry's navigation bridge and console, add an opt-in PX4 raw IMU input, and verify behavior-tree authority on Nano.
Architecture: retain stereo as default; optionally consume SensorCombined in body FRD through a bounded buffer with bracketed image intervals. Both use the existing PX4 publication gate without relaxing inertial/calibration/freshness checks.
Constraints: ground/disarmed testing authorized; no arm, mode, flight or PX4 parameter commands. Camera faces forward, top upward; lever arm and temporal calibration postponed by user. Never substitute D435i extrinsics. Existing dirty work retained. Stop VIO before builds, at most -j2.
1. Backup and audit active code; verify IMU timestamp domain/rate on actual DDS.
2. Fix image timestamps at acquisition, validate rectified calibration, recover console/shutdown and bridge diagnostics.
3. Add optional IMU config, bounded interval buffer, ROS reader and fail-closed checks. Inertial use requires explicitly verified spatiotemporal calibration.
4. Compile on aarch64 with VIO stopped; test gaps, resets, bracketing and no synthetic inertial readiness.
5. Run isolated ROS behavior-tree tests; restore services and perform live stereo/bridge/health checks. Record remaining physical prerequisites and rollback.
