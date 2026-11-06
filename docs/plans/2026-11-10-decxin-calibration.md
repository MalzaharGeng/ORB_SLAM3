# DECXIN Camera calibration and readiness continuation
Goal: independently estimate temporal offset and verify mounting rotation on Nano before testing actual EKF fusion.
1. Capture raw MJPEG at <=10Hz plus timestamped PX4 SensorCombined, bounded queue, explicit completeness and disarmed checks.
2. Use runtime C++ OpenCV rectification, stereo depth and frame-to-frame PnP for independent visual rotation.
3. Fit camera/body rotation, temporal offset and gyro bias with held-out temporal blocks; reject poor excitation, disagreement or weak fit.
4. Keep candidate explicitly unverified; validate a second physical sequence and translation/rotation consistency before enabling any navigation output.
5. Check actual PX4 EV parameters/fusion and only progress with observed ground health. Do not issue flight/arm commands.
Validation: synthetic known delay and mounting rotation regression, capture I/O completeness, independent hardware data.
