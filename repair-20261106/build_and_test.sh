set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/nano/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
export LD_LIBRARY_PATH=/home/nano/ORB_SLAM3/build/lib:/home/nano/orb_dependencies/pangolin-0.9.6/lib:/home/nano/orb_dependencies/sysroot/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}
cd /home/nano/ORB_SLAM3
cmake -S . -B build > /home/nano/vio_repair_20261106/configure.log 2>&1
cmake --build build --target stereo_inertial_realsense_D435i px4_vio_regression px4_recording_gate_regression px4_bridge_fixture d435i_recovery_replay -j1 > /home/nano/vio_repair_20261106/build.log 2>&1
ctest --test-dir build --output-on-failure > /home/nano/vio_repair_20261106/ctest.log 2>&1
ROS_DOMAIN_ID=43 python3 /home/nano/test_vio_ned_dds.py > /home/nano/vio_repair_20261106/ned_dds.log 2>&1
ROS_DOMAIN_ID=43 VIO_TEST_AIRBORNE=1 python3 /home/nano/test_vio_ned_dds.py > /home/nano/vio_repair_20261106/ned_air_dds.log 2>&1
git diff --check > /home/nano/vio_repair_20261106/diff_check.log 2>&1
echo REPAIR_BUILD_AND_TEST_PASS
