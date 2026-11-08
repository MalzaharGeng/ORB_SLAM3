#!/usr/bin/env bash
set -euo pipefail
cat >&2 <<'EOF'
build_ros.sh 是上游 ROS1 rosbuild 的历史入口，不用于本工程 ROS2/PX4。
请先加载对应分支的 ROS2 与 px4_msgs 环境，再运行：
  bash ./build.sh -DORB_SLAM3_WITH_PX4=ON -DORB_SLAM3_BUILD_TESTS=ON
历史 ROS1 源码位于 Examples_old/ROS；未在当前 C++17 环境验证。
EOF
exit 2
