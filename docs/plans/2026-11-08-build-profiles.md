# 构建配置与持续集成实施计划

目标：master 使用本机 CModel 与 ROS 2 Lyrical 环境；nano 保留 D435i、Ubuntu 22.04/Humble/aarch64 设备约束。两分支只同步本轮通用构建修改，分别提交；不合并分支，不连接设备，不搬运构建产物。

1. 保存远端起点，使用独立 worktree，原工作区不 checkout/reset。
2. 主工程和 DBoW2/g2o 按构建类型管理编译参数；native 在 master 默认关、nano 默认开，aarch64 使用 -mcpu=native。Boost、OpenSSL、Eigen、Threads 使用导入目标。
3. 拆分数据集、相机、历史示例开关；历史默认关。g2o config.h 移到构建目录，消费者用明确头路径；RealSense 只链接采集目标。可显式要求 RealSense，避免生产目标静默消失。
4. build.sh 在新目录可选 Ninja，自动使用可关闭的 ccache，保留已有生成器，检查并行数；master 历史 build_ros.sh 明确退出，提示真正 ROS2 构建入口。
5. 新增构建回归：目标集合、Release/Debug 参数、g2o 不写源码；现有 CTest 确认运行逻辑未改变。
6. CI 分 core 与 ROS/PX4 两类，Ubuntu22/Humble 仅作兼容检查，不冒充 WSL Lyrical 或 Nano 实机。固定 Pangolin/px4_msgs revision，缓存键按架构/编译器/配置区分，产物限于测试与构建日志。
7. master/nano 分别保留不同相机路径与默认 native 设置；共用文件 SHA256 核对。设备脚本、参数、相机标定均不改。
8. 代码、CI配置、文档分开提交，提交说明记录本地构建/测试数量及未验证边界。推送前 fetch 防止覆盖远端新增提交，普通 fast-forward 推送，回读引用与文件哈希。

本轮不拆 Viewer 架构，不引入 LTO/PCH，不删除数据集大文件或改写历史。CI 实际运行结果以远端完成状态为准。
