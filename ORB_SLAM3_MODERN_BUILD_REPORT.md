# ORB-SLAM3 现代 Linux 工具链修复与验证行动报告

执行日期：2026-10-27（Asia/Shanghai）。项目：`/home/gengyinghao/ORB_SLAM3`。

## 1. 最终结论

| 项目 | 实际结果 |
| --- | --- |
| 最终配置 | 成功，退出码 0；不再需要 `CMAKE_POLICY_VERSION_MINIMUM` |
| 核心库 | `ORB_SLAM3` 完整编译、链接成功，退出码 0 |
| 内置第三方库 | DBoW2、g2o 由同一构建图实际编译；Sophus 使用随项目提供的头文件 |
| 示例 | 24 个数据集示例全部编译、链接成功：当前版 12 个、旧版 12 个 |
| 动态库检查 | 核心库和 24 个示例共 25 项通过；检查进程中清除了 `LD_LIBRARY_PATH` |
| 启动检查 | 24 个示例均能执行并输出 Usage，按预期返回 1，而非加载器错误 127 |
| 实际运行 | 原版 `rgbd_tum`，保留可视化，读取本地生成的 40 帧合成 RGB-D，最终退出码 0 |
| 跟踪、建图输出 | 初始化 1004 个地图点；40 个有限位姿、3 个关键帧；合成平面位移最大误差约 0.005795 m |
| 真实传感器数据集 | **未验证**。本次有界搜索未找到本地数据集，没有擅自下载大型数据集 |
| 尚存边界 | Qt 退出警告和 EGL/DRI3 软件渲染警告仍存在；ROS1、RealSense、IMU、真实回环等未验证 |

**构建及本次合成输入的基本运行阻断已解决，但不能据此宣称真实数据集 SLAM 精度、长期稳定性或全部传感器模式通过。** 合成输入、真实算法执行与真实传感器数据集验证是不同层次。

没有升级或降级系统依赖，没有删除核心源文件或禁用可视化、跟踪、建图、优化、回环或 IMU 功能。没有提交、推送源码，也没有删除原 `build`、已有库或用户备份。

## 2. 实际环境版本表

| 项目 | 实测版本 / 来源 | 选择与兼容性证据 |
| --- | --- | --- |
| Ubuntu | 26.04.1 LTS / `/etc/os-release` | 保留现有系统 |
| WSL2 内核 | `6.18.33.2-microsoft-standard-WSL2` / `uname` | 当前运行环境 |
| WSLg | 1.0.73.2 / `/mnt/wslg/versions.txt` | 实际连接显示服务运行 Viewer |
| GCC / G++ | Ubuntu `15.2.0-16ubuntu1`，版本 15.2.0 | 显式使用 `/usr/bin/gcc`、`/usr/bin/g++`；全量构建通过 |
| CMake | 4.2.3 | 修正实际参与构建的最低版本声明后，无策略兼容参数配置通过 |
| C++ 标准 | C++14，禁用 GNU 扩展 | `compile_commands.json` 中全部 87 个 C++ 编译单元实际使用 `-std=c++14`；另有 1 个 C 单元 |
| OpenCV | 4.10.0；Ubuntu 包 `4.10.0+dfsg-7ubuntu5` | 头文件 `/usr/include/opencv4`，库 `/usr/lib/x86_64-linux-gnu`；构建和实际图像读取通过 |
| Eigen | 3.4.0；Ubuntu 包 `3.4.0-5` | `/usr/include/eigen3`；未修改 Eigen，保留其警告 |
| Pangolin | 安装包元数据 0.9.6；前缀 `$HOME/local/pangolin-0.9.6` | 使用该前缀的导入目标、头文件、库；实际 Viewer 创建成功 |
| Pangolin 源码来源 | `42963caffb354f8e0db127ed90ff5fa4396dd864`，`v0.9.6-7-g42963caf` | **不是恰好位于 v0.9.6 tag**；当前源码干净。安装与源码构建的 `libpango_core.so.0.9.6` SHA-256 相同 |
| Boost.Serialization | 1.90.0，开发元包 `1.90.0.1ubuntu3` | 实际加载 `libboost_serialization.so.1.90.0` |
| OpenSSL | 开发包 `3.5.5-1ubuntu3.7` | 实际链接 `libcrypto.so.3`；旧 MD5 API 为弃用警告，不是本次失败原因 |
| Sophus | 随仓库提供，CMake 标记 1.1.0 | 核心直接使用头文件；没有独立构建其测试、Ceres/Python 扩展，也无需修改其 CMake |
| DBoW2 / g2o | 随 ORB-SLAM3 提供的源码快照，非独立 Git 子模块 | 仅修改各自 CMake；没有第三方算法补丁 |
| 图形后端 | Mesa 26.0.8，llvmpipe / LLVM 21.1.8，OpenGL 4.5 | `glxinfo -B` 返回 0；**软件渲染，不代表 GPU 加速** |
| 已安装 ROS | ROS2 Lyrical，`ROS_DISTRO=lyrical` | 不满足旧 ROS 示例的 ROS1 `rosbuild` 要求 |
| RealSense | CMake 未找到 `realsense2` | 相机及录制示例不属于本次已构建的 24 个目标 |

### 源码身份与初始状态

- 分支：`master`。
- ORB-SLAM3 提交：`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，2022-02-10，`Update Dependencies.md`。
- origin：`https://github.com/UZ-SLAMLab/ORB_SLAM3.git`。
- 实际只读查询 `git ls-remote origin HEAD refs/heads/master` 返回 0；两项均为上述提交，执行时本地与官方远端 master 相同。没有 fetch、checkout 或更新源码。
- 初始未提交内容只有已修改的 `CMakeLists.txt` 和未跟踪的 `CMakeLists.txt.backup_cpp11`。前者已是用户的 C++14 修改，后者未覆盖。
- 初始差异保存于 `build_codex/audit/initial-user.diff`，初始状态保存于 `initial-status.txt`。
- 原始第三方树：DBoW2 `12327062ef0475f6f87e078bc41c7f2446d84113`；g2o `2f48b4d9f9c33daa849d6c5623c31f75e2914e6d`；Sophus `4fab6742e6c5063d5afc9513d2a5e562cbc7780f`。

网络访问最初受到沙箱限制，Git 查询经授权后完成；没有将访问失败误报为“上游不存在修复”。图形服务也需要跨沙箱访问，授权后可用。

## 3. 根本原因与验证链

### 3.1 上次构建的第一个真实错误：DBoW2 根本没有生成

`/tmp/orbslam3_build_cxx14.log:4062`，完整原日志也保存在 `build_codex/audit/original-build.log`：

```text
gmake[2]: *** No rule to make target '/home/gengyinghao/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so', needed by '/home/gengyinghao/ORB_SLAM3/lib/libORB_SLAM3.so'. Stop.
```

旧配置把 DBoW2、g2o 写成源码目录下 `.so` 文件的绝对路径。根 CMake 只 `add_subdirectory` 了 g2o，没有把 DBoW2 加入依赖图；审计也确认 DBoW2 的 `lib` 目录不存在。配置成功并不检查这种外部文件将来是否真的能生成。

所以最后两行 `Error 2` 不是根因；搜索时还必须匹配 `No rule to make target` / `***`，不能只搜索 `error:`。原日志没有发现 sigslot/C++14 编译错误，也没有证据支持将 Eigen 的警告认定为此次失败原因。

修复：添加 DBoW2 子目录，并直接链接 `DBoW2`、`g2o` CMake 目标。新构建日志证明两库实际编译，依赖顺序由 CMake 管理，不再要求预先手动构建源码目录中的库。

### 3.2 C++ 标准与旧示例宏不是同一问题

Pangolin 的导入目标声明了 `cxx_decltype_auto`，其 sigslot 头文件也使用 C++14 类型别名。C++14 足够，无证据要求升级 C++17。

用户初始补丁启用 C++14 的方向正确，但把 `COMPILEDWITHC11` 改成了 `COMPILEDWITHC14`。示例的 `#ifdef COMPILEDWITHC11` 实际用于选择 `std::chrono::steady_clock`；没有该宏时进入旧 `monotonic_clock` 分支。该宏是历史兼容开关，而非必须与标准数字同步的名称。

修复只在 CMake 恢复此宏，没有批量修改示例。用 CMake 标准属性统一新建目标的 C++14，不再手工向全局 flags 拼接 `-std=`。最终详细命令证明核心、DBoW2、g2o 和示例使用同一标准；全部示例实际构建通过。这一示例问题由代码分支审计确认，并非冒称原日志已经出现相应编译错误。

### 3.3 CMake 4 的旧策略兼容入口

原根目录和 DBoW2 最低版本为 2.8，g2o 为 2.6，依赖外部 `CMAKE_POLICY_VERSION_MINIMUM=3.5` 才绕过新 CMake 的旧兼容策略限制。

将实际参与此次构建的三个入口调整为根 3.13、DBoW2/g2o 3.10。根使用的 `add_compile_definitions`、`target_link_options` 分别需要至少 3.12、3.13。没有无依据地修改 Sophus、ROS 或所有其他 CMake 文件。

最终从新的 `build_codex/fresh_config` 重新配置，退出码 0，没有传入策略降限参数。

### 3.4 编译通过后的真实运行时失败：Pangolin 间接依赖丢失

第一次启动已编译的 `rgbd_tum`，退出码 **127**：

```text
error while loading shared libraries: libpango_windowing.so.0: cannot open shared object file
```

`readelf -d` 显示核心库有 `DT_RUNPATH`，Pangolin 的 `libpango_display.so.0` 自身却没有 RPATH/RUNPATH，并依赖 `libpango_windowing.so.0`、`libpango_image.so.0`。RUNPATH 不为这种间接依赖提供传递搜索，故 CMake 找到了包、链接器也成功，并不代表运行时能够找到依赖。

修复：只在 Linux 的 ORB-SLAM3 核心库链接时使用 `LINKER:--disable-new-dtags`，让 CMake 已推导的构建库路径以可传递的 `DT_RPATH` 输出。路径仍来自依赖查找，没有硬编码本机前缀；没有改安装好的 Pangolin、`ld.so.conf` 或 shell 启动文件。

修复后 `readelf` 确认 `RPATH`；在清除 `LD_LIBRARY_PATH` 的进程环境中，核心和全部示例均无 `not found`。该局部 ELF 兼容方案会采用传统 RPATH 的搜索优先级；更长期的依赖侧方案是给 Pangolin 自身设置正确的安装 RPATH，但本次不改动影响其他项目的安装。

### 3.5 实际跟踪后退出崩溃：未完成线程收尾

正确加载词袋后，原版示例已经读取 40 帧、创建地图并写出轨迹，但退出码是 **139**，不能宣称运行成功。

诊断日志 `runtime-segv-trace.log` 中的真实栈包含：

```text
libQt5Gui / QPainter / QWidget
libopencv_highgui / cvWaitKey / cv::waitKey
libORB_SLAM3 / ORB_SLAM3::Viewer::Run
```

源码中 `System::Shutdown()` 对 Viewer 的退出请求、等待，以及建图/回环线程的等待都被注释。主线程结束时，Viewer 仍在 Qt 的事件处理中，和进程退出的 GUI 资源清理冲突。

修复：请求 Viewer 结束并在外部调用时 join，避免 Viewer 的菜单回调 join 自己；join 局部建图和回环线程，等待活动 GBA；Viewer 的暂停等待允许结束请求打断，并在创建窗口的线程中销毁 OpenCV/Pangolin 窗口。没有删除、替换或禁用算法线程。

修改后重新构建全部目标并运行同一输入：**退出码 0**。最后一次运行没有预加载诊断器，也清除了 `LD_LIBRARY_PATH`。

### 3.6 验证命令自身的一次路径错误

首次 GUI 命令把词袋相对路径多写一级，返回 255，日志 `runtime-rgbd.log`。随后改为 `../../../Vocabulary/ORBvoc.txt`。这不是源码故障；没有为错误命令修改词袋加载逻辑。失败记录保留在汇总中。

## 4. 修改清单

备份基目录：`build_codex/audit/backups/`。备份为每个文件**本次修改前**的内容，而不是统一回到 Git HEAD。

| 文件 | 修改内容与目的 | 功能影响 / 备份 |
| --- | --- | --- |
| `CMakeLists.txt` | 最低 3.13；C++14 属性；保留 C11 历史宏；添加 DBoW2；目标方式链接第三方；构建产物移入 binary tree；Linux 核心 RPATH 兼容 | 不删源文件，不改算法；备份同名文件，保留用户原 C++14 修改 |
| `Thirdparty/DBoW2/CMakeLists.txt` | 最低 3.10；输出位置改为构建目录 | 第三方算法无变化；备份为同相对路径 |
| `Thirdparty/g2o/CMakeLists.txt` | 最低 3.10；默认库输出位置改为构建目录 | 优化实现、Eigen 对齐及警告选项未修改；备份为同相对路径 |
| `build.sh` | Bash 严格错误处理；从脚本位置确定源码；一张依赖图；可指定 `BUILD_DIR`、`BUILD_JOBS`；转发 CMake 参数；只在词袋未解压时解压 | 避免前一步失败仍继续及无上限并行；Sophus 是头文件依赖，不再强制构建其可选测试；备份同名文件 |
| `src/System.cc` | 修复 Shutdown 的线程收尾 | 仅生命周期管理；保留计算功能；备份 `src/System.cc` |
| `src/Viewer.cc` | 暂停时响应结束；同线程销毁窗口 | 保留可视化；备份 `src/Viewer.cc` |
| `.gitignore` | 忽略本次 `/build_codex/` | 防止备份、日志和生成数据混入源码差异；备份同名文件 |
| 本报告 | 新增行动报告 | 不修改已有文档 |

`build_codex/audit/` 还保留生成器、轨迹检查器、产物检查器、临时崩溃诊断代码/共享库，以及完整日志；均不是链接进最终程序的核心代码。最终运行没有使用诊断器。

新增解压的 `Vocabulary/ORBvoc.txt` 是仓库原有压缩包的内容；没有下载或替换词袋。压缩包校验值记录于 `vocabulary.sha256`。原 `build` 和原有 `Thirdparty/g2o/lib` 没有删除或作为新构建的依赖使用。g2o 原构建逻辑仍会生成其忽略的 `config.h`；该文件也留有备份。

## 5. 完整复现命令

### 5.1 配置与构建

以下命令对应已验证的环境与参数；重新执行时会复用独立构建目录，不会清理旧 `build`。

```bash
cd /home/gengyinghao/ORB_SLAM3
set -o pipefail
mkdir -p build_codex/audit

cmake -S . -B build_codex \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_PREFIX_PATH="$HOME/local/pangolin-0.9.6" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  2>&1 | tee build_codex/audit/repro-configure.log

cmake --build build_codex --target ORB_SLAM3 --parallel 2 --verbose \
  2>&1 | tee build_codex/audit/repro-core.log
cmake --build build_codex --parallel 2 --verbose \
  2>&1 | tee build_codex/audit/repro-all.log
```

更新后的脚本也已实际执行通过：

```bash
BUILD_DIR="$PWD/build_codex" BUILD_JOBS=2 bash ./build.sh \
  -DCMAKE_PREFIX_PATH="$HOME/local/pangolin-0.9.6" \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

需要另一个完全独立构建时，把上述 `BUILD_DIR` 改成 `$PWD/build_codex/reproduce` 即可；不要复制旧缓存。此次最初的 `build_codex` 是全新目录，完成了实际编译；最终补丁在其上进行了受影响目标的增量重编译及全目标链接验证。另一个全新目录 `build_codex/fresh_config` 已对最终源码实际配置成功，**没有再重复做第二次完整 clean 编译**。

实测机器 32 个逻辑 CPU、约 15 GiB RAM，使用 2 路并行避免盲目 `make -j` 的内存压力。保留上游 `-O3`、`-march=native`；因此产物是本机 CPU 定向构建，不承诺跨 CPU 二进制可移植性或逐字节一致。

### 5.2 动态依赖与启动检查

```bash
python3 build_codex/audit/validate_artifacts.py
env -u LD_LIBRARY_PATH ldd build_codex/lib/libORB_SLAM3.so
readelf -d build_codex/lib/libORB_SLAM3.so | rg 'PATH'
```

检查脚本逐一检查核心库和 24 个可执行文件的依赖，再启动全部示例验证 Usage / 退出码 1，同时验证 C++ 编译标准。它不会把 `ldd` 自身返回 0 误当成所有库可解析，而是额外检查 `not found`。

### 5.3 合成 RGB-D 基本运行

生成器和检查器已保存在本次审计目录中；需要本机已安装的 Python3、NumPy、OpenCV Python 绑定。生成器不访问网络，不改项目原 YAML。

```bash
cd /home/gengyinghao/ORB_SLAM3
python3 build_codex/audit/generate_rgbd_smoke.py
test -f Vocabulary/ORBvoc.txt || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
mkdir -p build_codex/audit/runtime

cd build_codex/audit/runtime
env -u LD_LIBRARY_PATH timeout 90 ../../Examples/RGB-D/rgbd_tum \
  ../../../Vocabulary/ORBvoc.txt \
  ../synthetic_rgbd/settings.yaml \
  ../synthetic_rgbd \
  ../synthetic_rgbd/associations.txt \
  > ../runtime-repro.log 2>&1
runtime_exit=$?
printf 'runtime_exit=%s\n' "$runtime_exit"
cd ../../..
python3 build_codex/audit/verify_rgbd_smoke.py
```

GUI 命令需要能够连接 WSLg；在受限工具沙箱中需授权访问显示服务。不要把沙箱内 `unable to open display` 直接解释为 WSLg 未安装。

输入定义：固定随机种子 `20261027`，640×480 图像，40 对实际 PNG 文件，30 Hz，2 m 恒深平面（16 位深度值 10000、比例 5000），每帧视场平移 2 像素，无畸变，使用生成配置中的真实数值内参。原始 TUM1 配置未被修改。检查器要求恰好 40 个有限位姿、有效关键帧，并验证相对于已知合成平面运动的三维位移误差小于 0.05 m。该阈值只是冒烟断言，不是实际数据集精度指标。

## 6. 测试结果与证据

所有路径如无特殊说明均相对于 `build_codex/audit/`。逐次退出码集中在 `results.txt`，既保存失败，也保存最终通过，不隐藏中间过程。

| 命令 / 阶段 | 退出码 | 结果与日志 |
| --- | --- | --- |
| 原日志只读审计 | 非本次重跑 | 第 4062 行缺 DBoW2；`original-build.log` |
| 初次独立 `cmake -S . -B build_codex ...` | 0 | `configure.log` |
| `cmake --build build_codex --target ORB_SLAM3 --parallel 2 --verbose` | 0 | `build-core.log`，包含真实编译与链接命令 |
| `cmake --build build_codex --parallel 2 --verbose` | 0 | `build-all.log`，24 个示例 |
| 修复 RPATH 前启动 `rgbd_tum` | 127 | 加载器缺 Pangolin 间接库；`startup-before-rpath.log` |
| RPATH 修复后全量目标增量构建 | 0 | `build-rpath.log` |
| `rgbd_tum` 无参数启动 | 1（预期） | 正确 Usage，`startup-fixed.log` |
| 首次 GUI，词袋相对路径错误 | 255 | `runtime-rgbd.log`，命令自身错误已纠正 |
| 正确路径、旧 Shutdown 运行 | 139 | 已建图并保存轨迹但退出崩溃；`runtime-rgbd-fixed-path.log` |
| 附加栈打印器诊断运行 | 139 | `runtime-segv-trace.log`，Viewer/Qt 栈，未伪装通过 |
| Shutdown/Viewer 修复后全目标构建 | 0 | `build-shutdown.log` |
| 最终源码独立配置 `build_codex/fresh_config` | 0 | `configure-fresh-final.log`，不使用策略最低版本参数 |
| 更新后的 `build.sh` 实际执行 | 0 | `build-script-final.log`；语法检查 `bash -n build.sh` 也返回 0 |
| 最终原版 `rgbd_tum` + 40 帧合成输入 + Viewer | **0** | `runtime-final.log`，无预加载器、无 `LD_LIBRARY_PATH` |
| `python3 build_codex/audit/verify_rgbd_smoke.py` | 0 | `trajectory-check-final.log`：40 位姿、3 关键帧，最大位移误差 0.005795 m |
| `python3 build_codex/audit/validate_artifacts.py` | 0 | `artifact-checks-final.log`：25 项依赖、24 项启动、87 个 C++14 编译单元 |
| `git diff --check` | 0 | 无空白错误 |
| 授权后的 `glxinfo -B` | 0 | WSLg 软件渲染 OpenGL 4.5；授权前沙箱中访问显示失败 |
| 授权后的 `git ls-remote origin HEAD refs/heads/master` | 0 | 官方远端与本地提交一致；未改变工作树 |

最终运行日志中的关键输出：

```text
Images in the sequence: 40
New Map created with 1004 points
Starting the Viewer
Shutdown
median tracking time: 0.0164419
mean tracking time: 0.0167759
Saving camera trajectory to CameraTrajectory.txt ...
Saving keyframe trajectory to KeyFrameTrajectory.txt ...
```

轨迹位于 `build_codex/audit/runtime/CameraTrajectory.txt` 和 `KeyFrameTrajectory.txt`，未覆盖项目根目录的用户轨迹。上述计时仅为本次短合成序列的实测值，不作为性能基准。

### 已构建示例清单

以下 12 个当前示例及其 12 个 `_old` 对应目标全部通过；完整可执行文件路径保存在 `executables.txt`：

```text
rgbd_tum
mono_tum mono_kitti mono_euroc mono_tum_vi
stereo_kitti stereo_euroc stereo_tum_vi
mono_inertial_euroc mono_inertial_tum_vi
stereo_inertial_euroc stereo_inertial_tum_vi
```

没有把 RealSense 条件目标、ROS1 工程或 Sophus 独立单元测试计入“全部通过”。

## 7. 尚未解决或尚未验证的问题

1. **真实数据集未验证**：有界搜索覆盖 Linux 用户目录及部分 Windows 用户目录，没有发现 `rgb.txt`、`associations.txt`、EuRoC `data.csv` 等输入。这不是全磁盘存在性证明。提供有效 TUM / EuRoC / KITTI 序列路径后才能进行真实传感器输入验证，本次不擅自下载大数据。
2. **Qt 退出警告仍有**：最终退出 0，但日志仍有 `QObject::killTimer`、`QObject::~QObject` 跨线程计时器提示。已修复并验证实际 SIGSEGV；尚未做 Qt GUI 全局对象归属重构或长期/反复开关窗口压力测试，没有屏蔽这些警告。
3. **WSLg 软件渲染**：仍有 EGL/DRI3 警告；Viewer 可用但 llvmpipe 并非 GPU 加速。没有修改 Windows 驱动、WSL 或 Mesa。
4. **ROS**：已有的是 ROS2 Lyrical，旧示例需要 ROS1 `rosbuild`。没有为核心库强装 ROS1，也没有把工程迁移到 ROS2。该部分未构建。
5. **RealSense**：SDK、相机输入未具备。所有由 `realsense2_FOUND` 控制的相机/IMU/录制目标未生成、未构建、未运行。
6. **其他传感器与算法场景**：单目、双目、IMU 的示例已编译和 Usage 检查，但没有对应真实输入；真实回环、地图合并、GBA 活动关闭场景、Atlas 序列化、交互菜单并发关闭等未验证。线程存在不代表这些功能被本次序列触发。
7. **警告不是错误**：保留 Eigen 对齐 API 弃用、OpenSSL MD5 弃用、旧代码符号性/初始化顺序等编译警告。未新增全局消警或降低检查。Pangolin 自身传递的特定 `-Wno-error=...` 原样保留，并非本次为掩盖失败而加入。
8. **构建分离边界**：库和程序位于新构建树；g2o 仍按上游方式在源码树生成被忽略的 `config.h`。不声称源码树绝对零生成文件，也未测试不同工具链同时生成该文件的并发构建。

下一步最有价值的是提供小段真实数据集，使用已编译的对应示例验证持续跟踪与轨迹，而不是盲目更换已经通过编译的 GCC/OpenCV/Pangolin 版本。

## 8. 回滚方法

**不要用 `git reset --hard` 或直接 `git restore CMakeLists.txt`，后者会连用户先前的 C++14 修改一起撤销。**

以下命令恢复本次任务开始时各文件的内容，包括原有 C++14 补丁；不会删除原 `build`、数据集、备份或日志。执行前如任务结束后又有新编辑，应先另行保存，避免覆盖后续工作。

```bash
cd /home/gengyinghao/ORB_SLAM3
backup=build_codex/audit/backups
for relative_path in \
  .gitignore \
  CMakeLists.txt \
  Thirdparty/DBoW2/CMakeLists.txt \
  Thirdparty/g2o/CMakeLists.txt \
  build.sh \
  src/System.cc \
  src/Viewer.cc
do
  cp -- "$backup/$relative_path" "$relative_path"
done
git status --short
git diff
```

如果还要恢复 g2o 生成配置，可另执行 `cp -- "$backup/Thirdparty/g2o/config.h" Thirdparty/g2o/config.h`。本报告、新构建目录和解压词袋可以保留，不影响恢复源码；本报告不提供自动删除用户目录的命令。回滚源码后已有二进制仍是修复版，若要运行回滚版，应另行构建，不要混淆两者。

完整最终代码差异：`build_codex/audit/final-full.diff`；原有用户差异：`build_codex/audit/initial-user.diff`。二者与备份共同区分了“用户原来改过什么”和“本次新增改了什么”。
