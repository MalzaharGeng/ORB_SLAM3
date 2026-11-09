/**
* This file is part of ORB-SLAM3
*
* 德创信 UVC 双目模组的实时双目入口（纯 Stereo，无 IMU）。
*
* 背景：2026-11-06 起硬件从 D435i 换成 UVC 双目光模块（1bcf:2d52），
* 原 stereo_inertial_realsense_D435i 走 librealsense，打不开这顆相机；
* 模组的 LSM6DSV 又没有暴露 HID 接口，Stereo-Inertial 暂时无惯导可用。
* 本入口因此只做纯双目：
*   V4L2 取 2720x960 -> 转灰度 -> 裁两目 1280x960 -> 按标定 R/T 立体校正
*   -> 交给 ORB-SLAM3 的 STEREO 模式。
*
* 性能取舍（分辨率与算法均不改）：
*   1. 校正前先转灰度。remap 是逐像素采样，单通道成本约为三通道的三分之一，
*      而 ORB-SLAM3 的 Frame 支持直接吃单通道，内部不会再转一次。
*   2. 取流+校正放独立线程，与 SLAM 重叠。单槽交接，处理不过来时丢旧帧，
*      而不是排队——实时系统要的是最新画面，不是完整性。
*
* 用法：
*   ./stereo_uvc path_to_vocabulary path_to_settings path_to_rectify_yaml [trajectory_name]
*
* 校正参数（K/D/R/T、裁剪偏移、设备节点）放在单独的 rectify yaml 里，
* 与 settings 分离：settings 每分支只保留一份，校正参数是模组级的。
*/

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>

using namespace std;

struct RectifyConfig {
    int frameWidth = 0, frameHeight = 0;
    int imgWidth = 0, imgHeight = 0;
    cv::Rect leftRoi, rightRoi;
    cv::Size size;
    cv::Mat K1, D1, K2, D2, R, T;
    cv::Mat mapLx, mapLy, mapRx, mapRy;
    string device = "/dev/video0";
    double fps = 30.0;
};

static bool readMat(cv::FileStorage &fs, const char *key, cv::Mat &out) {
    cv::FileNode n = fs[key];
    if (n.empty()) {
        cerr << "rectify 配置缺少 " << key << endl;
        return false;
    }
    fs[key] >> out;
    return true;
}

static bool loadRectify(const string &path, RectifyConfig &cfg) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "打不开校正配置：" << path << endl;
        return false;
    }
    cfg.frameWidth = (int)fs["Frame.width"];
    cfg.frameHeight = (int)fs["Frame.height"];
    cfg.imgWidth = (int)fs["Image.width"];
    cfg.imgHeight = (int)fs["Image.height"];
    int lx = (int)fs["Left.x0"], rx = (int)fs["Right.x0"];
    if (fs["Device"].isString()) cfg.device = (string)fs["Device"];
    if (!fs["Fps"].empty()) cfg.fps = (double)fs["Fps"];

    if (cfg.frameWidth <= 0 || cfg.frameHeight <= 0 || cfg.imgWidth <= 0 || cfg.imgHeight <= 0) {
        cerr << "rectify 配置的尺寸字段不完整" << endl;
        return false;
    }
    cfg.leftRoi = cv::Rect(lx, 0, cfg.imgWidth, cfg.imgHeight);
    cfg.rightRoi = cv::Rect(rx, 0, cfg.imgWidth, cfg.imgHeight);
    cfg.size = cv::Size(cfg.imgWidth, cfg.imgHeight);

    if (!readMat(fs, "K1", cfg.K1) || !readMat(fs, "D1", cfg.D1) ||
        !readMat(fs, "K2", cfg.K2) || !readMat(fs, "D2", cfg.D2) ||
        !readMat(fs, "R", cfg.R) || !readMat(fs, "T", cfg.T))
        return false;

    cfg.T = cfg.T.reshape(1, 3);  // stereoRectify 要 3x1

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cfg.K1, cfg.D1, cfg.K2, cfg.D2, cfg.size, cfg.R, cfg.T,
                      R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0.0);

    cv::initUndistortRectifyMap(cfg.K1, cfg.D1, R1, P1, cfg.size, CV_16SC2,
                                cfg.mapLx, cfg.mapLy);
    cv::initUndistortRectifyMap(cfg.K2, cfg.D2, R2, P2, cfg.size, CV_16SC2,
                                cfg.mapRx, cfg.mapRy);

    cout << "[Rectify] 帧 " << cfg.frameWidth << "x" << cfg.frameHeight
         << "  左目 x0=" << cfg.leftRoi.x << "  右目 x0=" << cfg.rightRoi.x
         << "  每目 " << cfg.imgWidth << "x" << cfg.imgHeight << endl;
    cout << "[Rectify] P1 fx=" << P1.at<double>(0, 0) << " cx=" << P1.at<double>(0, 2)
         << " cy=" << P1.at<double>(1, 2)
         << "  基线=" << cv::norm(cfg.T) * 1000.0 << " mm" << endl;
    return true;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cerr << "Usage: ./stereo_uvc path_to_vocabulary path_to_settings "
                "path_to_rectify_yaml [trajectory_name]" << endl;
        return 1;
    }

    RectifyConfig cfg;
    if (!loadRectify(argv[3], cfg))
        return 1;

    cv::VideoCapture cap(cfg.device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "打不开相机 " << cfg.device << endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.frameWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.frameHeight);
    cap.set(cv::CAP_PROP_FPS, cfg.fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cv::Mat probe;
    cap >> probe;
    if (probe.empty()) {
        cerr << "取不到帧" << endl;
        return 1;
    }
    if (probe.cols != cfg.frameWidth || probe.rows != cfg.frameHeight) {
        cerr << "实际取到 " << probe.cols << "x" << probe.rows
             << "，与配置的 " << cfg.frameWidth << "x" << cfg.frameHeight << " 不符" << endl;
        return 1;
    }
    cout << "[Camera] " << cfg.device << " " << probe.cols << "x" << probe.rows
         << " @" << cfg.fps << "fps" << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
    cout << "[SLAM] STEREO 模式已启动（无 IMU）" << endl;

    const string trajName = (argc >= 5) ? argv[4] : "stereo_uvc";
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    struct Slot {
        cv::Mat left, right;
        bool has = false;
        bool stop = false;
    } slot;
    mutex mtx;
    condition_variable cv;

    double msGrab = 0, msRect = 0;
    long nProduced = 0, nDropped = 0, nProcessed = 0;

    thread producer([&] {
        cv::Mat frame;
        int fails = 0;
        while (true) {
            auto t0 = chrono::steady_clock::now();
            cap >> frame;
            auto t1 = chrono::steady_clock::now();
            if (frame.empty()) {
                if (++fails > 20) {
                    lock_guard<mutex> lk(mtx);
                    slot.stop = true;
                    cv.notify_one();
                    break;
                }
                continue;
            }
            fails = 0;

            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::Mat leftR, rightR;
            cv::remap(gray(cfg.leftRoi), leftR, cfg.mapLx, cfg.mapLy, cv::INTER_LINEAR);
            cv::remap(gray(cfg.rightRoi), rightR, cfg.mapRx, cfg.mapRy, cv::INTER_LINEAR);
            auto t2 = chrono::steady_clock::now();

            {
                lock_guard<mutex> lk(mtx);
                if (slot.has) nDropped++;   // 上一帧还没被取走，直接丢
                slot.left = leftR;
                slot.right = rightR;
                slot.has = true;
                nProduced++;
                msGrab += chrono::duration<double, milli>(t1 - t0).count();
                msRect += chrono::duration<double, milli>(t2 - t1).count();
            }
            cv.notify_one();
        }
    });

    auto tStart = chrono::steady_clock::now();
    int lastState = -1;
    auto lastStats = chrono::steady_clock::now();
    long statFrames = 0;
    double msTrack = 0;

    while (true) {
        cv::Mat left, right;
        double tFrame = 0;
        {
            unique_lock<mutex> lk(mtx);
            cv.wait(lk, [&] { return slot.has || slot.stop; });
            if (slot.has) {
                left = slot.left;
                right = slot.right;
                slot.has = false;
                tFrame = chrono::duration<double>(chrono::steady_clock::now() - tStart).count();
            } else if (slot.stop) {
                break;
            }
        }

        auto tT0 = chrono::steady_clock::now();
        SLAM.TrackStereo(left, right, tFrame, vImuMeas, trajName);
        auto tT1 = chrono::steady_clock::now();
        nProcessed++;
        statFrames++;
        msTrack += chrono::duration<double, milli>(tT1 - tT0).count();

        int state = SLAM.GetTrackingState();
        if (state != lastState) {
            // ORB-SLAM3 的 eTrackingState：-1 未就绪, 0 无图像, 1 未初始化,
            // 2 正常, 3 刚丢失, 4 丢失。双目首帧即可初始化，通常很快进 2。
            cout << "[SLAM] t=" << tFrame << "s 状态 -> " << state
                 << " (1 未初始化 2 正常 3 刚丢失 4 丢失)" << endl;
            lastState = state;
        }

        double sinceStats = chrono::duration<double>(tT1 - lastStats).count();
        if (sinceStats >= 5.0) {
            double fGrab, fRect;
            long prod, drop;
            {
                lock_guard<mutex> lk(mtx);
                fGrab = msGrab / max(1L, nProduced);
                fRect = msRect / max(1L, nProduced);
                prod = nProduced;
                drop = nDropped;
                msGrab = msRect = 0;
                nProduced = nDropped = 0;
            }
            printf("[STAT] %.1f fps | 取流 %.1f ms  校正 %.1f ms  SLAM %.1f ms"
                   " | 产 %ld 帧 丢 %ld 帧\n",
                   statFrames / sinceStats, fGrab, fRect,
                   msTrack / max(1L, statFrames), prod, drop);
            cout.flush();
            msTrack = 0;
            statFrames = 0;
            lastStats = tT1;
        }
    }

    producer.join();
    cap.release();
    SLAM.Shutdown();
    // 默认不落轨迹文件：daemon 起来时工作目录是仓库根，每次启动都写一份会污染。
    // 需要时设 VIO_SAVE_TRAJ=1。
    if (const char *save = getenv("VIO_SAVE_TRAJ"); save && string(save) == "1")
        SLAM.SaveTrajectoryEuRoC(trajName + "_trajectory.txt");
    cout << "[SLAM] 结束，共处理 " << nProcessed << " 帧" << endl;
    return 0;
}
