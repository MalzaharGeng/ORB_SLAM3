/**
* This file is part of ORB-SLAM3
*
* 德创信 UVC 双目模组的实时双目入口（纯 Stereo，无 IMU）。
*
* 背景：2026-11-06 起硬件从 D435i 换成 UVC 双目光模块（1bcf:2d52），
* 原 stereo_inertial_realsense_D435i 走 librealsense，打不开这顆相机；
* 模组的 LSM6DSV 又没有暴露 HID 接口，Stereo-Inertial 暂时无惯导可用。
* 本入口因此只做纯双目：
*   V4L2 取 2720x960 -> 按偏移裁出两目 1280x960 -> 用标定的 R/T 做立体校正
*   -> 交给 ORB-SLAM3 的 STEREO 模式。
*
* 用法：
*   ./stereo_uvc path_to_vocabulary path_to_settings path_to_rectify_yaml [trajectory_name]
*
* 校正参数（K/D/R/T、裁剪偏移、设备节点）放在单独的 rectify yaml 里，
* 与 settings 分离：settings 每分支只保留一份，校正参数是模组级的。
*/

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
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
    int nFrames = 0, nFail = 0;
    int lastState = -1;
    auto tStart = chrono::steady_clock::now();
    int lastReport = 0;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            if (++nFail > 20) {
                cerr << "连续取帧失败，退出" << endl;
                break;
            }
            continue;
        }
        nFail = 0;

        const double t = chrono::duration_cast<chrono::duration<double>>(
                             chrono::steady_clock::now() - tStart).count();

        cv::Mat left = frame(cfg.leftRoi);
        cv::Mat right = frame(cfg.rightRoi);
        cv::Mat leftR, rightR;
        cv::remap(left, leftR, cfg.mapLx, cfg.mapLy, cv::INTER_LINEAR);
        cv::remap(right, rightR, cfg.mapRx, cfg.mapRy, cv::INTER_LINEAR);

        SLAM.TrackStereo(leftR, rightR, t, vImuMeas, trajName);
        nFrames++;

        int state = SLAM.GetTrackingState();
        if (state != lastState) {
            // ORB-SLAM3 的 eTrackingState：-1 未就绪, 0 无图像, 1 未初始化,
            // 2 正常, 3 刚丢失, 4 丢失。双目首帧即可初始化，所以通常很快进 2。
            cout << "[SLAM] t=" << t << "s 状态 -> " << state
                 << " (1 未初始化 2 正常 3 刚丢失 4 丢失)" << endl;
            lastState = state;
        }
        if (static_cast<int>(t) >= lastReport + 5) {
            lastReport = static_cast<int>(t);
            cout << "[SLAM] t=" << t << "s 帧数 " << nFrames
                 << " 状态 " << state << endl;
        }
    }

    cap.release();
    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC(trajName + "_trajectory.txt");
    cout << "[SLAM] 结束，共处理 " << nFrames << " 帧" << endl;
    return 0;
}
