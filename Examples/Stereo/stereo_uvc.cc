/**
* This file is part of ORB-SLAM3
*
* 德创信 UVC 双目入口：默认 Stereo，经过标定后可选 PX4 原始 IMU。
*
* 背景：2026-11-06 起硬件从 D435i 换成 UVC 双目光模块（1bcf:2d52），
* 原 stereo_inertial_realsense_D435i 走 librealsense，打不开这顆相机；
* 模组的 LSM6DSV 又没有暴露 HID 接口，Stereo-Inertial 暂时无惯导可用。
* 默认保持纯双目；标定完成后 --imu-config 可消费 PX4 SensorCombined：
*   V4L2 取 2720x960 -> 转灰度 -> 裁两目 1280x960 -> 按标定 R/T 立体校正
*   -> 交给 ORB-SLAM3 的 STEREO 或经过标定的 IMU_STEREO 模式。
*
* 采集与处理安排：
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
#include <atomic>
#include <csignal>
#include <cmath>
#include <stdexcept>
#include "Examples/Utils/RecoveryConsole.h"
#ifdef ORB_SLAM3_WITH_PX4
#include "Px4Bridge.h"
#include "Px4Imu.h"
#endif
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

#include <InputRecording.h>
#include <AsyncNavigationRecording.h>
#include "LatestCameraInput.h"
#include "Examples/Utils/UvcGray.h"
#include <sstream>
#include <RollingInputRecording.h>
#include <System.h>
#ifdef ORB_SLAM3_WITH_CUDA_FAST
#include "CudaRectify.h"
#endif

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>

using namespace std;
static volatile std::sig_atomic_t interrupted=0;
static void stopSignal(int) { interrupted=1; }


struct RectifyConfig {
    int frameWidth = 0, frameHeight = 0;
    int imgWidth = 0, imgHeight = 0;
    cv::Rect leftRoi, rightRoi;
    cv::Size size;
    cv::Mat K1, D1, K2, D2, R, T;
    cv::Mat mapLx, mapLy, mapRx, mapRy;
    double fx=0, fy=0, cx=0, cy=0, baseline=0;
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

    cfg.fx=P1.at<double>(0,0);cfg.fy=P1.at<double>(1,1);
    cfg.cx=P1.at<double>(0,2);cfg.cy=P1.at<double>(1,2);cfg.baseline=cv::norm(cfg.T);
    const cv::Rect bounds(0,0,cfg.frameWidth,cfg.frameHeight);
    if((cfg.leftRoi & bounds)!=cfg.leftRoi || (cfg.rightRoi & bounds)!=cfg.rightRoi)
        throw std::runtime_error("Camera crop is outside the captured frame");
    cv::initUndistortRectifyMap(cfg.K1, cfg.D1, R1, P1, cfg.size, CV_16SC2,
                                cfg.mapLx, cfg.mapLy);
    cv::initUndistortRectifyMap(cfg.K2, cfg.D2, R2, P2, cfg.size, CV_16SC2,
                                cfg.mapRx, cfg.mapRy);

    cout << "[Rectify] 帧 " << cfg.frameWidth << "x" << cfg.frameHeight
         << "  左目 x0=" << cfg.leftRoi.x << "  右目 x0=" << cfg.rightRoi.x
         << "  每目 " << cfg.imgWidth << "x" << cfg.imgHeight << endl;
    cout << std::setprecision(12) << "[Rectify] P1 fx=" << P1.at<double>(0, 0) << " cx=" << P1.at<double>(0, 2)
         << " cy=" << P1.at<double>(1, 2)
         << "  基线=" << cv::norm(cfg.T) * 1000.0 << " mm" << endl;
    return true;
}

static int run(int argc, char **argv) {
    if (argc < 4) {
        cerr << "Usage: ./stereo_uvc path_to_vocabulary path_to_settings "
                "path_to_rectify_yaml [trajectory_name]" << endl;
        return 1;
    }

    std::signal(SIGINT,stopSignal);std::signal(SIGTERM,stopSignal);
    string recordDirectory,px4Config,imuConfig,trajName="stereo_uvc";
    bool consoleEnabled=false,checkConfig=false,imuBench=false;
    for(int i=4;i<argc;++i) {
        const string arg=argv[i];
        if((arg=="--record" || arg=="--px4-config" || arg=="--imu-config") && i+1>=argc)
            throw std::runtime_error("Missing value for "+arg);
        if(arg=="--record") recordDirectory=argv[++i];
        else if(arg=="--px4-config") px4Config=argv[++i];
        else if(arg=="--imu-config") imuConfig=argv[++i];
        else if(arg=="--console") consoleEnabled=true;
        else if(arg=="--no-console") consoleEnabled=false;
        else if(arg=="--headless") {}
        else if(arg=="--check-config") checkConfig=true;
        else if(arg=="--imu-bench") imuBench=true;
        else if(arg.rfind("--",0)==0) throw std::runtime_error("Unsupported UVC option: "+arg);
        else if(trajName=="stereo_uvc") trajName=arg;
        else throw std::runtime_error("Unexpected argument: "+arg);
    }
    const bool inertial=!imuConfig.empty();
    if(imuBench && !inertial) throw std::runtime_error("--imu-bench requires --imu-config");
    double imageTimeOffset=0.;
    RectifyConfig cfg;
    if(!loadRectify(argv[3],cfg)) return 1;
    cv::FileStorage settings(argv[2],cv::FileStorage::READ);
    if(!settings.isOpened()) throw std::runtime_error("Cannot open SLAM settings");
    auto check=[&](const char* key,double expected,double tolerance) {
        if(settings[key].empty() || !std::isfinite(double(settings[key])) || std::abs(double(settings[key])-expected)>tolerance)
            throw std::runtime_error(std::string("Rectified calibration mismatch: ")+key);
    };
    check("Camera1.fx",cfg.fx,.01);check("Camera1.fy",cfg.fy,.01);
    check("Camera1.cx",cfg.cx,.01);check("Camera1.cy",cfg.cy,.01);
    check("Stereo.b",cfg.baseline,1e-5);
    check("Camera.width",cfg.imgWidth,0);check("Camera.height",cfg.imgHeight,0);
    if(std::string(settings["Camera.type"])!="Rectified")
        throw std::runtime_error("Rectified stereo images require Camera.type=Rectified");
    if(inertial) {
        cv::FileStorage imuSettings(imuConfig,cv::FileStorage::READ);
        if(!imuSettings.isOpened() || (!imuBench && int(imuSettings["calibration_verified"])!=1))
            throw std::runtime_error("PX4 IMU disabled: camera/body extrinsics and temporal calibration are NOT verified");
        if(int(imuSettings["calibration_verified"])==1) {
            if(imuSettings["capture_fps"].empty() ||
               std::abs(double(imuSettings["capture_fps"])-cfg.fps)>.01 ||
               std::string(imuSettings["capture_format"])!="MJPG" ||
               int(imuSettings["capture_buffer_count"])!=1)
                throw std::runtime_error("Calibrated timing requires matching MJPG/fps/single-buffer capture profile");
        }
        imageTimeOffset=double(imuSettings["image_time_offset_seconds"]);
        if(!std::isfinite(imageTimeOffset) || std::abs(imageTimeOffset)>.5)
            throw std::runtime_error("Invalid image time offset");
        cv::Mat transform;imuSettings["T_b_c1"]>>transform;
        cv::Mat active;settings["IMU.T_b_c1"]>>active;
        if(transform.rows!=4 || transform.cols!=4 || active.rows!=4 || active.cols!=4)
            throw std::runtime_error("IMU config must include verified T_b_c1 matching the SLAM settings");
        transform.convertTo(transform,CV_64F);active.convertTo(active,CV_64F);
        if(cv::norm(transform,active,cv::NORM_INF)>1e-6 || !cv::checkRange(transform))
            throw std::runtime_error("Camera/body transforms differ");
        cv::Mat rotation=transform(cv::Rect(0,0,3,3));
        const cv::Mat bottom=(cv::Mat_<double>(1,4)<<0,0,0,1);
        if(cv::norm(rotation.t()*rotation-cv::Mat::eye(3,3,CV_64F))>1e-5 ||
           std::abs(cv::determinant(rotation)-1.)>1e-5 ||
           cv::norm(transform.row(3)-bottom)>1e-6)
            throw std::runtime_error("Invalid rigid camera/body transform");
    }
    if(checkConfig) {cout<<"[Calibration] rectified camera/settings verified"<<endl;return 0;}
#ifdef ORB_SLAM3_WITH_PX4
    std::unique_ptr<ORB_SLAM3::px4::Bridge> px4Bridge;
    std::unique_ptr<ORB_SLAM3::px4::ImuReceiver> imu;
    auto bridgeConfig=px4Config.empty()?ORB_SLAM3::px4::Config{}:ORB_SLAM3::px4::Config::Load(px4Config);
    if(inertial) {
        if(px4Config.empty()) throw std::runtime_error("--imu-config requires --px4-config");
        if((bridgeConfig.bodyFromImu-Eigen::Matrix3d::Identity()).norm()>1e-6)
            throw std::runtime_error("PX4 SensorCombined is body FRD: R_BI must be identity, not D435i extrinsics");
        if(imuBench) {
            bridgeConfig.extrinsicsValid=false;
            cout<<"[IMU] BENCH ONLY: provisional calibration; navigation publication INHIBITED"<<endl;
        }
        imu=std::make_unique<ORB_SLAM3::px4::ImuReceiver>(bridgeConfig.domainId,bridgeConfig.topicPrefix);
    } else {
        // Pure stereo has no inertial readiness; stale D435i extrinsics must not authorize it.
        bridgeConfig.extrinsicsValid=false;
    }
    if(!px4Config.empty()) px4Bridge=std::make_unique<ORB_SLAM3::px4::Bridge>(bridgeConfig);
#else
    if(inertial || !px4Config.empty()) throw std::runtime_error("Rebuild with ORB_SLAM3_WITH_PX4=ON");
#endif

    int captureBuffers=1;
    if(const char* requested=std::getenv("VIO_CAPTURE_BUFFERS")) {
        const std::string value(requested);
        if(value!="1" && value!="2" && value!="3" && value!="4")
            throw std::runtime_error("VIO_CAPTURE_BUFFERS must be 1, 2, 3 or 4");
        captureBuffers=value[0]-'0';
    }
    if(inertial && captureBuffers!=1) {
        cout<<"[Camera] retaining one buffer for calibrated IMU timing"<<endl;
        captureBuffers=1;
    }
    cv::VideoCapture cap(cfg.device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "打不开相机 " << cfg.device << endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.frameWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.frameHeight);
    cap.set(cv::CAP_PROP_FPS, cfg.fps);
    if(!cap.set(cv::CAP_PROP_BUFFERSIZE,captureBuffers))
        throw std::runtime_error("Cannot configure V4L2 capture buffers");
    if(int(cap.get(cv::CAP_PROP_BUFFERSIZE))!=captureBuffers)
        throw std::runtime_error("V4L2 capture buffer count differs from requested profile");
    cout<<"[Camera] V4L2 buffers="<<captureBuffers<<endl;
    // 让后端吐原始 MJPEG 字节，本进程直接解成灰度：
    // 走默认路径会解成 BGR（多解两路色度）再由我们转灰度，白白多花一次时间。
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

    int startupSkipped=0;
    cv::Mat probeDecoded=ORB_SLAM3::UvcStartupGray(
        [&](cv::Mat& raw){cap>>raw;},[&]{return bool(interrupted);},
        cfg.frameWidth,cfg.frameHeight,startupSkipped);
    if(interrupted)return 0;
    if(probeDecoded.empty()) {
        cerr<<"[Camera] no valid startup image after "<<startupSkipped<<" attempts"<<endl;
        return 1;
    }
    if(startupSkipped)cout<<"[Camera] skipped invalid startup frames="<<startupSkipped<<endl;
    cout << "[Camera] DECXIN Camera " << cfg.device << " " << probeDecoded.cols << "x" << probeDecoded.rows
         << " @" << cfg.fps << "fps（原始 MJPEG -> 直接灰度解码）" << endl;

    // 图像录制：rolling 按配置启用环形缓冲，--record 为整段图像录制。
    // 两者都在独立线程写盘，不阻塞跟踪线程。
    const auto timeOrigin=chrono::steady_clock::now();
    const double wallOrigin=chrono::duration<double>(chrono::system_clock::now().time_since_epoch()).count();
    std::unique_ptr<ORB_SLAM3::RollingInputRecording> rolling;
    {
        cv::FileStorage fs(argv[2], cv::FileStorage::READ);
        if (!fs["Recording.rolling_seconds"].empty() && double(fs["Recording.rolling_seconds"]) > 0) {
            ORB_SLAM3::RollingInputRecording::Options opt;
            opt.seconds = double(fs["Recording.rolling_seconds"]);
            if (!fs["Recording.overview_seconds"].empty())
                opt.overviewSeconds = double(fs["Recording.overview_seconds"]);
            if (!fs["Recording.overview_period_seconds"].empty())
                opt.overviewPeriod = double(fs["Recording.overview_period_seconds"]);
            const char *eventRoot = getenv("VIO_EVENT_DIR");
            const char *home = getenv("HOME");
            const auto events = eventRoot ? std::filesystem::path(eventRoot)
                : (home ? std::filesystem::path(home) : std::filesystem::current_path()) / "vio_image_events";
            // 环形目录只能创建一次。上一轮若是干净退出（Close 会写 closed.txt），
            // 轮转掉重新开始；若是崩溃退出，保留现场当证据，这一轮就不录，
            // 不能为了"能录"把事故数据删了。
            const auto ring = std::filesystem::current_path() / "image_ring";
            bool canRoll = true;
            if (std::filesystem::exists(ring)) {
                if (std::filesystem::exists(ring / "closed.txt")) {
                    std::filesystem::remove_all(ring);
                    cout << "[RollingInput] 上一轮已干净退出，环形目录轮转" << endl;
                } else {
                    canRoll = false;
                    cerr << "[RollingInput] " << ring
                         << " 存在且无 closed.txt，疑似上次未正常退出，保留现场，本轮不录环形缓冲" << endl;
                }
            }
            if (!canRoll) {
                // 跳过创建
            } else
            try {
                rolling = std::make_unique<ORB_SLAM3::RollingInputRecording>(
                    ring, events, argv[2], opt);
                std::ofstream meta(std::filesystem::current_path() / "image_ring/clock_origin.txt");
                meta << std::setprecision(17)
                     << "sensor_time_origin_system_seconds=" << wallOrigin
                     << "\nimage_system_seconds=image_timestamp+sensor_time_origin_system_seconds\n";
                cout << "[RollingInput] 环形缓冲已启动：" << ring
                     << "  保留 " << opt.seconds << "s  事件目录 " << events << endl;
            } catch (const std::exception &e) {
                cerr << "[RollingInput] 不可用：" << e.what() << "；跟踪继续" << endl;
            }
        }
    }
    std::shared_ptr<ORB_SLAM3::InputRecording> recording;
    if (!recordDirectory.empty()) {
        recording = std::make_shared<ORB_SLAM3::InputRecording>(recordDirectory, argv[2]);
        cout << "[Record] 整段录制写入 " << recordDirectory << endl;
    }

    std::unique_ptr<ORB_SLAM3::AsyncNavigationRecording> navigationRecording;
    if(const char* path=std::getenv("VIO_NAV_LOG");path && *path) {
        navigationRecording=std::make_unique<ORB_SLAM3::AsyncNavigationRecording>(path,
            "kind,frame,sensor_time,system_us,tracking,valid,imu,ba2,epoch,px,py,pz,qw,qx,qy,qz,vx,vy,vz,gx,gy,gz,ax,ay,az,imu_count,imu_first,imu_last");
        cout<<"[NAV_RECORD] ready "<<path<<endl;
    }
    ORB_SLAM3::System SLAM(argv[1], argv[2], inertial?ORB_SLAM3::System::IMU_STEREO:ORB_SLAM3::System::STEREO, false);


    SLAM.SetInputRecording(recording);
    ORB_SLAM3::RecoveryConsole console(SLAM,"orbslam3_first_map",false);
    if(consoleEnabled) console.Help();
    cout<<"[Mode] "<<(inertial?"STEREO_INERTIAL / PX4 body FRD":"STEREO / navigation publication inhibited")
        <<" | software capture timestamps; exposure delay requires calibration"<<endl;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    struct Slot {
        cv::Mat left, right;
        double timestamp=0;
        bool has = false;
        bool stop = false;
    } slot;
    mutex mtx;
    condition_variable cv;

    double msGrab = 0, msRect = 0, msRead=0, msDecode=0;
    long nProduced = 0, nDropped = 0, nProcessed = 0;
    std::atomic<long> nBadFrames{0};

    const char* pipelineEnv=std::getenv("ORB_SLAM3_CAPTURE_PIPELINE");
    const bool capturePipeline=pipelineEnv && std::string(pipelineEnv)=="1" && !inertial;
    if(pipelineEnv && std::string(pipelineEnv)!="0" && std::string(pipelineEnv)!="1")
        throw std::runtime_error("ORB_SLAM3_CAPTURE_PIPELINE must be 0 or 1");
    if(pipelineEnv && std::string(pipelineEnv)=="1" && inertial)
        cout<<"[Camera] capture pipeline disabled for IMU: calibrated timing path retained"<<endl;
    cout<<"[Camera] capture pipeline="<<capturePipeline<<endl;
    ORB_SLAM3::LatestCameraInput cameraInput;
#ifdef ORB_SLAM3_WITH_CUDA_FAST
    ORB_SLAM3::CudaStereoRectifier gpuRectifier;
#endif
    int decodeFailures=0;
    auto processInput=[&](const ORB_SLAM3::TimedCameraInput& input) {
        const auto decodeStart=chrono::steady_clock::now();
        cv::Mat gray=ORB_SLAM3::UvcGray(input.image,"MJPG",cfg.frameWidth,cfg.frameHeight);
        const auto decoded=chrono::steady_clock::now();
        if(gray.empty() || gray.cols!=cfg.frameWidth || gray.rows!=cfg.frameHeight) {
            ++nBadFrames;
            return ++decodeFailures<=20;
        }
        decodeFailures=0;
        const double decodeMs=chrono::duration<double,milli>(decoded-decodeStart).count();
        const auto rectStart=chrono::steady_clock::now();
        cv::Mat leftR, rightR;
        bool gpuRemapped=false;
#ifdef ORB_SLAM3_WITH_CUDA_FAST
        gpuRemapped=gpuRectifier.Remap(gray,cfg.leftRoi,cfg.rightRoi,
            cfg.mapLx,cfg.mapLy,cfg.mapRx,cfg.mapRy,leftR,rightR);
#endif
        if(!gpuRemapped) {
            cv::remap(gray(cfg.leftRoi), leftR, cfg.mapLx, cfg.mapLy, cv::INTER_LINEAR);
            cv::remap(gray(cfg.rightRoi), rightR, cfg.mapRx, cfg.mapRy, cv::INTER_LINEAR);
        }
        auto t2 = chrono::steady_clock::now();

        {
            lock_guard<mutex> lk(mtx);
            if(slot.stop)return false;
            if (slot.has) nDropped++;   // 上一帧还没被取走，直接丢
            slot.timestamp=input.timestamp;
            slot.left = leftR;
            slot.right = rightR;
            slot.has = true;
            nProduced++;
            msGrab += input.grabMs+decodeMs;
            msRead += input.grabMs;msDecode += decodeMs;
            msRect += chrono::duration<double, milli>(t2 - rectStart).count();
        }
        cv.notify_one();
        return true;
    };
    thread processingWorker;
    thread producer([&] {
      try {
        int fails = 0;
        while (!interrupted) {
            {lock_guard<mutex> lk(mtx);if(slot.stop)break;}
            auto t0 = chrono::steady_clock::now();
            cv::Mat raw;
            cap >> raw;
            // Stamp acquisition, before JPEG decoding/remapping and the latest-frame mailbox.
            const double captured=chrono::duration<double>(chrono::steady_clock::now()-timeOrigin).count()+imageTimeOffset;
            const auto readDone=chrono::steady_clock::now();
            if(raw.empty()) {
                ++nBadFrames;
                if(++fails>20)break;
                continue;
            }
            fails=0;
            // Own the MJPEG bytes before the next V4L2 read reuses its buffer.
            ORB_SLAM3::TimedCameraInput input{capturePipeline?raw.clone():std::move(raw),captured,
                chrono::duration<double,milli>(readDone-t0).count()};
            if(capturePipeline) {
                if(!cameraInput.Put(std::move(input)))break;
            } else if(!processInput(input))break;
        }
      } catch(const std::exception& e) {
        cerr<<"[Camera] capture failed: "<<e.what()<<endl;
      }
        cameraInput.Stop();
        {lock_guard<mutex> lk(mtx);slot.stop=true;}
        cv.notify_one();
    });

    int lastState = -1;
    auto lastStats = chrono::steady_clock::now();
    long statFrames = 0;
    double msTrack = 0;

    struct CaptureJoin {
        std::thread& thread;std::thread& worker;std::mutex& mutex;Slot& slot;
        ORB_SLAM3::LatestCameraInput& input;
        ~CaptureJoin() {
            {lock_guard<std::mutex> lock(mutex);slot.stop=true;}
            input.Stop();
            if(thread.joinable())thread.join();
            if(worker.joinable())worker.join();
        }
    } captureJoin{producer,processingWorker,mtx,slot,cameraInput};
    if(capturePipeline)processingWorker=thread([&] {
        try {
            ORB_SLAM3::TimedCameraInput input;
            while(cameraInput.Take(input)) {
                {lock_guard<mutex> lk(mtx);if(slot.stop)break;}
                if(!processInput(input))break;
            }
        } catch(const std::exception& e) {
            cerr<<"[Camera] rectification failed: "<<e.what()<<endl;
        }
        cameraInput.Stop();
        {lock_guard<mutex> lk(mtx);slot.stop=true;}
        cv.notify_one();
    });
    double previousImage=0;
    uint64_t previousImuEpoch=0;
    int result=0;
    while (!interrupted && !SLAM.isShutDown()) {
        if(consoleEnabled && !console.Poll()) break;
        cv::Mat left, right;
        double tFrame = 0;
        {
            unique_lock<mutex> lk(mtx);
            if(!cv.wait_for(lk,chrono::milliseconds(100),[&]{return slot.has || slot.stop || interrupted;})) continue;
            if (slot.has) {
                left = slot.left;
                right = slot.right;
                slot.has = false;
                tFrame=slot.timestamp;
            } else if (slot.stop) {
                break;
            }
        }

#ifdef ORB_SLAM3_WITH_PX4
        uint64_t imuEpoch=0;
        if(imu) {
            std::vector<ORB_SLAM3::px4::ImuReading> batch;std::string reason;
            const double start=previousImage>0?previousImage:tFrame-.01;
            if(!imu->Interval(wallOrigin+start,wallOrigin+tFrame,batch,imuEpoch,reason)) {
                if(previousImage>0) {
                    cerr<<"[IMU] STOP "<<reason<<" "<<imu->Status()<<endl;
                    if(px4Bridge)px4Bridge->InvalidateClock(imuEpoch);
                    result=3;break;
                }
                continue;
            }
            if(previousImage>0 && imuEpoch!=previousImuEpoch) {
                cerr<<"[IMU] STOP discontinuity; fresh initialization required"<<endl;
                if(px4Bridge)px4Bridge->InvalidateClock(imuEpoch);
                result=3;break;
            }
            vImuMeas.clear();
            // ORB retains the previous end sample in its queue. Re-submitting the
            // shared start endpoint creates duplicate timestamps and a zero dt.
            for(size_t i=previousImage>0?1:0;i<batch.size();++i) {
                const auto& r=batch[i];
                vImuMeas.emplace_back(r.accel[0],r.accel[1],r.accel[2],
                    r.gyro[0],r.gyro[1],r.gyro[2],r.time-wallOrigin);
            }
            previousImuEpoch=imuEpoch;
        }
#endif
        previousImage=tFrame;
        if (rolling) rolling->Frame(left, right, tFrame, vImuMeas);

        auto tT0 = chrono::steady_clock::now();
        SLAM.TrackStereo(left, right, tFrame, vImuMeas, trajName);
        auto tT1 = chrono::steady_clock::now();
        if(navigationRecording) {
            const auto n=SLAM.GetNavigationState();
            const auto p=n.imuToWorld.translation();const auto q=n.imuToWorld.unit_quaternion();
            std::ostringstream row;row<<std::setprecision(12);
            row<<"frame,"<<nProcessed<<","<<tFrame<<","<<uint64_t(std::llround((wallOrigin+tFrame)*1e6))
                <<","<<n.trackingState<<","<<n.valid<<","<<n.inertialInitialized<<","<<n.inertialBA2<<","<<n.referenceEpoch
                <<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z()
                <<","<<n.velocityWorld.x()<<","<<n.velocityWorld.y()<<","<<n.velocityWorld.z()
                <<",,,,,,,"<<vImuMeas.size()<<","<<(vImuMeas.empty()?0:vImuMeas.front().t)
                <<","<<(vImuMeas.empty()?0:vImuMeas.back().t)<<"\n";
            for(const auto& point:vImuMeas) {
                row<<"imu,"<<nProcessed<<","<<point.t<<","<<uint64_t(std::llround((wallOrigin+point.t)*1e6))
                    <<",,,,,,,,,,,,,,,,"<<point.w.x()<<","<<point.w.y()<<","<<point.w.z()
                    <<","<<point.a.x()<<","<<point.a.y()<<","<<point.a.z()<<",,,\n";
            }
            if(!navigationRecording->Write(row.str()) || !navigationRecording->Error().empty()) {
                cerr<<"[NAV_RECORD] FAILED "<<navigationRecording->Error()<<endl;result=5;break;
            }
        }
#ifdef ORB_SLAM3_WITH_PX4
        if(px4Bridge) px4Bridge->Submit(SLAM.GetNavigationState(),
            uint64_t(std::llround((wallOrigin+tFrame)*1e6)),imuEpoch);
#endif
        if(consoleEnabled) console.PrintChanges();
        if(rolling && std::filesystem::exists("freeze_images")) {
            string reason;ifstream request("freeze_images");getline(request,reason);
            rolling->Trigger(reason.empty()?"manual_freeze":reason);
            std::filesystem::remove("freeze_images");
        }
        nProcessed++;
        statFrames++;
        msTrack += chrono::duration<double, milli>(tT1 - tT0).count();

        int state = SLAM.GetTrackingState();
        if (state != lastState) {
            // ORB-SLAM3 的 eTrackingState：-1 未就绪, 0 无图像, 1 未初始化,
            // 2 正常, 3 刚丢失, 4 丢失。双目首帧即可初始化，通常很快进 2。
            cout << "[SLAM] t=" << tFrame << "s 状态 -> " << state
                 << " (1 未初始化 2 正常 3 刚丢失 4 丢失)" << endl;
            if (rolling && (state == 3 || state == 4))
                rolling->Trigger(state == 4 ? "tracking_lost" : "tracking_recently_lost");
            lastState = state;
        }

        double sinceStats = chrono::duration<double>(tT1 - lastStats).count();
        if (sinceStats >= 5.0) {
            double fGrab, fRect, fRead, fDecode;
            long prod, drop;
            {
                lock_guard<mutex> lk(mtx);
                fGrab = msGrab / max(1L, nProduced);
                fRect = msRect / max(1L, nProduced);
                fRead = msRead / max(1L,nProduced);fDecode = msDecode / max(1L,nProduced);
                prod = nProduced;
                drop = nDropped;
                msGrab = msRect = msRead = msDecode = 0;
                nProduced = nDropped = 0;
            }
            printf("[STAT] %.1f fps | 取流 %.1f ms  校正 %.1f ms  SLAM %.1f ms"
                   " | 产 %ld 帧 丢 %ld 帧 坏帧 %ld\n",
                   statFrames / sinceStats, fGrab, fRect,
                   msTrack / max(1L, statFrames), prod, drop, nBadFrames.load());
            cout<<"[CAPTURE_STAGE] read_ms="<<fRead<<" decode_ms="<<fDecode<<endl;
            cout<<"[CAPTURE_QUEUE] overwritten="<<cameraInput.Dropped()<<" waiting_limit=1"<<endl;
            const auto nav=SLAM.GetNavigationState();
            cout<<"[NAV] mode="<<(inertial?"vio":"stereo")<<" valid="<<nav.valid
                <<" imu="<<nav.inertialInitialized<<" ba2="<<nav.inertialBA2
                <<" epoch="<<nav.referenceEpoch
                <<" frame_age_ms="<<chrono::duration<double,milli>(tT1-timeOrigin).count()-tFrame*1000<<endl;
#ifdef ORB_SLAM3_WITH_PX4
            if(imu) cout<<"[IMU] "<<imu->Status()<<endl;
#endif
            cout.flush();
            msTrack = 0;
            statFrames = 0;
            lastStats = tT1;
        }
    }

    {lock_guard<mutex> lk(mtx);slot.stop=true;}
    cameraInput.Stop();
    producer.join();
    if(processingWorker.joinable())processingWorker.join();
    cap.release();
    if (recording && !recording->Close()) {
        cerr << "[Record] 关闭失败：录制可能不完整" << endl;
        result=4;
    }
    if (rolling) rolling->Close();
    SLAM.Shutdown();
    if(navigationRecording && !navigationRecording->Close())result=5;
    // 默认不落轨迹文件：daemon 起来时工作目录是仓库根，每次启动都写一份会污染。
    // 需要时设 VIO_SAVE_TRAJ=1。
    if (const char *save = getenv("VIO_SAVE_TRAJ"); save && string(save) == "1")
        SLAM.SaveTrajectoryEuRoC(trajName + "_trajectory.txt");
    cout << "[SLAM] 结束，共处理 " << nProcessed << " 帧" << endl;
    return result;
}

int main(int argc,char** argv) {
    try {return run(argc,argv);}
    catch(const std::exception& e) {cerr<<"[UVC] ERROR "<<e.what()<<endl;return 2;}
}
