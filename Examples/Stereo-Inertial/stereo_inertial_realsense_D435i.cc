/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>
#include "InputRecording.h"
#include "RollingInputRecording.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include "Examples/Utils/RecoveryConsole.h"
#ifdef ORB_SLAM3_WITH_PX4
#include "Px4Bridge.h"
#endif

using namespace std;

volatile sig_atomic_t b_continue_session = 1;

void exit_loop_handler(int s){
    b_continue_session = false;

}

void interpolateData(const std::vector<double> &vBase_times,
                     std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                     const rs2_vector &prev_data, const double &prev_time);

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int run(int argc, char **argv) {

    auto usage=[](){
        cout<<"Usage: stereo_inertial_realsense_D435i vocabulary settings [trajectory_name]"
              " [--headless|--gui] [--console|--no-console] [--px4-config FILE] [--record DIR]\n"
              "--headless: no windows, terminal stage commands enabled\n";
    };
    if(argc==2 && string(argv[1])=="--help") { usage(); return 0; }
    if (argc < 3) {
        cerr << endl
             << "Usage: ./stereo_inertial_realsense_D435i path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name, px4Config, recordDirectory;
    bool useViewer=true, consoleEnabled=true;
    for(int i=3;i<argc;++i) {
        const string option=argv[i];
        if(option=="--headless" || option=="--no-gui") useViewer=false;
        else if(option=="--gui") useViewer=true;
        else if(option=="--console") consoleEnabled=true;
        else if(option=="--no-console") consoleEnabled=false;
        else if(option=="--px4-config" && i+1<argc) px4Config=argv[++i];
        else if(option=="--record" && i+1<argc) recordDirectory=argv[++i];
        else if(option=="--help") { usage();return 0; }
        else if(option.rfind("--",0)==0 || !file_name.empty()) {
            cerr<<"Unknown argument: "<<option<<endl;usage();return 2;
        } else file_name=option;
    }
    if(!useViewer && !consoleEnabled) { cerr<<"Headless live mode requires --console"<<endl;return 2; }
#ifdef ORB_SLAM3_WITH_PX4
    std::unique_ptr<ORB_SLAM3::px4::Bridge> px4Bridge;
    if(!px4Config.empty()) {
        const auto config=ORB_SLAM3::px4::Config::Load(px4Config);
        cv::FileStorage policy(argv[2],cv::FileStorage::READ);
        if(config.extrinsicsValid && (policy["loopClosing"].empty() || int(policy["loopClosing"])!=0))
            throw std::runtime_error("Hardware EV publication requires loopClosing: 0; use receive-only PX4 config for mapping");
        px4Bridge=std::make_unique<ORB_SLAM3::px4::Bridge>(config);
    }
    ORB_SLAM3::px4::SensorClock sensorClock;
    const double sensorTimeOrigin=px4Bridge ? ORB_SLAM3::px4::SystemMicros()*1e-6-1. : 0.;
#else
    if(!px4Config.empty()) {cerr<<"Rebuild with -DORB_SLAM3_WITH_PX4=ON to use --px4-config"<<endl;return 2;}
    const double sensorTimeOrigin=0.;
#endif
    cout<<"[Run] input=D435i live; GUI="<<(useViewer ? "on" : "off")
        <<"; terminal="<<(consoleEnabled ? "on" : "off")<<endl;
    std::unique_ptr<ORB_SLAM3::RollingInputRecording> rolling;
    {
        cv::FileStorage settings(argv[2],cv::FileStorage::READ);
        if(!settings["Recording.rolling_seconds"].empty() && double(settings["Recording.rolling_seconds"])>0) {
            ORB_SLAM3::RollingInputRecording::Options options;
            options.seconds=double(settings["Recording.rolling_seconds"]);
            if(!settings["Recording.overview_seconds"].empty())options.overviewSeconds=double(settings["Recording.overview_seconds"]);
            if(!settings["Recording.overview_period_seconds"].empty())options.overviewPeriod=double(settings["Recording.overview_period_seconds"]);
            const char* eventRoot=std::getenv("VIO_EVENT_DIR");
            const char* userHome=std::getenv("HOME");
            const auto events=eventRoot?std::filesystem::path(eventRoot):
                (userHome?std::filesystem::path(userHome):std::filesystem::current_path())/"vio_image_events";
            try {
                rolling=std::make_unique<ORB_SLAM3::RollingInputRecording>(std::filesystem::current_path()/"image_ring",events,argv[2],options);
                std::ofstream clockMetadata(std::filesystem::current_path()/"image_ring/clock_origin.txt");
                clockMetadata<<std::setprecision(17)<<"sensor_time_origin_system_seconds="<<sensorTimeOrigin
                    <<"\nimage_system_seconds=image_timestamp+sensor_time_origin_system_seconds\n";
            } catch(const std::exception& e) {
                std::cerr<<"[RollingInput] unavailable: "<<e.what()<<"; telemetry and VIO continue"<<std::endl;
            }
        }
    }
    bool rollingWasReady=false;uint64_t rollingReference=0;double performanceReport=0;
    std::string lastRollingFault;
    std::shared_ptr<ORB_SLAM3::InputRecording> recording;
    if(!recordDirectory.empty())recording=std::make_shared<ORB_SLAM3::InputRecording>(recordDirectory,argv[2]);

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 1;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
#ifdef ORB_SLAM3_WITH_PX4
    if(px4Bridge) for(auto sensor:sensors) {
        bool selected=false;
        for(const auto& profile:sensor.get_stream_profiles())
            selected=selected || profile.stream_type()==RS2_STREAM_INFRARED ||
                profile.stream_type()==RS2_STREAM_GYRO || profile.stream_type()==RS2_STREAM_ACCEL;
        if(selected) {
            if(!sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                throw std::runtime_error("PX4 requires RealSense global time for stereo and motion sensors");
            sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,1.f);
        }
    }
#endif
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            // D435i 的 UVC 控制通道偶发超时(EAGAIN)。任何 set_option/get_option 抛出的
            // rs2::error 若未被捕获，都会让整个进程直接退出。故逐个 try/catch，
            // 失败只告警，不影响 VIO 主流程。
            if (index == 1) {
                try {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                    sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,5000);
                    sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); // switch off emitter
                } catch (const rs2::error &e) {
                    std::cerr << "[D435i] WARN(set#1): " << e.what() << std::endl;
                }
            }
            // [已禁用] get_sensor_option(sensor)：纯诊断打印，遍历全部 RS2_OPTION 逐个读值，
            // 返回值恒为 0 且无人使用。它在启动阶段连做几十次 UVC 控制查询，
            // 任一次超时即抛 rs2::error -> 进程退出（日志尾部为 get_xu failed! EAGAIN，
            // 表现为启动 8 秒后 rc=1）。需要诊断时把下一行调用恢复即可。
            // get_sensor_option(sensor);
            if (index == 2){
                // RGB camera (not used here...)
                try { sensor.set_option(RS2_OPTION_EXPOSURE,100.f); }
                catch (const rs2::error &e) { std::cerr << "[D435i] WARN(set#2): " << e.what() << std::endl; }
            }

            if (index == 3){
                try { sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0); }
                catch (const rs2::error &e) { std::cerr << "[D435i] WARN(set#3): " << e.what() << std::endl; }
            }

        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Camera.fps must control physical acquisition, not just the SLAM model.
    cv::FileStorage captureSettings(argv[2],cv::FileStorage::READ);
    if(!captureSettings.isOpened())throw std::runtime_error("Cannot read capture settings");
    const double requestedFps=double(captureSettings["Camera.fps"]);
    if(!std::isfinite(requestedFps) || requestedFps<1 || requestedFps>90 || requestedFps!=std::floor(requestedFps))
        throw std::runtime_error("Camera.fps must be an integer in [1,90]");
    const int captureFps=static_cast<int>(requestedFps);
    std::cout<<"[D435i] requested stereo capture="<<captureFps<<" Hz"<<std::endl;
    cfg.enable_device(selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, captureFps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, captureFps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data{};
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data{};
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;

    cv::Mat imCV, imRightCV;
    int width_img=640, height_img=480;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames
    uint64_t captureStereo=0,captureGyro=0,captureAccel=0,captureClockRejected=0;
    double captureLastAgeMs=0;
    int captureLastDomain=-1;
    auto captureReport=std::chrono::steady_clock::now();

    auto imu_callback = [&](const rs2::frame& frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);
        if(frame.as<rs2::frameset>())++captureStereo;
        else if(auto motion=frame.as<rs2::motion_frame>()) {
            if(motion.get_profile().stream_type()==RS2_STREAM_GYRO)++captureGyro;
            else if(motion.get_profile().stream_type()==RS2_STREAM_ACCEL)++captureAccel;
        }
#ifdef ORB_SLAM3_WITH_PX4
        if(px4Bridge) {
            const auto wall=ORB_SLAM3::px4::SystemMicros(),mono=ORB_SLAM3::px4::SteadyMicros();
            bool accepted=true;
            auto accept=[&](const rs2::frame& f,unsigned stream) {
                captureLastDomain=int(f.get_frame_timestamp_domain());
                captureLastAgeMs=double(wall)*1e-3-f.get_timestamp();
                return sensorClock.Accept(stream,f.get_frame_timestamp_domain()==RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME,
                                          f.get_timestamp(),wall,mono);
            };
            if(auto fs=frame.as<rs2::frameset>()) {
                if(std::abs(timestamp_image-(fs.get_infrared_frame(1).get_timestamp()*1e-3-sensorTimeOrigin))<.001)return;
                accepted=accept(fs.get_infrared_frame(1),0);
                accepted=accept(fs.get_infrared_frame(2),1) && accepted;
            } else if(auto motion=frame.as<rs2::motion_frame>()) {
                accepted=accept(motion,motion.get_profile().stream_type()==RS2_STREAM_GYRO?2:3);
            }
            if(!accepted || !sensorClock.Ready()) {
                ++captureClockRejected;
                v_gyro_data.clear();v_gyro_timestamp.clear();v_accel_data_sync.clear();v_accel_timestamp_sync.clear();
                prev_accel_timestamp=current_accel_timestamp=0;
                timestamp_image=-1;image_ready=false;count_im_buffer=0;
                px4Bridge->InvalidateClock(sensorClock.Epoch());
                return;
            }
        }
#endif

        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            count_im_buffer++;

            double new_timestamp_image = fs.get_infrared_frame(1).get_timestamp()*1e-3-sensorTimeOrigin;
            if(abs(timestamp_image-new_timestamp_image)<0.001){
                // cout << "Two frames with the same timeStamp!!!\n";
                count_im_buffer--;
                return;
            }

            rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
            rs2::video_frame ir_frameR = fs.get_infrared_frame(2);

            // Own the pixels after the SDK callback returns; dimensions must
            // come from this frame, since streaming starts before profile setup.
            imCV = cv::Mat(ir_frameL.get_height(),ir_frameL.get_width(),CV_8U,
                          (void*)ir_frameL.get_data(),ir_frameL.get_stride_in_bytes()).clone();
            imRightCV = cv::Mat(ir_frameR.get_height(),ir_frameR.get_width(),CV_8U,
                               (void*)ir_frameR.get_data(),ir_frameR.get_stride_in_bytes()).clone();

            timestamp_image = new_timestamp_image;
            image_ready = true;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
            {

                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                v_accel_data_sync.push_back(current_accel_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            lock.unlock();
            cond_image_rec.notify_all();
        }
        else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>())
        {
            if (m_frame.get_profile().stream_name() == "Gyro")
            {
                // It runs at 200Hz
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp()+offset)*1e-3-sensorTimeOrigin);
                //rs2_vector gyro_sample = m_frame.get_motion_data();
                //std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            }
            else if (m_frame.get_profile().stream_name() == "Accel")
            {
                // It runs at 60Hz
                prev_accel_timestamp = current_accel_timestamp;
                prev_accel_data = current_accel_data;

                current_accel_data = m_frame.get_motion_data();
                current_accel_timestamp = (m_frame.get_timestamp()+offset)*1e-3-sensorTimeOrigin;

                while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
                {
                    int index = v_accel_timestamp_sync.size();
                    double target_time = v_gyro_timestamp[index];

                    rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp,
                                                                prev_accel_data, prev_accel_timestamp);

                    v_accel_data_sync.push_back(interp_data);
                    v_accel_timestamp_sync.push_back(target_time);

                }
                // std::cout << "Accel:" << current_accel_data.x << ", " << current_accel_data.y << ", " << current_accel_data.z << std::endl;
            }
        }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);
    std::cout<<"[D435i] negotiated stereo capture="
             <<pipe_profile.get_stream(RS2_STREAM_INFRARED,1).fps()<<"/"
             <<pipe_profile.get_stream(RS2_STREAM_INFRARED,2).fps()<<" Hz"<<std::endl;
    struct StopPipeline {
        rs2::pipeline& pipeline;
        ~StopPipeline() {try {pipeline.stop();} catch(const rs2::error&) {}}
    } stopPipeline{pipe}; // Stop callbacks before their captured buffers are destroyed on exception.

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    rs2::stream_profile cam_left = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
    rs2::stream_profile cam_right = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);


    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
    const auto camera_to_imu=cam_left.get_extrinsics_to(imu_stream);
    const float* Rbc = camera_to_imu.rotation;
    const float* tbc = camera_to_imu.translation;
    std::cout << "Tbc (left) = " << std::endl;
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++)
            std::cout << Rbc[j*3 + i] << ", "; // SDK rotations are column-major.
        std::cout << tbc[i] << "\n";
    }

    const auto right_to_left=cam_right.get_extrinsics_to(cam_left);
    const float* Rlr = right_to_left.rotation;
    const float* tlr = right_to_left.translation;
    std::cout << "Tlr  = " << std::endl;
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++)
            std::cout << Rlr[j*3 + i] << ", ";
        std::cout << tlr[i] << "\n";
    }



    rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_left.width;
    height_img = intrinsics_left.height;
    cout << "Left camera: \n";
    std::cout << " fx = " << intrinsics_left.fx << std::endl;
    std::cout << " fy = " << intrinsics_left.fy << std::endl;
    std::cout << " cx = " << intrinsics_left.ppx << std::endl;
    std::cout << " cy = " << intrinsics_left.ppy << std::endl;
    std::cout << " height = " << intrinsics_left.height << std::endl;
    std::cout << " width = " << intrinsics_left.width << std::endl;
    std::cout << " Coeff = " << intrinsics_left.coeffs[0] << ", " << intrinsics_left.coeffs[1] << ", " <<
        intrinsics_left.coeffs[2] << ", " << intrinsics_left.coeffs[3] << ", " << intrinsics_left.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_left.model << std::endl;

    rs2_intrinsics intrinsics_right = cam_right.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_right.width;
    height_img = intrinsics_right.height;
    cout << "Right camera: \n";
    std::cout << " fx = " << intrinsics_right.fx << std::endl;
    std::cout << " fy = " << intrinsics_right.fy << std::endl;
    std::cout << " cx = " << intrinsics_right.ppx << std::endl;
    std::cout << " cy = " << intrinsics_right.ppy << std::endl;
    std::cout << " height = " << intrinsics_right.height << std::endl;
    std::cout << " width = " << intrinsics_right.width << std::endl;
    std::cout << " Coeff = " << intrinsics_right.coeffs[0] << ", " << intrinsics_right.coeffs[1] << ", " <<
        intrinsics_right.coeffs[2] << ", " << intrinsics_right.coeffs[3] << ", " << intrinsics_right.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_right.model << std::endl;


    // Audit the ACTIVE pipeline profile directly, not rs-enumerate-devices or
    // cached console digits. Refuse calibration/profile mismatches before SLAM.
    {
        std::vector<std::string> errors;
        auto match=[&](const std::string& name,double actual,double expected,double tolerance) {
            if(!std::isfinite(actual)||!std::isfinite(expected)||std::abs(actual-expected)>tolerance)
                errors.push_back(name);
        };
        const auto& left=intrinsics_left;const auto& right=intrinsics_right;
        for(const auto* k:{&left,&right}) {
            match("fx",k->fx,double(captureSettings["Camera1.fx"]),.002);
            match("fy",k->fy,double(captureSettings["Camera1.fy"]),.002);
            match("cx",k->ppx,double(captureSettings["Camera1.cx"]),.002);
            match("cy",k->ppy,double(captureSettings["Camera1.cy"]),.002);
            match("width",k->width,double(captureSettings["Camera.width"]),0);
            match("height",k->height,double(captureSettings["Camera.height"]),0);
            for(float c:k->coeffs)match("rectified_distortion",c,0,1e-6);
        }
        std::string cameraType;captureSettings["Camera.type"]>>cameraType;
        if(cameraType!="Rectified")errors.push_back("Camera.type");
        match("baseline",tlr[0],double(captureSettings["Stereo.b"]),1e-6);
        for(int i=1;i<3;++i)match("stereo_translation",tlr[i],0,1e-6);
        cv::Mat configuredT;captureSettings["IMU.T_b_c1"]>>configuredT;configuredT.convertTo(configuredT,CV_64F);
        if(configuredT.rows!=4||configuredT.cols!=4)errors.push_back("IMU.T_b_c1_shape");
        else for(int row=0;row<3;++row) {
            for(int col=0;col<3;++col)match("IMU_rotation",Rbc[col*3+row],configuredT.at<double>(row,col),1e-5);
            match("IMU_translation",tbc[row],configuredT.at<double>(row,3),1e-6);
        }
        for(int row=0;row<3;++row)for(int col=0;col<3;++col)
            match("rectified_stereo_rotation",Rlr[col*3+row],row==col?1:0,1e-5);
        cv::FileStorage audit("camera_calibration.json",cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
        if(!audit.isOpened())throw std::runtime_error("Cannot save active calibration audit");
        audit<<"source"<<"librealsense pipeline.start(cfg): active get_intrinsics/get_extrinsics_to"
             <<"serial"<<selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
             <<"firmware"<<selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
             <<"sdk_version"<<RS2_API_VERSION_STR<<"settings_path"<<argv[2]
             <<"camera_to_imu_direction"<<"p_imu = R * p_left + t"
             <<"rotation_storage"<<"row-major in this JSON; transposed indexing from SDK column-major"
             <<"capture_fps"<<captureFps<<"gyro_fps"<<imu_stream.fps()
             <<"accel_fps"<<pipe_profile.get_stream(RS2_STREAM_ACCEL).fps();
        auto writeIntrinsics=[&](const char* name,const rs2_intrinsics& k) {
            audit<<name<<"{"<<"fx"<<double(k.fx)<<"fy"<<double(k.fy)<<"cx"<<double(k.ppx)<<"cy"<<double(k.ppy)
                 <<"width"<<k.width<<"height"<<k.height<<"distortion_model"<<int(k.model)
                 <<"coefficients"<<std::vector<float>(k.coeffs,k.coeffs+5)<<"}";
        };
        writeIntrinsics("left",left);writeIntrinsics("right",right);
        std::vector<double> bc,lr;
        for(int row=0;row<3;++row)for(int col=0;col<3;++col) {bc.push_back(Rbc[col*3+row]);lr.push_back(Rlr[col*3+row]);}
        audit<<"camera_to_imu_R"<<bc<<"camera_to_imu_t_m"<<std::vector<float>(tbc,tbc+3)
             <<"right_to_left_R"<<lr<<"right_to_left_t_m"<<std::vector<float>(tlr,tlr+3)
             <<"validation_passed"<<int(errors.empty())<<"mismatches"<<errors;
        audit.release();
        if(rolling)std::filesystem::copy_file("camera_calibration.json","image_ring/camera_calibration.json");
        if(recording)std::filesystem::copy_file("camera_calibration.json",std::filesystem::path(recordDirectory)/"camera_calibration.json");
        if(!errors.empty())throw std::runtime_error("Active camera calibration differs from YAML: "+errors.front()+"; see camera_calibration.json");
        std::cout<<"[D435i] ACTIVE_CALIBRATION_VERIFIED snapshot=camera_calibration.json"<<std::endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, useViewer, 0, file_name);
    SLAM.SetInputRecording(recording);
    ORB_SLAM3::RecoveryConsole console(SLAM,"orbslam3_first_map",!useViewer);
    if(consoleEnabled) { console.Help();console.PrintChanges(); }
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, imRight;

    // Clear IMU vectors
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        v_gyro_data.clear();
        v_gyro_timestamp.clear();
        v_accel_data_sync.clear();
        v_accel_timestamp_sync.clear();
        image_ready=false;
    }

    double t_resize = 0.f;
    double t_track = 0.f;
#ifdef ORB_SLAM3_WITH_PX4
    uint64_t processedClockEpoch=0;
#endif

    while (b_continue_session && !SLAM.isShutDown())
    {
        if(consoleEnabled) {
            if(!console.Poll()) break;
            console.PrintChanges();
        }
        // A stalled sensor produces no TrackStereo call. Handle the watchdog's
        // freeze request here too, so the last good window survives recovery.
        if(rolling && std::filesystem::exists("freeze_images")) {
            std::string reason;std::ifstream request("freeze_images");std::getline(request,reason);
            rolling->Trigger(reason.empty()?"manual_freeze":reason);
            std::filesystem::remove("freeze_images");
        }
        std::vector<rs2_vector> vGyro;
        std::vector<double> vGyro_times;
        std::vector<rs2_vector> vAccel;
        std::vector<double> vAccel_times;
#ifdef ORB_SLAM3_WITH_PX4
        uint64_t frameClockEpoch=0;
#endif

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!cond_image_rec.wait_for(lk,std::chrono::milliseconds(50),[&](){return image_ready;})) {
                const auto now=std::chrono::steady_clock::now();
                if(now-captureReport>std::chrono::seconds(2)) {
                    std::cout<<"[D435i_CAPTURE] waiting stereo="<<captureStereo<<" gyro="<<captureGyro
                             <<" accel="<<captureAccel<<" clock_rejected="<<captureClockRejected
                             <<" last_domain="<<captureLastDomain<<" last_age_ms="<<captureLastAgeMs<<std::endl;
                    captureReport=now;
                }
                continue; // Poll stdin/signals even when the sensor stops sending frames.
            }
            if(!b_continue_session || SLAM.isShutDown()) break;

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point time_Start_Process = std::chrono::monotonic_clock::now();
#endif

            if(count_im_buffer>1)
                cout << count_im_buffer -1 << " dropped frs\n";
            count_im_buffer = 0;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
            {
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp, prev_accel_data, prev_accel_timestamp);

                v_accel_data_sync.push_back(interp_data);
                // v_accel_data_sync.push_back(current_accel_data); // 0 interpolation
                v_accel_timestamp_sync.push_back(target_time);
            }

            // Copy the IMU data
            vGyro = v_gyro_data;
            vGyro_times = v_gyro_timestamp;
            vAccel = v_accel_data_sync;
            vAccel_times = v_accel_timestamp_sync;
            timestamp = timestamp_image;
#ifdef ORB_SLAM3_WITH_PX4
            frameClockEpoch=sensorClock.Epoch();
#endif
            im = imCV.clone();
            imRight = imRightCV.clone();

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_accel_data_sync.clear();
            v_accel_timestamp_sync.clear();

            image_ready = false;
        }

#ifdef ORB_SLAM3_WITH_PX4
        if(px4Bridge && frameClockEpoch!=processedClockEpoch) {
            SLAM.ResetActiveMap();vImuMeas.clear();processedClockEpoch=frameClockEpoch;
        }
#endif

        for(int i=0; i<vGyro.size(); ++i)
        {
            ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                  vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                  vGyro_times[i]);
            vImuMeas.push_back(lastPoint);
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
        // Record the exact input batch; compression and eviction run asynchronously.
        if(rolling)rolling->Frame(im,imRight,timestamp,vImuMeas);
        const auto performanceStart=std::chrono::steady_clock::now();
        SLAM.TrackStereo(im, imRight, timestamp, vImuMeas);
        const auto rollingNav=SLAM.GetNavigationState();
        const bool currentReady=rollingNav.valid && rollingNav.inertialInitialized && rollingNav.inertialBA2;
        if(rolling) {
            if(rollingWasReady && !currentReady)rolling->Trigger("tracking_or_inertial_lost");
            if(rollingWasReady && rollingReference!=rollingNav.referenceEpoch)rolling->Trigger("map_reference_change");
            rollingWasReady=currentReady;rollingReference=rollingNav.referenceEpoch;
        }
        const double performanceNow=std::chrono::duration<double>(performanceStart.time_since_epoch()).count();
        if(performanceNow-performanceReport>=1.) {
            const double latency=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-performanceStart).count();
            std::cout<<"[VIO_PERF] track_ms="<<latency<<" features="<<SLAM.GetTrackedKeyPointsUn().size()
                     <<" tracking="<<SLAM.GetTrackingState()<<" map="<<rollingNav.mapId
                     <<" epoch="<<rollingNav.referenceEpoch<<" ring_dropped="<<(rolling?rolling->Dropped():0)<<std::endl;
            performanceReport=performanceNow;
        }
#ifdef ORB_SLAM3_WITH_PX4
        if(px4Bridge) {
            if(rolling) {
                const auto fault=px4Bridge->FaultReason();
                if(!fault.empty() && fault!=lastRollingFault)rolling->Trigger(fault);
                lastRollingFault=fault;
            }
            const auto nav=SLAM.GetNavigationState();
            std::lock_guard<std::mutex> lock(imu_mutex);
            if(frameClockEpoch==sensorClock.Epoch() && sensorClock.Ready())
                px4Bridge->Submit(nav,uint64_t(std::llround((timestamp+sensorTimeOrigin)*1e6)),frameClockEpoch);
        }
#endif
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif



        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();
    }
    pipe.stop();
    if(!SLAM.isShutDown()) SLAM.Shutdown();
    const bool recordingOk=!recording || recording->Close();
    if(rolling)rolling->Close();
    cout << "System shutdown!\n";
    return recordingOk?0:1;
}

int main(int argc,char** argv) {
    try {return run(argc,argv);}
    catch(const std::exception& e) {std::cerr<<"D435i startup/runtime error: "<<e.what()<<std::endl;return 1;}
}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time)
{

    // If there are not previous information, the current data is propagated
    if(prev_time == 0)
    {
        return current_data;
    }

    rs2_vector increment;
    rs2_vector value_interp;

    if(target_time > current_time) {
        value_interp = current_data;
    }
    else if(target_time > prev_time)
    {
        increment.x = current_data.x - prev_data.x;
        increment.y = current_data.y - prev_data.y;
        increment.z = current_data.z - prev_data.z;

        double factor = (target_time - prev_time) / (current_time - prev_time);

        value_interp.x = prev_data.x + increment.x * factor;
        value_interp.y = prev_data.y + increment.y * factor;
        value_interp.z = prev_data.z + increment.z * factor;

        // zero interpolation
        value_interp = current_data;
    }
    else {
        value_interp = prev_data;
    }

    return value_interp;
}
