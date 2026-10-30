#ifndef ORB_SLAM3_IMU_SETTINGS_H
#define ORB_SLAM3_IMU_SETTINGS_H

#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace ORB_SLAM3
{
// Shared by the File.version=1.0 and legacy configuration paths. Values are
// validated before vocabulary loading or starting SLAM worker threads.
struct ImuSettings
{
    bool fastInit = false;
    bool insertKFsWhenLost = true;
    float viba1Time = 5.f;
    float viba2Time = 15.f;
    float frequency = 0;
    float noiseGyro = 0, noiseAcc = 0, gyroWalk = 0, accWalk = 0;
    cv::Mat Tbc;

    static float Number(const cv::FileNode& node, const std::string& name)
    {
        if(node.empty() || (!node.isInt() && !node.isReal()))
            throw std::invalid_argument(name+" must be a number");
        const double value=node.real();
        if(!std::isfinite(value) || std::abs(value)>std::numeric_limits<float>::max())
            throw std::invalid_argument(name+" must be finite");
        return static_cast<float>(value);
    }

    static bool Flag(const cv::FileNode& node, const std::string& name, bool fallback)
    {
        if(node.empty()) return fallback;
        const float value=Number(node,name);
        if(value!=0 && value!=1) throw std::invalid_argument(name+" must be 0 or 1");
        return value==1;
    }

    static ImuSettings Read(cv::FileStorage& file)
    {
        ImuSettings result;
        result.fastInit=Flag(file["IMU.fastInit"],"IMU.fastInit",false);
        const auto insert=file["IMU.InsertKFsWhenLost"], legacyInsert=file["InsertKFsWhenLost"];
        result.insertKFsWhenLost=Flag(insert.empty()?legacyInsert:insert,"IMU.InsertKFsWhenLost",true);
        if(!insert.empty() && !legacyInsert.empty() &&
           result.insertKFsWhenLost!=Flag(legacyInsert,"InsertKFsWhenLost",true))
            throw std::invalid_argument("IMU.InsertKFsWhenLost conflicts with InsertKFsWhenLost");
        if(!file["IMU.VIBA1Time"].empty()) result.viba1Time=Number(file["IMU.VIBA1Time"],"IMU.VIBA1Time");
        if(!file["IMU.VIBA2Time"].empty()) result.viba2Time=Number(file["IMU.VIBA2Time"],"IMU.VIBA2Time");
        if(!(result.viba1Time>0 && result.viba2Time>result.viba1Time))
            throw std::invalid_argument("Require 0 < IMU.VIBA1Time < IMU.VIBA2Time (seconds)");
        result.frequency=Number(file["IMU.Frequency"],"IMU.Frequency");
        result.noiseGyro=Number(file["IMU.NoiseGyro"],"IMU.NoiseGyro");
        result.noiseAcc=Number(file["IMU.NoiseAcc"],"IMU.NoiseAcc");
        result.gyroWalk=Number(file["IMU.GyroWalk"],"IMU.GyroWalk");
        result.accWalk=Number(file["IMU.AccWalk"],"IMU.AccWalk");
        if(result.frequency<=0 || result.noiseGyro<=0 || result.noiseAcc<=0 ||
           result.gyroWalk<0 || result.accWalk<0)
            throw std::invalid_argument("IMU frequency/noise must be positive; bias walks must be nonnegative");

        auto transform=file["IMU.T_b_c1"];
        if(transform.empty()) transform=file["Tbc"];
        if(transform.empty()) throw std::invalid_argument("Missing IMU.T_b_c1 (legacy: Tbc)");
        cv::Mat matrix=transform.mat();
        if(matrix.rows!=4 || matrix.cols!=4 || matrix.channels()!=1 ||
           (matrix.depth()!=CV_32F && matrix.depth()!=CV_64F) || !cv::checkRange(matrix))
            throw std::invalid_argument("IMU.T_b_c1 must be a finite 4x4 float/double matrix");
        matrix.convertTo(result.Tbc,CV_32F);
        if(!cv::checkRange(result.Tbc)) throw std::invalid_argument("IMU.T_b_c1 exceeds float range");
        const cv::Mat rotation=result.Tbc(cv::Rect(0,0,3,3));
        const cv::Mat bottom=(cv::Mat_<float>(1,4)<<0,0,0,1);
        if(cv::norm(rotation.t()*rotation-cv::Mat::eye(3,3,CV_32F))>1e-3 ||
           std::abs(cv::determinant(rotation)-1)>1e-3 || cv::norm(result.Tbc.row(3)-bottom)>1e-5)
            throw std::invalid_argument("IMU.T_b_c1 must contain a rigid rotation and homogeneous bottom row");
        if(!file["IMU.T_b_c1"].empty() && !file["Tbc"].empty()) {
            cv::Mat legacy=file["Tbc"].mat(), asFloat;
            if(legacy.size()!=matrix.size() || legacy.channels()!=1 || !cv::checkRange(legacy))
                throw std::invalid_argument("IMU.T_b_c1 conflicts with Tbc");
            legacy.convertTo(asFloat,CV_32F);
            if(cv::norm(asFloat-result.Tbc)>1e-6) throw std::invalid_argument("IMU.T_b_c1 conflicts with Tbc");
        }
        return result;
    }
};
}
#endif
