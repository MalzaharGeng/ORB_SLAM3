#include "System.h"
#include "Settings.h"
#include "CameraModels/GeometricCamera.h"
#include <opencv2/core.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv) {
    if(argc!=2)return 2;
    try {
        ORB_SLAM3::Settings settings(argv[1],ORB_SLAM3::System::STEREO);
        cv::FileStorage fs(argv[1],cv::FileStorage::READ);
        const double originalFx=double(fs["Camera1.fx"]),originalFy=double(fs["Camera1.fy"]);
        const double width=double(fs["Camera.width"]),height=double(fs["Camera.height"]);
        const double sx=fs["Camera.newWidth"].empty()?1.:double(fs["Camera.newWidth"])/width;
        const double sy=fs["Camera.newHeight"].empty()?1.:double(fs["Camera.newHeight"])/height;
        const double fx=settings.camera1()->getParameter(0),fy=settings.camera1()->getParameter(1);
        if(std::abs(fx-originalFx*sx)>.001 || std::abs(fy-originalFy*sy)>.001)
            throw std::runtime_error("scaled focal length mismatch");
        if(std::abs(settings.bf()-fx*settings.b())>1e-5)
            throw std::runtime_error("resized stereo bf differs from fx*baseline");
        const double depth=settings.bf()/(20.*sx);
        const double reference=originalFx*double(fs["Stereo.b"])/20.;
        if(std::abs(depth-reference)>1e-5)throw std::runtime_error("resize changed physical depth scale");
        std::cout<<"RESIZE_GEOMETRY_PASS sx="<<sx<<" fx="<<fx<<" bf="<<settings.bf()<<" depth="<<depth<<std::endl;
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
