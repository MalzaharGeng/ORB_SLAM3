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


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <iomanip>
#include <sstream>
#include <fstream>

namespace ORB_SLAM3
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStop("menu.Stop",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);
    pangolin::Var<bool> menuFirstInit("menu.First Init Done",false,false);
    pangolin::Var<bool> menuStartSecond("menu.Start Second Map",false,false);
    pangolin::Var<bool> menuSecondInit("menu.Second Init Done",false,false);
    pangolin::Var<bool> menuReadFirst("menu.Read First Map",false,false);
    pangolin::Var<std::string> menuRecoveryStatus("menu.Recovery", "First initialization");
    pangolin::Var<std::string> menuSaveStatus("menu.Save Status", "Not saved");
    pangolin::Var<std::string> menuPose("menu.Current Pose", "tracking unavailable");
    pangolin::Var<std::string> menuImuAttitude("menu.IMU Quaternion", "tracking unavailable");
    pangolin::Var<std::string> menuWorld("menu.World Frame", "Initial IMU axes; origin at initial IMU");
    pangolin::Var<std::string> menuImuLoad("menu.IMU at Load", "N/A: waiting for alignment");
    pangolin::Var<std::string> menuLoadOffset("menu.Camera at Load", "N/A: waiting for alignment");
    pangolin::Var<std::string> menuOriginOffset("menu.Second Origin", "N/A: waiting for alignment");
    pangolin::Var<std::string> menuConfidence("menu.Load Confidence", "N/A: waiting for alignment");

    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    cv::namedWindow("ORB-SLAM3: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    float trackedImageScale = mpTracker->GetImageScale();

    cout << "Starting the Viewer" << endl;
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);

        if(mbStopTrack)
        {
            menuStepByStep = true;
            mbStopTrack = false;
        }

        if(menuFollowCamera && bFollow)
        {
            if(bCameraView)
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if(bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
        }

        if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
            s_cam.Follow(Ow);
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(menuStepByStep && !bStepByStep)
        {
            //cout << "Viewer: step by step" << endl;
            mpTracker->SetStepByStep(true);
            bStepByStep = true;
        }
        else if(!menuStepByStep && bStepByStep)
        {
            mpTracker->SetStepByStep(false);
            bStepByStep = false;
        }

        if(menuStep)
        {
            mpTracker->mbStep = true;
            menuStep = false;
        }


        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph, menuShowOptLba);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();
        if(menuShowPoints) mpMapDrawer->DrawRecoveryReference();
        // Navigation world axes: red X, green Y, blue Z (initial IMU axes).
        glLineWidth(2.f);
        glBegin(GL_LINES);
        glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(0.2f,0,0);
        glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,0.2f,0);
        glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,0.2f);
        glEnd();

        pangolin::FinishFrame();

        cv::Mat toShow;
        cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }

        const auto visibleRecovery=mpSystem->GetRecoveryStatus();
        const bool aligned=visibleRecovery.stage==System::RecoveryStage::ALIGNED && visibleRecovery.report.valid;
        const auto navigation=mpSystem->GetNavigationState();
        const bool inertialNavigation=mpTracker->mSensor==mpSystem->IMU_STEREO;
        if(navigation.valid)
        {
            const Eigen::Vector3f p = navigation.imuToWorld.translation();
            const float distance = p.norm();
            std::ostringstream poseText;
            poseText << std::fixed << std::setprecision(3)
                     << (inertialNavigation ? "IMU XYZ: " : "Camera XYZ: ") << p.x() << ", " << p.y() << ", " << p.z()
                     << "  Distance: " << distance << " m";
            menuPose = poseText.str();
            menuWorld = inertialNavigation ? (navigation.inFirstMap ? "FIRST initial IMU" : "Current map initial IMU") : "Current map";
            const auto q=navigation.imuToWorld.unit_quaternion();
            std::ostringstream attitude;
            attitude<<std::fixed<<std::setprecision(3)<<(inertialNavigation ? "IMU q(wxyz): " : "Camera q(wxyz): ")<<q.w()<<", "<<q.x()<<", "<<q.y()<<", "<<q.z();
            menuImuAttitude=attitude.str();
            cv::putText(toShow, poseText.str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                        0.55, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::putText(toShow,attitude.str(),cv::Point(10,47),cv::FONT_HERSHEY_SIMPLEX,
                        0.45,cv::Scalar(0,255,0),1,cv::LINE_AA);
        }
        else
        {
            menuPose = "tracking unavailable";
            menuImuAttitude = "tracking unavailable";
        }

        if(mImageViewerScale != 1.f)
        {
            int width = toShow.cols * mImageViewerScale;
            int height = toShow.rows * mImageViewerScale;
            cv::resize(toShow, toShow, cv::Size(width, height));
        }

        cv::putText(toShow,visibleRecovery.message,cv::Point(10,69),cv::FONT_HERSHEY_SIMPLEX,
                    0.45,cv::Scalar(0,220,255),1,cv::LINE_AA);
        if(aligned)
        {
            const auto& report=visibleRecovery.report;
            auto offsetText=[](const char* label,const Eigen::Vector3f& offset,float distance) {
                std::ostringstream out;
                out<<std::fixed<<std::setprecision(3)<<label<<offset.x()<<", "<<offset.y()<<", "<<offset.z()
                   <<" m; d="<<distance<<" m";
                return out.str();
            };
            const string imu=offsetText("IMU at load in FIRST IMU0: ",report.imuInFirst,report.imuDistance);
            const string camera=offsetText("Camera at load in FIRST IMU0: ",report.cameraInFirst,report.cameraDistance);
            const string origin=offsetText("Second IMU0 in FIRST IMU0: ",report.secondOriginInFirst,report.originDistance);
            std::ostringstream quality,details;
            quality<<std::fixed<<std::setprecision(1)<<"Load confidence: "<<report.confidence
                   <<"/100 (heuristic, at alignment)";
            details<<std::fixed<<std::setprecision(3)<<"PnP "<<report.match.inliers<<"; pairs "
                   <<report.match.pairs<<"/"<<report.match.candidatePairs<<"; RMSE "<<report.match.rmse
                   <<" m; delta "<<report.translationDisagreement<<" m / "
                   <<std::setprecision(2)<<report.rotationDisagreement*180.f/3.14159265f<<" deg";
            menuImuLoad=imu;menuLoadOffset=camera;menuOriginOffset=origin;menuConfidence=quality.str();
            const std::vector<string> lines={imu,camera,origin,quality.str(),details.str()};
            for(size_t row=0;row<lines.size();++row)
                cv::putText(toShow,lines[row],cv::Point(10,91+22*row),cv::FONT_HERSHEY_SIMPLEX,
                            0.43,cv::Scalar(0,220,255),1,cv::LINE_AA);
        }
        else
        {
            menuLoadOffset="N/A: waiting for alignment";
            menuImuLoad="N/A: waiting for alignment";
            menuOriginOffset="N/A: waiting for alignment";
            menuConfidence="N/A: waiting for alignment";
        }
        cv::imshow("ORB-SLAM3: Current Frame",toShow);
        cv::waitKey(mT);

        const auto recovery=mpSystem->GetRecoveryStatus();
        menuRecoveryStatus=recovery.message;
        const auto save=mpSystem->GetAtlasSaveStatus();
        menuSaveStatus=save.message;
        if(menuFirstInit)
        {
            menuFirstInit=false;
            if(recovery.stage==System::RecoveryStage::FIRST_MAPPING)
                mpSystem->RequestAtlasSave("orbslam3_first_map");
        }
        if(menuStartSecond)
        {
            menuStartSecond=false;
            if(!mpSystem->StartNewMap()) std::cout<<"[Recovery] Reset requires a successfully saved first map"<<std::endl;
        }
        if(menuSecondInit)
        {
            menuSecondInit=false;
            if(!mpSystem->ConfirmSecondInitialization()) std::cout<<"[Recovery] Second confirmation is not available at this stage"<<std::endl;
        }
        if(menuReadFirst)
        {
            menuReadFirst=false;
            if(!mpSystem->RequestLoadFirstMap()) std::cout<<"[Recovery] Read requires confirmed second initialization"<<std::endl;
        }

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->ResetActiveMap();
            menuReset = false;
        }

        if(menuStop)
        {
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();

            // Stop all threads
            mpSystem->Shutdown();

            // Save camera trajectory
            mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
            mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
            menuStop = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
