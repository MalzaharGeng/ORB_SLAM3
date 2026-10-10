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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false),
    mTrackingState(Tracking::NO_IMAGES_YET), mbCurrentPoseValid(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;
    else if(mSensor==IMU_RGBD)
        cout << "RGB-D-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile = settings_->atlasSaveFile();

        cout << (*settings_) << endl;
    }
    else{
        settings_ = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if(!node.empty() && node.isString())
        {
            mStrLoadAtlasFromFile = (string)node;
        }

        node = fsSettings["System.SaveAtlasToFile"];
        if(!node.empty() && node.isString())
        {
            mStrSaveAtlasToFile = (string)node;
        }
    }

    node = fsSettings["loopClosing"];
    bool activeLC = true;
    if(!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if(mStrLoadAtlasFromFile.empty())
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Atlas
        cout << "Initialization of Atlas from scratch " << endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        //clock_t start = clock();
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if(!isRead)
        {
            cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }
        //mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


        //cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file read in " << msElapsed << " ms" << endl;

        //usleep(10*1000*1000);
    }


    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if(settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //usleep(10*1000*1000);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    //if(false) // TODO
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if(settings_ && settings_->needToRectify()){
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if(settings_ && settings_->needToResize()){
        cv::resize(imLeft,imLeftToFeed,settings_->newImSize());
        cv::resize(imRight,imRightToFeed,settings_->newImSize());
    }
    else{
        imLeftToFeed = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    ProcessAtlasSaveRequest();
    ProcessRecoveryActions();

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed,imRightToFeed,timestamp,filename);

    // std::cout << "out grabber" << std::endl;

    UpdateRecoveryAlignment();
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mCurrentPose = mpMapDrawer->GetRecoveryAlignment() * Tcw.inverse();
    mbCurrentPoseValid = mpTracker->mState == Tracking::OK && mpTracker->mCurrentFrame.HasPose();
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=RGBD  && mSensor!=IMU_RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap,imDepthToFeed,settings_->newImSize());
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_RGBD)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed,imDepthToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }

    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    cout << "Shutdown" << endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    // RequestFinish is asynchronous. Serialization and trajectory export must
    // wait until no worker can mutate the maps (including an outstanding GBA).
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        usleep(1000);
    if(mptLocalMapping->joinable())
        mptLocalMapping->join();
    if(mptLoopClosing->joinable())
        mptLoopClosing->join();
    {
        lock_guard<mutex> lock(mMutexAtlasSave);
        if(mAtlasSaveStatus.state == AtlasSaveState::QUEUED)
        {
            mAtlasSaveStatus.state = AtlasSaveState::FAILED;
            mAtlasSaveStatus.message = "Save cancelled: system shut down before next stereo frame";
        }
    }

    if(!mStrSaveAtlasToFile.empty())
    {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
        SaveAtlas(FileType::BINARY_FILE, mStrSaveAtlasToFile);
    }

    if(mpViewer)
    {
        mpViewer->RequestFinish();
        // Viewer may call Shutdown from its own Stop button; never self-join.
        if(mptViewer->get_id()!=std::this_thread::get_id() && mptViewer->joinable())
        {
            mpViewer->Release();
            mptViewer->join();
        }
    }

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif


}

bool System::isShutDown() {
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    int numMaxKFs = 0;
    Map* pBiggerMap;
    std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    for(Map* pMap :vpMaps)
    {
        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::SaveTrajectoryEuRoC(const string &filename, Map* pMap)
{

    cout << endl << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    int numMaxKFs = 0;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose_();
    else
        Twb = vpKFs[0]->GetPoseInverse_();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;


        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}*/


/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap)
{
    cout << endl << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        if(!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << endl;
    }
    f.close();
}


void System::SaveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +to_string(mpLocalMapper->mInitSect)+ "_" + to_string(initIdx)+".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0,0) << "," << mpLocalMapper->mRwg(0,1) << "," << mpLocalMapper->mRwg(0,2) << endl;
    f << mpLocalMapper->mRwg(1,0) << "," << mpLocalMapper->mRwg(1,1) << "," << mpLocalMapper->mRwg(1,2) << endl;
    f << mpLocalMapper->mRwg(2,0) << "," << mpLocalMapper->mRwg(2,1) << "," << mpLocalMapper->mRwg(2,2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" +to_string(mpLocalMapper->mInitSect)+ "_" +to_string(initIdx)+".txt", ios_base::app);
    f << fixed;
    for(int i=0; i<mpLocalMapper->mcovInertial.rows(); i++)
    {
        for(int j=0; j<mpLocalMapper->mcovInertial.cols(); j++)
        {
            if(j!=0)
                f << ",";
            f << setprecision(15) << mpLocalMapper->mcovInertial(i,j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

Sophus::SE3f System::GetCurrentPose() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return mCurrentPose;
}

bool System::HasCurrentPose() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return mbCurrentPoseValid;
}

bool System::RequestAtlasSave(const string &baseName)
{
    lock_guard<mutex> lock(mMutexAtlasSave);
    if(mAtlasSaveStatus.state == AtlasSaveState::QUEUED ||
       mAtlasSaveStatus.state == AtlasSaveState::SAVING)
        return false;
    mAtlasSaveStatus = AtlasSaveStatus();
    if(baseName.empty() || mSensor != IMU_STEREO || isShutDown())
    {
        mAtlasSaveStatus.state = AtlasSaveState::FAILED;
        mAtlasSaveStatus.message = "Save requires a running stereo-inertial system and a filename";
        return false;
    }
    mAtlasSaveBaseName = baseName;
    mpAtlasSaveMap = mpAtlas->GetCurrentMap();
    mAtlasSaveStatus.state = AtlasSaveState::QUEUED;
    mAtlasSaveStatus.message = "Save queued; waiting for next stereo frame";
    return true;
}

System::AtlasSaveStatus System::GetAtlasSaveStatus()
{
    lock_guard<mutex> lock(mMutexAtlasSave);
    return mAtlasSaveStatus;
}

void System::ProcessAtlasSaveRequest()
{
    string baseName;
    Map* requestedMap;
    {
        lock_guard<mutex> lock(mMutexAtlasSave);
        if(mAtlasSaveStatus.state != AtlasSaveState::QUEUED)
            return;
        baseName = mAtlasSaveBaseName;
        requestedMap = mpAtlasSaveMap;
        mAtlasSaveStatus.state = AtlasSaveState::SAVING;
        mAtlasSaveStatus.message = "Waiting for mapping and loop closing to pause";
    }

    AtlasSaveStatus result;
    result.state = AtlasSaveState::FAILED;
    bool resumeLocalMapper = false;
    bool resumeLoopCloser = false;
    try
    {
        // Tracking is at a frame boundary. Pause loop closing first: a loop or
        // GBA may itself stop/release LocalMapping, so stopping LM first races it.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        auto waitUntil = [&](const auto &ready, const char* description) {
            while(!ready())
            {
                if(isShutDown() || std::chrono::steady_clock::now() >= deadline)
                    throw std::runtime_error(string("Save cancelled while waiting for ") + description);
                usleep(1000);
            }
        };
        mpLoopCloser->RequestPause();
        resumeLoopCloser = true;
        waitUntil([&](){ return mpLoopCloser->isPaused(); }, "loop closing");
        waitUntil([&](){ return !mpLoopCloser->isRunningGBA(); }, "global BA");

        resumeLocalMapper = !mpLocalMapper->isStopped();
        mpLocalMapper->RequestStop();
        waitUntil([&](){ return mpLocalMapper->isStopped(); }, "local mapping");
        if(isShutDown())
            throw std::runtime_error("System is shutting down");
        {
            lock_guard<mutex> lock(mMutexReset);
            if(mbReset || mbResetActiveMap)
                throw std::runtime_error("A reset/map switch is pending; save again after initialization");
        }
        Map* current = mpAtlas->GetCurrentMap();
        if(current != requestedMap || current->IsBad() ||
           mpTracker->mState != Tracking::OK || current->KeyFramesInMap() == 0 ||
           !current->isImuInitialized() || !current->GetIniertialBA2())
            throw std::runtime_error("Map is not ready: require tracking OK and completed VIBA 2");

        // All atlas writers are now quiescent, not just the map-update sections.
        if(!SaveAtlas(FileType::BINARY_FILE, baseName))
            throw std::runtime_error("Archive write failed; see terminal for the file error");
        result.mapId = current->GetId();
        result.keyFrames = current->KeyFramesInMap();
        result.mapPoints = current->MapPointsInMap();
        result.state = AtlasSaveState::SUCCEEDED;
        result.message = "Atlas save succeeded: " + baseName + ".osa";
        {
            lock_guard<mutex> lock(mMutexRecovery);
            if(mRecoveryStatus.stage == RecoveryStage::FIRST_MAPPING)
            {
                mRecoveryArchive=baseName;
                mRecoveryStatus.stage=RecoveryStage::FIRST_SAVED;
                mRecoveryStatus.firstMapId=result.mapId;
                mRecoveryStatus.firstKeyFrames=result.keyFrames;
                mRecoveryStatus.firstPoints=result.mapPoints;
                mRecoveryStatus.message="First map saved; ready for reset";
            }
        }
    }
    catch(const std::exception &e)
    {
        result.message = string("Atlas save failed: ") + e.what();
    }
    // Restore only modes paused by this operation, including every error path.
    if(resumeLocalMapper)
        mpLocalMapper->ResumeAfterSave();
    if(resumeLoopCloser)
        mpLoopCloser->ReleasePause();
    cout << result.message << endl;
    {
        lock_guard<mutex> lock(mMutexAtlasSave);
        mAtlasSaveStatus = result;
    }
}

System::RecoveryStatus System::GetRecoveryStatus()
{
    lock_guard<mutex> lock(mMutexRecovery);
    return mRecoveryStatus;
}

bool System::StartNewMap()
{
    lock_guard<mutex> lock(mMutexRecovery);
    if(mRecoveryAction || mRecoveryStatus.stage!=RecoveryStage::FIRST_SAVED) return false;
    mRecoveryAction=1;
    mRecoveryStatus.stage=RecoveryStage::RESETTING;
    mRecoveryStatus.message="Full reset queued";
    return true;
}

bool System::ConfirmSecondInitialization()
{
    lock_guard<mutex> lock(mMutexRecovery);
    if(mRecoveryAction || mRecoveryStatus.stage!=RecoveryStage::SECOND_MAPPING) return false;
    mRecoveryAction=2;
    return true;
}

bool System::RequestLoadFirstMap()
{
    lock_guard<mutex> lock(mMutexRecovery);
    if(mRecoveryAction || mRecoveryStatus.stage!=RecoveryStage::SECOND_READY) return false;
    mRecoveryAction=3;
    mRecoveryStatus.stage=RecoveryStage::LOADING;
    mRecoveryStatus.message="Reading first map from disk";
    return true;
}

void System::ProcessRecoveryActions()
{
    int action;
    { lock_guard<mutex> lock(mMutexRecovery); action=mRecoveryAction; mRecoveryAction=0; }
    if(action==1)
    {
        cout<<"[Recovery] FULL RESET: clearing atlas and restarting stereo-inertial initialization"<<endl;
        mpMapDrawer->ClearRecoveryView();
        mpMapDrawer->SetCurrentCameraPose(Sophus::SE3f());
        mpTracker->Reset();
        mpTracker->NewDataset();
        { lock_guard<mutex> lock(mMutexState); mbCurrentPoseValid=false; }
        { lock_guard<mutex> lock(mMutexRecovery);
          mRecoveryStatus.report=RecoveryReport();
          mRecoveryStatus.stage=RecoveryStage::SECOND_MAPPING;
          mRecoveryStatus.message="Map cleared; second initialization: provide motion excitation"; }
        cout<<"[Recovery] RESET COMPLETE: active KFs="<<mpAtlas->KeyFramesInMap()
            <<" MPs="<<mpAtlas->MapPointsInMap()<<endl;
    }
    else if(action==2)
    {
        Map* map=mpAtlas->GetCurrentMap();
        lock_guard<mutex> lock(mMutexRecovery);
        if(mpTracker->mState!=Tracking::OK || !map->GetIniertialBA2() || !map->isImuInitialized() || mpLocalMapper->IsInitializing())
            mRecoveryStatus.message="Second initialization not ready; wait for end VIBA 2";
        else
        {
            mRecoveryStatus.secondMapId=map->GetId();
            mRecoveryStatus.secondKeyFrames=map->KeyFramesInMap();
            mRecoveryStatus.secondPoints=map->MapPointsInMap();
            mRecoveryStatus.stage=RecoveryStage::SECOND_READY;
            mRecoveryStatus.message="Second initialization ready; press Read First Map";
            cout<<"[Recovery] SECOND READY: map="<<map->GetId()<<" KFs="<<map->KeyFramesInMap()
                <<" MPs="<<map->MapPointsInMap()<<endl;
        }
    }
    else if(action==3) LoadRecoveryReference();
}

bool System::LoadRecoveryReference()
{
    // The archive has process-global ID counters in its serialization format.
    // Deserialize only while all writers are stopped and restore those counters.
    struct Counters {
        unsigned long frame=Frame::nNextId,key=KeyFrame::nNextId,point=MapPoint::nNextId,
                      map=Map::nNextId,camera=GeometricCamera::nNextId;
        ~Counters(){Frame::nNextId=frame;KeyFrame::nNextId=key;MapPoint::nNextId=point;
                    Map::nNextId=map;GeometricCamera::nNextId=camera;}
    };
    bool resumeLocal=false,pausedLoop=false,success=false;
    string message;
    try
    {
        auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
        auto wait=[&](const auto& predicate){while(!predicate()) {
            if(isShutDown() || std::chrono::steady_clock::now()>deadline) throw std::runtime_error("Timeout pausing map writers");
            usleep(1000);
        }};
        mpLoopCloser->RequestPause();pausedLoop=true;
        wait([&](){return mpLoopCloser->isPaused();});
        wait([&](){return !mpLoopCloser->isRunningGBA();});
        resumeLocal=!mpLocalMapper->isStopped();mpLocalMapper->RequestStop();
        wait([&](){return mpLocalMapper->isStopped();});
        Counters counters;
        std::ifstream file(mRecoveryArchive+".osa",std::ios::binary);
        if(!file) throw std::runtime_error("Archive does not exist: "+mRecoveryArchive+".osa");
        Atlas* raw=nullptr;
        string vocabularyName,checksum;
        boost::archive::binary_iarchive archive(file);
        archive>>vocabularyName>>checksum;
        if(checksum!=CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE))
            throw std::runtime_error("Vocabulary checksum mismatch");
        archive>>raw;
        std::unique_ptr<Atlas> atlas(raw);
        auto database=std::make_unique<KeyFrameDatabase>(*mpVocabulary);
        atlas->SetKeyFrameDababase(database.get());atlas->SetORBVocabulary(mpVocabulary);atlas->PostLoad();
        Map* reference=nullptr;
        for(Map* map:atlas->GetAllMaps())if(map->GetId()==mRecoveryStatus.firstMapId)reference=map;
        if(!reference || reference->KeyFramesInMap()==0 || !reference->isImuInitialized())
            throw std::runtime_error("Saved inertial reference map is missing or empty");
        if(reference->KeyFramesInMap()!=mRecoveryStatus.firstKeyFrames || reference->MapPointsInMap()!=mRecoveryStatus.firstPoints)
            throw std::runtime_error("Archive differs from the first saved snapshot");
        mpMapDrawer->SetRecoveryReference(reference);
        mRecoveryAtlas=std::move(atlas);mRecoveryDatabase=std::move(database);mpRecoveryReference=reference;
        mAlignmentConfirmations=0;mLastAlignmentFrame=0;
        success=true;
        message="FILE LOADED; waiting for geometric match to first map";
        cout<<"[Recovery] FILE LOAD SUCCESS: "<<mRecoveryArchive<<".osa; KFs="<<reference->KeyFramesInMap()
            <<" MPs="<<reference->MapPointsInMap()<<endl;
    }
    catch(const std::exception& e){message=string("FILE LOAD FAILED: ")+e.what();cerr<<"[Recovery] "<<message<<endl;}
    if(resumeLocal)mpLocalMapper->ResumeAfterSave();
    if(pausedLoop)mpLoopCloser->ReleasePause();
    { lock_guard<mutex> lock(mMutexRecovery); mRecoveryStatus.message=message;
      mRecoveryStatus.report=RecoveryReport();
      mRecoveryStatus.stage=success?RecoveryStage::LOADED:RecoveryStage::SECOND_READY; }
    return success;
}

void System::UpdateRecoveryAlignment()
{
    const auto status=GetRecoveryStatus();
    if(status.stage!=RecoveryStage::LOADED && status.stage!=RecoveryStage::ALIGNED) return;
    Map* map=mpAtlas->GetCurrentMap();
    if(map->GetId()!=status.secondMapId || !map->isImuInitialized())
    {
        mpMapDrawer->ClearRecoveryView();
        lock_guard<mutex> lock(mMutexRecovery);
        mRecoveryStatus.report=RecoveryReport();
        mRecoveryStatus.stage=RecoveryStage::SECOND_MAPPING;
        mRecoveryStatus.message="Second map was reset; alignment invalidated";
        return;
    }
    if(status.stage==RecoveryStage::ALIGNED || mpTracker->mState!=Tracking::OK) return;
    if(mpTracker->mCurrentFrame.mnId<mLastAlignmentFrame+15) return;
    mLastAlignmentFrame=mpTracker->mCurrentFrame.mnId;
    Sophus::SE3f transform;
    RecoveryMatchMetrics metrics;
    // LM may refine the second map. Its update mutex protects the 3-D comparison.
    unique_lock<mutex> mapLock(map->mMutexMapUpdate);
    bool matched=mpTracker->MatchReferenceMap(mpRecoveryReference,mRecoveryDatabase.get(),transform,metrics);
    mapLock.unlock();
    if(!matched){mAlignmentConfirmations=0;return;}
    const auto difference=mAlignmentCandidate.inverse()*transform;
    const float translationDelta=difference.translation().norm();
    const float rotationDelta=difference.so3().log().norm();
    if(mAlignmentConfirmations && translationDelta<0.15f && rotationDelta<0.10f) ++mAlignmentConfirmations;
    else mAlignmentConfirmations=1;
    mAlignmentCandidate=transform;
    if(mAlignmentConfirmations<2) return;
    const auto report=MakeRecoveryReport(transform,mpTracker->mCurrentFrame.GetPose().inverse(),metrics,
        translationDelta,rotationDelta,mAlignmentConfirmations,mpTracker->mCurrentFrame.mTimeStamp);
    if(!report.valid){mAlignmentConfirmations=0;return;}
    mFirstFromSecond=transform;
    mpMapDrawer->SetRecoveryAlignment(transform);
    { lock_guard<mutex> lock(mMutexRecovery);mRecoveryStatus.stage=RecoveryStage::ALIGNED;
      mRecoveryStatus.inliers=metrics.inliers;mRecoveryStatus.mapPairs=metrics.pairs;mRecoveryStatus.residual=metrics.rmse;
      mRecoveryStatus.report=report;
      mRecoveryStatus.message="ALIGNMENT VERIFIED; pose/distance now in FIRST map frame"; }
    cout<<"[Recovery] ALIGNMENT SUCCESS: PnP inliers="<<metrics.inliers<<" 3D pairs="<<metrics.pairs<<" RMSE="<<metrics.rmse
        <<" m; second-to-first transform:\n"<<transform.matrix()<<endl;
    std::ostringstream output;
    output<<std::fixed<<std::setprecision(3)
          <<"[Recovery] CAMERA OFFSET FROM FIRST ORIGIN: XYZ=["<<report.cameraInFirst.transpose()
          <<"] m; distance="<<report.cameraDistance<<" m; at alignment t="<<report.timestamp<<"\n"
          <<"[Recovery] SECOND ORIGIN OFFSET FROM FIRST ORIGIN: XYZ=["<<report.secondOriginInFirst.transpose()
          <<"] m; distance="<<report.originDistance<<" m\n"
          <<"[Recovery] LOAD CONFIDENCE: "<<std::setprecision(1)<<report.confidence
          <<"/100 (heuristic match quality, not a probability; snapshot at alignment)"
          <<"; PnP inliers="<<metrics.inliers<<"; 3D pairs="<<metrics.pairs<<"/"<<metrics.candidatePairs
          <<" ("<<report.pairRatio*100.f<<"%); RMSE="<<std::setprecision(3)<<metrics.rmse
          <<" m; agreement="<<translationDelta<<" m / "<<rotationDelta*180.f/3.14159265f
          <<" deg; confirmations="<<report.confirmations;
    cout<<output.str()<<endl;
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

bool System::SaveAtlas(int type, const string &baseName)
{
    // Caller must stop Tracking, LocalMapping, LoopClosing and any GBA first.
    string temporary;
    try
    {
        if(baseName.empty())
            throw std::runtime_error("Empty archive filename");
        const string destination = baseName + ".osa";
        const string checksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
        if(checksum.empty())
            throw std::runtime_error("Cannot checksum vocabulary");
        const string vocabularyName = mStrVocabularyFilePath.substr(mStrVocabularyFilePath.find_last_of("/\\")+1);
        // Write to a unique file in the same directory; a failed save must not
        // destroy the last good archive, nor count an old file as success.
        string pattern = destination + ".tmp.XXXXXX";
        vector<char> tempName(pattern.begin(), pattern.end());
        tempName.push_back('\0');
        const int fd = mkstemp(tempName.data());
        if(fd < 0)
            throw std::runtime_error(string("Cannot create archive: ") + std::strerror(errno));
        temporary = tempName.data();
        close(fd);
        mpAtlas->PreSave();
        std::ofstream ofs;
        ofs.exceptions(std::ios::failbit | std::ios::badbit);
        ofs.open(temporary, std::ios::binary | std::ios::trunc);
        if(type == TEXT_FILE)
        {
            boost::archive::text_oarchive archive(ofs);
            archive << vocabularyName << checksum << mpAtlas;
        }
        else
        {
            boost::archive::binary_oarchive archive(ofs);
            archive << vocabularyName << checksum << mpAtlas;
        }
        ofs.flush();
        ofs.close();
        if(std::rename(temporary.c_str(), destination.c_str()) != 0)
            throw std::runtime_error(string("Cannot publish archive: ") + std::strerror(errno));
        return true;
    }
    catch(const std::exception &e)
    {
        if(!temporary.empty())
            std::remove(temporary.c_str());
        cerr << "Atlas save failed: " << e.what() << endl;
        return false;
    }
}

bool System::LoadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool isRead = false;

    string pathLoadFileName = "./";
    pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
    pathLoadFileName = pathLoadFileName.append(".osa");

    if(type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file " << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if(type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file"  << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if(isRead)
    {
        //Check if the vocabulary is the same
        string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);

        if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session was created " << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

string System::CalculateCheckSum(string filename, int type)
{
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if(type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if ( !f.is_open() )
    {
        cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
        return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init (&md5Context);
    while(f.read(buffer, sizeof(buffer)) || f.gcount() > 0)
    {
        MD5_Update(&md5Context, buffer, f.gcount());
    }
    if(f.bad())
        return "";

    f.close();

    MD5_Final(c, &md5Context );

    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux,"%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

} //namespace ORB_SLAM
