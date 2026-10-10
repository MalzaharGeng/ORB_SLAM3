#include "Atlas.h"
#include "ORBextractor.h"
#include "KeyFrameDatabase.h"
#include <iostream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <stdexcept>
#include <sstream>
using namespace ORB_SLAM3;
static void require(bool condition, const char* message) {
    if(!condition) throw std::runtime_error(message);
}

int main() {
    ORBVocabulary voc;
    KeyFrameDatabase db(voc);
    Atlas atlas(0);
    auto* cam = atlas.AddCamera(new Pinhole({400,400,320,240}));
    cv::Mat left(480,640,CV_8U), right(480,640,CV_8U,cv::Scalar(0));
    cv::RNG rng(42); rng.fill(left,cv::RNG::UNIFORM,0,255);
    left(cv::Rect(8,0,632,480)).copyTo(right(cv::Rect(0,0,632,480)));
    cv::Mat K=(cv::Mat_<float>(3,3)<<400,0,320,0,400,240,0,0,1);
    cv::Mat distortion=cv::Mat::zeros(4,1,CV_32F);
    ORBextractor le(1000,1.2,8,20,7), re(1000,1.2,8,20,7);
    IMU::Calib calib(Sophus::SE3f(),0.001,0.01,0.0001,0.001);
    IMU::Preintegrated pre(IMU::Bias(),calib);
    pre.IntegrateNewMeasurement(Eigen::Vector3f(0,0,9.81),Eigen::Vector3f::Zero(),0.005);
    Frame frame(left,right,0.1,&le,&re,&voc,K,distortion,20,40,cam,nullptr,calib);
    frame.mvDepth.assign(frame.N,2.0f); frame.mvuRight.assign(frame.N,100.0f);
    frame.SetPose(Sophus::SE3f()); frame.mpImuPreintegrated=&pre; frame.mnDataset=0;
    Map* map=atlas.GetCurrentMap();
    auto* first=new KeyFrame(frame,map,&db);
    auto* second=new KeyFrame(frame,map,&db);
    map->AddKeyFrame(first); map->AddKeyFrame(second);
    map->mvpKeyFrameOrigins.push_back(first);
    map->SetInertialSensor(); map->SetImuInitialized(); map->SetIniertialBA1(); map->SetIniertialBA2();
    for(int i=0;i<frame.N;i++) if(frame.mvDepth[i]>0) {
        Eigen::Vector3f pos; frame.UnprojectStereo(i,pos);
        auto* p=new MapPoint(pos,first,map);
        p->AddObservation(first,i); first->AddMapPoint(p,i);
        p->AddObservation(second,i); second->AddMapPoint(p,i);
        map->AddMapPoint(p);
    }
    require(map->MapPointsInMap() > 500, "Fixture must contain sparse stereo points");
    const size_t initialPoints = map->MapPointsInMap();
    // Repeated save/read must not accumulate duplicated maps in backup buffers.
    for(int attempt=0; attempt<5; ++attempt) {
        atlas.PreSave();
        std::stringstream stream;
        { boost::archive::binary_oarchive out(stream); Atlas* ptr=&atlas; out<<ptr; }
        Atlas* loaded=nullptr;
        { boost::archive::binary_iarchive in(stream); in>>loaded; }
        KeyFrameDatabase loadedDB(voc);
        loaded->SetKeyFrameDababase(&loadedDB); loaded->SetORBVocabulary(&voc); loaded->PostLoad();
        auto maps=loaded->GetAllMaps();
        require(maps.size()==1, "Roundtrip changed map count");
        auto keys=maps[0]->GetAllKeyFrames();
        require(keys.size()==2, "Roundtrip changed keyframe count");
        require(maps[0]->MapPointsInMap()==initialPoints, "Roundtrip changed point count");
        require(maps[0]->GetIniertialBA2(), "Lost VIBA 2 flag");
        for(auto* key:keys) {
            require(key->mpImuPreintegrated != nullptr, "Missing IMU preintegration");
            require(std::abs(key->mpImuPreintegrated->dT-0.005f)<1e-6f, "Changed IMU interval");
            require(cv::norm(key->mDescriptors,first->mDescriptors,cv::NORM_INF)==0, "Changed descriptors");
        }
        for(auto* point:maps[0]->GetAllMapPoints()) {
            require(point->GetObservations().size()==2, "Lost observations");
            require(point->GetReferenceKeyFrame()!=nullptr, "Lost reference keyframe");
        }
        delete loaded;
    }
    // A stale cross-map observation used to erase the current set iterator,
    // producing SIGSEGV in std::_Rb_tree_increment inside Map::PreSave.
    second->UpdateMap(nullptr);
    atlas.PreSave();
    require(map->MapPointsInMap()==0, "Stale observations were not culled");

    // Direct MapPoint pre-save also used to erase its active observation iterator.
    auto* point=new MapPoint(Eigen::Vector3f(0,0,2),first,map);
    map->AddMapPoint(point);
    point->AddObservation(first,0); first->AddMapPoint(point,0);
    std::set<KeyFrame*> noKeyframes;
    std::set<MapPoint*> points{point};
    point->PreSave(noKeyframes,points);
    require(point->isBad(), "Last observation removal should discard point");
    require(point->GetReferenceKeyFrame()==nullptr, "Empty observation set has invalid reference");

    // Saving an empty active map must not mark it bad or archive it.
    atlas.CreateNewMap();
    Map* empty=atlas.GetCurrentMap();
    atlas.PreSave();
    require(!empty->IsBad() && atlas.GetCurrentMap()==empty, "Save changed empty active map");
    std::cout<<"PASS: repeated inertial roundtrip, stale observations, last observation, empty map"<<std::endl;
}
