#pragma once
#include "InputRecording.h"
#include "System.h"
#include "Examples/Utils/RecoveryConsole.h"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <functional>
#include <iomanip>
#include <deque>

namespace ORB_SLAM3 {
inline volatile std::sig_atomic_t recordingReplayRunning=1;
inline void StopRecordingReplay(int) {recordingReplayRunning=0;}

// Generic replay has no fixed 42/85/100-second schedule. A session may contain
// only tracking, one saved map, or the entire recovery workflow.
inline int ReplayRecordedSession(const std::string& vocabulary,const std::string& settings,
    const std::filesystem::path& input,const std::filesystem::path& output,
    const std::shared_ptr<InputRecording>& recording,
    const std::function<void(const System::NavigationState&,uint64_t)>& publish)
{
    const auto data=ReadInputRecording(input);
    if(const char* value=std::getenv("ORB_REPLAY_STRIDE"))
        if(std::atoi(value)!=1)throw std::runtime_error("Recorded sessions require stride=1 to preserve frame-specific IMU/actions");
    recordingReplayRunning=1;std::signal(SIGINT,StopRecordingReplay);std::signal(SIGTERM,StopRecordingReplay);
    System slam(vocabulary,settings,System::IMU_STEREO,std::getenv("ORB_REPLAY_VIEWER")!=nullptr);
    slam.SetInputRecording(recording);
    try {
        RecoveryConsole console(slam,(output/"first").string(),false);
        const bool terminal=std::getenv("ORB_REPLAY_CONSOLE")!=nullptr;
        if(terminal)console.Help();
        std::ofstream navFile(output/"navigation.csv"),timing(output/"timing.csv");
        navFile<<"index,timestamp,valid,tracking,imu,ba2,first_map_frame,imu_x,imu_y,imu_z,qw,qx,qy,qz\n"<<std::setprecision(17);
        timing<<"index,timestamp,read_ms,track_ms,cycle_ms\n"<<std::setprecision(17);
        if(!navFile || !timing)throw std::runtime_error("Cannot write replay outputs");
        size_t processed=0,actions=0,valid=0;bool loadRequested=false;
        std::deque<std::string> pending;
        std::string waiting;
        std::cout<<"[Replay] recorded session: frames="<<data.frames.size()<<" finalized="<<data.complete<<" stride=1"<<std::endl;
        for(size_t i=0;i<data.frames.size() && recordingReplayRunning && !slam.isShutDown();++i) {
            if(terminal && !console.Poll())break;
            const auto start=std::chrono::steady_clock::now();const auto& f=data.frames[i];
            const cv::Mat left=cv::imread((input/f.left).string(),0),right=cv::imread((input/f.right).string(),0);
            if(left.empty()||right.empty())throw std::runtime_error("Cannot decode recorded stereo pair "+std::to_string(i));
            const auto trackStart=std::chrono::steady_clock::now();
            const auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // Recorded stage times are the earliest request time. Mapping is
            // asynchronous, so retry soft stages when actually ready. Never
            // move a recorded reset or consume segment 2 to initialize map 1.
            std::vector<std::string> apply;
            const auto previous=slam.GetNavigationState();
            auto stage=slam.GetRecoveryStatus().stage;
            const bool initReady=previous.valid && previous.inertialInitialized && previous.inertialBA2;
            for(const auto& action:f.actions)pending.push_back(action);
            while(!pending.empty()) {
                const auto& action=pending.front();bool ready=true;
                if(action=="first")ready=initReady && stage==System::RecoveryStage::FIRST_MAPPING;
                else if(action=="second")ready=initReady && stage==System::RecoveryStage::SECOND_MAPPING;
                else if(action=="load")ready=stage==System::RecoveryStage::SECOND_READY;
                if(!ready) {
                    if(std::any_of(pending.begin(),pending.end(),[](const auto& a){return a=="reset" || a=="reset_active" || a=="reset_full";}))
                        throw std::runtime_error("Recorded reset boundary reached before stage completion; refusing to postpone the reset or consume another map's input");
                    if(waiting!=action) {std::cout<<"[Replay] waiting for recorded action: "<<action<<" (real initialization/stage guard)"<<std::endl;waiting=action;}
                    break;
                }
                if(action=="reset" && stage!=System::RecoveryStage::FIRST_SAVED)
                    throw std::runtime_error("Recorded reset requires a successfully saved first map");
                apply.push_back(action);
                if(action=="first")stage=System::RecoveryStage::FIRST_SAVED;
                else if(action=="reset")stage=System::RecoveryStage::SECOND_MAPPING;
                else if(action=="second")stage=System::RecoveryStage::SECOND_READY;
                else if(action=="load")stage=System::RecoveryStage::LOADED;
                pending.pop_front();waiting.clear();
            }
            slam.SetRecordedActions(apply,(output/"first").string());
            slam.TrackStereo(left,right,f.timestamp,f.imu);
            const auto trackEnd=std::chrono::steady_clock::now();
            const auto nav=slam.GetNavigationState();
            if(publish)publish(nav,stamp);
            for(const auto& action:apply) {++actions;if(action=="load")loadRequested=true;}
            ++processed;if(nav.valid)++valid;
            const auto p=nav.imuToWorld.translation();const auto q=nav.imuToWorld.unit_quaternion();
            navFile<<i<<","<<f.timestamp<<","<<nav.valid<<","<<nav.trackingState<<","<<nav.inertialInitialized<<","<<nav.inertialBA2
                   <<","<<nav.inFirstMap<<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<"\n";
            if(terminal)console.PrintChanges();
            if(i%150==0)std::cout<<"SESSION_REPLAY frame="<<i<<" t="<<f.timestamp<<" tracking="<<nav.trackingState<<std::endl;
            if(!std::getenv("ORB_REPLAY_UNPACED") && i+1<data.frames.size()) {
                // Preserve elapsed sensor time, including clock-reset boundaries
                // (a backwards clock has no negative sleep). Poll during gaps.
                const auto& next=data.frames[i+1];
                const bool clockReset=std::find(next.actions.begin(),next.actions.end(),"reset_active")!=next.actions.end();
                const double gap=next.timestamp-f.timestamp;
                const auto end=start+std::chrono::duration<double>(clockReset && gap>1.?0.:std::max(0.,gap));
                while(recordingReplayRunning && !slam.isShutDown() && std::chrono::steady_clock::now()<end) {
                    if(terminal && !console.Poll())break;
                    std::this_thread::sleep_until(std::min(end,start+std::chrono::duration<double>(
                        std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()+.02)));
                }
            }
            const auto elapsed=[](auto t){return std::chrono::duration<double,std::milli>(t).count();};
            timing<<i<<","<<f.timestamp<<","<<elapsed(trackStart-start)<<","<<elapsed(trackEnd-trackStart)
                  <<","<<elapsed(std::chrono::steady_clock::now()-start)<<"\n";
        }
        const auto recovery=slam.GetRecoveryStatus();
        const bool stopped=!recordingReplayRunning || console.QuitRequested() || processed!=data.frames.size();
        if(!slam.isShutDown())slam.Shutdown();
        if(recording && !recording->Close())return 1;
        navFile.close();timing.close();if(!navFile||!timing)throw std::runtime_error("Cannot finalize replay outputs");
        if(stopped) {std::cout<<"REPLAY_STOPPED_BY_USER frames="<<processed<<std::endl;return 0;}
        if(!pending.empty()) {std::cerr<<"REPLAY_STAGE_INCOMPLETE: input ended before recorded action "<<pending.front()<<std::endl;return 1;}
        if(loadRequested && recovery.stage!=System::RecoveryStage::ALIGNED) {
            std::cerr<<"RECOVERY_NOT_ALIGNED: recorded load replayed, but geometry was not confirmed"<<std::endl;return 1;
        }
        std::cout<<"RECORDING_REPLAY_COMPLETE frames="<<processed<<" actions="<<actions<<" valid_poses="<<valid
                 <<" recovery="<<RecoveryConsole::StageName(recovery.stage)<<std::endl;
        if(loadRequested)std::cout<<"RECOVERY_REPLAY_PASS inliers="<<recovery.inliers<<" pairs="<<recovery.mapPairs<<" residual="<<recovery.residual<<std::endl;
        return 0;
    } catch(...) {if(!slam.isShutDown())slam.Shutdown();if(recording)recording->Close();throw;}
}
}
