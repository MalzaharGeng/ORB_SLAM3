#ifndef ORB_SLAM3_RECOVERY_CONSOLE_H
#define ORB_SLAM3_RECOVERY_CONSOLE_H

#include "System.h"
#include <cerrno>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <unistd.h>

namespace ORB_SLAM3
{

// Poll only from the acquisition/tracking thread, outside TrackStereo and the
// sensor callback mutex. Every mutation goes through the GUI's queued APIs.
class RecoveryConsole
{
public:
    RecoveryConsole(System& system, std::string archive = "orbslam3_first_map",
                    bool stopOnEof = true)
        : slam(system), archiveBase(std::move(archive)), stopAtEof(stopOnEof) {}

    static const char* StageName(System::RecoveryStage stage)
    {
        switch(stage) {
        case System::RecoveryStage::FIRST_MAPPING: return "FIRST_MAPPING";
        case System::RecoveryStage::FIRST_SAVED: return "FIRST_SAVED";
        case System::RecoveryStage::RESETTING: return "RESETTING";
        case System::RecoveryStage::SECOND_MAPPING: return "SECOND_MAPPING";
        case System::RecoveryStage::SECOND_READY: return "SECOND_READY";
        case System::RecoveryStage::LOADING: return "LOADING";
        case System::RecoveryStage::LOADED: return "LOADED";
        case System::RecoveryStage::ALIGNED: return "ALIGNED";
        }
        return "UNKNOWN";
    }

    static void Help()
    {
        std::cout<<"\n[Console] 输入指令后回车 / commands:\n"
                 <<"  first  (1)  首次 VIBA 2 完成后存图 / First Init Done\n"
                 <<"  reset  (2)  存图成功后重置并建第二张地图 / Start Second Map\n"
                 <<"  second (3)  确认第二次初始化 / Second Init Done\n"
                 <<"  load   (4)  读取首次地图并匹配 / Read First Map\n"
                 <<"  status      查看阶段、存图状态和最近位姿\n"
                 <<"  pose        查看初始 IMU 世界系中的 IMU 位姿和距离\n"
                 <<"  help        显示帮助\n"
                 <<"  quit        停止采集并正常退出（Ctrl+C 也可）\n"
                 <<"[Console] queued 仅表示已排队，请等待完成日志再输入下一阶段。\n"<<std::flush;
    }

    bool Execute(const std::string& line)
    {
        std::istringstream input(line);
        std::string command, extra;
        if(!(input>>command)) return true;
        if(input>>extra) { std::cout<<"[Console] REJECTED: 每次只输入一条指令；输入 help 查看用法。"<<std::endl; return false; }
        bool accepted=false;
        if(command=="help" || command=="?") { Help(); return true; }
        if(command=="status") { PrintStatus(); return true; }
        if(command=="pose") { PrintPose(); return true; }
        if(command=="quit" || command=="exit" || command=="q") {
            quit=true; std::cout<<"[Console] quit requested"<<std::endl; return true;
        }
        if(command=="first" || command=="1") {
            if(slam.GetRecoveryStatus().stage==System::RecoveryStage::FIRST_MAPPING)
                accepted=slam.RequestAtlasSave(archiveBase);
        } else if(command=="reset" || command=="2") {
            accepted=slam.StartNewMap();
        } else if(command=="second" || command=="3") {
            accepted=slam.ConfirmSecondInitialization();
        } else if(command=="load" || command=="4") {
            accepted=slam.RequestLoadFirstMap();
        } else {
            std::cout<<"[Console] UNKNOWN command: "<<command<<"; use help"<<std::endl;
            return false;
        }
        std::cout<<"[Console] "<<command<<(accepted ? " queued" : " REJECTED")
                 <<"; stage="<<StageName(slam.GetRecoveryStatus().stage)<<std::endl;
        if(!accepted) PrintStatus();
        return accepted;
    }

    // No getline(): a partial line, an idle terminal, or a stopped camera must
    // never block acquisition or prevent quit. HUP is drained before EOF.
    bool Poll()
    {
        if(quit || inputClosed) return !quit;
        pollfd fd{STDIN_FILENO, POLLIN, 0};
        const int ready=poll(&fd,1,0);
        if(ready<0 && errno==EINTR) return true;
        if(ready<0 || (fd.revents & (POLLERR|POLLNVAL))) { CloseInput(); return !quit; }
        if(ready==0 || !(fd.revents & (POLLIN|POLLHUP))) return true;
        char buffer[4096];
        const ssize_t count=read(STDIN_FILENO,buffer,sizeof(buffer));
        if(count<0 && (errno==EINTR || errno==EAGAIN)) return true;
        if(count<=0) {
            if(!discardLine && !pending.empty()) Execute(pending);
            pending.clear(); CloseInput(); return !quit;
        }
        for(ssize_t i=0;i<count && !quit;++i) {
            const char c=buffer[i];
            if(c=='\n') {
                if(!discardLine) Execute(pending);
                pending.clear();discardLine=false;
            } else if(c!='\r' && !discardLine) {
                if(pending.size()>=1024) {
                    pending.clear();discardLine=true;
                    std::cout<<"[Console] REJECTED: command exceeds 1024 bytes"<<std::endl;
                } else pending.push_back(c);
            }
        }
        return !quit;
    }

    void PrintChanges()
    {
        const auto recovery=slam.GetRecoveryStatus();
        const auto save=slam.GetAtlasSaveStatus();
        const std::string next=std::string(StageName(recovery.stage))+" | "+recovery.message+" | "+save.message;
        if(next!=lastStatus) {
            lastStatus=next;
            std::cout<<"[Console] "<<next<<std::endl;
        }
    }

    void PrintStatus()
    {
        const auto recovery=slam.GetRecoveryStatus();
        const auto save=slam.GetAtlasSaveStatus();
        std::cout<<"[Console] stage="<<StageName(recovery.stage)<<"; "<<recovery.message
                 <<"; save="<<save.message<<std::endl;
        PrintPose();
        if(recovery.stage==System::RecoveryStage::ALIGNED && recovery.report.valid)
            std::cout<<"[Console] load confidence="<<recovery.report.confidence
                     <<"/100 (heuristic at alignment); IMU at load="
                     <<recovery.report.imuInFirst.transpose()<<" m"<<std::endl;
    }

    void PrintPose()
    {
        const auto nav=slam.GetNavigationState();
        if(!nav.valid) { std::cout<<"[Console] pose unavailable; tracking="<<slam.GetTrackingState()<<std::endl; return; }
        const auto p=nav.imuToWorld.translation();const auto q=nav.imuToWorld.unit_quaternion();
        std::ostringstream out;
        out<<std::fixed<<std::setprecision(3)<<"[Console] IMU XYZ=["<<p.transpose()
           <<"] m; distance="<<p.norm()<<" m; q(wxyz)=["<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()
           <<"]; world="<<(nav.inFirstMap ? "FIRST_IMU0" : "CURRENT_IMU0")
           <<"; sample_t="<<nav.timestamp;
        std::cout<<out.str()<<std::endl;
    }

    bool QuitRequested() const { return quit; }

private:
    void CloseInput()
    {
        inputClosed=true;
        if(stopAtEof) { quit=true; std::cout<<"[Console] stdin closed; stopping headless session"<<std::endl; }
        else std::cout<<"[Console] stdin closed; terminal commands disabled"<<std::endl;
    }
    System& slam;
    std::string archiveBase, pending, lastStatus;
    bool stopAtEof, quit=false, inputClosed=false, discardLine=false;
};
}
#endif
