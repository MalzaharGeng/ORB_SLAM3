#include "Examples/Utils/RecoveryConsole.h"
#include <chrono>
#include <stdexcept>

using namespace ORB_SLAM3;
static void require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
int main(int argc,char** argv)
{
    if(argc!=3) return 2;
    System slam(argv[1],argv[2],System::IMU_STEREO,false);
    int fds[2];require(pipe(fds)==0,"pipe failed");
    const int original=dup(STDIN_FILENO);
    require(dup2(fds[0],STDIN_FILENO)>=0,"dup2 failed");close(fds[0]);
    auto send=[&](const std::string& s){ require(write(fds[1],s.data(),s.size())==ssize_t(s.size()),"write failed"); };
    RecoveryConsole console(slam,"unused_console_test",true);
    const auto started=std::chrono::steady_clock::now();
    require(console.Poll(),"Idle stdin stopped console");
    require(std::chrono::steady_clock::now()-started<std::chrono::milliseconds(100),"Idle stdin blocked tracking");
    send("fir");require(console.Poll(),"Partial input stopped console");
    require(slam.GetAtlasSaveStatus().state==System::AtlasSaveState::IDLE,"Partial line executed before newline");
    send("st extra\nload\nreset\nsecond\n");console.Poll();
    require(slam.GetAtlasSaveStatus().state==System::AtlasSaveState::IDLE,"Invalid command queued a save");
    require(slam.GetRecoveryStatus().stage==System::RecoveryStage::FIRST_MAPPING,"Wrong-stage commands changed recovery state");
    send(std::string(1100,'x')+"\n");console.Poll();
    send("status\r\npose\nfirst\r\n");require(console.Poll(),"CRLF command parsing failed");
    require(slam.GetAtlasSaveStatus().state==System::AtlasSaveState::QUEUED,"first did not enqueue the GUI save API");
    require(!console.Execute("reset"),"Reset accepted before first save completed");
    send("quit\n");require(!console.Poll() && console.QuitRequested(),"quit failed without sensor frames");
    close(fds[1]);
    RecoveryConsole eof(slam,"unused_console_test",true);
    require(!eof.Poll(),"Headless stdin EOF must stop the session");
    RecoveryConsole guiEof(slam,"unused_console_test",false);
    require(guiEof.Poll() && !guiEof.QuitRequested(),"GUI/replay must survive closed stdin");
    if(original>=0) { dup2(original,STDIN_FILENO);close(original); }
    slam.Shutdown();
    require(slam.GetAtlasSaveStatus().state==System::AtlasSaveState::FAILED,"Shutdown did not cancel queued save");
    std::cout<<"RECOVERY_CONSOLE_PASS: nonblocking partial input, CRLF, stage guards, quit and EOF"<<std::endl;
}
