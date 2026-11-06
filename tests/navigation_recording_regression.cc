#include "AsyncNavigationRecording.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
int main() {
    const auto root=std::filesystem::temp_directory_path()/("navigation-record-"+std::to_string(getpid()));
    std::filesystem::create_directory(root);
    try {
        const auto path=root/"nav.csv";
        ORB_SLAM3::AsyncNavigationRecording recorder(path.string(),"kind,frame,time");
        for(int i=0;i<1000;++i) {
            if(!recorder.Write("frame,"+std::to_string(i)+","+std::to_string(i*.01)+"\n"))
                throw std::runtime_error("write rejected");
        }
        if(!recorder.Close() || !recorder.Close())throw std::runtime_error("close failed");
        std::ifstream input(path);std::string line;int count=0;
        while(std::getline(input,line))++count;
        if(count!=1001)throw std::runtime_error("recording row loss");
        std::ifstream mf(path.string()+".summary.json");std::string summary((std::istreambuf_iterator<char>(mf)),{});
        if(summary.find("\"writer_complete\":true")==std::string::npos)throw std::runtime_error("summary incomplete");
        bool rejected=false;
        try {ORB_SLAM3::AsyncNavigationRecording invalid((root/"missing/nav.csv").string(),"header");}
        catch(const std::exception&){rejected=true;}
        if(!rejected)throw std::runtime_error("invalid recording path accepted");
        std::filesystem::remove_all(root);
        std::cout<<"NAVIGATION_RECORDING_PASS packets=1000 exact_flush=1 invalid_path=1\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;std::filesystem::remove_all(root);return 1;}
}
