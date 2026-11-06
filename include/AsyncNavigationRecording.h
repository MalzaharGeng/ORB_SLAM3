#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
namespace ORB_SLAM3 {
class AsyncNavigationRecording {
public:
    AsyncNavigationRecording(const std::string& path,const std::string& header):path_(path) {
        file_.open(path_,std::ios::out|std::ios::trunc);
        if(!file_)throw std::runtime_error("Cannot open navigation recording: "+path_);
        file_<<header<<"\n";file_.flush();
        if(!file_)throw std::runtime_error("Cannot write navigation recording header");
        std::ofstream initial(path_+".summary.json");
        initial<<R"({"closed_cleanly":false,"writer_complete":false})"<<"\n";
        if(!initial)throw std::runtime_error("Cannot create navigation recording summary");
        initial.close();
        worker_=std::thread([this]{Run();});
    }
    ~AsyncNavigationRecording(){Close();}
    bool Write(std::string packet) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || !error_.empty())return false;
        if(queue_.size()>=2048){error_="navigation writer queue full";wake_.notify_all();return false;}
        queue_.push_back(std::move(packet));++accepted_;wake_.notify_one();return true;
    }
    std::string Error()const {std::lock_guard<std::mutex> lock(mutex_);return error_;}
    bool Close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(closed_)return error_.empty() && accepted_==written_;
            stopping_=true;
        }
        wake_.notify_all();if(worker_.joinable())worker_.join();
        file_.flush();if(!file_ && error_.empty())error_="navigation recording flush failed";
        file_.close();
        std::ofstream manifest(path_+".summary.json");
        manifest<<R"({"closed_cleanly":true,"writer_complete":)"<<(error_.empty() && accepted_==written_?"true":"false")
            <<R"(,"accepted_packets":)"<<accepted_<<R"(,"written_packets":)"<<written_<<R"(,"error":")"<<error_<<"\"}\n";
        if(!manifest && error_.empty())error_="navigation summary write failed";
        closed_=true;return error_.empty() && accepted_==written_;
    }
private:
    void Run() {
        auto flush=std::chrono::steady_clock::now();
        while(true) {
            std::string packet;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait_for(lock,std::chrono::milliseconds(200),[this]{return stopping_ || !queue_.empty() || !error_.empty();});
                if(!error_.empty())break;
                if(queue_.empty()){if(stopping_)break;continue;}
                packet=std::move(queue_.front());queue_.pop_front();
            }
            file_<<packet;
            const auto now=std::chrono::steady_clock::now();
            if(now-flush>=std::chrono::seconds(1)){file_.flush();flush=now;}
            std::lock_guard<std::mutex> lock(mutex_);
            if(!file_){error_="navigation data write failed";break;}
            ++written_;
        }
    }
    const std::string path_;
    std::ofstream file_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<std::string> queue_;
    std::thread worker_;
    std::uint64_t accepted_=0,written_=0;
    bool stopping_=false,closed_=false;
    std::string error_;
};
}
