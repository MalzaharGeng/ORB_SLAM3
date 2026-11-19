#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
namespace ORB_SLAM3 { namespace px4 {
struct ImuReading {
    double time=0;
    std::array<float,3> accel{},gyro{};
};
struct ImuIntervalCoverage {
    double start=0,end=0,first=std::numeric_limits<double>::quiet_NaN(),last=std::numeric_limits<double>::quiet_NaN();
    double missingBefore=0,missingAfter=0;
    size_t buffered=0;
    bool requestValid=false,historyCovered=false,futureCovered=false;
};
class ImuBuffer {
public:
    uint64_t epoch=0, received=0, rejected=0;
    std::deque<ImuReading> data;
    ImuIntervalCoverage Coverage(double start,double end) const {
        ImuIntervalCoverage c;c.start=start;c.end=end;c.buffered=data.size();
        c.requestValid=std::isfinite(start) && std::isfinite(end) && end>start;
        if(!data.empty()) {
            c.first=data.front().time;c.last=data.back().time;
            c.historyCovered=c.requestValid && c.first<=start;
            c.futureCovered=c.requestValid && c.last>=end;
            if(c.requestValid) {
                c.missingBefore=std::max(0.,c.first-start);
                c.missingAfter=std::max(0.,end-c.last);
            }
        }
        return c;
    }
    void Reset() { data.clear(); ++epoch; }
    bool Add(const ImuReading& s) {
        bool valid=std::isfinite(s.time) && s.time>0;
        for(int i=0;i<3;++i) valid=valid && std::isfinite(s.accel[i]) && std::isfinite(s.gyro[i]);
        if(!valid) {++rejected; Reset(); return false;}
        if(!data.empty() && (s.time<=data.back().time || s.time-data.back().time>.03)) {
            ++rejected; Reset();
        }
        data.push_back(s); ++received;
        while(data.size()>2048) data.pop_front();
        return true;
    }
    bool Slice(double start,double end,std::vector<ImuReading>& out,std::string& reason) {
        out.clear();
        if(!(end>start)) {reason="image_time_invalid";return false;}
        if(data.size()<2 || data.back().time<end) {reason="waiting_imu";return false;}
        if(data.front().time>start) {reason="imu_interval_uncovered";return false;}
        auto interp=[](const ImuReading& a,const ImuReading& b,double t) {
            ImuReading r; r.time=t; const double u=(t-a.time)/(b.time-a.time);
            for(int i=0;i<3;++i) {r.accel[i]=a.accel[i]+u*(b.accel[i]-a.accel[i]);r.gyro[i]=a.gyro[i]+u*(b.gyro[i]-a.gyro[i]);}
            return r;
        };
        size_t i=1;
        while(i<data.size() && data[i].time<start) ++i;
        out.push_back(interp(data[i-1],data[i],start));
        while(i<data.size() && data[i].time<end) {
            if(data[i].time>start) out.push_back(data[i]);
            ++i;
        }
        out.push_back(interp(data[i-1],data[i],end));
        while(data.size()>2 && data[1].time<=end) data.pop_front();
        reason="ok";return true;
    }
};
}}
