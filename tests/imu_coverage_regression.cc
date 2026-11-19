#include "Px4ImuBuffer.h"
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3::px4;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){
    ImuBuffer b;
    auto c=b.Coverage(1.,1.02);check(c.buffered==0 && !c.futureCovered && !c.historyCovered,"empty coverage");
    ImuReading r;r.time=1.;b.Add(r);r.time=1.01;b.Add(r);r.time=1.02;b.Add(r);
    const auto epoch=b.epoch,received=b.received;
    c=b.Coverage(1.005,1.03);
    check(c.requestValid && c.historyCovered && !c.futureCovered && std::abs(c.missingAfter-.01)<1e-9,"future coverage deficit");
    c=b.Coverage(.99,1.015);
    check(!c.historyCovered && c.futureCovered && std::abs(c.missingBefore-.01)<1e-9,"missing history deficit");
    c=b.Coverage(1.005,1.015);check(c.historyCovered && c.futureCovered && c.buffered==3,"full coverage");
    check(b.epoch==epoch && b.received==received && b.data.size()==3,"diagnostics changed state");
    c=b.Coverage(2.,1.);check(!c.requestValid,"reversed request");
    c=b.Coverage(NAN,1.);check(!c.requestValid,"nonfinite request");
    std::vector<ImuReading> out;std::string reason;
    check(!b.Slice(1.,1.03,out,reason) && reason=="waiting_imu","waiting policy changed");
    r.time=1.050001;b.Add(r);check(b.epoch==epoch+1 && b.rejected==1,"30ms source gap was relaxed");
    c=b.Coverage(1.02,1.06);check(!c.historyCovered && !c.futureCovered && c.buffered==1,"reset evidence");
    std::cout<<"PASS empty, future deficit, missing history, full coverage, invalid request, immutable diagnostics, unchanged gap rejection\n";
}
