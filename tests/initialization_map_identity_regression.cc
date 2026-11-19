#include "InitializationMapIdentity.h"
#include "KeyFrame.h"
#include <iostream>
#include <stdexcept>
using namespace ORB_SLAM3;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){
    Map original(0),replacement(0);
    KeyFrame origin;origin.mnId=10;origin.UpdateMap(&original);original.AddKeyFrame(&origin);
    InitializationMapIdentity guard(&original);
    check(guard.Matches(&original),"unchanged map rejected");
    check(!guard.Matches(nullptr) && !guard.Matches(&replacement),"different active map accepted");
    const auto id=original.GetId();replacement.ChangeId(id);
    check(!guard.Matches(&replacement),"same id on a different map accepted");
    original.ChangeId(id+100);
    check(!guard.Matches(&original),"map id mutation accepted");original.ChangeId(id);
    check(guard.Matches(&original),"unchanged identity not restored");
    original.clear();
    check(!guard.Matches(&original),"in-place reset was mistaken for the original map");
    KeyFrame next;next.mnId=11;next.UpdateMap(&original);original.AddKeyFrame(&next);
    check(!guard.Matches(&original),"replacement origin accepted");
    InitializationMapIdentity fresh(&original);check(fresh.Matches(&original),"fresh map cannot initialize");
    std::cout<<"PASS original map, null/different active map, reused id, changed id, in-place clear, replaced origin, fresh initialization\n";
}
