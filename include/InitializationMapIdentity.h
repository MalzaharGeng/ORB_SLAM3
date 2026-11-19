#pragma once
#include "Map.h"

namespace ORB_SLAM3 {
// Initialization results belong to one map and its original first keyframe.
// Comparing the origin also rejects an in-place clear of the same Map object.
class InitializationMapIdentity {
public:
    explicit InitializationMapIdentity(Map* map):map_(map),id_(map?map->GetId():0),origin_(map?map->GetOriginKF():nullptr) {}
    bool Matches(Map* active) const {
        return map_ && active==map_ && active->GetId()==id_ && active->GetOriginKF()==origin_;
    }
private:
    Map* map_;
    unsigned long id_;
    KeyFrame* origin_;
};
}
