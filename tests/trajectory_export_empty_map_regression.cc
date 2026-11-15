// [NP-01 FIX14] Offline reproduction of the SaveTrajectoryTUM crash.
//
// Observed in logs/vio_closed_loop_np01_fix13_func1m_20261115_182729/slam.log:
//
//   Saving camera trajectory to /tmp/slam_imu_stereo_trajectory_tum.txt ...
//   Segmentation fault
//
// Root cause candidate: Atlas::GetAllKeyFrames() returns only the CURRENT
// map's keyframes (Atlas.cc:191-195 -> mpCurrentMap->GetAllKeyFrames()).
// After a map reset, if inertial/stereo initialisation never succeeded in the
// new current map, that vector is EMPTY and System::SaveTrajectoryTUM does
//
//     Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();     // empty vector!
//
// This test builds a real System in IMU_STEREO mode, never feeds a frame
// (so the current map has zero keyframes), and then exports the trajectory.
// It requires no simulator, no flight and no image data.
//
// Before the fix this aborts with SIGSEGV. After the fix it must complete and
// write a well-formed file with zero pose rows, reporting the empty map.

#include "System.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

using namespace ORB_SLAM3;

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: trajectory_export_empty_map_regression <ORBvoc.txt> <settings.yaml>\n";
        return 2;
    }
    const std::string vocabFile    = argv[1];
    const std::string settingsFile = argv[2];
    const std::string outFile      = "/tmp/np01_traj_empty_map_regression.txt";

    std::remove(outFile.c_str());

    // IMU_STEREO keeps SaveTrajectoryTUM on the non-monocular path.
    System sys(vocabFile, settingsFile, System::IMU_STEREO, false);

    // No frame was ever submitted: mpAtlas->GetCurrentMap() has no keyframes.
    std::cout << "[TEST] exporting trajectory from an empty current map" << std::endl;
    sys.SaveTrajectoryTUM(outFile);

    std::ifstream in(outFile);
    if (!in) {
        std::cerr << "[TEST] FAIL: export did not create " << outFile << std::endl;
        return 1;
    }

    int rows = 0;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty()) ++rows;
    }

    if (rows != 0) {
        std::cerr << "[TEST] FAIL: expected 0 pose rows from an empty map, got " << rows << std::endl;
        return 1;
    }

    std::cout << "[TEST] PASS: empty map exported without crash (rows=0)" << std::endl;
    return 0;
}
