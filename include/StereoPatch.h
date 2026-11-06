#pragma once
#include <cstddef>
#include <cstdint>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif
namespace ORB_SLAM3 { namespace stereo_detail {
// Exact uint8 11x11 NORM_L1. The largest sum is 121*255=30855.
inline int PatchL1(const std::uint8_t* left,std::size_t leftStep,
                   const std::uint8_t* right,std::size_t rightStep) {
    int total=0;
    for(int row=0;row<11;++row) {
        int column=0;
#if defined(__aarch64__) && defined(__ARM_NEON)
        total+=vaddlv_u8(vabd_u8(vld1_u8(left),vld1_u8(right)));
        column=8;
#endif
        for(;column<11;++column) {
            const int d=int(left[column])-int(right[column]);total+=d<0?-d:d;
        }
        left+=leftStep;right+=rightStep;
    }
    return total;
}
}}
