#pragma once
// Gate 诊断快照（FIX11），覆盖提交、接受、拒绝及序列化输出。
// Submit 侧在持有 mMutexState 时复制原始 SLAM 状态和地图身份，之后不覆盖；
// Gate 侧在更新 previous_ 之前复制比较操作数；Mark 仅填充决策字段。
// src_sim_us 为源时钟，sample_us 为映射后的 epoch，wall/mono 分别记录主机时钟。
// 20 ms 仅用于 VIO/真值配对，不作为样本年龄门限。
// 默认环形缓冲 4096 条，每 1024 个序列批量写入并 flush，另存首次拒绝窗口。
// 仅在设置 NP01_GATE_SNAPSHOT 时启用；沿用 Submit 的互斥锁，不逐帧写文件。
// 快照不参与 Gate、锁存、年龄或融合判定，真值不进入控制路径。

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>

namespace ORB_SLAM3 { namespace px4 {

using SnapVec3 = std::array<double,3>;
using SnapQuat = std::array<double,4>;   // w,x,y,z

struct GateSnapshotRecord {
    // ---- submit side (immutable after Add) ----
    uint64_t seq = 0, pair = 0, submitWallUs = 0, submitMonoUs = 0;
    int64_t srcSimUs = 0, mapOffsetUs = 0;
    uint64_t sampleUs = 0, clockEpoch = 0, refEpoch = 0, continuity = 0;
    uint64_t mapId = 0, initialKfId = 0, frameId = 0;
    SnapVec3 srcP{}, srcV{}, rawP{}, mapT{}, initT{};
    SnapQuat srcQ{}, rawQ{}, mapQ{}, initQ{};
    int valid = 0, imuInit = 0, ba2 = 0, velValid = 0;
    SnapVec3 gyroIntegral{};             // rad, trapezoid integration over this frame's IMU interval
    double gyroSpanS = 0;
    uint64_t imuCount = 0;

    // ---- decision side (filled by Mark) ----
    int decision = 0;                    // 0 pending,1 publish,2 reject,3 skip,4 wait_px4_local
    std::string reason;
    uint64_t evalWallUs = 0, evalMonoUs = 0;
    double ageMs = 0;
    uint64_t sent = 0, rejected = 0;
    int armed = 0, navState = 0, landed = 0;
    int havePrevious = 0, comparable = 0, crossTransform = 0;
    double dt = 0, angleDeg = 0, limitDeg = 0, posResidual = 0;
    uint64_t prevSeq = 0, prevSampleUs = 0, prevRefEpoch = 0, prevClockEpoch = 0;
    uint64_t prevMapId = 0, prevInitialKfId = 0, prevFrameId = 0;
    SnapVec3 prevP{}, prevV{}, prevMapT{};
    SnapQuat prevQ{}, prevMapQ{};
    int haveReference = 0, airborneFault = 0, published = 0;
    SnapQuat flightQ{};
    SnapVec3 origin{}, outputOrigin{};
    uint64_t resetCounter = 0;
    SnapVec3 outP{}, outV{};
    SnapQuat outQ{};
    int poseFrame = -1;
    std::string faultReason;

    // Only decision-side fields are copied. Submit-side fields are left intact,
    // which makes an already observed sample immutable after recording.
    void Overlay(const GateSnapshotRecord& d) {
        decision = d.decision; reason = d.reason;
        evalWallUs = d.evalWallUs; evalMonoUs = d.evalMonoUs; ageMs = d.ageMs;
        sent = d.sent; rejected = d.rejected;
        armed = d.armed; navState = d.navState; landed = d.landed;
        havePrevious = d.havePrevious; comparable = d.comparable;
        crossTransform = d.crossTransform;
        dt = d.dt; angleDeg = d.angleDeg; limitDeg = d.limitDeg;
        posResidual = d.posResidual;
        prevSeq = d.prevSeq; prevSampleUs = d.prevSampleUs;
        prevRefEpoch = d.prevRefEpoch; prevClockEpoch = d.prevClockEpoch;
        prevMapId = d.prevMapId; prevInitialKfId = d.prevInitialKfId;
        prevFrameId = d.prevFrameId;
        prevP = d.prevP; prevV = d.prevV; prevMapT = d.prevMapT;
        prevQ = d.prevQ; prevMapQ = d.prevMapQ;
        haveReference = d.haveReference; airborneFault = d.airborneFault;
        published = d.published; flightQ = d.flightQ;
        origin = d.origin; outputOrigin = d.outputOrigin;
        resetCounter = d.resetCounter;
        outP = d.outP; outV = d.outV; outQ = d.outQ;
        poseFrame = d.poseFrame; faultReason = d.faultReason;
    }
};

// 标记地图、参考身份或 mapToWorld 发生变化的样本对。
// 局部 BA 可能更新初始关键帧 ImuPose 和 mapToWorld，而不递增 referenceEpoch；
// 因此除 map/epoch 外还要核对变换本身。
inline bool GateSnapshotCrossTransform(const GateSnapshotRecord& r) {
    if(!r.havePrevious) return false;
    if(r.mapId != r.prevMapId || r.initialKfId != r.prevInitialKfId ||
       r.refEpoch != r.prevRefEpoch || r.clockEpoch != r.prevClockEpoch) return true;
    double dot = 0;
    for(int i=0;i<4;++i) dot += r.mapQ[i]*r.prevMapQ[i];
    if(std::abs(std::abs(dot)-1.0) > 1e-9) return true;   // q and -q are the same rotation
    double d2 = 0;
    for(int i=0;i<3;++i) {
        const double v = r.mapT[i]-r.prevMapT[i];
        d2 += v*v;
    }
    return d2 > 1e-12;
}

inline const char* GateSnapshotHeader() {
    return "seq,pair,submit_wall_us,submit_mono_us,src_sim_us,sample_us,map_offset_us,"
           "clock_epoch,ref_epoch,continuity,map_id,initial_kf_id,frame_id,"
           "src_px,src_py,src_pz,src_vx,src_vy,src_vz,src_qw,src_qx,src_qy,src_qz,"
           "raw_px,raw_py,raw_pz,raw_qw,raw_qx,raw_qy,raw_qz,"
           "map_qw,map_qx,map_qy,map_qz,map_tx,map_ty,map_tz,"
           "init_qw,init_qx,init_qy,init_qz,init_tx,init_ty,init_tz,"
           "valid,imu_init,ba2,vel_valid,"
           "gyro_ix,gyro_iy,gyro_iz,gyro_span_s,imu_count,"
           "decision,reason,eval_wall_us,eval_mono_us,age_ms,sent,rejected,armed,nav_state,landed,"
           "have_previous,comparable,cross_transform,dt,angle_deg,limit_deg,pos_residual,"
           "prev_seq,prev_sample_us,prev_ref_epoch,prev_clock_epoch,prev_map_id,prev_initial_kf_id,prev_frame_id,"
           "prev_px,prev_py,prev_pz,prev_vx,prev_vy,prev_vz,prev_qw,prev_qx,prev_qy,prev_qz,"
           "prev_map_qw,prev_map_qx,prev_map_qy,prev_map_qz,prev_map_tx,prev_map_ty,prev_map_tz,"
           "have_reference,airborne_fault,published,flight_qw,flight_qx,flight_qy,flight_qz,"
           "origin_x,origin_y,origin_z,output_origin_x,output_origin_y,output_origin_z,reset_counter,"
           "out_px,out_py,out_pz,out_vx,out_vy,out_vz,out_qw,out_qx,out_qy,out_qz,pose_frame,fault_reason";
}

namespace detail {
template<class O> inline void Emit3(O& o, const SnapVec3& v) {
    o << ',' << v[0] << ',' << v[1] << ',' << v[2];
}
template<class O> inline void Emit4(O& o, const SnapQuat& q) {
    o << ',' << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3];
}
}

inline void WriteGateSnapshotRow(std::ostream& o, const GateSnapshotRecord& r) {
    // Forensic replay needs full double precision; the default 6 significant
    // digits would silently corrupt positions, residuals and angles.
    o << std::setprecision(12);
    o << r.seq << ',' << r.pair << ',' << r.submitWallUs << ',' << r.submitMonoUs
      << ',' << r.srcSimUs << ',' << r.sampleUs << ',' << r.mapOffsetUs
      << ',' << r.clockEpoch << ',' << r.refEpoch << ',' << r.continuity
      << ',' << r.mapId << ',' << r.initialKfId << ',' << r.frameId;
    detail::Emit3(o,r.srcP); detail::Emit3(o,r.srcV); detail::Emit4(o,r.srcQ);
    detail::Emit3(o,r.rawP); detail::Emit4(o,r.rawQ);
    detail::Emit4(o,r.mapQ); detail::Emit3(o,r.mapT);
    detail::Emit4(o,r.initQ); detail::Emit3(o,r.initT);
    o << ',' << r.valid << ',' << r.imuInit << ',' << r.ba2 << ',' << r.velValid;
    detail::Emit3(o,r.gyroIntegral);
    o << ',' << r.gyroSpanS << ',' << r.imuCount
      << ',' << r.decision << ',' << r.reason << ',' << r.evalWallUs << ',' << r.evalMonoUs
      << ',' << r.ageMs << ',' << r.sent << ',' << r.rejected
      << ',' << r.armed << ',' << r.navState << ',' << r.landed
      << ',' << r.havePrevious << ',' << r.comparable << ',' << r.crossTransform
      << ',' << r.dt << ',' << r.angleDeg << ',' << r.limitDeg << ',' << r.posResidual
      << ',' << r.prevSeq << ',' << r.prevSampleUs << ',' << r.prevRefEpoch << ',' << r.prevClockEpoch
      << ',' << r.prevMapId << ',' << r.prevInitialKfId << ',' << r.prevFrameId;
    detail::Emit3(o,r.prevP); detail::Emit3(o,r.prevV); detail::Emit4(o,r.prevQ);
    detail::Emit4(o,r.prevMapQ); detail::Emit3(o,r.prevMapT);
    o << ',' << r.haveReference << ',' << r.airborneFault << ',' << r.published;
    detail::Emit4(o,r.flightQ);
    detail::Emit3(o,r.origin); detail::Emit3(o,r.outputOrigin);
    o << ',' << r.resetCounter;
    detail::Emit3(o,r.outP); detail::Emit3(o,r.outV); detail::Emit4(o,r.outQ);
    o << ',' << r.poseFrame << ',' << r.faultReason << '\n';
}

class GateSnapshotTrace {
public:
    explicit GateSnapshotTrace(const std::string& path, size_t cap = 4096)
        : path_(path), cap_(cap), ring_(cap), out_(path, std::ios::out | std::ios::trunc) {
        if(out_) out_ << GateSnapshotHeader() << '\n';
        win_.reserve(2*WIN_HALF+1);
    }

    // Called on the submitting thread, under Bridge::Impl::mutex. No file IO.
    void Add(const GateSnapshotRecord& r) {
        std::lock_guard<std::mutex> lk(mu_);
        ring_[r.seq % cap_] = r;              // full value copy
        if(r.seq > seq_) seq_ = r.seq;
        ++nAdd_;
        if(r.seq % cap_ == 0) ++nWrap_;
    }

    // Called on the Bridge worker thread immediately after Gate::Evaluate.
    void Mark(uint64_t seq, const GateSnapshotRecord& d) {
        std::lock_guard<std::mutex> lk(mu_);
        if(seq == 0 || seq + cap_ <= seq_) { ++nLostMark_; return; }
        GateSnapshotRecord& r = ring_[seq % cap_];
        if(r.seq != seq) { ++nLostMark_; return; }
        r.Overlay(d);
        // Rolling pre-window: keep the last WIN_HALF finalised records so the
        // first reject can be analysed with real BEFORE context, not only after.
        if(d.decision != 0 && d.decision != 4) {
            recent_.push_back(r);
            while(recent_.size() > WIN_HALF) recent_.pop_front();
        }
        const bool first = (d.decision == 2 && firstRejectSeq_ == 0);
        if(d.decision == 2) {
            ++nReject_;
            if(firstRejectSeq_ == 0) {
                firstRejectSeq_ = seq;
                win_.assign(recent_.begin(),recent_.end());
            }
        }
        if(firstRejectSeq_ != 0 && seq >= firstRejectSeq_ &&
           seq <= firstRejectSeq_ + WIN_HALF && win_.size() < 2*WIN_HALF+1 &&
           !first) {
            win_.push_back(r);
        }
    }

    // Batched write. Caller holds no locks; one flush per batch, never per frame.
    void Flush(bool force) {
        std::vector<GateSnapshotRecord> snap, wsnap;
        {
            std::lock_guard<std::mutex> lk(mu_);
            if(!force && seq_ - flushedSeq_ < FLUSH_EVERY) return;
            uint64_t upto = flushedSeq_;
            const uint64_t from = seq_ + 1 > cap_ ? seq_ + 1 - cap_ : 1;
            uint64_t begin = flushedSeq_ + 1;
            if(begin < from) begin = from;
            for(uint64_t s = begin; s <= seq_; ++s) {
                const GateSnapshotRecord& r = ring_[s % cap_];
                if(r.seq != s) { upto = s; continue; }   // ring already wrapped
                if(!force && (r.decision == 0 || r.decision == 4)) break; // not finalised yet
                snap.push_back(r);                       // immutable value copy
                upto = s;
            }
            wsnap = win_;
            flushedSeq_ = upto;
            ++nFlush_;
        }
        if(out_) {
            for(const auto& r : snap) WriteGateSnapshotRow(out_,r);
            out_.flush();                                 // one flush per BATCH
        }
        if(force && !wsnap.empty()) {
            std::ofstream w(path_ + ".first_reject_window.csv", std::ios::out | std::ios::trunc);
            if(w) {
                w << GateSnapshotHeader() << '\n';
                for(const auto& r : wsnap) WriteGateSnapshotRow(w,r);
            }
        }
    }

    void Counters(std::string* s) const {
        char b[512];
        std::snprintf(b,sizeof(b),
            "GATE_SNAPSHOT add=%llu reject=%llu ring_wrap=%llu lost_mark=%llu "
            "flushes=%llu first_reject_seq=%llu flushed_seq=%llu",
            (unsigned long long)nAdd_.load(),(unsigned long long)nReject_.load(),
            (unsigned long long)nWrap_.load(),(unsigned long long)nLostMark_.load(),
            (unsigned long long)nFlush_.load(),(unsigned long long)firstRejectSeq_,
            (unsigned long long)flushedSeq_);
        *s = b;
    }

    ~GateSnapshotTrace() { Flush(true); }

private:
    // [NP-01 FIX13] A short run may never reach a second 1024-frame
    // batch. 256 keeps batched IO (never per-frame) while bounding
    // the loss if shutdown aborts before the destructor's force flush.
    static constexpr uint64_t FLUSH_EVERY = 256;
    static constexpr uint64_t WIN_HALF = 128;
    std::string path_;
    size_t cap_;
    std::vector<GateSnapshotRecord> ring_;
    std::vector<GateSnapshotRecord> win_;
    std::deque<GateSnapshotRecord> recent_;
    std::ofstream out_;
    std::mutex mu_;
    uint64_t seq_ = 0, flushedSeq_ = 0, firstRejectSeq_ = 0;
    std::atomic<uint64_t> nAdd_{0}, nReject_{0}, nWrap_{0}, nLostMark_{0}, nFlush_{0};
};

}} // namespace ORB_SLAM3::px4
