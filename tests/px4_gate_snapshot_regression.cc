// [NP-01 FIX11] Offline validation of the consistent Gate snapshot chain.
// NO ROS, NO simulation, NO truth in the control path, NO threshold change.
// Links only against orbslam3_px4_core (Px4Vio.cc) + the header-only trace.
#include "Px4GateSnapshot.h"
#include "Px4Vio.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

using namespace ORB_SLAM3::px4;

static constexpr double kPi = 3.14159265358979323846;
static int g_assertions = 0;
static void require(bool ok, const char* what) {
    ++g_assertions;
    if(!ok) throw std::runtime_error(std::string("ASSERT FAILED: ")+what);
}
static void near(double a, double b, double eps, const char* what) {
    ++g_assertions;
    if(!(std::abs(a-b) <= eps))
        throw std::runtime_error(std::string("ASSERT FAILED: ")+what);
}
static SnapQuat Q(const Eigen::Matrix3d& R) {
    const Eigen::Quaterniond q(R);
    return SnapQuat{q.w(),q.x(),q.y(),q.z()};
}
static SnapVec3 V(const Eigen::Vector3d& v) {
    return SnapVec3{v.x(),v.y(),v.z()};
}
static std::vector<std::vector<std::string>> ReadCsv(const std::string& path) {
    std::ifstream f(path);
    std::vector<std::vector<std::string>> out;
    std::string line;
    while(std::getline(f,line)) {
        std::vector<std::string> row;
        std::stringstream ss(line);
        std::string cell;
        while(std::getline(ss,cell,',')) row.push_back(cell);
        out.push_back(row);
    }
    return out;
}
static std::string Col(const std::vector<std::string>& hdr,
                       const std::vector<std::string>& row, const char* name) {
    for(size_t i=0;i<hdr.size();++i)
        if(hdr[i]==name) return i<row.size()?row[i]:std::string();
    throw std::runtime_error(std::string("missing column ")+name);
}
static double Num(const std::vector<std::string>& hdr,
                  const std::vector<std::string>& row, const char* name) {
    return std::stod(Col(hdr,row,name));
}

struct Run {
    uint64_t wall=1700000000000000ULL, mono=1000000;
    Sample s;
    Telemetry t;
    Gate gate;
    explicit Run(const Config& c):gate(c) {
        s.valid=s.imuInitialized=s.ba2=s.velocityValid=true;
        s.gravity=Eigen::Vector3d(0,0,-9.81);
        s.imuToWorld=Eigen::Matrix3d::Identity();
        s.mapToWorld=Eigen::Matrix3d::Identity();
        s.referenceEpoch=1;
        s.clockEpoch=0;
        t.haveStatus=t.haveSync=t.haveReader=true;
    }
    std::optional<Observation> step(uint64_t dt=33333) {
        wall+=dt;mono+=dt;++s.sequence;s.sampleUs=wall-10000;
        t.statusSteadyUs=t.syncSteadyUs=mono;
        return gate.Evaluate(s,t,wall,mono);
    }
};

static Config BaseConfig() {
    Config c;
    c.extrinsicsValid=true;          // do not change any Gate threshold / limit
    return c;
}

// (1) A FIXED rigid transform must not manufacture any Gate residual. Both the
// position/velocity pair and the attitude pair are expressed through the same
// constant R,t, so the Gate must see angle ~= 0 and posResidual ~= 0.
static void TestFixedRigidTransform() {
    Run r(BaseConfig());
    const Eigen::Matrix3d R =
        (Eigen::AngleAxisd(0.4,Eigen::Vector3d::UnitZ())*
         Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitY())).toRotationMatrix();
    const Eigen::Vector3d t(1.0,2.0,3.0);
    const Eigen::Vector3d vLocal(0.1,-0.05,0.02);
    r.s.imuToWorld=R;
    r.s.mapToWorld=R;
    r.s.mapTranslation=t;
    r.s.mapId=7;r.s.initialKeyFrameId=3;r.s.frameId=100;
    r.s.velocity=R*vLocal;
    Eigen::Vector3d pLocal(0.2,0.3,0.4);
    for(int i=0;i<6;++i) {
        r.s.position=R*pLocal+t;
        r.step();
        pLocal+=vLocal*(33333.0/1e6);   // exactly the Gate's dt for this step
        if(i>=1) {
            const GateDiagnostic d=r.gate.Diagnostic();
            require(d.comparison.havePair,"fixed transform: Gate comparison missing");
            require(d.comparison.comparable,"fixed transform: comparison not comparable");
            near(d.comparison.angleDeg,0.0,1e-6,"fixed transform creates attitude residual");
            near(d.comparison.posResidual,0.0,1e-9,"fixed transform creates position residual");
            require(d.comparison.prevSeq==r.s.sequence-1,"fixed transform: wrong prev seq");
            require(d.comparison.prevSampleUs<r.s.sampleUs,"fixed transform: wrong prev sample time");
        }
    }
    require(r.gate.Reason()=="stabilizing" || r.gate.Reason()=="publishing",
            "fixed transform: unexpected Gate reason");
}

// (2) Cross-transform / cross-epoch comparison must be EXPLICITLY identified,
// including the code-level gap: same referenceEpoch but changed mapToWorld.
static void TestCrossTransform() {
    const Eigen::Matrix3d R1=Eigen::AngleAxisd(0.3,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d R2=Eigen::AngleAxisd(0.9,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    GateSnapshotRecord cur;
    cur.havePrevious=1;
    cur.mapId=7;cur.initialKfId=3;cur.refEpoch=1;cur.clockEpoch=0;
    cur.prevMapId=7;cur.prevInitialKfId=3;cur.prevRefEpoch=1;cur.prevClockEpoch=0;
    cur.mapQ=Q(R1);cur.mapT=SnapVec3{1,2,3};
    cur.prevMapQ=Q(R1);cur.prevMapT=SnapVec3{1,2,3};
    require(!GateSnapshotCrossTransform(cur),"identical transform flagged as cross-transform");
    // q and -q denote the SAME rotation; must not be a false positive.
    for(int i=0;i<4;++i) cur.prevMapQ[i]=-cur.prevMapQ[i];
    require(!GateSnapshotCrossTransform(cur),"quaternion sign flip is a false cross-transform");
    for(int i=0;i<4;++i) cur.prevMapQ[i]=-cur.prevMapQ[i];
    cur.prevMapQ=Q(R2);
    require(GateSnapshotCrossTransform(cur),"changed mapToWorld with same epoch not identified");
    cur.prevMapQ=cur.mapQ;
    cur.prevMapT=SnapVec3{1,2,3.5};
    require(GateSnapshotCrossTransform(cur),"changed map translation not identified");
    cur.prevMapT=cur.mapT;
    cur.prevRefEpoch=2;
    require(GateSnapshotCrossTransform(cur),"reference epoch change not identified");
    cur.prevRefEpoch=1;
    cur.prevMapId=8;
    require(GateSnapshotCrossTransform(cur),"map identity change not identified");
}

static std::string TmpPath(const char* suffix) {
    return std::string("/tmp/px4_gate_snapshot_regression_")+
           std::to_string((long long)getpid())+suffix;
}

// (3) Saved samples are immutable VALUE copies; a rejected sample can be fully
// replayed from the CSV (submit side + Gate comparison + decision + reason).
static void TestImmutableAndReplayableReject() {
    // Gate::previous_ is a Sample value member, not a shared mutable object.
    Sample a;a.position=Eigen::Vector3d(1,2,3);a.imuToWorld=Eigen::Matrix3d::Identity();
    Sample b=a;
    a.position.x()=999.0;
    near(b.position.x(),1.0,1e-12,"Sample copy is not a value copy (previous_ aliasing)");

    // A real Gate reject with a 30 deg attitude step (limit ~16 deg at dt=33ms).
    Run bad(BaseConfig());
    const Eigen::Matrix3d R0=Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R1=Eigen::AngleAxisd(30.0*kPi/180.0,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    bad.s.imuToWorld=R0;
    bad.s.position=Eigen::Vector3d(0.5,0.6,0.7);
    bad.s.velocity=Eigen::Vector3d(0.01,0.02,0.0);
    bad.s.mapId=7;bad.s.initialKeyFrameId=3;bad.s.frameId=100;
    bad.step();                                  // establishes previous_ as a value copy
    const Sample before=bad.s;
    bad.s.imuToWorld=R1;
    bad.s.position+=bad.s.velocity*(1.0/30.0);
    const std::optional<Observation> out=bad.step();
    require(!out,"30 deg attitude step should be rejected");
    require(bad.gate.Reason()=="attitude_discontinuity","wrong rejection reason");
    const GateDiagnostic d=bad.gate.Diagnostic();
    require(d.comparison.havePair && d.comparison.comparable,"reject: comparison not captured");
    require(d.comparison.angleDeg>d.comparison.limitDeg,"reject: angle/limit not captured");
    near(d.comparison.angleDeg,30.0,1e-6,"reject: angle_deg mismatch");
    require(d.comparison.prevSeq==before.sequence,"reject: prev seq not value-copied before previous_=s");
    require(d.comparison.prevSampleUs==before.sampleUs,"reject: prev sample time incorrect");
    near(d.comparison.prevPosition.x(),before.position.x(),1e-12,"reject: prev position mutated");
    near(d.comparison.prevRotation(0,0),before.imuToWorld(0,0),1e-12,"reject: prev rotation mutated");

    const std::string path=TmpPath("_a.csv");
    {
        GateSnapshotTrace tr(path,64);
        GateSnapshotRecord r;
        r.seq=bad.s.sequence;r.pair=bad.s.sequence;
        r.srcSimUs=65000000;r.mapOffsetUs=1000000;r.sampleUs=bad.s.sampleUs;
        r.clockEpoch=bad.s.clockEpoch;r.refEpoch=bad.s.referenceEpoch;
        r.mapId=7;r.initialKfId=3;r.frameId=bad.s.frameId;
        r.srcP=V(bad.s.position);r.srcV=V(bad.s.velocity);r.srcQ=Q(bad.s.imuToWorld);
        r.mapQ=Q(bad.s.mapToWorld);r.mapT=V(bad.s.mapTranslation);
        r.valid=1;r.imuInit=1;r.ba2=1;r.velValid=1;
        tr.Add(r);
        GateSnapshotRecord dec;
        dec.decision=2;dec.reason=bad.gate.Reason();
        dec.havePrevious=1;dec.comparable=1;
        dec.dt=d.comparison.dt;dec.angleDeg=d.comparison.angleDeg;
        dec.limitDeg=d.comparison.limitDeg;dec.posResidual=d.comparison.posResidual;
        dec.prevSeq=d.comparison.prevSeq;dec.prevSampleUs=d.comparison.prevSampleUs;
        dec.prevP=V(d.comparison.prevPosition);dec.prevV=V(d.comparison.prevVelocity);
        dec.prevQ=Q(d.comparison.prevRotation);
        dec.prevMapQ=Q(d.comparison.prevMapToWorld);dec.prevMapT=V(d.comparison.prevMapTranslation);
        dec.prevMapId=d.comparison.prevMapId;dec.prevInitialKfId=d.comparison.prevInitialKeyFrameId;
        dec.prevRefEpoch=d.comparison.prevRefEpoch;dec.prevClockEpoch=d.comparison.prevClockEpoch;
        // Deliberately different submit-side values: Overlay must NOT touch them.
        dec.srcP=SnapVec3{77,77,77};dec.sampleUs=999;
        tr.Mark(r.seq,dec);
        tr.Flush(true);
    }
    const auto csv=ReadCsv(path);
    require(csv.size()>=2,"reject replay: CSV missing rows");
    const auto& hdr=csv[0];
    const auto& row=csv[1];
    require(Col(hdr,row,"decision")=="2","reject replay: decision not recorded");
    require(Col(hdr,row,"reason")=="attitude_discontinuity","reject replay: reason not recorded");
    near(Num(hdr,row,"angle_deg"),30.0,1e-6,"reject replay: angle_deg not replayable");
    require(Num(hdr,row,"limit_deg")>0,"reject replay: limit_deg missing");
    near(Num(hdr,row,"prev_px"),before.position.x(),1e-9,"reject replay: prev position missing");
    near(Num(hdr,row,"src_px"),bad.s.position.x(),1e-9,"reject replay: submit-side sample was mutated");
    require(Num(hdr,row,"sample_us")==(double)bad.s.sampleUs,
            "reject replay: submit-side sample_us was mutated");
    std::remove(path.c_str());
    std::remove((path+".first_reject_window.csv").c_str());
}

// (5) seq / pair / source-time / sample-time association across a batch.
static void TestSeqTimeAssociation() {
    const std::string path=TmpPath("_b.csv");
    const int N=48;
    {
        GateSnapshotTrace tr(path,128);
        for(int i=1;i<=N;++i) {
            GateSnapshotRecord r;
            r.seq=(uint64_t)i;r.pair=(uint64_t)i;
            r.submitWallUs=1700000000000000ULL+(uint64_t)i*33333ULL;
            r.submitMonoUs=(uint64_t)i*33333ULL;
            r.srcSimUs=(int64_t)65000000+(int64_t)i*32000;
            r.mapOffsetUs=1794724277402255LL;
            r.sampleUs=(uint64_t)(r.srcSimUs+r.mapOffsetUs);
            r.clockEpoch=0;r.refEpoch=1;r.continuity=0;
            r.mapId=7;r.initialKfId=3;r.frameId=(uint64_t)(100+i);
            r.mapQ=SnapQuat{1,0,0,0};
            r.valid=1;r.imuInit=1;r.ba2=1;r.velValid=1;
            tr.Add(r);
            GateSnapshotRecord d;
            d.decision=1;d.reason="publishing";d.sent=(uint64_t)i;
            d.evalWallUs=r.submitWallUs+1000;
            tr.Mark(r.seq,d);
        }
        tr.Flush(true);
    }
    const auto csv=ReadCsv(path);
    require((int)csv.size()==N+1,"seq association: wrong row count");
    const auto& hdr=csv[0];
    for(int i=1;i<=N;++i) {
        const auto& row=csv[i];
        require(Num(hdr,row,"seq")==(double)i,"seq association: seq mismatch");
        require(Num(hdr,row,"pair")==(double)i,"pair association: pair mismatch");
        const double src=Num(hdr,row,"src_sim_us");
        const double off=Num(hdr,row,"map_offset_us");
        require(Num(hdr,row,"sample_us")==src+off,
                "time association: sample_us != src_sim_us + map_offset_us");
        require(Col(hdr,row,"reason")=="publishing","decision association: reason mismatch");
    }
    std::remove(path.c_str());
    std::remove((path+".first_reject_window.csv").c_str());
}

int main() {
    try {
        TestFixedRigidTransform();
        TestCrossTransform();
        TestImmutableAndReplayableReject();
        TestSeqTimeAssociation();
        std::cout<<"px4_gate_snapshot_regression PASS assertions="<<g_assertions<<std::endl;
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"px4_gate_snapshot_regression FAIL after "<<g_assertions
                 <<" assertions: "<<e.what()<<std::endl;
        return 1;
    }
}
