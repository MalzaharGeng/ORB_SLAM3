// NP-01: executable frame-contract verification for the REAL publication path.
//
// It calls Gate::Evaluate() (the exact code that produces Observation, which
// Px4Bridge serialises into px4_msgs::VehicleOdometry) and checks closed-form
// expectations for: static hold, pure translation, a known NED heading, and a
// known body rotation. It intentionally does NOT read /slam/pose camera Euler
// angles: those are a different representation (camera/world) and must never be
// used to judge what PX4 receives.
//
// No gate limit, threshold, or acceptance criterion is modified here. The Gate
// config below only widens the *synthetic input* envelope so the constructed
// motions are not rejected; the real config is loaded unchanged in production.
#include "Px4Vio.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace ORB_SLAM3::px4;

static void require(bool ok, const std::string& what) {
    if (!ok) throw std::runtime_error(what);
}
static void near(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                 const std::string& what, double eps = 1e-7) {
    require((a - b).norm() < eps, what + " got (" + std::to_string(a.x()) + "," +
            std::to_string(a.y()) + "," + std::to_string(a.z()) + ") expected (" +
            std::to_string(b.x()) + "," + std::to_string(b.y()) + "," +
            std::to_string(b.z()) + ")");
}
static void nearAngle(double a, double b, const std::string& what, double eps = 1e-6) {
    double d = std::atan2(std::sin(a - b), std::cos(a - b));
    require(std::abs(d) < eps, what + " got " + std::to_string(a) + " expected " + std::to_string(b));
}
static double yawOf(const Eigen::Quaterniond& q) {
    // NED heading: body +x direction expressed in the local NED frame.
    const Eigen::Vector3d forward = q * Eigen::Vector3d::UnitX();
    return std::atan2(forward.y(), forward.x());
}
static const Eigen::Matrix3d R_BI =
    (Eigen::Matrix3d() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished();  // IMU FLU -> body FRD
static Eigen::Matrix3d rz(double a) {
    return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
// SLAM "world" is the gravity-aligned initial IMU frame (FLU, z up).
// PX4 local frame is NED (x north, y east, z down). With Gazebo x=east,
// y=north, the world->NED change of basis is C = [[0,1,0],[1,0,0],[0,0,-1]].
static const Eigen::Matrix3d C_ENU_TO_NED =
    (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();

struct Fixture {
    Config c;
    Gate gate;
    Sample s;
    Telemetry t;
    uint64_t wall = 1700000000000000ULL, mono = 1000000;
    double heading = 0.;
    Eigen::Vector3d localOrigin = Eigen::Vector3d::Zero();
    explicit Fixture(double h, Eigen::Vector3d origin)
        : gate(Config()), heading(h), localOrigin(origin) {
        c.extrinsicsValid = true;
        c.alignToPx4Ned = true;
        c.bodyFromImu = R_BI;
        // Synthetic-input envelope only. Production Gate limits are untouched.
        c.maxSpeed = 20.;
        c.maxPositionResidual = 5.;
        c.consistencyWindowSeconds = 10.;
        gate = Gate(c);
        s.valid = s.imuInitialized = s.ba2 = s.velocityValid = true;
        s.gravity = Eigen::Vector3d(0, 0, -9.81);
        // ORB's imuToWorld is IMU(FLU) -> SLAM world (the gravity-aligned
        // initial IMU frame). It is NOT the IMU->body(FRD) mounting R_BI;
        // Px4Vio applies bodyFromImu exactly once. Setting it to R_BI here is
        // the "duplicated extrinsic" fault checked in Case 6.
        s.imuToWorld = Eigen::Matrix3d::Identity();
        s.position = Eigen::Vector3d(0, 0, 0);
        s.velocity = Eigen::Vector3d::Zero();
        t.haveStatus = t.haveSync = t.haveReader = true;
        t.haveLocalFrame = t.localXYValid = t.localZValid = true;
        t.localHeading = h;
        t.localPosition = origin;
        t.navState = 1;                         // ALTCTL
        t.armed = false;
    }
    std::optional<Observation> step(uint64_t dt = 33333) {
        wall += dt; mono += dt;
        ++s.sequence;
        s.sampleUs = wall - 10000;
        t.statusSteadyUs = t.syncSteadyUs = t.localSteadyUs = mono;
        t.localSampleUs = s.sampleUs;
        return gate.Evaluate(s, t, wall, mono);
    }
    Observation ready() {
        for (int i = 0; i < 60; ++i)
            if (auto o = step()) return *o;
        throw std::runtime_error("never published: " + gate.Reason());
    }
};

int main() {
    int checks = 0;
    try {
        // ---- Case 1: STATIC. No motion, no rotation, fixed reference. ----
        {
            const double h = 0.4;
            const Eigen::Vector3d origin(1.25, -0.5, -1.1);
            Fixture f(h, origin);
            Observation first = f.ready();
            near(first.position, origin, "static: EV position = PX4 local origin");
            near(first.velocity, Eigen::Vector3d::Zero(), "static: EV velocity = 0");
            require(first.nedFrame, "static: pose/velocity frame must be NED");
            require(first.resetCounter == 0, "static: initial reset_counter must be 0");
            nearAngle(yawOf(first.attitude), h, "static: EV yaw = PX4 local heading");
            ++checks;
            // Attitude quaternion must be a pure NED-down rotation, [w,x,y,z]
            // produced by Eigen and serialised in that order by Px4Bridge.
            const Eigen::Quaterniond expected(rz(h));
            require(first.attitude.angularDistance(expected) < 1e-7,
                    "static: attitude quaternion must equal Rz(heading)");
            ++checks;
            // Holding still for more frames must not move the output or re-anchor.
            for (int i = 0; i < 5; ++i) {
                auto o = f.step();
                require(bool(o), "static: publication must continue");
                near(o->position, origin, "static: position must stay put");
                near(o->velocity, Eigen::Vector3d::Zero(), "static: velocity must stay zero");
                nearAngle(yawOf(o->attitude), h, "static: yaw must stay put");
                require(o->resetCounter == 0, "static: no silent re-anchor");
            }
            ++checks;
        }

        // ---- Case 2: PURE TRANSLATION. Same fixed transform for p, v and q. ----
        {
            const double h = 0.9;
            const Eigen::Vector3d origin(-2., 3., -0.8);
            Fixture f(h, origin);
            f.ready();  // establish the reference; Case 1 checks the static output
            // F = Rz(h) * R_BI for a level initial attitude (F0 rows are the
            // projected forward / down x forward / down basis).
            const Eigen::Matrix3d F = rz(h) * R_BI;
            const Eigen::Vector3d vWorld(0.6, -0.2, 0.1);
            const uint64_t dtUs = 33333;
            f.s.velocity = vWorld;
            f.s.position = Eigen::Vector3d::Zero();
            auto o1 = f.step(dtUs);
            require(bool(o1), "translation: rejected a valid small motion");
            const Eigen::Vector3d vNed = F * vWorld;
            near(o1->velocity, vNed, "translation: EV velocity = F * world velocity");
            nearAngle(yawOf(o1->attitude), h, "translation: attitude unchanged by pure translation");
            ++checks;
            // Position increment must equal the velocity integral in the SAME frame.
            const Eigen::Vector3d dWorld = vWorld * (double(dtUs) * 1e-6);
            f.s.position += dWorld;
            auto o2 = f.step(dtUs);
            require(bool(o2), "translation: second step rejected");
            near(o2->position - o1->position, vNed * (double(dtUs) * 1e-6),
                 "translation: position derivative equals published velocity", 1e-6);
            near(o2->position, origin + F * f.s.position,
                 "translation: EV position = origin + F * (slam position - slam origin)");
            ++checks;
            // Independent cross-check: with the nose initially along Gazebo +x
            // (East), PX4 local heading is +90 deg, and the constructed fixed
            // transform F must equal the ENU->NED change of basis C exactly.
            // This is the matrix used to compare published EV velocity against
            // truth.csv velocity in the offline causality probe.
            const Eigen::Matrix3d F_at_90 = rz(M_PI / 2) * R_BI;
            require((F_at_90 - C_ENU_TO_NED).norm() < 1e-9,
                    "translation: F at heading 90deg must equal C(ENU->NED)");
            ++checks;
        }

        // ---- Case 3: KNOWN YAW. Headings 0, +90, -90, 180 deg. ----
        {
            for (double h : {0.0, M_PI / 2, -M_PI / 2, M_PI - 0.01}) {
                Fixture f(h, Eigen::Vector3d::Zero());
                Observation o = f.ready();
                nearAngle(yawOf(o.attitude), h, "known yaw " + std::to_string(h));
                require(std::abs(o.attitude.norm() - 1.) < 1e-9, "known yaw: unit quaternion");
                // q must be [w,x,y,z] of Rz(h): w=cos(h/2), z=sin(h/2), x=y=0.
                require(std::abs(o.attitude.w() - std::cos(h / 2)) < 1e-7 &&
                        std::abs(o.attitude.z() - std::sin(h / 2)) < 1e-7 &&
                        std::abs(o.attitude.x()) < 1e-7 && std::abs(o.attitude.y()) < 1e-7,
                        "known yaw: quaternion component order / sign");
            }
            ++checks;
        }

        // ---- Case 4: KNOWN BODY ROTATION after the reference is fixed. ----
        // A rotation of +psi about world +z (up) is -psi about NED +z (down),
        // so the published NED heading must become h - psi. A left/right
        // multiplication error, a doubled R_BI, or an ad-hoc +90 deg patch
        // all fail this check.
        {
            const double h = 0.3, psi = 30. * M_PI / 180.;
            Fixture f(h, Eigen::Vector3d::Zero());
            Observation first = f.ready();
            nearAngle(yawOf(first.attitude), h, "rotation: initial yaw");
            const int n = 6;
            for (int i = 1; i <= n; ++i) {
                const double a = psi * i / n;
                f.s.imuToWorld = rz(a);          // 5 deg per step, inside Gate's angular envelope
                auto o = f.step();
                require(bool(o), "rotation: valid rotation rejected at step " + std::to_string(i));
                nearAngle(yawOf(o->attitude), h - a,
                          "rotation: EV yaw must be h - psi (NED down-axis sign)", 1e-5);
                // F is fixed: position/velocity must NOT be rotated again.
                near(o->position, Eigen::Vector3d::Zero(), "rotation: no position re-anchor");
                near(o->velocity, Eigen::Vector3d::Zero(), "rotation: no spurious velocity");
            }
            ++checks;
            // A single 25 deg jump must be rejected by the unchanged Gate
            // (fault input regression: no smoothing, no threshold relaxation).
            f.s.imuToWorld = rz(psi + 25. * M_PI / 180.);
            require(!f.step(), "rotation: oversized attitude step must be rejected");
            ++checks;
        }

        // ---- Case 6: FAULT INPUT - applying R_BI twice. ----
        // If ORB's imuToWorld were (wrongly) pre-multiplied by R_BI, the
        // published attitude picks up an extra 180 deg roll while its NED
        // heading still looks plausible. This is exactly why /slam/pose Euler
        // angles must not be used to "fix" a suspected 90 deg error.
        {
            Fixture bad(0.4, Eigen::Vector3d::Zero());
            bad.s.imuToWorld = R_BI;   // duplicated extrinsic
            Observation o = bad.ready();
            nearAngle(yawOf(o.attitude), 0.4, "duplicate R_BI: yaw still looks right");
            require(o.attitude.angularDistance(Eigen::Quaterniond(rz(0.4))) > 1.0,
                    "duplicate R_BI: must NOT be a pure NED yaw rotation");
            ++checks;
        }

        // ---- Case 5: frame flags and covariance are present and finite. ----
        {
            Fixture f(0.2, Eigen::Vector3d::Zero());
            Observation o = f.ready();
            require(o.nedFrame, "frame: NED flag");
            require(std::isfinite(o.positionVariance) && o.positionVariance > 0, "frame: pos var");
            require(std::isfinite(o.velocityVariance) && o.velocityVariance > 0, "frame: vel var");
            require(std::isfinite(o.orientationVariance) && o.orientationVariance > 0, "frame: ori var");
            require(o.sampleUs == f.s.sampleUs, "frame: sample timestamp is the SOURCE time");
            require(o.timestampUs == f.wall, "frame: message timestamp is the publish time");
            ++checks;
        }

        std::printf("PX4_VIO_FRAME_CONTRACT_OK checks=%d\n", checks);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "PX4_VIO_FRAME_CONTRACT_FAIL: " << e.what() << std::endl;
        return 1;
    }
}
