// Standalone Monte-Carlo test of DualESKFFilter vs the existing MEKFFilter.
//
// Build (from repo root):
//   g++ -O2 -std=c++17 -Iinclude -Isim sim/test_dual_eskf.cpp sim/mekf_signfix.cpp
//       src/filter/dual_eskf_filter.cpp src/filter/mekf_filter.cpp
//       src/quaternion.cpp src/math3d.cpp -o sim/test_dual_eskf.exe
//
// Run:
//   sim/test_dual_eskf.exe [numSeeds=10] [csvPath=sim/dual_eskf_run.csv]
//
// Scenario: UGV-like yaw motion (straight / turn segments, stops every
// 2-3 min, gentle terrain roll/pitch), 30 min at 200 Hz. Two IMUs at the same
// point (no lever arm). Each IMU's gyro bias (sensor frame) =
//   turn-on bias + random walk + shared temperature-like warm-up term c(t)*s
// where s is identical in each sensor's OWN frame. IMU2 is mounted rotated
// (180 deg about Z, or 180 deg about X), data is generated in the sensor
// frame and rotated back to body like the pipeline does.

#include "filter/dual_eskf_filter.h"
#include "filter/mekf_filter.h"
#include "mekf_signfix.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace
{

constexpr double PI      = 3.14159265358979323846;
constexpr double DEG     = PI / 180.0;
constexpr double GRAVITY = 9.80665;

// ---------------------------------------------------------------- config
constexpr double DT          = 1.0 / 200.0;
constexpr double DURATION    = 30.0 * 60.0;

constexpr double ARW         = 0.30 * DEG / 60.0;   // rad/sqrt(s)  (0.30 deg/sqrt(h))
constexpr double BIAS_RW     = 5.0e-6;              // rad/s/sqrt(s)
constexpr double BIAS0_STD   = 0.10 * DEG;          // rad/s, turn-on bias per axis
constexpr double SHARED_STD  = 0.10 * DEG;          // rad/s, shared warm-up amplitude per axis
constexpr double SHARED_TAU  = 300.0;               // s
constexpr double ACC_NOISE   = 0.03;                // m/s^2 per sample
constexpr double MAG_NOISE   = 0.3;                 // uT per sample
constexpr double ZARU_MARGIN = 0.5;                 // s, detector latency at both ends

const double MAG_WORLD[3] = { 20.0, 0.0, -45.0 };   // uT, world z up

// ---------------------------------------------------------------- small math
struct V3 { double x, y, z; };

struct M3
{
    double m[3][3];

    V3 operator*(const V3& v) const
    {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }

    M3 T() const
    {
        M3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }
};

M3 rotX(double a)
{
    double c = std::cos(a), s = std::sin(a);
    return {{ {1,0,0}, {0,c,-s}, {0,s,c} }};
}

M3 rotZ(double a)
{
    double c = std::cos(a), s = std::sin(a);
    return {{ {c,-s,0}, {s,c,0}, {0,0,1} }};
}

struct QD { double w, x, y, z; };

QD qmul(const QD& a, const QD& b)
{
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

QD qconj(const QD& q) { return { q.w, -q.x, -q.y, -q.z }; }

QD qFromEuler(double roll, double pitch, double yaw)   // q = Rz(yaw) Ry(pitch) Rx(roll)
{
    QD qz = { std::cos(yaw/2),   0, 0, std::sin(yaw/2) };
    QD qy = { std::cos(pitch/2), 0, std::sin(pitch/2), 0 };
    QD qx = { std::cos(roll/2),  std::sin(roll/2), 0, 0 };
    return qmul(qz, qmul(qy, qx));
}

M3 qToM(const QD& q)
{
    double w = q.w, x = q.x, y = q.y, z = q.z;
    return {{
        { 1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y) },
        { 2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x) },
        { 2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y) }
    }};
}

// Rotation vector of q (log map * 2)
V3 qLog2(QD q)
{
    if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12) return { 2*q.x, 2*q.y, 2*q.z };
    double a = 2.0 * std::atan2(n, q.w);
    return { a*q.x/n, a*q.y/n, a*q.z/n };
}

double yawOf(double w, double x, double y, double z)
{
    return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

double tiltErr(const QD& qt, const Quaternion& qe)
{
    // angle between estimated and true world-up seen in body
    M3 Rt = qToM(qt);
    M3 Re = qToM({ qe.w, qe.x, qe.y, qe.z });
    double d = Rt.m[2][0]*Re.m[2][0] + Rt.m[2][1]*Re.m[2][1] + Rt.m[2][2]*Re.m[2][2];
    d = std::max(-1.0, std::min(1.0, d));
    return std::acos(d);
}

double wrapPi(double a)
{
    while (a >  PI) a -= 2*PI;
    while (a < -PI) a += 2*PI;
    return a;
}

// ---------------------------------------------------------------- trajectory
struct TruthSample
{
    QD   q;
    V3   fBody;        // specific force, body
    bool zaru;         // stationary flag as a (latency-limited) detector would report
};

// Generates a UGV profile: yaw rate and speed commands with rate limits.
std::vector<TruthSample> makeTrajectory(std::mt19937& rng)
{
    std::uniform_real_distribution<double> U(0.0, 1.0);

    const int steps = static_cast<int>(DURATION / DT) + 1;

    // Build a phase list.
    enum Phase { STOP, STRAIGHT, TURN };
    struct Seg { Phase p; double dur; double rate; double speed; };

    std::vector<Seg> segs;
    double t = 0.0;

    segs.push_back({ STOP, 10.0, 0.0, 0.0 });   // initial stationary period
    t += 10.0;

    while (t < DURATION)
    {
        double driveTime = 120.0 + 60.0 * U(rng);   // stop every 2-3 min
        double d = 0.0;
        double speed = 1.0 + 1.0 * U(rng);

        while (d < driveTime)
        {
            double ds = 10.0 + 30.0 * U(rng);
            segs.push_back({ STRAIGHT, ds, 0.0, speed });
            d += ds;

            double rate  = (0.2 + 0.4 * U(rng)) * (U(rng) < 0.5 ? -1.0 : 1.0);
            double angle = (30.0 + 150.0 * U(rng)) * DEG;
            double dur   = angle / std::fabs(rate);
            segs.push_back({ TURN, dur, rate, speed * 0.7 });
            d += dur;
        }

        segs.push_back({ STRAIGHT, 5.0, 0.0, speed });  // settle before stop
        segs.push_back({ STOP, 10.0 + 10.0 * U(rng), 0.0, 0.0 });
        t += d + 5.0 + segs.back().dur;
    }

    // Integrate with rate limits.
    std::vector<TruthSample> out(steps);

    const double accLim  = 0.5;    // m/s^2
    const double yawAccL = 0.8;    // rad/s^2

    double yaw = 0.0, yawRate = 0.0, v = 0.0, dist = 0.0;
    double rollAmp  = 1.5 * DEG;
    double pitchAmp = 2.0 * DEG;

    size_t si = 0;
    double segT = 0.0;

    double prevV = 0.0;
    double restTime = 0.0;

    for (int k = 0; k < steps; ++k)
    {
        const Seg& s = segs[std::min(si, segs.size() - 1)];

        double targetRate  = s.rate;
        double targetSpeed = s.speed;

        // Rate-limited tracking
        double dr = targetRate - yawRate;
        dr = std::max(-yawAccL*DT, std::min(yawAccL*DT, dr));
        yawRate += dr;

        double dv = targetSpeed - v;
        dv = std::max(-accLim*DT, std::min(accLim*DT, dv));
        v += dv;

        if (s.p == STOP && v < 1e-9) { v = 0.0; }

        yaw  += yawRate * DT;
        dist += v * DT;

        double roll  = rollAmp  * std::sin(dist / 3.1);
        double pitch = pitchAmp * std::sin(dist / 4.7 + 1.0);

        out[k].q = qFromEuler(roll, pitch, yaw);

        double vdot = (v - prevV) / DT;
        prevV = v;

        V3 aW = {
            vdot*std::cos(yaw) - v*yawRate*std::sin(yaw),
            vdot*std::sin(yaw) + v*yawRate*std::cos(yaw),
            0.0
        };

        M3 R = qToM(out[k].q);
        V3 f = R.T() * V3{ aW.x, aW.y, aW.z + GRAVITY };
        out[k].fBody = f;

        // ZARU flag: inside a STOP segment, truly at rest, with margins.
        bool atRest = (s.p == STOP) && v == 0.0 && std::fabs(yawRate) < 1e-12;
        restTime = atRest ? restTime + DT : 0.0;
        out[k].zaru = atRest && restTime > ZARU_MARGIN && segT < s.dur - ZARU_MARGIN;

        segT += DT;
        if (segT >= s.dur) { segT = 0.0; ++si; }
    }

    return out;
}

// ---------------------------------------------------------------- sensors
struct Bias
{
    V3 b0;          // turn-on, sensor frame
    V3 rw;          // random walk state, sensor frame
};

IMUSample toSample(const V3& a, const V3& g, const V3& m)
{
    IMUSample s;
    s.ax = (float)a.x; s.ay = (float)a.y; s.az = (float)a.z;
    s.gx = (float)g.x; s.gy = (float)g.y; s.gz = (float)g.z;
    s.mx = (float)m.x; s.my = (float)m.y; s.mz = (float)m.z;
    return s;
}

// ---------------------------------------------------------------- filters
// Filter rows. MEKF "repo" = src/filter/mekf_filter.cpp as-is; "fix" = the
// sim-only copy with the gravity-Jacobian sign corrected.
enum FilterId
{
    F_MEKF_IMU1 = 0,
    F_MEKF_AVG,
    F_MEKFX_IMU1,
    F_MEKFX_AVG,
    F_MEKFX_AVG_CAL,
    F_ESKF_NOZARU,
    F_ESKF_NOZARU_NODEC,
    F_ESKF_ZARU_START,
    F_ESKF_ZARU,
    F_ESKF_ZARU_NODIFF,
    F_ESKF_ZARU_NODEC,
    F_ESKF1_ZARU,
    F_ESKF_ZARU_MAG,
    F_COUNT
};

const char* FILTER_NAMES[F_COUNT] =
{
    "(a)  MEKF repo, IMU1 only",
    "(b)  MEKF repo, 50/50 average",
    "(a*) MEKF sign-fixed, IMU1 only",
    "(b*) MEKF sign-fixed, 50/50 average",
    "(b*')MEKF fixed, avg + 10 s startup cal",
    "(c)  Dual ESKF, no ZARU",
    "(c#) Dual ESKF, no ZARU, no decoupling",
    "(c') Dual ESKF, ZARU first 10 s only",
    "(d)  Dual ESKF, ZARU at stops",
    "(d-) Dual ESKF, ZARU stops, no diff upd",
    "(d#) Dual ESKF, ZARU stops, no decoupl.",
    "(d1) ESKF single-input IMU1, ZARU stops",
    "(e)  Dual ESKF, ZARU stops + mag yaw",
};

void configureESKF(DualESKFFilter& f)
{
    f.setGyroNoise((float)ARW, (float)ARW);
    f.setGyroBiasNoise(1.0e-5f, 1.0e-5f);   // > true RW to absorb warm-up drift
    f.setInitialBiasStd(5.0e-3f);
    f.setAccelNoise(0.05f);
    f.setAccelGate(0.5f, 0.35f);
    f.setZaruNoise(1.0e-3f);
    f.setMagNoise(0.05f);
}

template <class MEKF>
void configureMEKF(MEKF& f)
{
    f.setGyroNoise((float)(ARW*ARW));
    f.setGyroBiasNoise(1.0e-10f);
}

struct RunResult
{
    double rmsYaw[F_COUNT];
    double finalYaw[F_COUNT];
    double rmsTilt[F_COUNT];

    // ESKF (d) bias diagnostics at the end, body frame z axis
    double b1zTrue, b2zTrue, b1zEst, b2zEst;
    double fusedTrue[3], fusedEst[3];
    double fusedEstNoZaru[3];
    double b1Est[3], b2Est[3], b1True[3], b2True[3];
    double diffEstNoZaru[3], diffTrue[3];
};

RunResult runOnce(
    unsigned seed,
    const M3& mount2,          // sensor -> body rotation of IMU2
    FILE* csv
)
{
    std::mt19937 rngTraj(seed * 7919u + 1u);
    std::mt19937 rng(seed * 104729u + 17u);
    std::normal_distribution<double> N01(0.0, 1.0);

    std::vector<TruthSample> traj = makeTrajectory(rngTraj);

    const M3 mount1 = rotZ(0.0);
    const M3 mounts[2] = { mount1, mount2 };

    Bias bias[2];
    for (int i = 0; i < 2; ++i)
    {
        bias[i].b0 = { BIAS0_STD*N01(rng), BIAS0_STD*N01(rng), BIAS0_STD*N01(rng) };
        bias[i].rw = { 0, 0, 0 };
    }

    V3 shared = { SHARED_STD*N01(rng), SHARED_STD*N01(rng), SHARED_STD*N01(rng) };
    double phase = 2*PI*std::uniform_real_distribution<double>(0,1)(rng);

    const double gyroStd = ARW / std::sqrt(DT);
    const double rwStep  = BIAS_RW * std::sqrt(DT);

    MEKFFilter mekf[2];
    for (auto& f : mekf) configureMEKF(f);

    MEKFSignFix mekfx[3];
    for (auto& f : mekfx) configureMEKF(f);

    // eskf[0] no ZARU             [1] no ZARU, no decoupling
    // eskf[2] ZARU first 10 s     [3] ZARU stops
    // eskf[4] ZARU, no diff       [5] ZARU, no decoupling
    // eskf[6] single-input IMU1   [7] ZARU + mag
    DualESKFFilter eskf[8];
    for (auto& f : eskf) configureESKF(f);
    eskf[1].setAccelHeadingDecoupling(false);
    eskf[4].setGyroDiffEnabled(false);
    eskf[5].setAccelHeadingDecoupling(false);
    eskf[7].setMagEnabled(true);

    double sumYaw2[F_COUNT] = {}, sumTilt2[F_COUNT] = {};
    double prevErr[F_COUNT] = {}, unwrapErr[F_COUNT] = {};
    long   count = 0;

    // startup calibration for (b')
    double calSum[3] = {}; long calN = 0; bool calDone = false; double cal[3] = {};

    RunResult res {};

    const int steps = (int)traj.size();

    V3 bodyBiasTrue[2] = {};

    for (int k = 1; k < steps; ++k)
    {
        double t = k * DT;

        // Interval rate [k-1, k] in body frame (exact for the truth path)
        V3 w = qLog2(qmul(qconj(traj[k-1].q), traj[k].q));
        w = { w.x / DT, w.y / DT, w.z / DT };

        M3 Rt = qToM(traj[k].q);
        V3 mB = Rt.T() * V3{ MAG_WORLD[0], MAG_WORLD[1], MAG_WORLD[2] };
        V3 f  = traj[k].fBody;

        double c = (1.0 - std::exp(-t / SHARED_TAU)) + 0.15*std::sin(2*PI*t/600.0 + phase);

        IMUSample body[2];

        for (int i = 0; i < 2; ++i)
        {
            bias[i].rw.x += rwStep*N01(rng);
            bias[i].rw.y += rwStep*N01(rng);
            bias[i].rw.z += rwStep*N01(rng);

            V3 bS = {
                bias[i].b0.x + bias[i].rw.x + c*shared.x,
                bias[i].b0.y + bias[i].rw.y + c*shared.y,
                bias[i].b0.z + bias[i].rw.z + c*shared.z
            };

            M3 Rm  = mounts[i];
            M3 RmT = Rm.T();

            // Generate in sensor frame
            V3 wS = RmT * w;
            V3 fS = RmT * f;
            V3 mS = RmT * mB;

            V3 gS = { wS.x + bS.x + gyroStd*N01(rng),
                      wS.y + bS.y + gyroStd*N01(rng),
                      wS.z + bS.z + gyroStd*N01(rng) };
            V3 aS = { fS.x + ACC_NOISE*N01(rng),
                      fS.y + ACC_NOISE*N01(rng),
                      fS.z + ACC_NOISE*N01(rng) };
            V3 mmS = { mS.x + MAG_NOISE*N01(rng),
                       mS.y + MAG_NOISE*N01(rng),
                       mS.z + MAG_NOISE*N01(rng) };

            // Pipeline: rotate back to body
            body[i] = toSample(Rm * aS, Rm * gS, Rm * mmS);
            bodyBiasTrue[i] = Rm * bS;
        }

        IMUSample noMag1 = body[0];
        noMag1.mx = noMag1.my = noMag1.mz = 0.0f;

        IMUSample avg;
        avg.ax = 0.5f*(body[0].ax + body[1].ax);
        avg.ay = 0.5f*(body[0].ay + body[1].ay);
        avg.az = 0.5f*(body[0].az + body[1].az);
        avg.gx = 0.5f*(body[0].gx + body[1].gx);
        avg.gy = 0.5f*(body[0].gy + body[1].gy);
        avg.gz = 0.5f*(body[0].gz + body[1].gz);
        avg.mx = avg.my = avg.mz = 0.0f;

        // Startup calibration over the first 10 s (vehicle at rest)
        IMUSample avgCal = avg;
        if (t < 9.5)
        {
            calSum[0] += avg.gx; calSum[1] += avg.gy; calSum[2] += avg.gz; ++calN;
        }
        else if (!calDone)
        {
            for (int i = 0; i < 3; ++i) cal[i] = calSum[i] / std::max(1L, calN);
            calDone = true;
        }
        avgCal.gx -= (float)cal[0];
        avgCal.gy -= (float)cal[1];
        avgCal.gz -= (float)cal[2];

        // ESKF inputs without magnetometer for (c)(c')(d)(d1)
        IMUSample e1 = body[0], e2 = body[1];
        e1.mx = e1.my = e1.mz = 0.0f;
        e2.mx = e2.my = e2.mz = 0.0f;

        bool zaru = traj[k].zaru;

        mekf[0].update(noMag1, (float)DT);
        mekf[1].update(avg, (float)DT);
        mekfx[0].update(noMag1, (float)DT);
        mekfx[1].update(avg, (float)DT);
        mekfx[2].update(avgCal, (float)DT);

        eskf[0].setStationary(false);
        eskf[1].setStationary(false);
        eskf[2].setStationary(zaru && t < 10.0);
        for (int j = 3; j < 8; ++j)
            eskf[j].setStationary(zaru);

        for (int j = 0; j < 6; ++j)
            eskf[j].update(e1, e2, (float)DT);
        eskf[6].update(e1, (float)DT);
        eskf[7].update(body[0], body[1], (float)DT);

        const Quaternion* est[F_COUNT] =
        {
            &mekf[0].orientation(), &mekf[1].orientation(),
            &mekfx[0].orientation(), &mekfx[1].orientation(), &mekfx[2].orientation(),
            &eskf[0].orientation(), &eskf[1].orientation(), &eskf[2].orientation(),
            &eskf[3].orientation(), &eskf[4].orientation(), &eskf[5].orientation(),
            &eskf[6].orientation(), &eskf[7].orientation()
        };

        const QD& qt = traj[k].q;
        double yawT = yawOf(qt.w, qt.x, qt.y, qt.z);

        for (int j = 0; j < F_COUNT; ++j)
        {
            const Quaternion& q = *est[j];
            double e = wrapPi(yawOf(q.w, q.x, q.y, q.z) - yawT);

            // unwrap
            unwrapErr[j] += wrapPi(e - prevErr[j]);
            prevErr[j] = e;

            double tilt = tiltErr(qt, q);

            sumYaw2[j]  += unwrapErr[j] * unwrapErr[j];
            sumTilt2[j] += tilt * tilt;
        }
        ++count;

        if (csv && (k % 20 == 0))
        {
            Vec3 b1 = eskf[3].bias1(), b2 = eskf[3].bias2();
            Vec3 b1n = eskf[0].bias1(), b2n = eskf[0].bias2();
            std::fprintf(csv, "%.3f,%d", t, zaru ? 1 : 0);
            for (int j = 0; j < F_COUNT; ++j)
                std::fprintf(csv, ",%.6f", unwrapErr[j] / DEG);
            std::fprintf(csv, ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                bodyBiasTrue[0].z / DEG, bodyBiasTrue[1].z / DEG,
                b1.z / DEG, b2.z / DEG, b1n.z / DEG, b2n.z / DEG);
        }
    }

    for (int j = 0; j < F_COUNT; ++j)
    {
        res.rmsYaw[j]   = std::sqrt(sumYaw2[j] / count) / DEG;
        res.finalYaw[j] = unwrapErr[j] / DEG;
        res.rmsTilt[j]  = std::sqrt(sumTilt2[j] / count) / DEG;
    }

    Vec3 b1 = eskf[3].bias1(), b2 = eskf[3].bias2(), fb = eskf[3].fusedBias();
    Vec3 fn = eskf[0].fusedBias();
    Vec3 b1n = eskf[0].bias1(), b2n = eskf[0].bias2();

    double bt1[3] = { bodyBiasTrue[0].x, bodyBiasTrue[0].y, bodyBiasTrue[0].z };
    double bt2[3] = { bodyBiasTrue[1].x, bodyBiasTrue[1].y, bodyBiasTrue[1].z };
    double be1[3] = { b1.x, b1.y, b1.z }, be2[3] = { b2.x, b2.y, b2.z };
    double bf[3]  = { fb.x, fb.y, fb.z }, bfn[3] = { fn.x, fn.y, fn.z };
    double dn[3]  = { b1n.x - b2n.x, b1n.y - b2n.y, b1n.z - b2n.z };

    for (int i = 0; i < 3; ++i)
    {
        res.b1True[i] = bt1[i] / DEG;  res.b2True[i] = bt2[i] / DEG;
        res.b1Est[i]  = be1[i] / DEG;  res.b2Est[i]  = be2[i] / DEG;
        res.fusedTrue[i] = 0.5*(bt1[i] + bt2[i]) / DEG;
        res.fusedEst[i]  = bf[i] / DEG;
        res.fusedEstNoZaru[i] = bfn[i] / DEG;
        res.diffTrue[i] = (bt1[i] - bt2[i]) / DEG;
        res.diffEstNoZaru[i] = dn[i] / DEG;
    }

    return res;
}

void meanStd(const std::vector<double>& v, double& m, double& s)
{
    m = 0; for (double x : v) m += x; m /= v.size();
    s = 0; for (double x : v) s += (x - m)*(x - m);
    s = std::sqrt(s / std::max<size_t>(1, v.size() - 1));
}

} // namespace

int main(int argc, char** argv)
{
    int numSeeds = (argc > 1) ? std::atoi(argv[1]) : 10;
    std::string csvPath = (argc > 2) ? argv[2] : "sim/dual_eskf_run.csv";

    struct Mount { const char* name; M3 R; const char* tag; };
    Mount cases[2] =
    {
        { "IMU2 rotated 180 deg about Z (same Z)",    rotZ(PI), "rotZ180" },
        { "IMU2 rotated 180 deg about X (opposed Z)", rotX(PI), "rotX180" },
    };

    std::printf("Dual ESKF Monte-Carlo: %d seeds, %.0f min @ %.0f Hz, ARW %.2f deg/sqrt(h)\n",
        numSeeds, DURATION/60.0, 1.0/DT, ARW/DEG*60.0);
    std::printf("bias: turn-on %.2f deg/s/axis, RW %.1e rad/s/sqrt(s), shared warm-up %.2f deg/s/axis (tau %.0f s)\n\n",
        BIAS0_STD/DEG, BIAS_RW, SHARED_STD/DEG, SHARED_TAU);

    for (int mc = 0; mc < 2; ++mc)
    {
        std::vector<double> rms[F_COUNT], fin[F_COUNT], tilt[F_COUNT];
        std::vector<double> fusedErrZ, fusedErrZnoZ, diffErrZnoZ, b1ErrZ, b2ErrZ;

        for (int s = 0; s < numSeeds; ++s)
        {
            FILE* csv = nullptr;

            if (s == 0)
            {
                std::string p = csvPath;
                size_t dot = p.rfind('.');
                p = p.substr(0, dot) + "_" + cases[mc].tag + p.substr(dot);
                csv = std::fopen(p.c_str(), "w");
                if (csv)
                {
                    std::fprintf(csv, "t,zaru");
                    for (int j = 0; j < F_COUNT; ++j) std::fprintf(csv, ",yawerr%d", j);
                    std::fprintf(csv, ",b1z_true,b2z_true,b1z_est_d,b2z_est_d,b1z_est_c,b2z_est_c\n");
                }
            }

            RunResult r = runOnce((unsigned)(s + 1), cases[mc].R, csv);
            if (csv) std::fclose(csv);

            for (int j = 0; j < F_COUNT; ++j)
            {
                rms[j].push_back(r.rmsYaw[j]);
                fin[j].push_back(std::fabs(r.finalYaw[j]));
                tilt[j].push_back(r.rmsTilt[j]);
            }

            fusedErrZ.push_back(std::fabs(r.fusedEst[2] - r.fusedTrue[2]));
            fusedErrZnoZ.push_back(std::fabs(r.fusedEstNoZaru[2] - r.fusedTrue[2]));
            diffErrZnoZ.push_back(std::fabs(r.diffEstNoZaru[2] - r.diffTrue[2]));
            b1ErrZ.push_back(std::fabs(r.b1Est[2] - r.b1True[2]));
            b2ErrZ.push_back(std::fabs(r.b2Est[2] - r.b2True[2]));

            if (s == 0)
            {
                std::printf("[%s] seed 1, final body-frame biases (deg/s):\n", cases[mc].tag);
                std::printf("   b1 true  % .4f % .4f % .4f | ESKF(d) % .4f % .4f % .4f\n",
                    r.b1True[0], r.b1True[1], r.b1True[2], r.b1Est[0], r.b1Est[1], r.b1Est[2]);
                std::printf("   b2 true  % .4f % .4f % .4f | ESKF(d) % .4f % .4f % .4f\n",
                    r.b2True[0], r.b2True[1], r.b2True[2], r.b2Est[0], r.b2Est[1], r.b2Est[2]);
                std::printf("   b1-b2    % .4f % .4f % .4f | ESKF(c) % .4f % .4f % .4f\n",
                    r.diffTrue[0], r.diffTrue[1], r.diffTrue[2],
                    r.diffEstNoZaru[0], r.diffEstNoZaru[1], r.diffEstNoZaru[2]);
                std::printf("   fused    % .4f % .4f % .4f | ESKF(d) % .4f % .4f % .4f | ESKF(c) % .4f % .4f % .4f\n\n",
                    r.fusedTrue[0], r.fusedTrue[1], r.fusedTrue[2],
                    r.fusedEst[0], r.fusedEst[1], r.fusedEst[2],
                    r.fusedEstNoZaru[0], r.fusedEstNoZaru[1], r.fusedEstNoZaru[2]);
            }
        }

        std::printf("=== %s ===\n", cases[mc].name);
        std::printf("%-42s %20s %20s %16s\n", "filter", "RMS yaw err [deg]", "|final yaw| [deg]", "RMS tilt [deg]");

        for (int j = 0; j < F_COUNT; ++j)
        {
            double m1, s1, m2, s2, m3, s3;
            meanStd(rms[j], m1, s1);
            meanStd(fin[j], m2, s2);
            meanStd(tilt[j], m3, s3);
            std::printf("%-42s %9.3f +- %-8.3f %9.3f +- %-8.3f %6.3f +- %-6.3f\n",
                FILTER_NAMES[j], m1, s1, m2, s2, m3, s3);
        }

        double m, sd;
        std::printf("\nend-of-run |bias error| z-axis, deg/s (mean +- std over seeds):\n");
        meanStd(fusedErrZ, m, sd);    std::printf("   fused bias, ESKF (d)      %.5f +- %.5f\n", m, sd);
        meanStd(b1ErrZ, m, sd);       std::printf("   b1,         ESKF (d)      %.5f +- %.5f\n", m, sd);
        meanStd(b2ErrZ, m, sd);       std::printf("   b2,         ESKF (d)      %.5f +- %.5f\n", m, sd);
        meanStd(fusedErrZnoZ, m, sd); std::printf("   fused bias, ESKF (c)      %.5f +- %.5f\n", m, sd);
        meanStd(diffErrZnoZ, m, sd);  std::printf("   b1-b2,      ESKF (c)      %.5f +- %.5f\n\n", m, sd);
    }

    return 0;
}
