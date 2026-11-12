#include "filter/dual_eskf_filter.h"

#include <cmath>
#include <algorithm>

namespace
{

constexpr int N = DualESKFFilter::N;

// Error-state indices
constexpr int ITH = 0;   // dtheta
constexpr int IB1 = 3;   // db1
constexpr int IB2 = 6;   // db2

bool validVector(float x, float y, float z)
{
    return (x*x + y*y + z*z) > 1.0e-12f;
}

Quaternion quatMultiply(
    const Quaternion& a,
    const Quaternion& b
)
{
    Quaternion q;

    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

    return q;
}

// Quaternion exponential of a rotation vector (rad).
Quaternion quatExp(
    double vx,
    double vy,
    double vz
)
{
    double angle = std::sqrt(vx*vx + vy*vy + vz*vz);

    Quaternion q;

    if (angle < 1.0e-9)
    {
        q.w = 1.0f;
        q.x = static_cast<float>(0.5 * vx);
        q.y = static_cast<float>(0.5 * vy);
        q.z = static_cast<float>(0.5 * vz);
    }
    else
    {
        double s = std::sin(0.5 * angle) / angle;

        q.w = static_cast<float>(std::cos(0.5 * angle));
        q.x = static_cast<float>(s * vx);
        q.y = static_cast<float>(s * vy);
        q.z = static_cast<float>(s * vz);
    }

    q.normalize();

    return q;
}

// Rotation matrix of q (body -> world): v_world = R v_body.
void quatToMatrix(
    const Quaternion& q,
    double R[3][3]
)
{
    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;

    R[0][0] = 1.0 - 2.0*(y*y + z*z);
    R[0][1] = 2.0*(x*y - w*z);
    R[0][2] = 2.0*(x*z + w*y);

    R[1][0] = 2.0*(x*y + w*z);
    R[1][1] = 1.0 - 2.0*(x*x + z*z);
    R[1][2] = 2.0*(y*z - w*x);

    R[2][0] = 2.0*(x*z - w*y);
    R[2][1] = 2.0*(y*z + w*x);
    R[2][2] = 1.0 - 2.0*(x*x + y*y);
}

// Rodrigues: R = Exp([v]x)
void rotationFromVector(
    double vx,
    double vy,
    double vz,
    double R[3][3]
)
{
    double angle = std::sqrt(vx*vx + vy*vy + vz*vz);

    double K[3][3] =
    {
        {  0.0, -vz,   vy  },
        {  vz,   0.0, -vx  },
        { -vy,   vx,   0.0 }
    };

    double a;
    double b;

    if (angle < 1.0e-6)
    {
        a = 1.0 - angle*angle / 6.0;
        b = 0.5 - angle*angle / 24.0;
    }
    else
    {
        a = std::sin(angle) / angle;
        b = (1.0 - std::cos(angle)) / (angle*angle);
    }

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            double KK = 0.0;

            for (int k = 0; k < 3; ++k)
                KK += K[r][k] * K[k][c];

            R[r][c] = ((r == c) ? 1.0 : 0.0) + a*K[r][c] + b*KK;
        }
    }
}

} // namespace

DualESKFFilter::DualESKFFilter()
{
    reset();
}

void DualESKFFilter::reset()
{
    m_orientation = Quaternion();

    for (int i = 0; i < 3; ++i)
    {
        m_b1[i] = 0.0;
        m_b2[i] = 0.0;
    }

    m_initialized = false;
    m_singleInput = false;
    m_magRefValid = false;
    m_hasLast     = false;
    m_diag        = Diagnostics();

    resetCovariance();
}

void DualESKFFilter::resetCovariance()
{
    for (int r = 0; r < N; ++r)
    {
        m_dx[r] = 0.0;

        for (int c = 0; c < N; ++c)
            m_P[r][c] = 0.0;
    }

    double tilt = static_cast<double>(m_initTiltStd) * m_initTiltStd;
    double bias = static_cast<double>(m_initBiasStd) * m_initBiasStd;

    // Roll / pitch uncertain; yaw is *defined* by the initial frame
    // (or by the mag reference captured at the same instant).
    m_P[ITH + 0][ITH + 0] = tilt;
    m_P[ITH + 1][ITH + 1] = tilt;
    m_P[ITH + 2][ITH + 2] = 1.0e-8;

    for (int i = 0; i < 3; ++i)
    {
        m_P[IB1 + i][IB1 + i] = bias;
        m_P[IB2 + i][IB2 + i] = bias;
    }
}

void DualESKFFilter::initialize(
    const IMUSample& s1,
    const IMUSample& s2
)
{
    float ax = 0.5f * (s1.ax + s2.ax);
    float ay = 0.5f * (s1.ay + s2.ay);
    float az = 0.5f * (s1.az + s2.az);

    float n = std::sqrt(ax*ax + ay*ay + az*az);

    ax /= n;
    ay /= n;
    az /= n;

    float roll  = std::atan2(ay, az);
    float pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));

    Quaternion qRoll;
    qRoll.w = std::cos(0.5f * roll);
    qRoll.x = std::sin(0.5f * roll);
    qRoll.y = 0.0f;
    qRoll.z = 0.0f;

    Quaternion qPitch;
    qPitch.w = std::cos(0.5f * pitch);
    qPitch.x = 0.0f;
    qPitch.y = std::sin(0.5f * pitch);
    qPitch.z = 0.0f;

    m_orientation = quatMultiply(qPitch, qRoll);
    m_orientation.normalize();

    // Capture magnetic horizontal reference in the (yaw = 0) world frame.
    float mx = 0.5f * (s1.mx + s2.mx);
    float my = 0.5f * (s1.my + s2.my);
    float mz = 0.5f * (s1.mz + s2.mz);

    m_magRefValid = false;

    if (validVector(mx, my, mz))
    {
        double R[3][3];
        quatToMatrix(m_orientation, R);

        double wx = R[0][0]*mx + R[0][1]*my + R[0][2]*mz;
        double wy = R[1][0]*mx + R[1][1]*my + R[1][2]*mz;
        double h  = std::sqrt(wx*wx + wy*wy);

        if (h > 1.0e-3)
        {
            m_magRef[0]   = static_cast<float>(wx / h);
            m_magRef[1]   = static_cast<float>(wy / h);
            m_magRefNorm  = std::sqrt(mx*mx + my*my + mz*mz);
            m_magRefValid = true;
        }
    }

    m_initialized = true;
}

void DualESKFFilter::predict(
    const IMUSample& s1,
    const IMUSample& s2,
    float dtf,
    double omega[3]
)
{
    double dt = dtf;

    double w1 = m_w1;
    double w2 = m_w2;

    double g1[3] = { s1.gx, s1.gy, s1.gz };
    double g2[3] = { s2.gx, s2.gy, s2.gz };

    for (int i = 0; i < 3; ++i)
        omega[i] = w1 * (g1[i] - m_b1[i]) + w2 * (g2[i] - m_b2[i]);

    // Nominal: q <- q (x) Exp(omega dt)
    m_orientation =
        quatMultiply(
            m_orientation,
            quatExp(omega[0]*dt, omega[1]*dt, omega[2]*dt)
        );

    m_orientation.normalize();

    // Error-state transition (Sola 2017, sec. 7, local attitude error):
    //   dtheta+ = R(omega dt)^T dtheta - (w1 db1 + w2 db2) dt - (w1 n1 + w2 n2) dt
    //   dbi+    = dbi + n_bi
    double Rw[3][3];
    rotationFromVector(omega[0]*dt, omega[1]*dt, omega[2]*dt, Rw);

    double F[N][N] = {};

    for (int i = 0; i < N; ++i)
        F[i][i] = 1.0;

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
            F[ITH + r][ITH + c] = Rw[c][r];   // transpose

        F[ITH + r][IB1 + r] = -w1 * dt;
        F[ITH + r][IB2 + r] = -w2 * dt;
    }

    // P = F P F^T + Q  (exploit sparsity: only the first 3 rows of F differ from I)
    double FP[N][N];

    for (int r = 0; r < N; ++r)
    {
        for (int c = 0; c < N; ++c)
        {
            if (r >= 3)
            {
                FP[r][c] = m_P[r][c];
                continue;
            }

            double sum = 0.0;

            for (int k = 0; k < N; ++k)
                sum += F[r][k] * m_P[k][c];

            FP[r][c] = sum;
        }
    }

    double newP[N][N];

    for (int r = 0; r < N; ++r)
    {
        for (int c = 0; c < N; ++c)
        {
            if (c >= 3)
            {
                newP[r][c] = FP[r][c];
                continue;
            }

            double sum = 0.0;

            for (int k = 0; k < N; ++k)
                sum += FP[r][k] * F[c][k];

            newP[r][c] = sum;
        }
    }

    // Process noise.
    double sg1 = m_gyroNoise1;
    double sg2 = m_gyroNoise2;

    double qTheta;

    if (m_singleInput)
    {
        // Same physical sensor in both slots: noise is fully correlated.
        qTheta = sg1 * sg1 * dt;
    }
    else
    {
        qTheta = (w1*w1*sg1*sg1 + w2*w2*sg2*sg2) * dt;
    }

    double qB1 = static_cast<double>(m_biasNoise1) * m_biasNoise1 * dt;
    double qB2 = static_cast<double>(m_biasNoise2) * m_biasNoise2 * dt;

    for (int i = 0; i < 3; ++i)
    {
        newP[ITH + i][ITH + i] += qTheta;
        newP[IB1 + i][IB1 + i] += qB1;
        newP[IB2 + i][IB2 + i] += qB2;
    }

    for (int r = 0; r < N; ++r)
        for (int c = 0; c < N; ++c)
            m_P[r][c] = newP[r][c];

    conditionCovariance();
}

bool DualESKFFilter::scalarUpdate(
    const double H[N],
    double r,
    double R,
    const double* upBody
)
{
    // h = P H^T
    double h[N];

    for (int i = 0; i < N; ++i)
    {
        double sum = 0.0;

        for (int k = 0; k < N; ++k)
            sum += m_P[i][k] * H[k];

        h[i] = sum;
    }

    double s = R;

    for (int k = 0; k < N; ++k)
        s += H[k] * h[k];

    if (!(s > 0.0) || !std::isfinite(s))
        return false;

    // Innovation relative to the error mean already accumulated this step.
    double y = r;

    for (int k = 0; k < N; ++k)
        y -= H[k] * m_dx[k];

    double K[N];

    for (int i = 0; i < N; ++i)
        K[i] = h[i] / s;

    if (upBody)
    {
        // Suboptimal (projected) gain: remove the components of the attitude
        // and both bias corrections along the body-frame vertical, so this
        // measurement cannot touch heading or the vertical gyro biases.
        for (int blk = 0; blk < N; blk += 3)
        {
            double d =
                K[blk + 0] * upBody[0] +
                K[blk + 1] * upBody[1] +
                K[blk + 2] * upBody[2];

            for (int c = 0; c < 3; ++c)
                K[blk + c] -= d * upBody[c];
        }
    }

    for (int i = 0; i < N; ++i)
        m_dx[i] += K[i] * y;

    // Joseph form for a scalar measurement, expanded (valid for any gain K):
    // (I-KH)P(I-KH)^T + K R K^T = P - K h^T - h K^T + s K K^T
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            m_P[i][j] +=
                - K[i] * h[j]
                - h[i] * K[j]
                + s * K[i] * K[j];
        }
    }

    return true;
}

void DualESKFFilter::updateGyroDifference(
    const IMUSample& s1,
    const IMUSample& s2,
    double dt,
    const double omega[3]
)
{
    double z[3] =
    {
        static_cast<double>(s1.gx) - s2.gx,
        static_cast<double>(s1.gy) - s2.gy,
        static_cast<double>(s1.gz) - s2.gz
    };

    double sg1 = m_gyroNoise1;
    double sg2 = m_gyroNoise2;

    double rate = std::sqrt(
        omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]
    );

    double infl = static_cast<double>(m_diffRateInfl) * rate;

    // Discrete-time variance of a white-noise sample with density sigma:
    // sigma^2 / dt.
    double R =
        (sg1*sg1 + sg2*sg2) / dt
        + static_cast<double>(m_diffFloor) * m_diffFloor
        + infl * infl;

    for (int i = 0; i < 3; ++i)
    {
        double H[N] = {};
        H[IB1 + i] =  1.0;
        H[IB2 + i] = -1.0;

        double r = z[i] - (m_b1[i] - m_b2[i]);

        scalarUpdate(H, r, R);
    }

    m_diag.diffUsed = true;
}

void DualESKFFilter::updateZeroRateInternal(
    const IMUSample& s1,
    const IMUSample& s2,
    double dt
)
{
    double g1[3] = { s1.gx, s1.gy, s1.gz };
    double g2[3] = { s2.gx, s2.gy, s2.gz };

    double sg1 = m_gyroNoise1;
    double sg2 = m_gyroNoise2;
    double zr  = static_cast<double>(m_zaruNoise) * m_zaruNoise;

    if (m_singleInput)
    {
        // Only the fused bias is observable from one sensor.
        double R = sg1*sg1 / dt + zr;

        for (int i = 0; i < 3; ++i)
        {
            double H[N] = {};
            H[IB1 + i] = m_w1;
            H[IB2 + i] = m_w2;

            double r = g1[i] - (m_w1*m_b1[i] + m_w2*m_b2[i]);

            scalarUpdate(H, r, R);
        }
    }
    else
    {
        double R1 = sg1*sg1 / dt + zr;
        double R2 = sg2*sg2 / dt + zr;

        for (int i = 0; i < 3; ++i)
        {
            double H1[N] = {};
            H1[IB1 + i] = 1.0;
            scalarUpdate(H1, g1[i] - m_b1[i], R1);

            double H2[N] = {};
            H2[IB2 + i] = 1.0;
            scalarUpdate(H2, g2[i] - m_b2[i], R2);
        }
    }

    m_diag.zaruUsed = true;
}

void DualESKFFilter::updateGravity(
    const IMUSample& s1,
    const IMUSample& s2,
    const double omega[3]
)
{
    double ax = 0.5 * (static_cast<double>(s1.ax) + s2.ax);
    double ay = 0.5 * (static_cast<double>(s1.ay) + s2.ay);
    double az = 0.5 * (static_cast<double>(s1.az) + s2.az);

    double n = std::sqrt(ax*ax + ay*ay + az*az);

    double rate = std::sqrt(
        omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]
    );

    m_diag.accelNormError = static_cast<float>(std::fabs(n - m_gravity));
    m_diag.rate           = static_cast<float>(rate);

    if (n < 1.0e-6)
        return;

    if (std::fabs(n - m_gravity) > m_accelNormGate)
        return;

    if (rate > m_accelRateGate)
        return;

    double u[3] = { ax / n, ay / n, az / n };

    // Predicted gravity direction in body: h = R^T e_z (third row of R).
    double R[3][3];
    quatToMatrix(m_orientation, R);

    double h[3] = { R[2][0], R[2][1], R[2][2] };

    // With q_t = q (x) Exp(dtheta):  h_t = (I - [dtheta]x) h = h + [h]x dtheta
    double S[3][3] =
    {
        {  0.0,  -h[2],  h[1] },
        {  h[2],  0.0,  -h[0] },
        { -h[1],  h[0],  0.0  }
    };

    double var = static_cast<double>(m_accelNoise) * m_accelNoise;

    for (int i = 0; i < 3; ++i)
    {
        double H[N] = {};

        for (int c = 0; c < 3; ++c)
            H[ITH + c] = S[i][c];

        scalarUpdate(H, u[i] - h[i], var, m_accelDecouple ? h : nullptr);
    }

    m_diag.accelUsed = true;
}

void DualESKFFilter::updateMagnetometer(
    const IMUSample& s1,
    const IMUSample& s2
)
{
    if (!m_magRefValid)
        return;

    double mx = 0.5 * (static_cast<double>(s1.mx) + s2.mx);
    double my = 0.5 * (static_cast<double>(s1.my) + s2.my);
    double mz = 0.5 * (static_cast<double>(s1.mz) + s2.mz);

    double n = std::sqrt(mx*mx + my*my + mz*mz);

    if (n < 1.0e-6)
        return;

    if (std::fabs(n - m_magRefNorm) > m_magRelGate * m_magRefNorm)
        return;

    double R[3][3];
    quatToMatrix(m_orientation, R);

    double wx = R[0][0]*mx + R[0][1]*my + R[0][2]*mz;
    double wy = R[1][0]*mx + R[1][1]*my + R[1][2]*mz;

    if (wx*wx + wy*wy < 1.0e-6 * n * n)
        return;

    // Signed angle from reference to measured horizontal direction.
    double cross = m_magRef[0]*wy - m_magRef[1]*wx;
    double dot   = m_magRef[0]*wx + m_magRef[1]*wy;
    double phi   = std::atan2(cross, dot);

    // R m_b = Exp(-R dtheta) m_w  ->  phi = -(R dtheta)_z
    double H[N] = {};
    H[ITH + 0] = -R[2][0];
    H[ITH + 1] = -R[2][1];
    H[ITH + 2] = -R[2][2];

    double var = static_cast<double>(m_magNoise) * m_magNoise;

    if (scalarUpdate(H, phi, var))
        m_diag.magUsed = true;
}

void DualESKFFilter::injectAndReset()
{
    double dth[3] = { m_dx[ITH + 0], m_dx[ITH + 1], m_dx[ITH + 2] };

    m_orientation =
        quatMultiply(
            m_orientation,
            quatExp(dth[0], dth[1], dth[2])
        );

    m_orientation.normalize();

    for (int i = 0; i < 3; ++i)
    {
        m_b1[i] += m_dx[IB1 + i];
        m_b2[i] += m_dx[IB2 + i];
    }

    // Reset Jacobian (Sola 2017, sec. 6.3): G = diag(I - [dtheta/2]x, I, I)
    bool nonzero =
        (dth[0] != 0.0) || (dth[1] != 0.0) || (dth[2] != 0.0);

    if (nonzero)
    {
        double G[3][3] =
        {
            {  1.0,           0.5*dth[2],  -0.5*dth[1] },
            { -0.5*dth[2],    1.0,          0.5*dth[0] },
            {  0.5*dth[1],   -0.5*dth[0],   1.0        }
        };

        // Rows 0..2: GP = G * P[0..2][:]
        double GP[3][N];

        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < N; ++c)
            {
                double sum = 0.0;

                for (int k = 0; k < 3; ++k)
                    sum += G[r][k] * m_P[k][c];

                GP[r][c] = sum;
            }
        }

        // Full G P G^T: top-left block GP G^T, top-right GP, bottom-left (GP)^T
        double TL[3][3];

        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                double sum = 0.0;

                for (int k = 0; k < 3; ++k)
                    sum += GP[r][k] * G[c][k];

                TL[r][c] = sum;
            }
        }

        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                m_P[r][c] = TL[r][c];

            for (int c = 3; c < N; ++c)
            {
                m_P[r][c] = GP[r][c];
                m_P[c][r] = GP[r][c];
            }
        }
    }

    for (int i = 0; i < N; ++i)
        m_dx[i] = 0.0;

    conditionCovariance();
}

void DualESKFFilter::conditionCovariance()
{
    for (int r = 0; r < N; ++r)
    {
        for (int c = r + 1; c < N; ++c)
        {
            double v = 0.5 * (m_P[r][c] + m_P[c][r]);
            m_P[r][c] = v;
            m_P[c][r] = v;
        }
    }

    for (int i = 0; i < N; ++i)
    {
        if (!(m_P[i][i] > 1.0e-18))
            m_P[i][i] = 1.0e-18;
    }

    // Keep correlation coefficients within [-1, 1] (cheap PSD guard).
    for (int r = 0; r < N; ++r)
    {
        for (int c = r + 1; c < N; ++c)
        {
            double lim = std::sqrt(m_P[r][r] * m_P[c][c]) * 0.999999;

            if (m_P[r][c] > lim)
            {
                m_P[r][c] = lim;
                m_P[c][r] = lim;
            }
            else if (m_P[r][c] < -lim)
            {
                m_P[r][c] = -lim;
                m_P[c][r] = -lim;
            }
        }
    }
}

void DualESKFFilter::step(
    const IMUSample& s1,
    const IMUSample& s2,
    float dt
)
{
    if (dt <= 0.0f)
        return;

    m_last1   = s1;
    m_last2   = s2;
    m_lastDt  = dt;
    m_hasLast = true;

    m_diag = Diagnostics();

    if (!m_initialized)
    {
        float ax = 0.5f * (s1.ax + s2.ax);
        float ay = 0.5f * (s1.ay + s2.ay);
        float az = 0.5f * (s1.az + s2.az);

        if (!validVector(ax, ay, az))
            return;

        initialize(s1, s2);
        return;
    }

    double omega[3];
    predict(s1, s2, dt, omega);

    if (m_stationary)
    {
        // ZARU already contains the information of the gyro difference
        // (same samples); using both would double count.
        updateZeroRateInternal(s1, s2, dt);
    }
    else if (m_diffEnabled && !m_singleInput)
    {
        updateGyroDifference(s1, s2, dt, omega);
    }

    if (m_accelEnabled)
        updateGravity(s1, s2, omega);

    if (m_magEnabled)
        updateMagnetometer(s1, s2);

    injectAndReset();
}

void DualESKFFilter::update(
    const IMUSample& s1,
    const IMUSample& s2,
    float dt
)
{
    m_singleInput = false;
    step(s1, s2, dt);
}

void DualESKFFilter::update(
    const IMUSample& sample,
    float dt
)
{
    m_singleInput = true;
    step(sample, sample, dt);
}

const Quaternion& DualESKFFilter::orientation() const
{
    return m_orientation;
}

void DualESKFFilter::setStationary(bool stationary)
{
    m_stationary = stationary;
}

bool DualESKFFilter::isStationary() const
{
    return m_stationary;
}

void DualESKFFilter::updateZeroRate()
{
    if (!m_initialized || !m_hasLast || m_lastDt <= 0.0f)
        return;

    updateZeroRateInternal(m_last1, m_last2, m_lastDt);
    injectAndReset();
}

void DualESKFFilter::setGyroNoise(float imu1, float imu2)
{
    m_gyroNoise1 = std::max(imu1, 1.0e-9f);
    m_gyroNoise2 = std::max(imu2, 1.0e-9f);
}

void DualESKFFilter::setGyroBiasNoise(float imu1, float imu2)
{
    m_biasNoise1 = std::max(imu1, 0.0f);
    m_biasNoise2 = std::max(imu2, 0.0f);
}

void DualESKFFilter::setGyroDiffNoiseFloor(float stdRadPerSec)
{
    m_diffFloor = std::max(stdRadPerSec, 0.0f);
}

void DualESKFFilter::setGyroDiffRateInflation(float relative)
{
    m_diffRateInfl = std::max(relative, 0.0f);
}

void DualESKFFilter::setAccelNoise(float stdDir)
{
    m_accelNoise = std::max(stdDir, 1.0e-6f);
}

void DualESKFFilter::setMagNoise(float stdRad)
{
    m_magNoise = std::max(stdRad, 1.0e-6f);
}

void DualESKFFilter::setZaruNoise(float stdRadPerSec)
{
    m_zaruNoise = std::max(stdRadPerSec, 0.0f);
}

void DualESKFFilter::setInitialBiasStd(float stdRadPerSec)
{
    m_initBiasStd = std::max(stdRadPerSec, 1.0e-9f);
}

void DualESKFFilter::setInitialTiltStd(float stdRad)
{
    m_initTiltStd = std::max(stdRad, 1.0e-9f);
}

void DualESKFFilter::setWeights(float w1, float w2)
{
    w1 = std::max(w1, 0.0f);
    w2 = std::max(w2, 0.0f);

    float sum = w1 + w2;

    if (sum <= 0.0f)
    {
        m_w1 = 0.5f;
        m_w2 = 0.5f;
        return;
    }

    m_w1 = w1 / sum;
    m_w2 = w2 / sum;
}

void DualESKFFilter::setAccelGate(float normTol, float rateTol)
{
    m_accelNormGate = std::max(normTol, 0.0f);
    m_accelRateGate = std::max(rateTol, 0.0f);
}

void DualESKFFilter::setMagGate(float relTol)
{
    m_magRelGate = std::max(relTol, 0.0f);
}

void DualESKFFilter::setGravity(float g)
{
    m_gravity = std::max(g, 1.0e-3f);
}

void DualESKFFilter::setAccelHeadingDecoupling(bool enabled)
{
    m_accelDecouple = enabled;
}

void DualESKFFilter::setGyroDiffEnabled(bool enabled)
{
    m_diffEnabled = enabled;
}

void DualESKFFilter::setAccelEnabled(bool enabled)
{
    m_accelEnabled = enabled;
}

void DualESKFFilter::setMagEnabled(bool enabled)
{
    m_magEnabled = enabled;
}

bool DualESKFFilter::initialized() const
{
    return m_initialized;
}

Vec3 DualESKFFilter::bias1() const
{
    return Vec3(
        static_cast<float>(m_b1[0]),
        static_cast<float>(m_b1[1]),
        static_cast<float>(m_b1[2])
    );
}

Vec3 DualESKFFilter::bias2() const
{
    return Vec3(
        static_cast<float>(m_b2[0]),
        static_cast<float>(m_b2[1]),
        static_cast<float>(m_b2[2])
    );
}

Vec3 DualESKFFilter::fusedBias() const
{
    return Vec3(
        static_cast<float>(m_w1*m_b1[0] + m_w2*m_b2[0]),
        static_cast<float>(m_w1*m_b1[1] + m_w2*m_b2[1]),
        static_cast<float>(m_w1*m_b1[2] + m_w2*m_b2[2])
    );
}

void DualESKFFilter::attitudeCovariance(float out[3][3]) const
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r][c] = static_cast<float>(m_P[ITH + r][ITH + c]);
}

void DualESKFFilter::errorCovariance(float out[N][N]) const
{
    for (int r = 0; r < N; ++r)
        for (int c = 0; c < N; ++c)
            out[r][c] = static_cast<float>(m_P[r][c]);
}

const DualESKFFilter::Diagnostics& DualESKFFilter::diagnostics() const
{
    return m_diag;
}
