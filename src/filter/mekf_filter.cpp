#include "filter/mekf_filter.h"

#include <cmath>
#include <algorithm>

static bool validVector(float x, float y, float z)
{
    return (x*x + y*y + z*z) > 1.0e-12f;
}

static void normalizeVector(float& x, float& y, float& z)
{
    float n = std::sqrt(x*x + y*y + z*z);

    if (n <= 0.0f)
        return;

    x /= n;
    y /= n;
    z /= n;
}

static Quaternion quatMultiply(
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

static Quaternion quatConjugate(const Quaternion& q)
{
    Quaternion out;

    out.w =  q.w;
    out.x = -q.x;
    out.y = -q.y;
    out.z = -q.z;

    return out;
}

static void rotateBodyToWorld(
    const Quaternion& q,
    float x,
    float y,
    float z,
    float& ox,
    float& oy,
    float& oz
)
{
    Quaternion v;
    v.w = 0.0f;
    v.x = x;
    v.y = y;
    v.z = z;

    Quaternion out =
        quatMultiply(
            quatMultiply(q, v),
            quatConjugate(q)
        );

    ox = out.x;
    oy = out.y;
    oz = out.z;
}

static void rotateWorldToBody(
    const Quaternion& q,
    float x,
    float y,
    float z,
    float& ox,
    float& oy,
    float& oz
)
{
    Quaternion v;
    v.w = 0.0f;
    v.x = x;
    v.y = y;
    v.z = z;

    Quaternion out =
        quatMultiply(
            quatMultiply(quatConjugate(q), v),
            q
        );

    ox = out.x;
    oy = out.y;
    oz = out.z;
}

static void makeSkew(
    float x,
    float y,
    float z,
    float S[3][3]
)
{
    S[0][0] =  0.0f; S[0][1] = -z;    S[0][2] =  y;
    S[1][0] =  z;    S[1][1] =  0.0f; S[1][2] = -x;
    S[2][0] = -y;    S[2][1] =  x;    S[2][2] =  0.0f;
}

MEKFFilter::MEKFFilter()
{
    reset();
}

void MEKFFilter::reset()
{
    m_orientation = Quaternion();

    m_biasX = 0.0f;
    m_biasY = 0.0f;
    m_biasZ = 0.0f;

    m_initialized = false;

    resetCovariance();
}

void MEKFFilter::resetCovariance()
{
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            m_P[r][c] = 0.0f;
        }
    }

    // Attitude uncertainty
    m_P[0][0] = 1.0e-2f;
    m_P[1][1] = 1.0e-2f;
    m_P[2][2] = 1.0e-2f;

    // Bias uncertainty
    m_P[3][3] = 1.0e-3f;
    m_P[4][4] = 1.0e-3f;
    m_P[5][5] = 1.0e-3f;
}

void MEKFFilter::normalizeQuaternion()
{
    m_orientation.normalize();
}

void MEKFFilter::initializeFromAccel(
    float ax,
    float ay,
    float az
)
{
    normalizeVector(ax, ay, az);

    float roll =
        std::atan2(ay, az);

    float pitch =
        std::atan2(
            -ax,
            std::sqrt(ay*ay + az*az)
        );

    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);

    Quaternion qRoll;
    qRoll.w = cr;
    qRoll.x = sr;
    qRoll.y = 0.0f;
    qRoll.z = 0.0f;

    Quaternion qPitch;
    qPitch.w = cp;
    qPitch.x = 0.0f;
    qPitch.y = sp;
    qPitch.z = 0.0f;

    m_orientation =
        quatMultiply(
            qPitch,
            qRoll
        );

    m_orientation.normalize();

    m_initialized = true;
}

void MEKFFilter::predict(
    float gx,
    float gy,
    float gz,
    float dt
)
{
    // Remove estimated gyro bias.
    gx -= m_biasX;
    gy -= m_biasY;
    gz -= m_biasZ;

    Quaternion omega;
    omega.w = 0.0f;
    omega.x = gx;
    omega.y = gy;
    omega.z = gz;

    Quaternion qDot =
        quatMultiply(
            m_orientation,
            omega
        );

    m_orientation.w += 0.5f * qDot.w * dt;
    m_orientation.x += 0.5f * qDot.x * dt;
    m_orientation.y += 0.5f * qDot.y * dt;
    m_orientation.z += 0.5f * qDot.z * dt;

    normalizeQuaternion();

    // Error-state transition.
    //
    // delta_theta_dot = -skew(omega) delta_theta - delta_bias
    // delta_bias_dot  = noise
    float F[6][6] = {};

    for (int i = 0; i < 6; ++i)
        F[i][i] = 1.0f;

    float W[3][3];
    makeSkew(gx, gy, gz, W);

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            F[r][c] += -W[r][c] * dt;
        }
    }

    F[0][3] = -dt;
    F[1][4] = -dt;
    F[2][5] = -dt;

    // P = F P F^T + Q
    float FP[6][6] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                FP[r][c] += F[r][k] * m_P[k][c];
            }
        }
    }

    float newP[6][6] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                newP[r][c] += FP[r][k] * F[c][k];
            }
        }
    }

    float qGyro = m_gyroNoise * dt;
    float qBias = m_biasNoise * dt;

    newP[0][0] += qGyro;
    newP[1][1] += qGyro;
    newP[2][2] += qGyro;

    newP[3][3] += qBias;
    newP[4][4] += qBias;
    newP[5][5] += qBias;

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            m_P[r][c] = newP[r][c];
        }
    }
}

void MEKFFilter::applyErrorState(
    const float dx[6]
)
{
    // Small-angle quaternion correction.
    Quaternion dq;
    dq.w = 1.0f;
    dq.x = 0.5f * dx[0];
    dq.y = 0.5f * dx[1];
    dq.z = 0.5f * dx[2];
    dq.normalize();

    m_orientation =
        quatMultiply(
            m_orientation,
            dq
        );

    normalizeQuaternion();

    m_biasX += dx[3];
    m_biasY += dx[4];
    m_biasZ += dx[5];
}

void MEKFFilter::updateDirection(
    float zx,
    float zy,
    float zz,
    float refX,
    float refY,
    float refZ,
    float noise
)
{
    if (!validVector(zx, zy, zz))
        return;

    normalizeVector(zx, zy, zz);
    normalizeVector(refX, refY, refZ);

    // Predicted reference direction in body frame.
    float hx;
    float hy;
    float hz;

    rotateWorldToBody(
        m_orientation,
        refX,
        refY,
        refZ,
        hx,
        hy,
        hz
    );

    normalizeVector(hx, hy, hz);

    // Residual.
    float y[3];

    y[0] = zx - hx;
    y[1] = zy - hy;
    y[2] = zz - hz;

    // Measurement Jacobian for small attitude error.
    // h ≈ h - skew(h) * delta_theta
    // H = [ -skew(h), 0 ]
    float S[3][3];
    makeSkew(hx, hy, hz, S);

    float H[3][6] = {};

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            H[r][c] = -S[r][c];
        }
    }

    // S_mat = H P H^T + R
    float HP[3][6] = {};

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                HP[r][c] += H[r][k] * m_P[k][c];
            }
        }
    }

    float S_mat[3][3] = {};

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                S_mat[r][c] += HP[r][k] * H[c][k];
            }
        }

        S_mat[r][r] += noise;
    }

    float det =
        S_mat[0][0] * (S_mat[1][1]*S_mat[2][2] - S_mat[1][2]*S_mat[2][1])
      - S_mat[0][1] * (S_mat[1][0]*S_mat[2][2] - S_mat[1][2]*S_mat[2][0])
      + S_mat[0][2] * (S_mat[1][0]*S_mat[2][1] - S_mat[1][1]*S_mat[2][0]);

    if (std::fabs(det) < 1.0e-12f)
        return;

    float invDet = 1.0f / det;

    float invS[3][3];

    invS[0][0] =  (S_mat[1][1]*S_mat[2][2] - S_mat[1][2]*S_mat[2][1]) * invDet;
    invS[0][1] = -(S_mat[0][1]*S_mat[2][2] - S_mat[0][2]*S_mat[2][1]) * invDet;
    invS[0][2] =  (S_mat[0][1]*S_mat[1][2] - S_mat[0][2]*S_mat[1][1]) * invDet;

    invS[1][0] = -(S_mat[1][0]*S_mat[2][2] - S_mat[1][2]*S_mat[2][0]) * invDet;
    invS[1][1] =  (S_mat[0][0]*S_mat[2][2] - S_mat[0][2]*S_mat[2][0]) * invDet;
    invS[1][2] = -(S_mat[0][0]*S_mat[1][2] - S_mat[0][2]*S_mat[1][0]) * invDet;

    invS[2][0] =  (S_mat[1][0]*S_mat[2][1] - S_mat[1][1]*S_mat[2][0]) * invDet;
    invS[2][1] = -(S_mat[0][0]*S_mat[2][1] - S_mat[0][1]*S_mat[2][0]) * invDet;
    invS[2][2] =  (S_mat[0][0]*S_mat[1][1] - S_mat[0][1]*S_mat[1][0]) * invDet;

    // K = P H^T inv(S)
    float PHt[6][3] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                PHt[r][c] += m_P[r][k] * H[c][k];
            }
        }
    }

    float K[6][3] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            for (int k = 0; k < 3; ++k)
            {
                K[r][c] += PHt[r][k] * invS[k][c];
            }
        }
    }

    float dx[6] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int k = 0; k < 3; ++k)
        {
            dx[r] += K[r][k] * y[k];
        }
    }

    applyErrorState(dx);

    // Joseph form:
    // P = (I-KH)P(I-KH)^T + K R K^T
    float KH[6][6] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 3; ++k)
            {
                KH[r][c] += K[r][k] * H[k][c];
            }
        }
    }

    float A[6][6];

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            A[r][c] =
                ((r == c) ? 1.0f : 0.0f) - KH[r][c];
        }
    }

    float AP[6][6] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                AP[r][c] += A[r][k] * m_P[k][c];
            }
        }
    }

    float newP[6][6] = {};

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 6; ++k)
            {
                newP[r][c] += AP[r][k] * A[c][k];
            }
        }
    }

    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            for (int k = 0; k < 3; ++k)
            {
                newP[r][c] += K[r][k] * noise * K[c][k];
            }
        }
    }

    // Symmetrize and clamp diagonal.
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            m_P[r][c] =
                0.5f * (newP[r][c] + newP[c][r]);
        }
    }

    for (int i = 0; i < 6; ++i)
    {
        if (m_P[i][i] < 1.0e-9f)
            m_P[i][i] = 1.0e-9f;
    }
}

void MEKFFilter::update(
    const IMUSample& sample,
    float dt
)
{
    if (dt <= 0.0f)
        return;

    float ax = sample.ax;
    float ay = sample.ay;
    float az = sample.az;

    if (!m_initialized)
    {
        if (!validVector(ax, ay, az))
            return;

        initializeFromAccel(ax, ay, az);
        return;
    }

    // Your current pipeline convention: gyro already rad/s.
    predict(
        sample.gx,
        sample.gy,
        sample.gz,
        dt
    );

    if (validVector(ax, ay, az))
    {
        normalizeVector(ax, ay, az);

        // Gravity reference in world frame.
        updateDirection(
            ax,
            ay,
            az,
            0.0f,
            0.0f,
            1.0f,
            m_accelNoise
        );
    }

    float mx = sample.mx;
    float my = sample.my;
    float mz = sample.mz;

    if (validVector(mx, my, mz))
    {
        normalizeVector(mx, my, mz);

        // Soft yaw reference:
        // Use measured horizontal magnetic direction. This avoids needing local declination.
        float horizontal =
            std::sqrt(mx*mx + my*my);

        if (horizontal > 1.0e-6f)
        {
            updateDirection(
                mx,
                my,
                mz,
                mx / horizontal,
                my / horizontal,
                0.0f,
                m_magNoise
            );
        }
    }

    normalizeQuaternion();
}

const Quaternion& MEKFFilter::orientation() const
{
    return m_orientation;
}

void MEKFFilter::setGyroNoise(float value)
{
    m_gyroNoise = std::max(value, 0.0f);
}

void MEKFFilter::setGyroBiasNoise(float value)
{
    m_biasNoise = std::max(value, 0.0f);
}

void MEKFFilter::setAccelNoise(float value)
{
    m_accelNoise = std::max(value, 1.0e-9f);
}

void MEKFFilter::setMagNoise(float value)
{
    m_magNoise = std::max(value, 1.0e-9f);
}
