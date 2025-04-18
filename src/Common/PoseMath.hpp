#pragma once
#include <linalg.h>
#include <algorithm> // For std::min and std::max
#include <cmath>     // For std::acos, std::sin

inline void normalizeQuat(double pose[]){
    double mag = sqrt(pose[3] * pose[3] +
        pose[4] * pose[4] +
        pose[5] * pose[5] +
        pose[6] * pose[6]);

    pose[3] /= mag;
    pose[4] /= mag;
    pose[5] /= mag;
    pose[6] /= mag;
}

inline void eulerToQuaternion(double eulerXYZ[], double* out_quat){
    double c1 = cos(eulerXYZ[0] / 2.0);
    double c2 = cos(eulerXYZ[1] / 2.0);
    double c3 = cos(eulerXYZ[2] / 2.0);
    double s1 = sin(eulerXYZ[0] / 2.0);
    double s2 = sin(eulerXYZ[1] / 2.0);
    double s3 = sin(eulerXYZ[2] / 2.0);
    double x = s1 * c2 * c3 + c1 * s2 * s3;
    double y = c1 * s2 * c3 - s1 * c2 * s3;
    double z = c1 * c2 * s3 + s1 * s2 * c3;
    double w = c1 * c2 * c3 - s1 * s2 * s3;
    out_quat[0] = x;
    out_quat[1] = y;
    out_quat[2] = z;
    out_quat[3] = w;
};

inline linalg::vec<float, 4> GetXAxisRotation(linalg::vec<float, 4> inQuat)
{
    float a = sqrt((inQuat.w * inQuat.w) + (inQuat.x * inQuat.x));
    return linalg::normalize(linalg::vec<float, 4>(inQuat.x, 0, 0, inQuat.w / a));
}

inline linalg::vec<float, 4> GetYAxisRotation(linalg::vec<float, 4> inQuat)
{
    float a = sqrt((inQuat.w * inQuat.w) + (inQuat.y * inQuat.y));
    return linalg::normalize(linalg::vec<float, 4>(0, inQuat.y, 0, inQuat.w / a));
}

inline linalg::vec<float, 4> GetZAxisRotation(linalg::vec<float, 4> inQuat)
{
    float a = sqrt((inQuat.w * inQuat.w) + (inQuat.z * inQuat.z));
    return linalg::normalize(linalg::vec<float, 4>(0, 0, inQuat.z, inQuat.w / a));
}

inline linalg::mat<float, 3, 3> GetRotationMatrixFromTransform(linalg::mat<float, 4, 4> inMatrix) {
    return linalg::mat<float, 3, 3>(
        linalg::vec<float, 3>(inMatrix.x.x, inMatrix.x.y, inMatrix.x.z),
        linalg::vec<float, 3>(inMatrix.y.x, inMatrix.y.y, inMatrix.y.z),
        linalg::vec<float, 3>(inMatrix.z.x, inMatrix.z.y, inMatrix.z.z)
        );
}

inline linalg::vec<float, 3> GetTranslationFromTransformMatrix(linalg::mat<float, 4, 4> inMatrix) {
    return linalg::vec<float, 3>(inMatrix.w.x, inMatrix.w.y, inMatrix.w.z);
}

inline linalg::mat<float, 4, 4> GetTransformMatrixFromVector(linalg::vec<float, 3> inVec, linalg::vec<float, 3> up) {
    
    linalg::vec<float, 3> xAxis = linalg::cross(up, inVec);
    linalg::vec<float, 3> yAxis = linalg::cross(inVec, xAxis);

    return linalg::mat<float, 4, 4>{
        linalg::vec<float, 4>{xAxis.x, xAxis.y, xAxis.z, 0.0},
        linalg::vec<float, 4>{yAxis.x, yAxis.y, yAxis.z, 0.0},
        linalg::vec<float, 4>{inVec.x, inVec.y, inVec.z, 0.0},
        linalg::vec<float, 4>{0.0, 0.0, 0.0, 1}
    };
}

inline linalg::mat<float, 4, 4> GetTransformMatrixFromVRHmdMatrix34(vr::HmdMatrix34_t vr_matrix)
{
    return linalg::mat<float, 4, 4>{
        linalg::vec<float, 4>{vr_matrix.m[0][0], vr_matrix.m[0][1], vr_matrix.m[0][2], vr_matrix.m[0][3]},
        linalg::vec<float, 4>{vr_matrix.m[1][0], vr_matrix.m[1][1], vr_matrix.m[1][2], vr_matrix.m[1][3]},
        linalg::vec<float, 4>{vr_matrix.m[2][0], vr_matrix.m[2][1], vr_matrix.m[2][2], vr_matrix.m[2][3]},
        linalg::vec<float, 4>{0.0, 0.0, 0.0, 1}
    };
}

inline linalg::vec<float, 3> YawPitchRollFromRotationMatrix(linalg::mat<float, 3, 3> inMatrix)
{
    return linalg::vec<float, 3>(atan(inMatrix.z.y/inMatrix.z.z), -asin(inMatrix.z.x), atan(inMatrix.y.x/inMatrix.x.x));
}

inline linalg::vec<float, 4> CreateFromAxisAngle(const double& xx, const double& yy, const double& zz, const double& a)
{
    // Here we calculate the sin( theta / 2) once for optimization
    double factor = sin(a / 2.0);

    // Calculate the x, y and z of the quaternion
    double x = xx * factor;
    double y = yy * factor;
    double z = zz * factor;

    // Calcualte the w value by cos( theta / 2 )
    double w = cos(a / 2.0);

    return linalg::normalize(linalg::vec<float, 4>(x, y, z, w));
}


// Space conversions
// -----------------

inline linalg::mat<float, 4, 4> ConvertZtoYUp(linalg::mat<float, 4, 4> inMatrix) {
    linalg::mat<float, 4, 4> ZtoYupMatrix(
        linalg::vec<float, 4>(0, 0, 1, 0),
        linalg::vec<float, 4>(1, 0, 0, 0),
        linalg::vec<float, 4>(0, 1, 0, 0),
        linalg::vec<float, 4>(0, 0, 0, 1)
    );
    return linalg::mul(linalg::mul(ZtoYupMatrix, inMatrix), linalg::inverse(ZtoYupMatrix));
}

//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

inline vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

inline vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

inline linalg::vec<float, 3> CalculateAngularVelocity(
    const linalg::vec<float, 4>& q0,  // quaternion at time t0 [x,y,z,w]
    const linalg::vec<float, 4>& q1,  // quaternion at time t1 [x,y,z,w]
    float dt)                         // time difference between t0 and t1
{
    // Ensure quaternions are normalized
    linalg::vec<float, 4> q0_normalized = linalg::normalize(q0);
    linalg::vec<float, 4> q1_normalized = linalg::normalize(q1);

    // Calculate quaternion inverse (for unit quaternions, inverse = conjugate)
    linalg::vec<float, 4> q0_inv = { -q0_normalized[0], -q0_normalized[1], -q0_normalized[2], q0_normalized[3] };

    // Manual quaternion multiplication for q_diff = q1 * q0_inv
    float x1 = q1_normalized[0], y1 = q1_normalized[1], z1 = q1_normalized[2], w1 = q1_normalized[3];
    float x2 = q0_inv[0], y2 = q0_inv[1], z2 = q0_inv[2], w2 = q0_inv[3];

    linalg::vec<float, 4> q_diff = {
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  // x
        w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,  // y
        w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,  // z
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2   // w
    };

    // Convert to axis-angle representation
    // The angle is 2 * acos(q_diff.w)
    float angle = 2.0f * std::acos(std::clamp(q_diff[3], -1.0f, 1.0f));

    // Avoid division by zero and numerical instability for small angles
    linalg::vec<float, 3> axis;
    if (angle < 1e-6f) {
        // For very small rotations, return zero angular velocity
        return linalg::vec<float, 3>{0, 0, 0};
    }
    else {
        // Extract the rotation axis
        float sin_half_angle = std::sin(angle / 2.0f);
        axis = linalg::vec<float, 3>{ q_diff[0], q_diff[1], q_diff[2] } / sin_half_angle;

        // Normalize axis
        axis = linalg::normalize(axis);
    }

    // Calculate angular velocity vector (axis * angle / dt)
    // Direction is the rotation axis, magnitude is angle/dt (radians/second)
    return axis * (angle / dt);
}
