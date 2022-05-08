#pragma once
#include <linalg.h>

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

inline linalg::vec<float, 4> eulerToQuaternion(linalg::vec<float, 3> euler) {
    float c1 = cos(euler.x / 2.0f);
    float c2 = cos(euler.y / 2.0f);
    float c3 = cos(euler.z / 2.0f);
    float s1 = sin(euler.x / 2.0f);
    float s2 = sin(euler.y / 2.0f);
    float s3 = sin(euler.z / 2.0f);
    /*float x = s1 * c2 * c3 + c1 * s2 * s3;
    float y = c1 * s2 * c3 - s1 * c2 * s3;
    float z = c1 * c2 * s3 + s1 * s2 * c3;
    float w = c1 * c2 * c3 - s1 * s2 * s3;*/
    float c1c2 = c1 * c2;
    float s1s2 = s1 * s2;
    float w = (c1c2 * c3) - (s1s2 * s3);
    float x = (c1c2 * s3) + (s1s2 * c3);
    float y = (s1 * c2 * c3) + (c1 * s2 * s3);
    float z = (c1 * s2 * c3) - (s1 * c2 * s3);
    return linalg::vec<float, 4>{x, y, z, w};

    // Assuming the angles are in radians.
    /*float c1 = cos(euler.y);
    float s1 = sin(euler.y);
    float c2 = cos(euler.x);
    float s2 = sin(euler.x);
    float c3 = cos(euler.z);
    float s3 = sin(euler.z);
    float w = sqrt(1.0f + c1 * c2 + c1 * c3 - s1 * s2 * s3 + c2 * c3) / 2.0f;
    float w4 = (4.0f * w);
    float x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4;
    float y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4;
    float z = (-s1 * s3 + c1 * s2 * c3 + s2) / w4;
    return linalg::vec<float, 4>{x, y, z, w4};*/
};

inline linalg::mat<float, 3, 3> GetRotationMatrixFromTransform(linalg::mat<float, 4, 4> inMatrix) {
    return linalg::mat<float, 3, 3>(
        linalg::vec<float, 3>(inMatrix.x.x, inMatrix.x.y, inMatrix.x.z),
        linalg::vec<float, 3>(inMatrix.y.x, inMatrix.y.y, inMatrix.y.z),
        linalg::vec<float, 3>(inMatrix.z.x, inMatrix.z.y, inMatrix.z.z)
    );
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


inline linalg::mat<float, 4, 4> ConvertLeftToRightHanded(linalg::mat<float, 4, 4> inMatrix) {
    linalg::mat<float, 4, 4> FlipMatrix(
        linalg::vec<float, 4>(0, 0, -1, 0),
        linalg::vec<float, 4>(0, 0, -1, 0),
        linalg::vec<float, 4>(0, 0, -1, 0),
        linalg::vec<float, 4>(0, 0, 0, 1)
    );
    return linalg::mul(linalg::mul(FlipMatrix, inMatrix), linalg::inverse(FlipMatrix));
}


