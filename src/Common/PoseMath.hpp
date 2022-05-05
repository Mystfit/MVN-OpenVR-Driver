#pragma once
#include <linalg.h>

void normalizeQuat(double pose[]){
    double mag = sqrt(pose[3] * pose[3] +
        pose[4] * pose[4] +
        pose[5] * pose[5] +
        pose[6] * pose[6]);

    pose[3] /= mag;
    pose[4] /= mag;
    pose[5] /= mag;
    pose[6] /= mag;
}

void eulerToQuaternion(double eulerXYZ[], double* out_quat){
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

linalg::mat<float, 4, 4> ZtoYupMatrix(
    linalg::vec<float, 4>(0, 0, 1, 0),
    linalg::vec<float, 4>(1, 0, 0, 0),
    linalg::vec<float, 4>(0, 1, 0, 0),
    linalg::vec<float, 4>(0, 0, 0, 1)
);
linalg::mat<float, 4, 4> ZtoYUpInv = linalg::inverse(ZtoYupMatrix);
linalg::vec<float, 3> DefaultScale(1.0f);

inline linalg::mat<float, 4, 4> ConvertZtoYUp(linalg::mat<float, 4, 4> inMatrix) {
    return linalg::mul(linalg::mul(ZtoYupMatrix, inMatrix), ZtoYUpInv);
}

