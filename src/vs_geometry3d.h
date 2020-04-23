#ifndef __VS_GEOMETRY3D_H__
#define __VS_GEOMETRY3D_H__
#include <Eigen/Dense>
#include <vector>

namespace vs
{

Eigen::Matrix3d skew(const Eigen::Vector3d& w);

double rotDiff(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2);

bool checkRot(const Eigen::Matrix3d& R);

Eigen::Vector4d weightMeanQuat(Eigen::Vector4d q1, Eigen::Vector4d q2,
                               double k1, double k2 = 0);

bool slerp(double t1, const Eigen::Vector4d& q1, double t2, const Eigen::Vector4d& q2,
           double t, Eigen::Vector4d& q);

Eigen::Vector4d quatMean(const std::vector<Eigen::Vector4d>& quats);

Eigen::Matrix3d rotMean(const std::vector<Eigen::Matrix3d>& Rs);

Eigen::Matrix3d rotMeanSO3(const std::vector<Eigen::Matrix3d>& Rs);

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w);

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

Eigen::Matrix4d expSE3(const Eigen::Matrix<double, 6, 1>& vec);

Eigen::Matrix<double, 6, 1> logSE3(const Eigen::Matrix4d& mat);

Eigen::Matrix4d Omega(Eigen::Vector3d w);

Eigen::Matrix<double, 6, 1> isom2vec(const Eigen::Isometry3d& T);

Eigen::Vector3d eulerConjugate(const Eigen::Vector3d& rpy);

/** type: 0-acute roll 1-positive roll*/
Eigen::Vector3d eulerAdjust(Eigen::Vector3d& rpy, int type = 0);

enum
{
    ROT_RDF2FLU = 0, // right-down-front to front-left-up
    ROT_FLU2RDF, // front-left-up to right-down-front
    ROT_RDF2FRD, // right-down-front to front-right-down
    ROT_FRD2RDF, // front-right-down to right-down-front
    ROT_FLU2FRD, // front-left-up to front-right-down
    ROT_FRD2FLU, // front-right-down to front-left-up
    ROT_RBD2FLU, // right-back-down to front-left up
    ROT_FLU2RBD, // front-left up to right-back-down
    ROT_RBD2FRD, // right-back-down to front-right-down
    ROT_FRD2RBD, // front-right-down to right-back-down
};
Eigen::Matrix3d typicalRot(int type);

Eigen::Vector3d Rbw2rpy(const Eigen::Matrix3d& R_b_w);

} /* namespace vs */
#endif//__VS_GEOMETRY3D_H__