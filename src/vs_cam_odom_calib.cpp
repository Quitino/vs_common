#include "vs_cam_odom_calib.h"
#include "vs_vecutils.h"
#include "vs_geometry3d.h"
#include "vs_numeric.h"
#include <iostream>
#include <opencv2/opencv.hpp>

template <typename T>
Eigen::Matrix<T, 4, 4> QuaternionMultMatLeft(const Eigen::Quaternion<T> &q)
{
    return (Eigen::Matrix<T, 4, 4>() << q.w(), -q.z(), q.y(), q.x(),
            q.z(), q.w(), -q.x(), q.y(),
            -q.y(), q.x(), q.w(), q.z(),
            -q.x(), -q.y(), -q.z(), q.w())
        .finished();
}

template <typename T>
Eigen::Matrix<T, 4, 4> QuaternionMultMatRight(const Eigen::Quaternion<T> &q)
{
    return (Eigen::Matrix<T, 4, 4>() << q.w(), q.z(), -q.y(), q.x(),
            -q.z(), q.w(), q.x(), q.y(),
            q.y(), -q.x(), q.w(), q.z(),
            -q.x(), -q.y(), -q.z(), q.w())
        .finished();
}

namespace vs
{

using namespace Eigen;

bool solveRxy(const std::vector<Isometry3d> &cam_motions, const std::vector<Isometry3d> &odom_motions, Matrix3d &R)
{
    int n = odom_motions.size();
    MatrixXd T(4 * n, 4);
    T.setZero();
    Matrix3d Rxy[2];
    for (int i = 0; i < n; i++)
    {
        Eigen::Quaterniond ql(odom_motions[i].linear());
        Eigen::Quaterniond qr(cam_motions[i].linear());
        ql.normalize();
        qr.normalize();
        T.block<4, 4>(i * 4, 0) = QuaternionMultMatLeft(ql) - QuaternionMultMatRight(qr);
    }
    JacobiSVD<Eigen::MatrixXd> svd(T, ComputeThinU | ComputeThinV);
    auto t1 = svd.matrixV().block<4, 1>(0, 2);
    auto t2 = svd.matrixV().block<4, 1>(0, 3);
    double a = (t1(0) * t1(1) + t1(2) * t1(3));
    double b = t1(1) * t2(0) + t1(0) * t2(1) + t1(3) * t2(2) + t1(2) * t2(3);
    double c = (t2(0) * t2(1) + t2(2) * t2(3));
    double delta = b * b - 4.0 * a * c;
    if (delta < 0)
    {
        return false;
    }
    double s[2];
    s[0] = (-b + sqrt(delta)) / (2.0 * a);
    s[1] = (-b - sqrt(delta)) / (2.0 * a);

    for (size_t i = 0; i != 2; i++)
    {
        auto lamda = s[i];
        double t = lamda * lamda * t1.dot(t1) + 2 * lamda * t1.dot(t2) + t2.dot(t2);
        double l2 = sqrt(1.0 / t);
        double l1 = lamda * l2;
        Eigen::Quaterniond qxy;
        qxy.coeffs() = (l1 * t1 + l2 * t2);
        qxy.normalize();
        auto &R = Rxy[i];
        R = qxy.toRotationMatrix();
    }
    double yaw0 = atan2(Rxy[0](1, 0), Rxy[0](0, 0));
    double yaw1 = atan2(Rxy[1](1, 0), Rxy[1](0, 0));

    R = (fabs(yaw0) < fabs(yaw1)) ? Rxy[0]: Rxy[1];
    return true;
}

bool camOdomCalib(const std::vector<Isometry3d> &cam_poses,
                  const std::vector<Isometry3d> &odom_poses,
                  Isometry3d &T_c_o, int method, bool verbose)
{
    T_c_o.setIdentity();
    int N = cam_poses.size();
    if (N < 3 || N != (int)odom_poses.size())
    {
        printf("[ERROR]vsCamOdomCalib: cam poses empty or not same size to odom poses\n");
        return false;
    }
    Matrix3d Rxy;
    std::vector<Isometry3d> T_cs, T_os;

    double height = 0.0;
    for (size_t i = 0; i < cam_poses.size(); i++)
    {
        height += cam_poses[i].translation()(2) / (double)cam_poses.size();
    }

    for (int i = 1; i < N; i++)
    {
        T_cs.push_back(cam_poses[i].inverse() * cam_poses[i - 1]);
        T_os.push_back(odom_poses[i].inverse() * odom_poses[i - 1]);
    }

    switch (method)
    {
    case 0:
        if (!solveRxy(T_cs, T_os, Rxy))
            return false;
        break;
    case 1:
        {
            std::vector<Matrix3d> Rxys;
            for (size_t i = 0; i < cam_poses.size(); i++)
            {
                Vector3d ypr = cam_poses[i].linear().eulerAngles(2, 1, 0);
                Matrix3d Rxyi = (AngleAxisd(ypr(1), Vector3d::UnitY())
                    * AngleAxisd(ypr(2), Vector3d::UnitX())).toRotationMatrix();
                Rxys.push_back(Rxyi);
            }
            Rxy = rotMean(Rxys);
        }
        break;
    case 2:
        {
            std::vector<Vector4d> qxys;
            for (size_t i = 0; i < cam_poses.size(); i++)
            {
                Vector3d ypr = cam_poses[i].linear().eulerAngles(2, 1, 0);
                Quaterniond qxyi = (AngleAxisd(ypr(1), Vector3d::UnitY()) * AngleAxisd(ypr(2), Vector3d::UnitX()));
                qxys.push_back(qxyi.coeffs());
            }
            Rxy = Quaterniond(quatMean(qxys)).toRotationMatrix();
        }
        break;
    default:
        break;
    }

    Matrix4d A = Matrix4d::Zero();
    Vector4d b = Vector4d::Zero();
    for (int i = 0; i < N - 1; i++)
    {
        MatrixXd JK(2, 4);
        JK.leftCols(2) = T_os[i].matrix().block<2, 2>(0, 0) - Matrix2d::Identity();
        Vector3d p = Rxy * T_cs[i].translation();
        JK.rightCols(2) << p(0), -p(1), p(1), p(0);
        A += JK.transpose() * JK;
        b += -JK.transpose() * T_os[i].translation().topRows(2);
    }
    Vector4d x = A.inverse() * b;

    double tx = x(0);
    double ty = x(1);
    double scale = hypotf(x(2), x(3));
    double yaw = atan2(-x(3), -x(2));

    if (fabs(scale - 1) > 0.1)
    {
        printf("[ERROR]vsCamOdomCalib: wrong scale %f\n", scale);
        return false;
    }

    T_c_o.linear() = AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix() * Rxy;
    T_c_o.translation() << tx, ty, height;

    if(verbose)
    {
        auto t = T_c_o.translation();
        auto v = vs::Rbw2rpy(T_c_o.linear() * vs::typicalRot(vs::ROT_FLU2RDF)) * 57.29578;
        printf("[INFO]CamOdomCalib t:(%.3f %.3f %.3f) "
               "euler-rpy:(%.3f %.3f %.3f)deg scale: %.3f\n",
               t(0), t(1), t(2), v(0), v(1), v(2), scale);
    }
    return true;
}

} /* namespace vs */