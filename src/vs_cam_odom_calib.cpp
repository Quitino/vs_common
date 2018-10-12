#include "vs_cam_odom_calib.h"
#include "vs_vecutils.h"
#include <iostream>

namespace vs
{

using namespace Eigen;

bool camOdomCalib(const std::vector<Isometry3d>& cam_poses,
                    const std::vector<Isometry3d>& odom_poses,
                    Isometry3d& T_c_o)
{
    T_c_o.setIdentity();
    int N = cam_poses.size();
    if(N < 3 || N != (int)odom_poses.size())
    {
        printf("[ERROR]vsCamOdomCalib: cam poses empty or not same size to odom poses\n");
        return false;
    }

    std::vector<double> pitches, rolls, heights;
    for(const auto& T_c_w : cam_poses)
    {
        Vector3d ypr = T_c_w.linear().eulerAngles(2, 1, 0);
        if(fabs(ypr(2)) < 0.08)
        {
            rolls.push_back(ypr(2));
            pitches.push_back(ypr(1));
        }
        else
        {
            Vector3d rpy = - T_c_w.linear().transpose().eulerAngles(0, 1, 2);
            rolls.push_back(rpy(0));
            pitches.push_back(rpy(1));
        }
        heights.push_back(T_c_w.translation().z());
    }
    double mean_roll = vecMean(rolls);
    double mean_pitch = vecMean(pitches);
    double mean_height = vecMean(heights);
    for(int i = 0; i < N; i++)
    {
        if(fabs(pitches[i] - mean_pitch) > 0.087 || fabs(rolls[i] - mean_roll) > 0.087
            || fabs(heights[i] - mean_height) > 0.02)
        {
            printf("[WARN]]vsCamOdomCalib: roll(%.4f) pitch(%.4f) height(%.4f) seems wrong.\n",
                    rolls[i], pitches[i], heights[i]);
        }
    }

    Matrix3d R_xy = (AngleAxisd(mean_pitch, Vector3d::UnitY())
                   * AngleAxisd(mean_roll, Vector3d::UnitX())).toRotationMatrix();

    std::vector<Isometry3d> T_cs, T_os;
    for(int i = 1; i < N; i++)
    {
        T_cs.push_back(cam_poses[i].inverse() * cam_poses[i - 1]);
        T_os.push_back(odom_poses[i].inverse() * odom_poses[i - 1]);
    }

    Matrix4d A = Matrix4d::Zero();
    Vector4d b = Vector4d::Zero();
    for(int i = 0; i < N - 1; i++)
    {
        MatrixXd JK(2, 4);
        JK.leftCols(2) = T_os[i].matrix().block<2, 2>(0, 0) - Matrix2d::Identity();
        Vector3d p = R_xy * T_cs[i].translation();
        JK.rightCols(2) << p(1), -p(2), p(2), p(1);
        A += JK.transpose() * JK;
        b += - JK.transpose() * T_os[i].translation().topRows(2);
    }
    if(fabs(A.determinant()) < 1e-3 || b.norm() < 1e-3)
    {
        printf("[ERROR]]vsCamOdomCalib: ill solution\n");
        return false;
    }
    Vector4d x = A.inverse() * b;

    double tx = x(0);
    double ty = x(1);
    double yaw = atan2(x(3), x(2));
    double scale = hypotf(x(2), x(3));

    if(fabs(scale - 1) > 0.1)
    {
        printf("[ERROR]]vsCamOdomCalib: wrong scale %f\n", scale);
        return false;
    }

    T_c_o.linear() = AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix() * R_xy;
    T_c_o.translation() << tx, ty, mean_height;
    return true;
}

} /* namespace vs */