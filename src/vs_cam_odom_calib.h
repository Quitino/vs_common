#ifndef __VS_CAM_ODOM_CALIB_H__
#define __VS_CAM_ODOM_CALIB_H__
#include <vector>
#include <Eigen/Dense>

namespace vs
{

// Simple hand-eye calibration case for camera odomtry calibration
// where odomtry is on ground plane, and camera poses can be collect by chessboard or marker
// Key eqution: $T_{o_i}_{o_{i+1}} * T_c_o = T_c_o * T_{c_i}_{c_{i+1}}$
// ref:
// 1. Guo C X, Mirzaei F M, Roumeliotis S I. An analytical least-squares solution to
//    the odometer-camera extrinsic calibration problem[C]// IEEE International Conference
//    on Robotics & Automation. 2012.
// 2. Heng L , Li B , Pollefeys M . CamOdoCal: Automatic intrinsic and extrinsic
//    calibration of a rig with multiple generic cameras and odometry[C]// IEEE/RSJ
//    International Conference on Intelligent Robots & Systems. IEEE, 2013.
// NOTE:
// 1. z plane of camera poses must be the ground plane, which is the same as odomter z plane
// 2. There is NO scale problem in cam poses. So one can put a chessboard or other marker
//    on ground plane to collect camera poses with pnp.
// 3. camera poses must be synchronized with odom poses.

// p_o = T_c_o * p_c
bool camOdomCalib(const std::vector<Eigen::Isometry3d>& cam_poses,
                    const std::vector<Eigen::Isometry3d>& odom_poses,
                    Eigen::Isometry3d& T_c_o);

} /* namespace vs */

#endif//__VS_CAM_ODOM_CALIB_H__