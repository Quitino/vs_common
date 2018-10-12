#ifndef __VS_PNP_H__
#define __VS_PNP_H__
#include <opencv2/opencv.hpp>

namespace vs
{

/** \brief project 2 points
        assert K as eye33, so 2d points are projected into normalized plane
    \param[in] pt1 the first point in 3D
    \param[in] uv1 the first projected point in 2D plane
    \param[in] pt2 the second point in 3D
    \param[in] uv2 the second projected point in 2D plane
    \param[out] trans translation which satisfy: s*uv_i = pt_i + trans for i=1, 2
*/
void p2pTrans(const cv::Point3f& pt1, const cv::Point2f& uv1,
              const cv::Point3f& pt2, const cv::Point2f& uv2,
              cv::Point3f& trans);

/** \brief project n points, need at least 2 points
        assert K as eye33, so 2d points are projected into normalized plane
    \param[in] pts3d points list in 3D
    \param[in] pts2d projected points list in 2D plane
    \param[out] trans translation which satisfy: s*uv_i = pt_i + trans for i=0, 1, ..., n-1
*/
void pnpTrans(const std::vector<cv::Point3f>& pts3d, const std::vector<cv::Point2f>& pts2d,
                cv::Point3f& trans);

/** \brief pnp to calculate translation and yaw
        assert K as eye33, so 2d points are projected into normalized plane
        assert the transformation as: s*uv_i = R * pt_i + trans for i = 0, 1, ..., n-1
        where R = c, -s, 0, s, c, 0, 0, 0, 1, c=cos(yaw), s=sin(yaw)
    \param[in] pts3d points list in 3D
    \param[in] pts2d projected points list in 2D plane
    \param[out] trans the result translation
    \param[out] yaw the result yaw angle in [deg]
*/
// projection su = Rz(yaw)p + t, where u in pts2d and p in pts3d
void pnpTransYaw(const std::vector<cv::Point3f>& pts3d, const std::vector<cv::Point2f>& pts2d,
                 cv::Point3f& trans, float& yaw);

/** \brief project n lines, need at least 3 lines
        assert K as eye33, so 2d lines are projected into normalized plane
    \param[in] lns3d line list in 3D with format (x1, y1, z1, x2, y2, z2)
    \param[in] lns2d projected line list in 2D plane with format (u1, v1, u2, v2)
    \param[out] trans translation which satisfy: lns2d_i // lns3d_i + trans for i=0, 1, ..., n-1
*/
void pnlTrans(const std::vector<cv::Vec6f>& lns3d, const std::vector<cv::Vec4f>& lns2d,
                cv::Point3f& trans);

/** \brief project n points and lines, need at least 3 lines, or at least 2 points
        assert K as eye33, so 2d lines are projected into normalized plane
    \param[in] pts3d points list in 3D
    \param[in] pts2d projected points list in 2D plane
    \param[in] lns3d line list in 3D with format (x1, y1, z1, x2, y2, z2)
    \param[in] lns2d projected line list in 2D plane with format (u1, v1, u2, v2)
    \param[out] trans translation which satisfy: lns2d_i//(lns3d_i+trans) and s*uv_i = pt_i+trans
*/
void pnplTrans(const std::vector<cv::Point3f>& pts3d, const std::vector<cv::Point2f>& pts2d,
                const std::vector<cv::Vec6f>& lns3d, const std::vector<cv::Vec4f>& lns2d,
                cv::Point3f& trans);

/** \brief pnp with RANSAC to calculate translation and yaw
        assert K as eye33, so 2d points are projected into normalized plane
        assert the transformation as: s*uv_i = R * pt_i + trans for i = 0, 1, ..., n-1
        where R = c, -s, 0, s, c, 0, 0, 0, 1, c=cos(yaw), s=sin(yaw)
    \param[in] pts3d points list in 3D
    \param[in] pts2d projected points list in 2D plane
    \param[out] trans the result translation
    \param[out] yaw the result yaw angle in [rad]
    \param[out] inliers inliers flag, the same size as pts3d, 1:inlier, 0:outlier
    \param[in] ite_cnt max iterate count
    \param[in] reprj_err the max reprojection error for inliers. Note, this error is in normailzed
                image plane, rather than image plane. Thus, reprj_err = pixel_error / focal_length
    \return if ransac succeed. If RANSAC run to max iterate count, this will return false.
*/
// projection su = Rz(yaw)p + t, where u in pts2d and p in pts3d
bool pnpRansacTransYaw(const std::vector<cv::Point3f>& pts3d,
                       const std::vector<cv::Point2f>& pts2d,
                       cv::Point3f& trans, float& yaw, std::vector<uchar>& inliers,
                       int ite_cnt = 100, float reprj_err = 0.1);

} /* namespace vs */
#endif//__VS_PNP_H__