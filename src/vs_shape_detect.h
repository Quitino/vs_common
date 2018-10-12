#ifndef __VS_SHAPE_DETECT_H__
#define __VS_SHAPE_DETECT_H__
#include "vs_geometry2d.h"

namespace vs
{

/** \brief find markers in image
    \param[in] gray: input gray image
    \param[out] centers: centers of markers
    \param[in] marker_type:
                0: con-centers circles
                1: reserved
                2: arrows pair
                3: con-centers triangles
                4: con-centers rectangles
                5: con-centers rhombus
    \param[in] draw: debug draw
    \return count of markers
*/
int detectMarker(const cv::Mat& gray, std::vector<cv::Point2f>& centers,
                   int marker_type = 0, bool draw = false);

/** \brief find all circles in image with hough circle transformation
    \param[in] gray: input gray image
    \param[out] centers: centers of detect circles
    \param[out] radius: radius of detect circles
    \param[in] min_radius: circles whose radius less than this won't be detected.
    \param[in] max_radius: circles whose radius beyond this won't be detected.
    \return count of detect circles
*/
int detectCircle(const cv::Mat& gray, std::vector<cv::Point2f>& centers,
                   std::vector<float>& radius, int min_radius,
                   int max_radius = INT_MAX);

/** \brief find concetric circles in image
    \param[in] gray: input gray image
    \param[out] centers: centers of detect circles
    \param[out] radius: radius of detect circles
    \param[in] min_radius: circles whose radius less than this won't be detected.
    \param[in] max_radius: circles whose radius beyond this won't be detected.
    \param[in] dist_thres: distance between two circle centers below thres are seen
                            as conteric.
    \param[in] k: at least k circles concentric are seen as concentric circle
    \param[in] min_outr: outer circle radius of concentric circle must be larger
                            than this thres. unable this check by setting to 0.
    \param[in] radius_ratio_thres: ratio inner_radius/outer_radius must be less than
                            this thes. unable this check by setting to 1.
    \return count of detect concentric circle center
*/
int detectCircleConcentric(const cv::Mat& gray, std::vector<cv::Point2f>& con_centers,
                             std::vector<std::vector<float>>& con_radius,
                             int min_radius = 5, int max_radius = INT_MAX,
                             float dist_thres = 5.0f, int k = 3, float min_outr = 0,
                             float radius_ratio_thres = 1, bool draw = false);

/** \brief find arrows in image
    \param[in] gray: input gray image
    \param[out] arrows: centers of detect circles
    \param[in] angle_min: min arrow angle in rad
    \param[in] angle_max: max arrow angle in rad
    \param[in] thres_connect: two line with endpoints distance below this thres
                            are seen as a arrow
    \return count of detect arrows
*/
int detectArrow(const cv::Mat& gray, std::vector<Arrow>& arrows,
                  float angle_min, float angle_max, float thres_connect = 5);

/** \brief find arrows in lines
    \param[in] lines: detect lines
    \param[out] arrows: centers of detect circles
    \param[in] angle_min: min arrow angle in rad
    \param[in] angle_max: max arrow angle in rad
    \param[in] thres_connect: two line with endpoints distance below this thres
                            are seen as a arrow
    \return count of detect arrows
*/
int detectArrow(const std::vector<cv::Vec4f>& lines, std::vector<Arrow>& arrows,
                  float angle_min, float angle_max, float thres_connect = 5);

/** \brief detect lines in gray image with rectangle ROI*/
int detectLine(const cv::Mat& gray, std::vector<cv::Vec4f>& lines,
                 const cv::Rect& roi = cv::Rect());

/** \brief detect lines in gray image with mask*/
int detectLine(const cv::Mat& gray, std::vector<cv::Vec4f>& lines,
                 const cv::Mat& mask);

/** \brief find special nested shapes in image
    \param[in] gray: input gray image
    \param[out] centers: centers of detect shapes
    \param[in] type: type of nested shape
                        2: nested two arrows such as >>>> <<<<<
                        3: nested equilateral triangle
                        4: nested equilateral rectangle
                        5: nested rhombus
    \param[in] draw: debug draw
    \return count of detect shapes
*/
int detectNestedShape(const cv::Mat& gray, std::vector<cv::Point2f>& centers,
                       int type = 3, bool draw = false);


} /* namespace vs */
#endif//__VS_SHAPE_DETECT_H__