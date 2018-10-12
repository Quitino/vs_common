#ifndef __VS_LANE_DETECT_H__
#define __VS_LANE_DETECT_H__
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

namespace vs
{

struct Lane
{
    Lane(const cv::Point2f& _p1, const cv::Point2f& _p2)
        : p1(_p1), p2(_p2)
    {
        dir = p2 - p1;
        center = (p1 + p2) / 2;
        len = hypotf(dir.x, dir.y);
        if(len > 1e-3) dir /= len;
    }
    cv::Point2f p1;
    cv::Point2f p2;
    cv::Point2f center;
    cv::Point2f dir;
    float len;
    int color;
};
typedef std::vector<Lane> LaneList;

// lane_type 0: pure red
//           1: pure green
//           2: pure blue
//           3: pure yellow
//           10: pure white
// NOTE: input BGR image
int laneDetect(const cv::Mat& img, LaneList& lanes,
               const cv::Mat& K, const cv::Mat& T_c_b,
               int lane_type = 3, bool draw = false);

} /* namespace vs */

#endif//__VS_LANE_DETECT_H__