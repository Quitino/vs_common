#ifndef __VS_DEBUG_DRAW_H__
#define __VS_DEBUG_DRAW_H__
#include <opencv2/opencv.hpp>

namespace vs
{

void drawMonoPoints(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& pts);

void drawMonoLK(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& prev_pts,
            const std::vector<cv::Point2f>& cur_pts,
            const std::vector<unsigned char>& inliers = std::vector<unsigned char>(0));

void showMatches(const char* title,
                 const cv::Mat& img1, const std::vector<cv::KeyPoint>& kpts1,
                 const cv::Mat& img2, const std::vector<cv::KeyPoint>& kpts2,
                 const std::vector<cv::DMatch>& matches1to2,
                 const std::vector<char>& inliers = std::vector<char>(),
                 bool draw_single_point = true);

void showKeypoints(const char* title, const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts);

void showRectify(const char* title, const cv::Mat& imgl, const cv::Mat& imgr,
                 int step = 40);

// input rate from [0,1], out a color from [red-orange-yellow-green-cyan-blue-purple]
void colorBar(double rate, uchar& R, uchar& G, uchar& B);

cv::Scalar colorBar(double rate);

void drawLines(const char* title, const cv::Mat& img, const std::vector<cv::Vec4f>& lines);

} /* namespace vs */
#endif//__DEBUG_DRAW_H__