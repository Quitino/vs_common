#ifndef __VS_DEBUG_DRAW_H__
#define __VS_DEBUG_DRAW_H__
#include <opencv2/opencv.hpp>

void drawMonoPoints(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& pts);

void drawMonoLK(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& prev_pts,
            const std::vector<cv::Point2f>& cur_pts,
            const std::vector<unsigned char>& inliers = std::vector<unsigned char>(0));

#endif//__DEBUG_DRAW_H__