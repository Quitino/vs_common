#ifndef __VS_IMPROC_H__
#define __VS_IMPROC_H__
#include <vector>
#include <opencv2/opencv.hpp>

/*\brief grab hue channel from bgr image */
cv::Mat bgr2hue(const cv::Mat& bgr);

/*\brief grab red region from bgr image.*/
cv::Mat grabRed(const cv::Mat& bgr);

/*\brief grab red region from bgr image with dense NN classifer.*/
cv::Mat grabRedMl(const cv::Mat& bgr);

/*\brief Gradient filter with Sobel*/
void sobelFilter(const cv::Mat& img, cv::Mat& grad_bw, int grad_thres=200);

/*\brief region filter, remove small patch.*/
void regionFilter(cv::Mat& img, int minRegion, int maxRegion = 1<<30);

/*\brief region filter, detect bounding boxes of regions with proper size*/
void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes, int minRegion, int maxRegion = 1<<30);

/*\brief region filter, detect bounding boxes and centers of regions with proper size*/
void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes, std::vector<cv::Point2f>& centers, int minRegion, int maxRegion = 1<<30);

#endif//__VS_IMPROC_H__