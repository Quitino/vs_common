#ifndef __VS_IMPROC_H__
#define __VS_IMPROC_H__
#include <vector>
#include <opencv2/opencv.hpp>

namespace vs
{

/*\brief grab hue channel from bgr image */
cv::Mat bgr2hue(const cv::Mat& bgr);

/*\brief grab saturation channel from bgr image */
cv::Mat bgr2saturation(const cv::Mat& bgr);

/*\brief grab red region from bgr image.*/
cv::Mat grabRed(const cv::Mat& bgr);

/*\brief grab red region from bgr image with dense NN classifer.*/
cv::Mat grabRedMl(const cv::Mat& bgr);

/*\brief gradient filter with Sobel*/
void sobelFilter(const cv::Mat& img, cv::Mat& grad_bw, int grad_thres=200);

/*\brief region filter, remove small patch.*/
void regionFilter(cv::Mat& img, int minRegion, int maxRegion = 1<<30);

/*\brief region filter, detect bounding boxes of regions with proper size*/
void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes,
                  int minRegion, int maxRegion = 1<<30);

/*\brief region filter, detect bounding boxes and centers of regions with proper size*/
void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes,
                  std::vector<cv::Point2f>& centers, int minRegion, int maxRegion = 1<<30);

/*\brief normalized histogram for image, return a 256 size vector, each bin store the frequency*/
void histn(const cv::Mat& gray, double normalized_hist[256], int step = 1);

} /* namespace vs */

#endif//__VS_IMPROC_H__