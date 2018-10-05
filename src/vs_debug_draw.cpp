#include "vs_debug_draw.h"

void drawMonoPoints(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& pts)
{
    cv::Mat img_show;
    if(img.channels()==1)
        cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
    else
        img.copyTo(img_show);

    for(const auto& p: pts)
        cv::circle(img_show, p, 2, cv::Scalar(0,255,0), -1);
    cv::imshow(title, img_show);
}

void drawMonoLK(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& prev_pts,
            const std::vector<cv::Point2f>& cur_pts,
            const std::vector<unsigned char>& inliers)
{
    cv::Mat img_show;
    if(img.channels()==1)
        cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
    else
        img.copyTo(img_show);
    if(inliers.empty())
    {
        for(size_t i=0; i<prev_pts.size(); i++)
        {
            cv::circle(img_show, cur_pts[i], 2, cv::Scalar(0,255,0), -1);
            cv::line(img_show, prev_pts[i], cur_pts[i], cv::Scalar(0,255,0), 1);
        }
    }
    else
    {
        for(size_t i=0; i<prev_pts.size(); i++)
        {
            if(inliers[i])
            {
                cv::circle(img_show, cur_pts[i], 2, cv::Scalar(0,255,0), -1);
                cv::line(img_show, prev_pts[i], cur_pts[i], cv::Scalar(0,255,0), 1);        
            }
        }
    }
    cv::imshow(title, img_show);
}