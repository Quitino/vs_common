#include "vs_debug_draw.h"
namespace vs
{

void drawMonoPoints(const char* title, const cv::Mat& img,
            const std::vector<cv::Point2f>& pts)
{
    cv::Mat img_show;
    if(img.channels()==1)
        cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
    else
        img.copyTo(img_show);

    for(const auto& p : pts)
        cv::circle(img_show, p, 2, cv::Scalar(0, 255, 0), -1);
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
            cv::circle(img_show, cur_pts[i], 2, cv::Scalar(0, 255, 0), -1);
            cv::line(img_show, prev_pts[i], cur_pts[i], cv::Scalar(0, 255, 0), 1);
        }
    }
    else
    {
        for(size_t i=0; i<prev_pts.size(); i++)
        {
            if(inliers[i])
            {
                cv::circle(img_show, cur_pts[i], 2, cv::Scalar(0, 255, 0), -1);
                cv::line(img_show, prev_pts[i], cur_pts[i], cv::Scalar(0, 255, 0), 1);
            }
        }
    }
    cv::imshow(title, img_show);
}

void showMatches(const char* title,
                 const cv::Mat& img1, const std::vector<cv::KeyPoint>& kpts1,
                 const cv::Mat& img2, const std::vector<cv::KeyPoint>& kpts2,
                 const std::vector<cv::DMatch>& matches1to2,
                 const std::vector<char>& inliers,
                 bool draw_single_point)
{
    cv::Mat img_show;
    cv::drawMatches(img1, kpts1, img2, kpts2, matches1to2, img_show,
                    cv::Scalar::all(-1), cv::Scalar::all(-1), inliers,
                    draw_single_point ? cv::DrawMatchesFlags::DEFAULT:
                                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow(title, img_show);
}

void showKeypoints(const char* title, const cv::Mat& img, const std::vector<cv::KeyPoint>& kpts)
{
    cv::Mat img_show;
    cv::drawKeypoints(img, kpts, img_show);
    cv::imshow(title, img_show);
}

void showRectify(const char* title, const cv::Mat& imgl, const cv::Mat& imgr, int step)
{
    cv::Mat img_show;
    cv::hconcat(imgl, imgr, img_show);
    if(img_show.channels()==1)
        cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2BGR);

    static std::vector<cv::Scalar> colors;
    static unsigned int seed = 0;
    if(colors.empty())
    {
        for(int i=0; i<100; i++)
        {
            cv::Scalar s(rand_r(&seed)%255, rand_r(&seed)%255, rand_r(&seed)%255);
            colors.push_back(s);
        }
    }
    for(int i=step, j=0; i<img_show.rows; i+=step, j++)
    {
        cv::line(img_show, cv::Point(0, i), cv::Point(img_show.cols, i),
                            colors[j%colors.size()], 1);
    }
    cv::imshow(title, img_show);
}

void colorBar(double rate, uchar& R, uchar& G, uchar& B)
{
    const static uchar tr[] = {255, 255, 255, 0, 0, 0,
                               255, 255, 255, 0, 0, 0};
    const static uchar tg[] = {0, 125, 255, 255, 255, 0,
                               0, 125, 255, 255, 255, 0};
    const static uchar tb[] = {0, 0, 0, 0, 255, 255,
                               0, 0, 0, 0, 255, 255};
    const static int N = sizeof(tb)/sizeof(tb[0]);
    if(rate <= 0)
    {
        R = tr[0];
        G = tg[0];
        B = tb[0];
    }
    else if(rate >= 1)
    {
        R = tr[N-1];
        G = tg[N-1];
        B = tb[N-1];
    }
    else
    {
        float k = rate * N;
        int i = k;
        k -= i;
        R = tr[i]*(1-k) + tr[i+1]*k;
        G = tg[i]*(1-k) + tg[i+1]*k;
        B = tb[i]*(1-k) + tb[i+1]*k;
    }
}

cv::Scalar colorBar(double rate)
{
    uchar r, g, b;
    colorBar(rate, r, g, b);
    return cv::Scalar(r, g, b);
}


void drawLines(const char* title, const cv::Mat& img, const std::vector<cv::Vec4f>& lines)
{
    cv::Mat img_show;
    if(img.channels()==1)
        cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
    else
        img.copyTo(img_show);

    for(const auto& l : lines)
    {
        cv::line(img_show, cv::Point(l(0), l(1)), cv::Point(l(2), l(3)),
                 cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
    }
    cv::imshow(title, img_show);
}

} /* namespace vs */