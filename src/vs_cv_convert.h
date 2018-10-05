#ifndef __VS_CV_CONVERT_H__
#define __VS_CV_CONVERT_H__

#include <opencv2/opencv.hpp>

inline cv::Mat Rt2T(const cv::Mat& R, const cv::Mat& t)
{
    cv::Mat T = cv::Mat::eye(4,4, CV_64FC1);
    R.copyTo(T.rowRange(0,3).colRange(0,3));
    t.copyTo(T.rowRange(0,3).col(3));
    return T;
}

inline cv::Mat R2T(const cv::Mat& R)
{
    cv::Mat T = cv::Mat::eye(4,4, CV_64FC1);
    R.copyTo(T.rowRange(0,3).colRange(0,3));
    return T;
}

inline void T2Rt(const cv::Mat& T, cv::Mat& R, cv::Mat& t)
{
    T.rowRange(0,3).colRange(0,3).copyTo(R);
    T.rowRange(0,3).col(3).copyTo(t);
}

inline cv::Mat vec2T(const std::vector<double>& v)
{
    assert(v.size()==16);
    cv::Mat T(4,4, CV_64FC1);
    double* Tdata = (double*)T.data;
    for(int i=0; i<16; i++)
    {
        Tdata[i] = v[i];
    }
    return T;
}

inline cv::Mat camMat(double fx, double fy, double cx, double cy)
{
    return (cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
}

#ifdef EIGEN_MAJOR_VERSION
#include <opencv2/core/eigen.hpp>

inline Eigen::Isometry3d rt2isometry(const cv::Mat& rvec, const cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R,r);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

inline Eigen::Isometry3d Rt2isometry(const cv::Mat& R, const cv::Mat& tvec)
{
    Eigen::Matrix3d r;
    cv::cv2eigen(R,r);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);
    return T;
}

#endif

#endif//__VS_CV_CONVERT_H__