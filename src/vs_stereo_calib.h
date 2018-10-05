#ifndef __VS_STEREO_CALIB_H__
#define __VS_STEREO_CALIB_H__
#include <opencv2/opencv.hpp>

struct StereoVioCalib
{
    cv::Size img_size;
    cv::Mat K0,K1,D0,D1;
    cv::Mat R_c0_c1, t_c0_c1; //rotation/translation point from camera left to camera right
    cv::Mat R_c0_imu, t_c0_imu; //rotation/translation point from camera left to body imu
    cv::Mat R_imu_body, t_imu_body;
    std::string distort_model;

    void deepCopy(const StereoVioCalib& rhs);
};

std::ostream& operator<<(std::ostream& os, const StereoVioCalib& c);

bool loadStereoVioCalib(const char* calib_file, StereoVioCalib& calib);

bool saveStereoVioCalib(const char* calib_file, const StereoVioCalib& calib);

class StereoRectifier
{
public:
    StereoRectifier();
    StereoRectifier(const char* calib_file);
    StereoRectifier(const StereoVioCalib& calib_raw);

    bool init(const char* calib_file);
    bool init(const StereoVioCalib& calib_raw);

    void rectify(const cv::Mat& img0, const cv::Mat& img1,
                      cv::Mat& out0, cv::Mat& out1);

    StereoVioCalib getCalibRaw() {return m_calib_raw;}

    StereoVioCalib getCalibRectify() {return m_calib_rectify;}

    bool rectified() {return m_rectified;}

private:
    bool m_rectified;
    StereoVioCalib m_calib_raw;
    StereoVioCalib m_calib_rectify;
    cv::Mat m_rmap[2][2];

    void calcRectify();
};

#endif//__VS_STEREO_CALIB_H__