#ifndef __VS_STEREO_CALIB_H__
#define __VS_STEREO_CALIB_H__
#include <opencv2/opencv.hpp>

namespace vs
{

struct StereoVioCalib
{
    cv::Size img_size;
    cv::Mat K0, K1, D0, D1;
    cv::Mat R_c0_c1, t_c0_c1; //rotation/translation point from camera left to camera right
    cv::Mat R_c0_imu, t_c0_imu; //rotation/translation point from camera left to body imu
    cv::Mat R_imu_body, t_imu_body;
    cv::Mat R_cam_gps, t_cam_gps;
    std::string distort_model;
    double time_delay;
    void deepCopy(const StereoVioCalib& rhs);
};

std::ostream& operator<<(std::ostream& os, const StereoVioCalib& c);

bool loadStereoVioCalib(const char* calib_file, StereoVioCalib& calib);

bool saveStereoVioCalib(const char* calib_file, const StereoVioCalib& calib);

class StereoRectifier
{
public:
    explicit StereoRectifier(bool enable_cl = false);
    explicit StereoRectifier(const char* calib_file, bool enable_cl = false);
    explicit StereoRectifier(const StereoVioCalib& calib_raw, bool enable_cl = false);

    bool init(const char* calib_file);
    bool init(const StereoVioCalib& calib_raw);

    void rectify(const cv::Mat& img0, const cv::Mat& img1,
                      cv::Mat& out0, cv::Mat& out1);

    StereoVioCalib getCalibRaw() {return m_calib_raw;}

    StereoVioCalib getCalibRectify() {return m_calib_rectify;}

    bool rectified() {return m_rectified;}

    void setOpenCL(bool enable) {m_enable_cl = enable;}

private:
    bool m_rectified;
    bool m_enable_cl;
    StereoVioCalib m_calib_raw;
    StereoVioCalib m_calib_rectify;
    cv::Mat m_rmap[2][2];
    cv::UMat m_urmap[2][2];

    void calcRectify();
};

} /* namespace vs */
#endif//__VS_STEREO_CALIB_H__