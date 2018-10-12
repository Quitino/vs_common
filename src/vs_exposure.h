#ifndef __VS_EXPOSURE_H__
#define __VS_EXPOSURE_H__
#include <opencv2/opencv.hpp>
#include <memory>

namespace vs
{

class ExposureControl
{
public:
    ExposureControl(float tar_bright = 75, float max_expo = 100,
                    float init_expo = 10)
        : m_tar_bright(tar_bright), m_max_expo(max_expo)
        , m_cur_expo(init_expo), m_mode(0)
        , m_kp(0.2), m_ki(0), m_kd(0.05)
        , m_err(0), m_prev_err(0), m_pprev_err(0)
    {}

    void init(float tar_bright, float max_expo, float init_expo = 10)
    {
        m_tar_bright = tar_bright;
        m_max_expo = max_expo;
        m_cur_expo = init_expo;
    }

    void setPID(double kp, double ki = -1, double kd = -1)
    {
        if(kp >= 0) m_kp = kp;
        if(ki >= 0) m_ki = ki;
        if(kd >= 0) m_kd = kd;
    }

    float exposure(const cv::Mat& m, const cv::Mat& mask = cv::Mat())
    {
        float cur_bright = mask.rows == 0 ?
                           cv::mean(m).val[0] : cv::mean(m, mask).val[0];
        exposureAdjust(cur_bright);
        return m_cur_expo;
    }

private:
    float m_tar_bright;
    float m_max_expo;
    float m_cur_expo;
    int m_mode;
    float m_kp, m_ki, m_kd;
    float m_err, m_prev_err, m_pprev_err;

    enum AutoExposureMode
    {
        AE_MODE_PID = 0,
        AE_MODE_DEADZONE
    };

    void exposureAdjust(float cur_bright)
    {
        float m_err = m_tar_bright - cur_bright;

        // handle state matchine
        switch(m_mode)
        {
            case AE_MODE_PID:
            {
                if(fabs(m_err) < 1) m_mode = AE_MODE_DEADZONE;
                break;
            }
            case AE_MODE_DEADZONE:
            {
                float abserr = fabs(m_err);
                if(abserr > 5)
                {
                    m_mode = AE_MODE_PID;
                }
                break;
            }
        }

        // process mode
        float u = 0;
        switch(m_mode)
        {
            case AE_MODE_PID:
            {
                u = m_kp * m_err + m_kd * (m_err - m_prev_err);
                m_cur_expo += u;
                break;
            }
            case AE_MODE_DEADZONE:
                break;
        }

        // clip
        if(m_cur_expo < 1) m_cur_expo = 1;
        else if(m_cur_expo > m_max_expo) m_cur_expo = m_max_expo;

    #if 0 // only for debug, and must be close when releasing, since it will affect capture fps
        static FILE* fp = fopen((g_log_dir+"/exposure.txt").c_str(), "w");
        if(fp)
        {
            fprintf(fp, "%f %d %.1f %.0f %.1f %.1f\n",
                getCurTimestamp(), mode, cur_bright, m_cur_expo, err, u);
            fflush(fp);
        }
        // printf("%d %.1f %.0f %.1f %.1f\n", mode, cur_bright, m_cur_expo, err, u);
    #endif

        m_pprev_err = m_prev_err;
        m_prev_err = m_err;
    }
};

} /* namespace vs */
#endif//__VS_EXPOSURE_H__
