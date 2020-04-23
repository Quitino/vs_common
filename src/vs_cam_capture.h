#ifndef __VS_CAM_CAPTURE_H__
#define __VS_CAM_CAPTURE_H__
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "vs_tictoc.h"
#include "vs_video_saver.h"
#include "vs_data_saver.h"
#include "vs_data_struct.h"

namespace vs
{

class CamCapture
{
public:
    CamCapture(): m_init(false), m_start(false), m_exit(false), m_idx(0), m_buffer(64)
    {}

    CamCapture(int device, const char* fsave = NULL,
               cv::Size set_size = cv::Size(0, 0))
        : m_init(false), m_start(false), m_exit(false), m_idx(0), m_buffer(64)
    {
        init(device, fsave, set_size);
    }

    CamCapture(const char* device, const char* fsave = NULL,
               cv::Size set_size = cv::Size(0, 0))
        : m_init(false), m_start(false), m_exit(false), m_idx(0), m_buffer(64)
    {
        init(device, fsave, set_size);
    }

    virtual ~CamCapture()
    {
        m_exit = true;
        if(m_cap_thread)
        {
            m_cap_thread->join();
            delete m_cap_thread;
        }
        if(m_vw_thread) delete m_vw_thread;
    }

    bool init(int device, const char* fsave = NULL,
              cv::Size set_size = cv::Size(0, 0))
    {
        m_cap.open(device);
        if(!m_cap.isOpened())
        {
            printf("[ERROR] cannot open camera %d\n", device);
            return false;
        }
        m_init = initElse(fsave, set_size);
        return m_init;
    }

    bool init(const char* device, const char* fsave = NULL,
              cv::Size set_size = cv::Size(0, 0))
    {
        m_cap.open(device);
        if(!m_cap.isOpened())
        {
            printf("[ERROR] cannot open camera %s\n", device);
            return false;
        }
        m_init = initElse(fsave, set_size);
        return m_init;
    }

    void start(){m_start = true;}

    void stop(){m_start = false;}

    double getLatest(cv::Mat& img)
    {
        if(!m_init)
        {
            printf("[ERROR]Get data failed. Init first.\n");
            return 0;
        }
        if(!m_start)
        {
            printf("[ERROR]Get data failed. Capture not start.\n");
            return 0;
        }
        if(m_idx <= 0)
        {
            usleep(1e5);
            if(m_idx <= 0)
            {
                printf("[WARN]Get data failed. Buffer is empty.\n");
                return 0;
            }
        }
        auto& p = m_buffer[m_idx - 1];
        img = p.second;
        return p.first;
    }

    double read(cv::Mat& img)
    {
        return getLatest(img);
    }


private:
    bool                            m_init;
    bool                            m_start;
    bool                            m_exit;
    int                             m_idx;
    cv::VideoCapture                m_cap;
    cv::VideoWriter                 m_vw;
    VideoSaver*                     m_vw_thread;
    std::thread*                    m_cap_thread;
    std::string                     m_imgts_file;
    cv::Size                        m_img_size;
    FastCircularQueue<std::pair<double, cv::Mat>>   m_buffer;
    std::shared_ptr<DataSaver<double>>              m_ts_saver;

    virtual bool initElse(const char* fsave, cv::Size set_size)
    {
        if(!m_cap.isOpened()) return false;

        if(set_size.width != 0 || set_size.height != 0)
        {
            m_cap.set(CV_CAP_PROP_FRAME_WIDTH, set_size.width);
            m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, set_size.height);
        }

        cv::Mat img;
        if(m_cap.read(img))
            m_img_size = img.size();
        else
        {
            printf("[ERROR] read image failed\n");
            return false;
        }

        if(fsave)
        {
            m_vw.open(fsave, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),
                      m_cap.get(CV_CAP_PROP_FPS), m_img_size);
            if(!m_vw.isOpened())
            {
                printf("[ERROR] cannot open video file %s\n", fsave);
                return false;
            }
            m_vw_thread = new VideoSaver(&m_vw);
            m_imgts_file = std::string(fsave) + ".txt";
        }

        m_cap_thread = new std::thread(std::bind(&CamCapture::captureThread, this));
        return true;
    }

    virtual bool capture()
    {
        if(!m_start) return false;
        auto& data = m_buffer[m_idx];
        if(!m_cap.read(data.second)) {return false;}
        data.first = getSysTs();
        m_idx++;
        
        if(m_vw_thread) m_vw_thread->write(data.second);
        if(m_imgts_file.length() > 0)
        {
            if(!m_ts_saver.get())
            {
                m_ts_saver.reset(new DataSaver<double>(m_imgts_file.c_str(),
                                [](FILE* fp, const double& d){fprintf(fp, "%f\n", d);}));
            }
            m_ts_saver->push(data.first);
        }
        return true;
    }

    void captureThread()
    {
        while(!m_exit)
        {
            capture();
            usleep(5e3);
        }
    }
};

} /* namespace vs */

#endif//__VS_CAM_CAPTURE_H__