#ifndef __VS_VIDEO_SAVER_H__
#define __VS_VIDEO_SAVER_H__
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <vector>
#include <unistd.h>

namespace vs
{

class VideoSaver
{
public:
    VideoSaver(cv::VideoWriter *writer = NULL)
        : m_vw_ptr(writer)
        , m_thread_ptr(new std::thread(std::bind(&VideoSaver::run, this)))
        , m_stop(false)
    {
        m_img_vec.reserve(20);
    }

    ~VideoSaver()
    {
        m_stop = true;
        m_thread_ptr->join();
        if(m_vw_ptr)
        {
            m_vw_ptr->release();
            m_vw_ptr = NULL;
        }
    }

    void setWriter(cv::VideoWriter *writer) {m_vw_ptr = writer;}

    void write(const cv::Mat& img)
    {
        m_mtx.lock();
        m_img_vec.push_back(img.clone());
        m_mtx.unlock();
    }

private:
    cv::VideoWriter *m_vw_ptr;
    std::shared_ptr<std::thread> m_thread_ptr;
    std::vector<cv::Mat> m_img_vec;
    std::mutex m_mtx;
    bool m_stop;

    void run()
    {
        try
        {
            while(!m_stop)
            {
                if(!m_vw_ptr)
                {
                    usleep(20000);
                    continue;
                }
                m_mtx.lock();
                auto img_vec = m_img_vec;
                m_img_vec.clear();
                m_mtx.unlock();
                if(!img_vec.empty())
                {
                    for(const auto& m : img_vec)
                    {
                        if(m.empty())
                        {
                            printf("[ERROR]VideoSaver: image is empty, not saving.\n");
                            continue;
                        }
                        else if(m.channels() == 1)
                        {
                            cv::Mat t;
                            cv::cvtColor(m, t, cv::COLOR_GRAY2BGR);
                            m_vw_ptr->write(t);
                        }
                        else
                        {
                            m_vw_ptr->write(m);
                        }
                    }
                }
                usleep(500000);
            }
        }
        catch(...)
        {
            printf("[ERROR]:Video saver thread quit expectedly.\n");
        }
    }
};

} /* namespace vs */
#endif//__VS_VIDEO_SAVER_H__