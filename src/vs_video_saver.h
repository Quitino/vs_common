#ifndef __VS_VIDEO_SAVER_H__
#define __VS_VIDEO_SAVER_H__
#include <opencv2/opencv.hpp>
#include <deque>
#include <thread>
#include <unistd.h>

namespace vs
{

class VideoSaver
{
public:
    VideoSaver(cv::VideoWriter *writer = NULL)
        : vw_ptr(writer)
        , thread_ptr(new std::thread(std::bind(&VideoSaver::run, this)))
        , stop(false){}

    ~VideoSaver()
    {
        stop = true;
        thread_ptr->join();
        if(vw_ptr)
        {
            vw_ptr->release();
            vw_ptr = NULL;
        }
    }

    void setWriter(cv::VideoWriter *writer) {vw_ptr = writer;}

    void write(const cv::Mat& img)
    {
        img_vec.push_back(img);
    }

private:
    cv::VideoWriter *vw_ptr;
    std::shared_ptr<std::thread> thread_ptr;
    std::deque<cv::Mat> img_vec;
    bool stop;

    void run()
    {
        try
        {
            while(!stop)
            {
                if(vw_ptr == NULL || img_vec.empty())
                {
                    usleep(20000);
                    continue;
                }
                const cv::Mat& m = img_vec[0];
                if(!m.empty())
                {
                    if(m.channels()==1)
                    {
                        cv::Mat t;
                        cv::cvtColor(m, t, cv::COLOR_GRAY2BGR);
                        vw_ptr->write(t);
                    }
                    else
                    {
                        vw_ptr->write(m);
                    }
                }
                else
                {
                    printf("[ERROR]VideoSaver: image is empty, not saving.\n");
                }
                img_vec.pop_front();
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