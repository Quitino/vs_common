#ifndef __VS_VIDEO_SAVER_H__
#define __VS_VIDEO_SAVER_H__
#include <opencv2/opencv.hpp>
#include <deque>
#include <thread>
#include <unistd.h>

class VideoSaverThread
{
public:
    VideoSaverThread(cv::VideoWriter *writer = NULL)
        : vw_ptr(writer)
        , thread_ptr(new std::thread(std::bind(&VideoSaverThread::threadFoo, this))){}

    void setWriter(cv::VideoWriter *writer) {vw_ptr = writer;}

    void write(const cv::Mat& img)
    {
        img_vec.push_back(img);
    }

    void close()
    {
        while(!img_vec.empty()){usleep(1000);}
        vw_ptr->release();
        vw_ptr = NULL;
    }

private:
    cv::VideoWriter *vw_ptr;
    std::shared_ptr<std::thread> thread_ptr;
    std::deque<cv::Mat> img_vec;

    void threadFoo()
    {
        while(1)
        {
            if(vw_ptr == NULL || img_vec.empty())
            {
                usleep(2000);
                continue;
            }
            vw_ptr->write(img_vec[0]);
            img_vec.pop_front();
        }
    }
};


#endif//__VS_VIDEO_SAVER_H__