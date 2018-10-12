#ifndef __VS_VIZ3D_H__
#define __VS_VIZ3D_H__
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>
#include <map>

namespace vs
{

class Viz3D
{
public:
    Viz3D()
        : thread_ptr(new std::thread(std::bind(&Viz3D::run, this)))
        , stop(false)
    {
        viz.setBackgroundColor();
    }

    ~Viz3D()
    {
        stop = true;
        thread_ptr->join();
        viz.removeAllWidgets();
        widget_table.clear();
    }

    void updateWidget(const std::string& id, const cv::viz::Widget& w)
    {
        widget_table[id] = w;
    }

private:
    cv::viz::Viz3d viz;
    std::map<std::string, cv::viz::Widget> widget_table;
    std::shared_ptr<std::thread> thread_ptr;
    bool stop;

    void run()
    {
        try
        {
            while(!viz.wasStopped() && !stop)
            {
                if(!widget_table.empty())
                {
                    for(const auto& m : widget_table)
                    {
                        viz.showWidget(m.first, m.second);
                    }
                    viz.spinOnce();
                }
                usleep(100000);
            }
        }
        catch(...)
        {
            printf("[ERROR]:Viz3d thread quit expectedly.\n");
        }
    }
};

} /* namespace vs */
#endif//__VS_VIZ3D_H__