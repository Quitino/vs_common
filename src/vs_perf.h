#ifndef __VS_PERFORMANCE_H__
#define __VS_PERFORMANCE_H__
#include <vector>
#include <float.h>
#include <stdio.h>
#include <deque>
#include "vs_tictoc.h"

namespace vs
{

class FpsCalculator
{
public:
    FpsCalculator(int queue_len = 10): m_queue(queue_len), m_queue_len(queue_len) {}

    void start() { m_timer.start(); }

    double stop()
    {
        double cost_ms = m_timer.stop();
        m_queue.push_back(cost_ms);
        if((int)m_queue.size() > m_queue_len) m_queue.pop_front();
        return cost_ms;
    }

    double fps()
    {
        if(m_queue.empty()) return -1;
        double s = 0;
        int c = 0;
        for(double a : m_queue)
        {
            s += a;
            c++;
        }
        s /= c;
        return s > 0 ? 1000.0 / s : -1;
    }

private:
    Timer m_timer;
    std::deque<double> m_queue;
    int m_queue_len;
};


class SCounter
{
public:
    SCounter():m_n(0), m_max(FLT_MIN), m_min(FLT_MAX), m_sum(0){}
    void add(float a)
    {
        m_n++;
        m_sum+=a;
        if(a<m_min) m_min = a;
        if(a>m_max) m_max = a;
    }

    float max() {return m_max;}
    float min() {return m_min;}
    float sum() {return m_sum;}
    float mean(){return m_n>0 ? m_sum/m_n : 0;}

private:
    int m_n;
    float m_max;
    float m_min;
    float m_sum;
};

class PerfCounter: public SCounter
{
public:
    void start(){timer.start();}

    void stop(){timer.stop(); add(timer.getMsec());}

    void print(const char* header = NULL, bool verbose = false)
    {
        if(header) printf("[%s]", header);
        if(verbose)
            printf("%.3f(%.3f, %.3f) ms.\n", mean(), min(), max());
        else
            printf("%.3f ms.\n", mean());
    }

private:
    Timer timer;
};

void perfEnable(bool enable);

void perfBegin(const char* name);

void perfEnd(const char* name);

void perfPrint(const char* name = NULL, bool verbose = false);

float perfAvg(const char* name);

inline void perfStart(const char* name)
{
    perfBegin(name);
}

inline void perfStop(const char* name)
{
    perfEnd(name);
}

} /* namespace vs */
#endif//__VS_PERFORMANCE_H__