#ifndef __VS_PERFORMANCE_H__
#define __VS_PERFORMANCE_H__
#include <vector>
#include <float.h>
#include <stdio.h>
#include "vs_tictoc.h"

class SCounter
{
public:
    SCounter():m_n(0),m_max(FLT_MIN),m_min(FLT_MAX),m_sum(0){}
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
            printf("%.3f(%.3f,%.3f) ms.\n", mean(), min(), max());
        else
            printf("%.3f ms.\n", mean());
    }

private:
    Timer timer;
};

/***************** C api *****************/
void perf_enable(bool enable);

void perf_begin(const char* name);

void perf_end(const char* name);

void perf_print(const char* name = NULL, bool verbose = false);

float perf_avg(const char* name);

inline void perf_start(const char* name)
{
    perf_begin(name);
}

inline void perf_stop(const char* name)
{
    perf_end(name);
}

/***************** C++ api *****************/
inline void perfEnable(bool enable)
{
    perf_enable(enable);
}

inline void perfBegin(const char* name)
{
    perf_begin(name);
}

inline void perfEnd(const char* name)
{
    perf_end(name);
}

inline void perfPrint(const char* name = NULL, bool verbose = false)
{
    perf_print(name, verbose);
}

inline float perfAvg(const char* name)
{
    return perf_avg(name);
}

inline void perfStart(const char* name)
{
    perf_begin(name);
}

inline void perfStop(const char* name)
{
    perf_end(name);
}

#endif//__VS_PERFORMANCE_H__