#ifndef __VS_TICTOC_H__
#define __VS_TICTOC_H__

#include <chrono>

class Timer
{
public:
    Timer(){start();}

    void start() { t1 = std::chrono::system_clock::now(); }

    void stop() { t2 = std::chrono::system_clock::now(); }

    double getSec() { return std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() * ks;}

    double getMsec() { return std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() * kms;}

private:
    std::chrono::system_clock::time_point t1,t2;
    const static double kms, ks;
};

void tictoc(const char* name);

void tic(const char* name);

/*return time between tic and toc [MS]*/
float toc(const char* name);

double getCurTimestamp();

#endif