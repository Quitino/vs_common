#include "vs_performance.h"
#include <map>

static std::map<std::string, PerfCounter> p_perf_counter;
static bool p_perf_enable = false;

void perf_enable(bool enable)
{
    p_perf_enable = enable;
}

void perf_begin(const char* name)
{
    if(p_perf_enable)
        p_perf_counter[std::string(name)].start();

}

void perf_end(const char* name)
{
    if(p_perf_enable)
        p_perf_counter[std::string(name)].stop();
}

void perf_print(const char* name, bool verbose)
{
    if(p_perf_enable)
    {
        if(name)
        {
            auto it = p_perf_counter.find(std::string(name));
            if(it == p_perf_counter.end())
            {
                printf("[ERROR] perf_avg(\"%s\"): no such performance.\n", name);
                return;
            }
            it->second.print(name, verbose);
        }
        else
        {
            for(auto it = p_perf_counter.begin(); it != p_perf_counter.end(); ++it)
            {
                it->second.print(it->first.c_str(), verbose);
            }
        }       
    }
}

float perf_avg(const char* name)
{
    if(!p_perf_enable) return 0;

    if(name == NULL)
    {
        printf("[ERROR] perf_avg: null name.\n");
        return -1;
    }
    auto it = p_perf_counter.find(std::string(name));
    if(it == p_perf_counter.end())
    {
        printf("[ERROR] perf_avg(\"%s\"): no such performance.\n", name);
        return -1;
    }
    return it->second.mean();
}
