#ifndef __VS_TIME_BUFFER_H__
#define __VS_TIME_BUFFER_H__
#include <deque>
#include <vector>
#include <functional>
#include "vs_numeric.h"

namespace vs
{

template<class T>
class TimeBuffer
{
public:
    TimeBuffer()
    {
        foo_weighted_sum = [](double k1, const T& a1, double k2, const T& a2)
        {return k1 * a1 + k2 * a2;};
    }

    TimeBuffer(std::function<T(double,const T&,double,const T&)> f_weight_sum)
    {
        foo_weighted_sum = f_weight_sum;
    }

    T front() {return buffer.front().second;}

    T back() {return buffer.back().second;}

    double frontTs() {return buffer.front().first;}

    double backTs() {return buffer.back().first;}

    bool empty() {return buffer.empty();}

    void add(double ts, const T& a)
    {
        buffer.push_back(std::make_pair(ts, a));
    }

    bool get(double ts, T& res) const
    {
        int nbuf = buffer.size();
        if(nbuf < 2 || ts < buffer[0].first || ts > buffer.back().first)
            return false;
        int left = 0, right = nbuf;
        while(left < right)
        {
            int mid = (left + right) / 2;
            const auto& b_mid = buffer[mid];
            if(fequal(b_mid.first, ts))
            {
                res = b_mid.second;
                return true;
            }
            else if(b_mid.first < ts) left = mid + 1;
            else right = mid;
        }
        if(left != right || right == nbuf)
        {
            printf("[ERROR] l:%d != r:%d\n", left, right);
            return false;
        }
        const auto& a1 = buffer[left];
        if(fequal(a1.first, ts))
        {
            res = a1.second;
            return true;
        }
        const auto& a2 = a1.first > ts ? buffer[left - 1] : buffer[left + 1];
        double dt = a2.first - a1.first;
        if(fequal(dt, 0)) return false;
        double k = (ts - a1.first) / dt;
        if(k < 0 || k > 1) return false;
        res = foo_weighted_sum(k, a2.second, (1 - k), a1.second);
        return true;
    }

    // return data in time duration (start_ts, end_ts]
    std::vector<T> getRange(double start_ts, double end_ts)
    {
        // TODO:accelerate with binary search
        std::vector<T> res;
        for(const auto& item : buffer)
        {
            if(item.first <= start_ts) continue;
            else if(item.first > end_ts) break;
            res.push_back(item.second);
        }
        return res;
    }

    void dropOld(double old_ts)
    {
        if(buffer.empty()) return;
        auto it = buffer.begin();
        while(it->first < old_ts && it != buffer.end()) it++;
        buffer.erase(buffer.begin(), it); 
    }

    std::deque<std::pair<double, T>> buffer;

private:
    std::function<T(double,const T&,double,const T&)> foo_weighted_sum;
};

} /* namespace vs */
#endif//__VS_TIME_BUFFER_H__