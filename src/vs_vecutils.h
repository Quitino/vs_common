#ifndef __VS_VEC_UTILS_H__
#define __VS_VEC_UTILS_H__
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdio.h>

namespace vs
{
/** \brief minimum value of vector*/
template <class T>
T vecMin(const std::vector<T> &a)
{
    T m = 0;
    if(a.empty()) return m;
    m = a[0];
    for(auto i : a) if(m > i) m = i;
    return m;
}

/** \brief maximum value of vector*/
template <class T>
T vecMax(const std::vector<T> &a)
{
    T m = 0;
    if(a.empty()) return m;
    m = a[0];
    for(auto i : a) if(m < i) m = i;
    return m;
}

/** \brief maximum value of vector*/
template <class T>
void vecMinMax(const std::vector<T> &a, T& amin, T& amax)
{
    amin = 0;
    amax = 0;
    if(a.empty()) return;
    amin = amax = a[0];
    for(auto i: a)
    {
        if(amin > i) amin = i;
        else if(amax < i) amax = i;
    }
}

/** \brief is all value in vecor zero.*/
template <class T>
bool vecAllZero(const std::vector<T> &a)
{
    for (size_t i = 0; i<a.size(); i++)
        if (fabs((double)a[i])>1e-8) return false;
    return true;
}

/** \brief find the kth largest number in vector.*/
template<class T>
T findKth(const std::vector<T> &vec, int k)
{
    if (vec.empty()) return T();
    int N = vec.size();
    if(k < 0) k = 0;
    else if(k > N-1) k = N-1;
    std::vector<T> a(vec.begin(), vec.end());
    std::nth_element(a.begin(), a.begin() + k, a.end());
    return a[k];
}

template<class T>
T vecMedian(const std::vector<T>& vec)
{
    return findKth(vec, vec.size()/2);
}

// 0:L0 norm 1:L1 norm 2:L2 norm -1:L_oo norm
template<class T>
double vecNorm(const std::vector<T>& a, int l = 2)
{
    double res = 0;
    switch(l)
    {
        case 0: for(auto i : a) if(i) res++; break;
        case 1: for(auto i : a) res += fabs(i); break;
        case 2: for(auto i : a) res += i * i; res = std::sqrt(res); break;
        case -1: res = vecMax(a); break;
        default: printf("[ERROR]vecNorm: invalid l %d, set(0,1,2,-1)\n", l);
    }
    return res;
}

/** \brief the sum of all values in vector*/
template<class T>
T vecSum(const std::vector<T>& vec)
{
    T sum = T(0);
    for(const auto& i : vec) sum += i;
    return sum;
}

/** \brief the mean of all values in vector*/
template<class T>
T vecMean(const std::vector<T> &vec)
{
    T a = T(0);
    if (vec.empty()) return a;
    T s = vecSum(vec);
    return s / (double)vec.size();
}

/** \brief the count of all non-zero values in vector*/
template<class T>
int vecCount(const std::vector<T> &vec)
{
    int cnt = 0;
    for(const auto& i : vec)
        if(i) cnt++;
    return cnt;
}

/** \brief any values is no zero*/
template<class T>
bool vecAny(const std::vector<T> &vec)
{
    int cnt = 0;
    for(const auto& i : vec)
        if(i) return true;
    return false;
}

/** \brief all values is no zero*/
template<class T>
bool vecAll(const std::vector<T> &vec)
{
    int cnt = 0;
    for(const auto& i : vec)
        if(i == 0) return false;
    return true;
}

/** \brief convert vector to normalize vector, which sum to 1.*/
template<class T>
void vecNormalize(std::vector<T> &vec)
{
    if (vec.empty())return;
    T sum = vecSum(vec);
    if (sum == 0) return;
    for (auto it = vec.begin(); it < vec.end(); ++it)
    {
        *it /= sum;
    }
}

/** \brief compute the mean and variance of vector.*/
template<class T>
void vecStatistics(const std::vector<T> &vec, T& mean_, T& variance)
{
    mean_ = 0;
    variance = 0;
    if (vec.empty()) return;
    mean_ = vecMean(vec);
    if (vec.size() == 1) return;
    for (auto it = vec.begin(); it < vec.end(); ++it)
        variance += (*it - mean_) * (*it - mean_);
    variance /= vec.size() - 1;
}

/** \brief Get subset of a vector with begin index and length.
    the similar use to substr() in std::string */
template <class T>
std::vector<T> subvec(const std::vector<T>& vec, int beg, int len = -1)
{
    std::vector<T> res;
    if (len == -1) res.insert(res.begin(), vec.begin() + beg, vec.end());
    else res.insert(res.begin(), vec.begin() + beg, vec.begin() + beg + len);
    return res;
}

/** \brief dot/inner product between a and b*/
template <class T>
T vecDot(const std::vector<T>& a, const std::vector<T>& b)
{
    int n = a.size();
    if(n <= 0) return T(0);
    if(b.size() != n)
    {
        printf("[ERROR]vecDot: size not match(%d, %d).\n", n, (int)b.size());
        return T(0);
    }
    T res = 0;
    for(int i = 0; i < n; i++)
        res += a[i] * b[i];
    return res;
}

/** \brief a[i] = a[i]*b[i] */
template <class T>
void vecElemMul(std::vector<T>& a, const std::vector<T>& b)
{
    int n = a.size();
    if(n <= 0) return;
    if(b.size() != n)
    {
        printf("[ERROR]vecElemMul: size not match(%d, %d).\n", n, (int)b.size());
        return;
    }
    for(int i = 0; i < n; i++)
        a[i] *= b[i];
}

/** \brief a[i] = a[i]+b[i] */
template <class T>
void vecElemAdd(std::vector<T>& a, const std::vector<T>& b)
{
    int n = a.size();
    if(n <= 0) return;
    if(b.size() != n)
    {
        printf("[ERROR]vecElemAdd: size not match(%d, %d).\n", n, (int)b.size());
        return;
    }
    for(int i = 0; i < n; i++)
        a[i] += b[i];
}

/** \brief a[i] = a[i]-b[i] */
template <class T>
void vecElemSub(std::vector<T>& a, const std::vector<T>& b)
{
    int n = a.size();
    if(n <= 0) return;
    if(b.size() != n)
    {
        printf("[ERROR]vecElemMul: size not match(%d, %d).\n", n, (int)b.size());
        return;
    }
    for(int i = 0; i < n; i++)
        a[i] -= b[i];
}

/** \brief a[i] = a[i]/b[i] */
template <class T>
void vecElemDiv(std::vector<T>& a, const std::vector<T>& b)
{
    int n = a.size();
    if(n <= 0) return;
    if(b.size() != n)
    {
        printf("[ERROR]vecElemAdd: size not match(%d, %d).\n", n, (int)b.size());
        return;
    }
    for(int i = 0; i < n; i++)
        a[i] /= b[i];
}

/** \brief a[i] = a[i]+b */
template <class T, class T2>
void vecAdd(std::vector<T>& a, T2 b)
{
    if(a.empty()) return;
    for(size_t i = 0; i < a.size(); i++)
        a[i] += b;
}

/** \brief a[i] = a[i]-b */
template <class T, class T2>
void vecSub(std::vector<T>& a, T2 b)
{
    if(a.empty()) return;
    for(size_t i = 0; i < a.size(); i++)
        a[i] -= b;
}

/** \brief a[i] = a[i]*b */
template <class T, class T2>
void vecMul(std::vector<T>& a, T2 b)
{
    if(a.empty()) return;
    for(size_t i = 0; i < a.size(); i++)
        a[i] *= b;
}

/** \brief a[i] = a[i]/b */
template <class T, class T2>
void vecDiv(std::vector<T>& a, T2 b)
{
    if(a.empty()) return;
    for(size_t i = 0; i < a.size(); i++)
        a[i] /= b;
}

template <class T, class T2>
int vecCmpCnt(const std::vector<T>& a, T2 thres)
{
    int cnt = 0;
    for(auto i : a)
        if(i > thres) cnt++;
    return cnt;
}

} /* namespace vs */
#endif//__VS_VEC_UTILS_H__