#ifndef __VS_GEOMETRY2D_H__
#define __VS_GEOMETRY2D_H__
#include <opencv2/opencv.hpp>
#include <functional>
#include <set>
#include "vs_numeric.h"
#include "vs_vecutils.h"

namespace vs
{

inline cv::Point2f normalize(const cv::Point2f& a)
{
    return a / (cv::norm(a) + 0.0001f);
}

inline float dist(const cv::Point2f& p1, const cv::Point2f& p2)
{
    return hypotf(p1.x - p2.x, p1.y - p2.y);
}

inline float dist2line(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p)
{
    cv::Point2f d = p2 - p1;
    float len = hypotf(d.x, d.y);
    if(len < 1e-4) return dist(p, p1);
    return fabs(d.cross(p - p1) / len);
}

inline cv::Point2f project2line(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p)
{
    cv::Point2f d = p2 - p1;
    float len = hypotf(d.x, d.y);
    if(len < 1e-4) return p1;
    d /= len;
    return p1 + (p - p1).dot(d) * d;
}

/** \brief check if two line direction are parallel
    \param[in] dir1: UNIT direction vector
    \param[in] dir2: UNIT direction vector
    \param[in] thres_parallel: cosine between two direction. cos(10deg)=0.996
    \param[in] mode 0: not concern direction   '-- --'
                    1: same direction          '-> ->'
                    2: opposite direction      '<- ->'
*/
inline bool isParallel(const cv::Point2f& dir1, const cv::Point2f& dir2,
                       int mode = 0, float thres_parallel = 0.996f)
{
    float c = dir1.dot(dir2);
    switch(mode)
    {
        case 0: return fabs(c) > thres_parallel;
        case 1: return c > thres_parallel;
        case 2: return -c > thres_parallel;
    }
    return false;
}

/** \brief check if two line are coincidence
    \param[in] p1: point in one line
    \param[in] dir1: UNIT direction vector
    \param[in] p2: point in another line
    \param[in] dir2: UNIT direction vector
    \param[in] thres_parallel: cosine between two direction. cos(5deg)=0.996
    \param[in] mode 0: not concern direction   '-- --'
                    1: same direction          '-> ->'
                    2: opposite direction      '<- ->'
*/
inline bool isCoincidence(const cv::Point2f& p1, const cv::Point2f& dir1,
                          const cv::Point2f& p2, const cv::Point2f& dir2,
                          int mode = 0, float thres_parallel = 0.996f,
                          float thres_coincidence = 3.0f)
{
    return isParallel(dir1, dir2, mode, thres_parallel)
           && (fabs((p2 - p1).cross(dir1)) < thres_coincidence
               ||/*&&*/ fabs((p2 - p1).cross(dir2)) < thres_coincidence);
}

inline float overlapLen(const cv::Point2f& a1, const cv::Point2f& a2,
                        const cv::Point2f& b1, const cv::Point2f& b2)
{
    cv::Point2f dir_a = a2 - a1;
    float len_a = hypotf(dir_a.x, dir_a.y);
    if(len_a <= 0) return 0;
    dir_a /= len_a;
    float k1 = dir_a.dot(b1 - a1);
    float k2 = dir_a.dot(b2 - a1);
    return fabs(clip(k1, 0.0f, len_a) - clip(k2, 0.0f, len_a));
}

inline float overlapRate(const cv::Point2f& a1, const cv::Point2f& a2,
                         const cv::Point2f& b1, const cv::Point2f& b2,
                         int mode = 0)
{
    cv::Point2f dir_a = a2 - a1;
    float len_a = hypotf(dir_a.x, dir_a.y);
    if(len_a <= 0) return 0;
    dir_a /= len_a;
    float len_b = hypotf(b1.x - b2.x, b1.y - b2.y);
    if(len_b <= 0) return 0;
    float k1 = dir_a.dot(b1 - a1);
    float k2 = dir_a.dot(b2 - a1);
    float olen = fabs(clip(k1, 0.0f, len_a) - clip(k2, 0.0f, len_a));
    switch(mode)
    {
        case 0: return olen / len_a;
        case 1: return olen / len_b;
        case 2: return olen / std::min(len_a, len_b);
        case 3: return olen / std::max(len_a, len_b);
        default: return olen / (max3(k1, k2, len_a) - min3(k1, k2, 0.0f)); //iou
    }
}

inline bool isPerpendicular(const cv::Point2f& dir1, const cv::Point2f& dir2,
                            float thres_perpendi = 0.1736f)
{
    return fabs(dir1.dot(dir2)) < thres_perpendi;
}

inline bool isAngleInRange(const cv::Point2f& dir1, const cv::Point2f& dir2,
                           float cos_min, float cos_max)
{
    return inRange(dir1.dot(dir2), cos_min, cos_max);
}

/* \brief get the intersection of two lines*/
inline void lineIntersect(float x1, float y1, float x2, float y2,
                          float x3, float y3, float x4, float y4,
                          float& x, float& y)
{
    float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));
    if(fabs(d) > 0.0001f)
    {
        x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
        y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
    }
    else
    {
        x = y = -1;
    }
}

/* \brief get the intersection of two lines*/
inline cv::Point2f lineIntersect(const cv::Vec4f& a, const cv::Vec4f& b)
{
    cv::Point2f p;
    lineIntersect(a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3], p.x, p.y);
    return p;
}

/* \brief get the intersection of two lines*/
inline cv::Point2f lineIntersect(const cv::Point2f& p1, const cv::Point2f& dir1,
                                 const cv::Point2f& p2, const cv::Point2f& dir2)
{
    cv::Point2f p;
    lineIntersect(p1.x, p1.y, p1.x + dir1.x, p1.y + dir1.y,
                  p2.x, p2.y, p2.x + dir2.x, p2.y + dir2.y,
                  p.x, p.y);
    return p;
}

/* \brief angle of vector*/
inline float vecAngle(const cv::Point2f& a, const cv::Point2f& b)
{
    return acos(a.dot(b)/(cv::norm(a) * cv::norm(b)));
}

struct Arrow
{
    Arrow(){}
    Arrow(const cv::Point2f& _top, const cv::Point2f& _dir1, const cv::Point2f& _dir2,
          int _id1 = 0, int _id2 = 0): top(_top), dir1(_dir1), dir2(_dir2),
          id1(_id1), id2(_id2)
    {
        ray = normalize(dir1 + dir2);
        if(dir1.cross(dir2) < 0)
        {
            std::swap(dir1, dir2);
            std::swap(id1, id2);
        }
    }

    cv::Point2f top;  //top point
    cv::Point2f ray;  //unit direction vector of arrow center
    cv::Point2f dir1; //unit direction vector of one edge
    cv::Point2f dir2; //unit direction vector of another edge
    int id1; //line id of one edge
    int id2; //line id of another edge
};

template<class T>
T angleMean(const std::vector<T>& angles)
{
    T m = 0;
    if(angles.empty()) return m;
    T neg_min = 0.0;
    T neg_max = -9999.0;
    T pos_min = 9999.0;
    T pos_max = 0.0;

    for(auto a : angles)
    {
        if(a < 0)
        {
            if(a < neg_min) neg_min = a;
            if(a > neg_max) neg_max = a;
        }
        else
        {
            if(a < pos_min) pos_min = a;
            if(a > pos_max) pos_max = a;
        }
    }
    // has positive angles and negative angles
    //         |-PI --- neg_min --- neg_max ------ 0 ---- pos_min ----- pos_max --- PI|
    // normal: |        *******************************************************       |
    // cross:  |***************************               ****************************|
    if(neg_min <= neg_max && pos_min <= pos_max
        && VS_2PI - (pos_min - neg_max) < pos_max - neg_min)
    {
        T sum = 0;
        for(auto& a : angles)
        {
            sum += a < 0 ? a + VS_2PI : a;
        }
        return sum / angles.size();
    }
    else
        return vecMean(angles);
}

template <class T>
void merge(std::vector<T>& list,
            std::function<bool(const T& a, const T& b)> foo_need_merge,
            std::function<T(const T& a, const T& b)> foo_merge)
{
    int cnt = list.size();
    if(cnt < 2) return;

    std::vector<T> new_list;
    new_list.reserve(cnt);

    std::set<int> ids;
    for(int i = 0; i < cnt; i++)
        ids.insert(i);

    while(!ids.empty())
    {
        auto it = ids.begin();
        T a = list[*it];
        ids.erase(it);
        bool no_merge = false;
        while(!no_merge)
        {
            no_merge = true;
            for(auto i : ids)
            {
                T li = list[i];
                if(foo_need_merge(a, li))
                {
                    a = foo_merge(a, li);
                    ids.erase(i);
                    no_merge = false;
                    break;
                }
            }
        }
        new_list.push_back(a);
    }
    list = new_list;
}

inline void mergeLine(std::vector<cv::Vec4f>& lines, int mode = 0,
                      float thres_parallel = 0.996f, float thres_coincidence = 3.0f,
                      float thres_overlap = 0.0f, float thres_connect = 1.0f)
{
    auto foo_need_merge = [mode, thres_parallel, thres_coincidence,
                           thres_overlap, thres_connect]
        (const cv::Vec4f& a, const cv::Vec4f& b)
    {
        cv::Point2f a1(a[0], a[1]), a2(a[2], a[3]);
        cv::Point2f b1(b[0], b[1]), b2(b[2], b[3]);
        cv::Point2f da = a2 - a1;
        cv::Point2f db = b2 - b1;
        float lena = hypotf(da.x, da.y);
        if(lena <= 1e-4) return false;
        float lenb = hypotf(db.x, db.y);
        if(lenb <= 1e-4) return false;
        da /= lena;
        db /= lenb;
        cv::Point2f ca = (a1 + a2) / 2;
        cv::Point2f cb = (b1 + b2) / 2;
    #if 0
        std::cout<<a<<" "<<b<<" : ";
        printf("%d(%.3f %.1f %.1f)&(%.2f|%.2f|%.2f|%.2f|%.2f)\n",
            isCoincidence(ca, da, cb, db, mode, thres_parallel, thres_coincidence),
            da.dot(db), fabs((ca - cb).cross(da)), fabs((ca - cb).cross(db)),
            overlapLen(a1, a2, b1, b2), dist(a1, b1), dist(a1, b2), dist(a2, b1), dist(a2, b2));
    #endif
        return isCoincidence(ca, da, cb, db, mode, thres_parallel, thres_coincidence)
                && (overlapLen(a1, a2, b1, b2) > thres_overlap
                 || dist(a1, b1) < thres_connect || dist(a1, b2) < thres_connect
                 || dist(a2, b1) < thres_connect || dist(a2, b2) < thres_connect);
    };

    auto foo_merge = [](const cv::Vec4f& a, const cv::Vec4f& b)
    {
        cv::Point2f a1(a[0], a[1]), a2(a[2], a[3]);
        cv::Point2f b1(b[0], b[1]), b2(b[2], b[3]);
        cv::Point2f da = a2 - a1;
        cv::Point2f db = b2 - b1;
        float lena = hypotf(da.x, da.y);
        float lenb = hypotf(db.x, db.y);
        da /= lena;
        db /= lenb;

        float kmin = 0.0f;
        float kmax = 1.0f;
        float k1 = (b1 - a1).dot(da) / lena;
        float k2 = (b2 - a1).dot(da) / lena;
        if(kmin > k1) kmin = k1;
        if(kmax < k1) kmax = k1;
        if(kmin > k2) kmin = k2;
        if(kmax < k2) kmax = k2;
        cv::Point2f p1, p2;
        if(fequal(kmin, 0)) p1 = a1;
        else if(fequal(kmin, k1)) p1 = b1;
        else p1 = b2;
        if(fequal(kmax, 1)) p2 = a2;
        else if(fequal(kmax, k1)) p2 = b1;
        else p2 = b2;
        return cv::Vec4f(p1.x, p1.y, p2.x, p2.y);
    };
    merge<cv::Vec4f>(lines, foo_need_merge, foo_merge);
}

} /* namespace vs */
#endif//__VS_GEOMETRY2D_H__