#ifndef __VS_NUMERIC_H__
#define __VS_NUMERIC_H__
#include <cmath>
#include <stdio.h>

#define VS_PI_2     1.5707963267948966
#define VS_PI       3.14159265358979323846
#define VS_2PI      6.28318530717958647692
#define VS_RAD1     0.017453292519943
#define VS_RAD5     0.087266462599715
#define VS_RAD10    0.17453292519943
#define VS_RAD20    0.34906585039886
#define VS_RAD30    0.52359877559829
#define VS_RAD45    0.7853981633974351
#define VS_RAD60    1.04719755119658
#define VS_RAD90    1.5707963267948966
#define VS_RAD120   2.09439510239316

namespace vs
{

inline bool fequal(double a, double b, double eta = 1e-5)
{
    return fabs(a - b) < eta;
}

inline int sign(double d)
{
    return (d >= 0) ? 1 : -1;
}

template<class T1, class T2, class T3>
bool inRange(T1 a, T2 amin, T3 amax)
{
    return amin <= a && a <= amax;
}

template<typename T>
T min3(T a, T b, T c)
{
    T t = a < b ? a : b;
    return t < c ? t : c;
}

template<typename T>
T max3(T a, T b, T c)
{
    T t = a > b ? a : b;
    return t > c ? t : c;
}

template<typename T>
T min4(T a, T b, T c, T d)
{
    T x = a < b ? a : b;
    T y = c < d ? c : d;
    return x < y ? x : y;
}

template<typename T>
T max4(T a, T b, T c, T d)
{
    T x = a > b ? a : b;
    T y = c > d ? c : d;
    return x > y ? x : y;
}

template<typename T>
T sqsum(T a, T b)
{
    return a * a + b * b;
}

template<typename T>
T sqsum2(T a, T b)
{
    return a * a + b * b;
}

template<typename T>
T sqsum3(T a, T b, T c)
{
    return a * a + b * b + c * c;
}

template<typename T>
T sqsum4(T a, T b, T c, T d)
{
    return a * a + b * b + c * c + d * d;
}

template<typename T>
double hypot3(T a, T b, T c)
{
    return std::sqrt(sqsum3(a, b, c));
}

template<typename T>
double hypot4(T a, T b, T c, T d)
{
    return std::sqrt(sqsum4(a, b, c, d));
}

template<class T>
inline T clip(T val, T min_val, T max_val)
{
    return val < min_val ? min_val : (val > max_val ? max_val : val);
}

/** \brief Normalize angle in rad to range (-pi, pi]
    \param[in] angle input angle in rad
    \return normalized angle in (-pi, pi] equal to input angle
*/
inline double normalizeRad(double angle)
{
    angle = fmod(angle, VS_2PI);
    if (angle <= -VS_PI) angle += VS_2PI;
    else if (angle > VS_PI) angle -= VS_2PI;
    return angle;
}

/** \brief Normalize angle in deg to range (-180, 180]
    \param[in] angle input angle in deg
    \return normalized angle in (-180, 180] equal to input angle
*/
inline double normalizeDeg(double theta)
{
    theta = fmod(theta, 360);
    if (theta <= -180) theta += 360;
    else if (theta > 180) theta -= 360;
    return theta;
}

/** \brief convert angle from unit deg to unit rad
    \param[in] angle input angle in deg
    \return angle in rad equivalent to input angle
*/
inline double deg2rad(double angle)
{
    return angle * VS_RAD1;
}

/** \brief convert angle from unit rad to unit deg
    \param[in] angle input angle in rad
    \return angle in deg equivalent to input angle
*/
inline double rad2deg(double angle)
{
    return angle * 57.295779513082;
}

/** \brief Solve linear equation y=ax+b.
    Given 2 points (x1, y1), (x2, y2) in line and x,
    find the y in line corresponding to x.
    \param[in] x1 x of first point(x1, y1)
    \param[in] y1 y of first point(x1, y1)
    \param[in] x2 x of second point(x2, y2)
    \param[in] y2 y of second point(x2, y2)
    \param[in] x the x to be solved
    \return y y coresponding to x. (y-y1)/(x-x1) = (y2-y1)/(x2-x1) = (y-y2)/(x-x2);
*/
inline double linearInter(double x1, double y1, double x2, double y2, double x)
{
    if (fequal(x1, x2))
    {
        printf("[ERROR] linearInter: assert x1 != x2\n");
        return 0;
    }
    return (y2 - y1) / (x2 - x1)*(x - x1) + y1;
}

inline double linearClip(double xl, double yl, double xr, double yr, double x)
{
    if(xl <= xr)
    {
        if(x <= xl) return yl;
        else if(x >= xr) return yr;
    }
    else
    {
        if(x <= xr) return yr;
        else if(x >= xl) return yl;
    }
    return linearInter(xl, yl, xr, yr, x);
}

inline double radSub(double rad1, double rad2)
{
    return normalizeRad(rad1 - rad2);
}

inline double radAddW(double k1, double rad1, double k2, double rad2)
{
    double r2 = rad1 + normalizeRad(rad2 - rad1);
    return normalizeRad(k1 * rad1 + k2 * r2);
}

/** \return (-pi, pi] */
inline double angleDiffSigned(double rad1, double rad2)
{
    return normalizeRad(rad1 - rad2);
}

// Absolute value angle difference
/** \return (0, pi] */
inline double angleDiff(double rad1, double rad2)
{
    return fabs(angleDiffSigned(rad1, rad2));
}

/** \return (0, pi/2] */
inline double angleDiffAcute(double rad1, double rad2)
{
    double a = angleDiff(rad1, rad2);
    return a > VS_PI / 2.0 ? VS_PI - a : a;
}

} /* namespace vs */
#endif//__VS_NUMERIC_H__