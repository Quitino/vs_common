#include "vs_improc.h"
#include "vs_numeric.h"
#include <functional>

namespace vs
{

static cv::Mat grabBgr(const cv::Mat& bgr,
                       const std::function<uchar(const cv::Vec3b& a)> &foo)
{
    cv::Mat res(bgr.size(), CV_8UC1);
    for(int i=0; i<bgr.rows; i++)
    {
        const cv::Vec3b *p_bgr = bgr.ptr<cv::Vec3b>(i);
        uchar *p_hue = res.ptr<uchar>(i);
        for(int j=0; j<bgr.cols; j++)
        {
            *p_hue++ = foo(*p_bgr++);
        }
    }
    return res;
}

static auto foo_bgr2hue = [](const cv::Vec3b& a)
{
    static int hdiv_table[256];
    static bool initialized = false;
    if(!initialized)
    {
        hdiv_table[0] = 0;
        for(int i = 1; i < 256; i++)
            hdiv_table[i] = cv::saturate_cast<int>((180 << 12)/(6.*i));
        initialized = true;
    }
    int b = a[0];
    int g = a[1];
    int r = a[2];
    int vmin = min3(b, g, r);
    int v = max3(b, g, r);
    int diff = v - vmin;
    int vr = v == r ? -1 : 0;
    int vg = v == g ? -1 : 0;
    int h = (vr & (g - b)) +
        (~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
    h = (h * hdiv_table[diff] + (1 << 11)) >> 12;
    h += h < 0 ? 180 : 0;
    return cv::saturate_cast<uchar>(h);
};

static auto foo_bgr2saturation = [](const cv::Vec3b& a)
{
    static int sdiv_table[256];
    static bool initialized = false;
    if(!initialized)
    {
        sdiv_table[0] = 0;
        for(int i = 1; i < 256; i++)
            sdiv_table[i] = cv::saturate_cast<int>((255 << 12)/(1.*i));
        initialized = true;
    }
    int b = a[0];
    int g = a[1];
    int r = a[2];
    int vmin = min3(b, g, r);
    int v = max3(b, g, r);
    int diff = v - vmin;
    int s = (diff * sdiv_table[v] + (1 << (11))) >> 12;
    return (uchar)s;
};

cv::Mat bgr2hue(const cv::Mat& bgr)
{
    return grabBgr(bgr, foo_bgr2hue);
}

cv::Mat bgr2saturation(const cv::Mat& bgr)
{
    return grabBgr(bgr, foo_bgr2saturation);
}

cv::Mat grabRed(const cv::Mat& bgr)
{
    cv::Mat red(bgr.rows, bgr.cols, CV_8UC1);
    for(int i=0; i<bgr.rows; i++)
    {
        const uchar *p_bgr = bgr.ptr<uchar>(i);
        uchar *p_hue = red.ptr<uchar>(i);
        for(int j=0; j<red.cols; j++)
        {
            uchar b = *p_bgr++;
            uchar g = *p_bgr++;
            uchar r = *p_bgr++;
            uchar amin = min3(b, g, r);
            uchar amax = max3(b, g, r);
            uchar h;
            if(amax == amin)
                // h = 0;
                h = 255;
            else if(amax == r)
                h = ((int)g-b)*30/(amax-amin) + (g >= b ? 0:180);
            else if(amax == g)
                h = ((int)b-r)*30/(amax-amin) + 60;
            else
                h = ((int)r-g)*30/(amax-amin) + 120;
            if(/*(0<=h && h<=10) ||*/ (156<=h && h<=180))
                *p_hue++ = 255;
            else
                *p_hue++ = 0;
        }
    }
    return red;
}

cv::Mat grabRedMl(const cv::Mat& bgr)
{
    /** net
    layers 4
    layer 0 Dense
    3 2
    [ 0.30742466 -0.34175044]
    [-0.57303673 -0.8127285 ]
    [ 0.26552096 -0.96885884]
    [-1.4555163  0.       ]
    layer 1 Activation
    relu
    layer 2 Dense
    2 2
    [-1.5953918  -0.48820382]
    [1.1496252 0.7166755]
    [ 2.8792114 -2.8792102]
    layer 3 Activation
    softmax
    */
    cv::Mat red(bgr.rows, bgr.cols, CV_8UC1);
    for(int i=0; i<bgr.rows; i++)
    {
        const uchar *p_bgr = bgr.ptr<uchar>(i);
        uchar *p_hue = red.ptr<uchar>(i);
        for(int j=0; j<red.cols; j++)
        {
            uchar b = *p_bgr++;
            uchar g = *p_bgr++;
            uchar r = *p_bgr++;
            float m1 = std::max(0.30742466*b - 0.57303673*g + 0.26552096*r - 1.4555163, 0.0);
            float m2 = std::max(-0.34175044*b - 0.8127285*g - 0.96885884*r, 0.0);
            float o1 = -1.5953918*m1 + 1.1496252*m2 + 2.8792114;
            float o2 = -0.48820382*m1 + 0.7166755*m2 -2.8792102;
            *p_hue++ = o1 < o2 ? 255 : 0;
        }
    }
    return red;
}

void regionFilter(cv::Mat& img, int minRegion, int maxRegion)
{
    typedef cv::Point_<int16_t> Point2s;
    int width = img.cols, height = img.rows, npixels = width*height;
    size_t bufSize = npixels*(int)(sizeof(Point2s) + sizeof(int) + sizeof(uchar));
    cv::Mat _buf;
    _buf.create(1, (int)bufSize, CV_8U);
    uchar* buf = _buf.ptr();
    int i, j, dstep = (int)(img.step/sizeof(uchar));
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf[0]);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for( i = 0; i < height; i++ )
    {
        uchar* ds = img.ptr<uchar>(i);
        int* ls = labels + width*i;

        for( j = 0; j < width; j++ )
        {
            if( ds[j] != 0 )   // not a bad disparity
            {
                if( ls[j] )     // has a label, check for bad label
                {
                    if( rtype[ls[j]] ) // small region, zero out disparity
                        ds[j] = (uchar)0;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((int16_t)j, (int16_t)i);  // current pixel
                    curlabel++; // next label
                    int count = 0;  // current region size
                    ls[j] = curlabel;

                    // wavefront propagation
                    while( ws >= wbuf ) // wavefront not empty
                    {
                        count++;
                        // put neighbors onto wavefront
                        uchar* dpp = &img.at<uchar>(p.y, p.x);
                        // uchar dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != 0)
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y+1);
                        }

                        if( p.y > 0 && !lpp[-width] && dpp[-dstep] != 0)
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y-1);
                        }

                        if( p.x < width-1 && !lpp[+1] && dpp[+1] != 0)
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x+1, p.y);
                        }

                        if( p.x > 0 && !lpp[-1] && dpp[-1] != 0)
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x-1, p.y);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if(count<minRegion || count>maxRegion)
                    {
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (uchar)0;
                    }
                    else
                    {
                        rtype[ls[j]] = 0;   // large region label
                    }
                }
            }
        }
    }
    return;
}


void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes, int minRegion, int maxRegion)
{
    typedef cv::Point_<int16_t> Point2s;
    bboxes.clear();
    int width = img.cols, height = img.rows, npixels = width*height;
    size_t bufSize = npixels*(int)(sizeof(Point2s) + sizeof(int) + sizeof(uchar));
    cv::Mat _buf;
    _buf.create(1, (int)bufSize, CV_8U);
    uchar* buf = _buf.ptr();
    int i, j, dstep = (int)(img.step/sizeof(uchar));
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf[0]);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for( i = 0; i < height; i++ )
    {
        uchar* ds = img.ptr<uchar>(i);
        int* ls = labels + width*i;

        for( j = 0; j < width; j++ )
        {
            if( ds[j] != 0 )   // not a bad disparity
            {
                if( ls[j] )     // has a label, check for bad label
                {
                    if( rtype[ls[j]] ) // small region, zero out disparity
                        ds[j] = (uchar)0;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((int16_t)j, (int16_t)i);  // current pixel
                    curlabel++; // next label
                    int count = 0;  // current region size
                    ls[j] = curlabel;
                    std::vector<Point2s> pt_list;

                    // wavefront propagation
                    while( ws >= wbuf ) // wavefront not empty
                    {
                        count++;
                        pt_list.push_back(p);
                        // put neighbors onto wavefront
                        uchar* dpp = &img.at<uchar>(p.y, p.x);
                        // uchar dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != 0)
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y+1);
                        }

                        if( p.y > 0 && !lpp[-width] && dpp[-dstep] != 0)
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y-1);
                        }

                        if( p.x < width-1 && !lpp[+1] && dpp[+1] != 0)
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x+1, p.y);
                        }

                        if( p.x > 0 && !lpp[-1] && dpp[-1] != 0)
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x-1, p.y);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if(count<minRegion || count>maxRegion)
                    {
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (uchar)0;
                    }
                    else
                    {
                        // check bbox
                        int min_x = 60000, min_y = 60000, max_x = 0, max_y = 0;
                        for(const auto& tp : pt_list)
                        {
                            if(min_x > tp.x) min_x = tp.x;
                            else if(max_x < tp.x) max_x = tp.x;
                            if(min_y > tp.y) min_y = tp.y;
                            else if(max_y < tp.y) max_y = tp.y;
                        }
                        bboxes.push_back(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));
                        rtype[ls[j]] = 0;   // large region label
                    }
                }
            }
        }
    }
    return;
}

void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes,
                  std::vector<cv::Point2f>& centers, int minRegion, int maxRegion)
{
    typedef cv::Point_<int16_t> Point2s;
    bboxes.clear();
    centers.clear();
    int width = img.cols, height = img.rows, npixels = width*height;
    size_t bufSize = npixels*(int)(sizeof(Point2s) + sizeof(int) + sizeof(uchar));
    cv::Mat _buf;
    _buf.create(1, (int)bufSize, CV_8U);
    uchar* buf = _buf.ptr();
    int i, j, dstep = (int)(img.step/sizeof(uchar));
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf[0]);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for( i = 0; i < height; i++ )
    {
        uchar* ds = img.ptr<uchar>(i);
        int* ls = labels + width*i;

        for( j = 0; j < width; j++ )
        {
            if( ds[j] != 0 )   // not a bad disparity
            {
                if( ls[j] )     // has a label, check for bad label
                {
                    if( rtype[ls[j]] ) // small region, zero out disparity
                        ds[j] = (uchar)0;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((int16_t)j, (int16_t)i);  // current pixel
                    curlabel++; // next label
                    int count = 0;  // current region size
                    ls[j] = curlabel;
                    std::vector<Point2s> pt_list;

                    // wavefront propagation
                    while( ws >= wbuf ) // wavefront not empty
                    {
                        count++;
                        pt_list.push_back(p);
                        // put neighbors onto wavefront
                        uchar* dpp = &img.at<uchar>(p.y, p.x);
                        // uchar dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != 0)
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y+1);
                        }

                        if( p.y > 0 && !lpp[-width] && dpp[-dstep] != 0)
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y-1);
                        }

                        if( p.x < width-1 && !lpp[+1] && dpp[+1] != 0)
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x+1, p.y);
                        }

                        if( p.x > 0 && !lpp[-1] && dpp[-1] != 0)
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x-1, p.y);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if(count<minRegion || count>maxRegion)
                    {
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (uchar)0;
                    }
                    else
                    {
                        // check bbox
                        int min_x = 60000, min_y = 60000, max_x = 0, max_y = 0;
                        float xsum = 0, ysum = 0;
                        for(const auto& tp : pt_list)
                        {
                            if(min_x > tp.x) min_x = tp.x;
                            else if(max_x < tp.x) max_x = tp.x;
                            if(min_y > tp.y) min_y = tp.y;
                            else if(max_y < tp.y) max_y = tp.y;
                            xsum += tp.x;
                            ysum += tp.y;
                        }
                        bboxes.push_back(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));
                        float n = pt_list.size();
                        centers.push_back(cv::Point2f(xsum/n, ysum/n));
                        rtype[ls[j]] = 0;   // large region label
                    }
                }
            }
        }
    }
    return;
}

void sobelFilter(const cv::Mat& img, cv::Mat& grad_bw, int grad_thres)
{
    grad_bw = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    int w = grad_bw.cols;
#if 1
    for(int i=1; i<img.rows-1; i++)
    {
        const uchar* ptr = img.ptr<uchar>(i);
        uchar* ptr_bw = grad_bw.ptr<uchar>(i);
        for(int j=1; j<img.cols-1; j++)
        {
            const uchar* p = ptr+j;
            int grad_x = - *(p-w-1) + *(p-w+1)
                         - *(p-1)*2 + *(p+1)*2
                         - *(p+w-1) + *(p+w+1);
            int grad_y = - *(p-w-1) - *(p-w)*2 - *(p-w+1)
                         + *(p+w-1) + *(p+w)*2 + *(p+w+1);
            if(abs(grad_x)>grad_thres || abs(grad_y)>grad_thres)
            {
                ptr_bw[j] = 255;
            }
        }
    }
#else
    uchar const *di = (uchar*) img.data;
    uchar *db = (uchar*) grad_bw.data;
    uchar const *p0=di, *p1=p0+w, *p2=p1+w;
    uchar *pb=db+w+1;
    for(int i=1; i<img.rows-1; i++)
    {
        for(int j=1; j<img.cols-1; j++)
        {
            int grad_x = - p0[0] + p0[2]
                         - p1[0] - p1[0] + p1[2] + p1[2]
                         - p2[0] + p2[2];
            int grad_y = - p0[0] - p0[1] - p0[1] - p0[2]
                         + p2[0] + p2[1] + p2[1] + p2[2];
            if(abs(grad_x)>grad_thres || abs(grad_y)>grad_thres)
            {
                *pb = 255;
            }
            pb++;
            p0++;
            p1++;
            p2++;
        }
        p0+=2;
        p1+=2;
        p2+=2;
        pb+=2;
    }
#endif
}

void histn(const cv::Mat& gray, double normalized_hist[256], int step)
{
    for(int i=0; i<256; i++) normalized_hist[i] = 0;
    const int N = gray.rows * gray.cols;
    uchar* p = (uchar *)gray.data;
    unsigned int cnt = N/step;
    for(unsigned int i = 0; i < cnt; i++, p += step) normalized_hist[*p]++;
    for(int i = 0; i < 256; i++) normalized_hist[i] /= cnt;
}

} /* namespace vs */