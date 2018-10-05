#include "vs_improc.h"
#include "vs_numeric.h"

cv::Mat bgr2hue(const cv::Mat& bgr)
{
    cv::Mat hue(bgr.rows, bgr.cols, CV_8UC1);
    for(int i=0; i<bgr.rows; i++)
    {
        const uchar *p_bgr = bgr.ptr<uchar>(i);
        uchar *p_hue = hue.ptr<uchar>(i);
        for(int j=0; j<hue.cols; j++)
        {
            uchar b = *p_bgr++;
            uchar g = *p_bgr++;
            uchar r = *p_bgr++;
            uchar amin = min3(b,g,r);
            uchar amax = max3(b,g,r);
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
            *p_hue++ = h;
        }
    }
    return hue;
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
            uchar amin = min3(b,g,r);
            uchar amax = max3(b,g,r);
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
    using namespace cv;
    typedef cv::Point_<short> Point2s;
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
                    Point2s p((short)j, (short)i);  // current pixel
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
    using namespace cv;
    typedef cv::Point_<short> Point2s;
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
                    Point2s p((short)j, (short)i);  // current pixel
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
                        for(const auto& p:pt_list)
                        {
                            if(min_x > p.x) min_x = p.x;
                            else if(max_x < p.x) max_x = p.x;
                            if(min_y > p.y) min_y = p.y;
                            else if(max_y < p.y) max_y = p.y;
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

void regionFilter(cv::Mat& img, std::vector<cv::Rect>& bboxes, std::vector<cv::Point2f>& centers, int minRegion, int maxRegion)
{
    using namespace cv;
    typedef cv::Point_<short> Point2s;
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
                    Point2s p((short)j, (short)i);  // current pixel
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
                        for(const auto& p:pt_list)
                        {
                            if(min_x > p.x) min_x = p.x;
                            else if(max_x < p.x) max_x = p.x;
                            if(min_y > p.y) min_y = p.y;
                            else if(max_y < p.y) max_y = p.y;
                            xsum += p.x;
                            ysum += p.y;
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
    uchar const *p0=di,*p1=p0+w,*p2=p1+w;
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
