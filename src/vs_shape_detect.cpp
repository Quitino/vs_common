#include "vs_shape_detect.h"

namespace vs
{

int detectMarker(const cv::Mat& gray, std::vector<cv::Point2f>& centers,
                   int marker_type, bool draw)
{
    switch(marker_type)
    {
        case 0:
            {
                std::vector<std::vector<float>> con_radius;
                return detectCircleConcentric(gray, centers, con_radius,
                        5, gray.cols << 1, 5.0f, 3, 15, 0.6, draw);
            }
        case 1: break;
        case 2: return detectNestedShape(gray, centers, 2, draw);
        case 3: return detectNestedShape(gray, centers, 3, draw);
        case 4: return detectNestedShape(gray, centers, 4, draw);
        case 5: return detectNestedShape(gray, centers, 5, draw);
        default: break;
    }
    printf("[ERROR]detectMarker: Unknown marker type:%d."
           " Only support(0, 2, 3, 4, 5)\n", marker_type);
    return 0;
}

static void houghCircles(const cv::Mat& edge, const cv::Mat& dx, const cv::Mat& dy,
                         std::vector<cv::Point2f>& centers, std::vector<float>& radius,
                         int min_radius, int max_radius)
{
    /*accumulate circle center*/
    int rows = edge.rows;
    int cols = edge.cols;
    int accum_rows = rows;
    int accum_cols = cols;
    cv::Mat accum(accum_rows, accum_cols, CV_32SC1, cv::Scalar(0));
    int boarder = 2;
    int boarder2 = boarder<<1;
    uchar const* ptr_edge = edge.ptr<uchar>(boarder) + boarder;
    int16_t const* ptr_dx = dx.ptr<int16_t>(boarder) + boarder;
    int16_t const* ptr_dy = dy.ptr<int16_t>(boarder) + boarder;
    for(int i=boarder; i<rows-boarder; i++)
    {
        for(int j=boarder; j<cols-boarder; j++, ptr_edge++, ptr_dx++, ptr_dy++)
        {
            if(*ptr_edge<=0) continue;
            float vx = *ptr_dx;
            float vy = *ptr_dy;
            float mag = hypotf(vx, vy);
            if(mag < 70) continue;
            vx /= mag;
            vy /= mag;
        #if 0 // search two directions
            if(fabs(vx)<fabs(vy))
            {
                float stepx = vx/vy;
                if(vy<0)
                {
                    vx = -vx;
                    vy = -vy;
                }
                float tx;
                int r, y;
                for(r = min_radius, y = i + vy * min_radius, tx = j + vx * min_radius;
                    r<max_radius; r++, tx+=stepx, y++)
                {
                    if(tx<0 || y<0 || tx>=accum_cols || y>=accum_rows) break;
                    accum.at<int>(y, (int)tx)++;
                }
                for(r = min_radius, y = i - vy * min_radius, tx = j - vx * min_radius;
                    r<max_radius; r++, tx-=stepx, y--)
                {
                    if(tx<0 || y<0 || tx>=accum_cols || y>=accum_rows) break;
                    accum.at<int>(y, (int)tx)++;
                }
            }
            else // fabs(vx)>fabs(vy)
            {
                float stepy = vy/vx;
                if(vx<0)
                {
                    vx = -vx;
                    vy = -vy;
                }
                float ty;
                int r, x;
                for(r = min_radius, ty = i + vy * min_radius, x = j + vx * min_radius;
                    r<max_radius; r++, x++, ty+=stepy)
                {
                    if(x<0 || ty<0 || x>=accum_cols || ty>=accum_rows) break;
                    accum.at<int>((int)ty, x)++;
                }
                for(r = min_radius, ty = i - vy * min_radius, x = j - vx * min_radius;
                    r<max_radius; r++, x--, ty-=stepy)
                {
                    if(x<0 || ty<0 || x>=accum_cols || ty>=accum_rows) break;
                    accum.at<int>((int)ty, x)++;
                }
            }
        #else  // only search one direction, the accum thresh must be decreased.
            if(fabs(vx)<fabs(vy))
            {
                float stepx = vx/vy;
                float tx;
                int r, y;
                if(vy>=0)
                {
                    for(r = min_radius, y = i + vy * min_radius,
                        tx = j + vx * min_radius; r<max_radius; r++, tx+=stepx, y++)
                    {
                        if(tx<0 || y<0 || tx>=accum_cols || y>=accum_rows) break;
                        accum.at<int>(y, (int)tx)++;
                    }
                }
                else
                {
                    for(r = min_radius, y = i - vy * min_radius,
                        tx = j - vx * min_radius; r<max_radius; r++, tx-=stepx, y--)
                    {
                        if(tx<0 || y<0 || tx>=accum_cols || y>=accum_rows) break;
                        accum.at<int>(y, (int)tx)++;
                    }
                }
            }
            else // fabs(vx)>fabs(vy)
            {
                float stepy = vy/vx;
                float ty;
                int r, x;
                if(vx>=0)
                {
                    for(r = min_radius, ty = i + vy * min_radius,
                        x = j + vx * min_radius; r<max_radius; r++, x++, ty+=stepy)
                    {
                        if(x<0 || ty<0 || x>=accum_cols || ty>=accum_rows) break;
                        accum.at<int>((int)ty, x)++;
                    }
                }
                else
                {
                    for(r = min_radius, ty = i - vy * min_radius,
                        x = j - vx * min_radius; r<max_radius; r++, x--, ty-=stepy)
                    {
                        if(x<0 || ty<0 || x>=accum_cols || ty>=accum_rows) break;
                        accum.at<int>((int)ty, x)++;
                    }
                }
            }
        #endif
        }
        ptr_edge += boarder2;
        ptr_dx += boarder2;
        ptr_dy += boarder2;
    }
    /*find maximun in accumulate mat to find circle centers*/
    const int thresh_accum = 80; //150;
    std::vector<cv::Point2f> hough_centers;
    std::vector<int> hough_vals;
    int* ptr_accum = (int*)accum.data + accum_cols + 1;
    for(int i=1; i<accum_rows-1; i++)
    {
        for(int j=1; j<accum_cols-1; j++, ptr_accum++)
        {
            int c = *ptr_accum;
            if(c < thresh_accum) continue;
            if((c>=*(ptr_accum-1) && c>=*(ptr_accum+1))
               || (c>=*(ptr_accum-accum_cols) && c>=*(ptr_accum+accum_cols)))
            {
                // non maximum suppression
                bool need = true;
                for(size_t k=0; k<hough_centers.size(); k++)
                {
                    auto& ck = hough_centers[k];
                    if(fabs(ck.x-j) + fabs(ck.y-i)<30)
                    {
                        if(c > hough_vals[k])
                        {
                            ck.x = j;
                            ck.y = i;
                            hough_vals[k] = c;
                        }
                        need = false;
                        break;
                    }
                }
                if(need)
                {
                    hough_centers.push_back(cv::Point2f(j, i));
                    hough_vals.push_back(c);
                }
            }
        }
        ptr_accum += 2;
    }
    #if 0
        double vmin, vmax;
        minMaxIdx(accum, &vmin, &vmax);
        printf("hough:%d  max:%.1f\n", (int)hough_centers.size(), vmax);
    #endif
    /*refine circle center with quadratic interpolation*/
    for(auto& p : hough_centers)
    {
        // for(int k=0;k<2;k++) //refine twice
        {
            std::vector<cv::Vec6f> as;
            std::vector<float> bs;
            int half_size = 5;
            for(int i = -half_size, r = p.y-half_size; i <= half_size; i++, r++)
            {
                if(r < 0) continue;
                else if(r >= accum_rows) break;
                int* ptr_accum = accum.ptr<int>(r) + int(p.x-half_size);
                for(int j=-half_size; j<=half_size; j++, ptr_accum++)
                {
                    if(p.x+j < 0) continue;
                    else if(p.x+j >= accum_cols) break;
                    else if(*ptr_accum < 30) continue;
                    as.push_back(cv::Vec6f(j*j, j*i, i*i, j, i, 1));
                    bs.push_back(*ptr_accum);
                }
            }
            int n = bs.size();
            if(n < 6) continue; //invalid circle
            cv::Mat A(n, 6, CV_32FC1);
            float* pA = (float*)A.data;
            for(const auto &a : as)
                for(int j=0;j<6;j++)
                    *pA++ = a[j];
            cv::Mat b(bs);
            cv::Mat x;
            cv::solve(A, b, x, cv::DECOMP_NORMAL);
            float* p_x = (float*)x.data;
            p_x[1] /= 2;
            float delta = (p_x[0] * p_x[2] - p_x[1] * p_x[1]) * 2;
            if(delta > 0 && p_x[0] < 0)
            {
                p.x += (p_x[1]*p_x[4]-p_x[2]*p_x[3]) / delta;
                p.y += (p_x[1]*p_x[3]-p_x[0]*p_x[4]) / delta;
            }
        }
    }
    if(hough_centers.empty()) return;
    /*hough radius*/
    int n_center = hough_centers.size();
    {
        int bin_step = 2;
        int n_bin = max_radius/bin_step + 1;
        std::vector<std::vector<std::vector<cv::Vec3f>>>
                center_maps(n_center, std::vector<std::vector<cv::Vec3f>>(n_bin));
        ptr_edge = edge.ptr<uchar>(boarder) + boarder;
        ptr_dx = dx.ptr<int16_t>(boarder) + boarder;
        ptr_dy = dy.ptr<int16_t>(boarder) + boarder;
        for(int i = boarder; i < rows-boarder; i++)
        {
            for(int j = boarder; j < cols-boarder; j++, ptr_edge++, ptr_dx++, ptr_dy++)
            {
                if(*ptr_edge<=0) continue;
                int16_t vx = *ptr_dx;
                int16_t vy = *ptr_dy;
                if(abs(vx)<70 && abs(vy)<70) continue;
                for(int k=0; k<n_center; k++)
                {
                    float dx = hough_centers[k].x - j;
                    float dy = hough_centers[k].y - i;
                    if(fabs(dx*vx + dy*vy) > 0.996f)
                    {
                        float r = hypotf(dx, dy);
                        if(r > max_radius || r < min_radius) continue;
                        int idxr = r / bin_step;
                        center_maps[k][idxr].push_back(cv::Vec3f(j, i, r));
                    }
                }
            }
            ptr_edge += boarder2;
            ptr_dx += boarder2;
            ptr_dy += boarder2;
        }
        for(int i = 0; i < n_center; i++)
        {
            const auto& bin_list = center_maps[i];
            int n_pre = bin_list[0].size();
            int n_cur = bin_list[1].size();
            int n_nex = bin_list[2].size();
        #if 0
            printf("radius hough:");
            for(int j=0; j<n_bin; j++)
            {
                printf("%d ", (int)bin_list[j].size());
                // printf("(%.1f %d) ", (j+0.5f)*bin_step, (int)bin_list[j].size());
            }
            printf("\n");
        #endif
            for(int j = 1; j < n_bin-1; j++)
            {
                float temp_r = (j + 0.5f) * bin_step;
                // need at least see 70% circle
                const static float thres_angle = 3.1415926f * 2.0f * 0.7f;
                const float radius_thres = std::max(thres_angle * temp_r, 10.0f);
                // local minima
                if(n_cur>=n_pre && n_cur>=n_nex && n_cur>5)
                {
                    if(n_cur>radius_thres)
                    {
                        float r_sum = 0;
                        for(const auto &v : bin_list[j])
                            r_sum += v[2];
                        centers.push_back(hough_centers[i]);
                        radius.push_back(r_sum/n_cur);
                        // printf("circle:%d r:%f n:%d(%d, %d)\n", j,
                        //         radius.back(), n_cur, n_pre, n_nex);
                    }
                    else if(n_cur > 50) //handle case that minima fall into two bins
                    {
                        // calculate average radius
                        float avg_r;
                        {
                            float r_sum = 0, w_sum;
                            for(const auto &v : bin_list[j])
                                r_sum += v[2];
                            avg_r = r_sum / bin_list[j].size();

                            auto f1 = [&r_sum, &w_sum, avg_r](float r){
                                float w = 1.0f/(fabs(r-avg_r)+0.5f);
                                r_sum += r*w;
                                w_sum += w;
                            };
                            for(int k = 0; k<2; k++) //calculate avg radius twice
                            {
                                r_sum = w_sum = 0;
                                for(const auto&v : bin_list[j]) f1(v[2]);
                                for(const auto&v : bin_list[j-1]) f1(v[2]);
                                for(const auto&v : bin_list[j+1]) f1(v[2]);
                                avg_r = r_sum / w_sum;
                            }
                        }
                        // find inlier cnt
                        int cnt = 0;
                        {
                            const float sigma = std::max(avg_r*0.02f, 1.0f);
                            float r_sum = 0;
                            auto f2 = [&cnt, &r_sum, sigma, avg_r](float r)
                            {
                                if(fabs(r-avg_r)<sigma)
                                {
                                    r_sum += r;
                                    cnt++;
                                }
                            };
                            for(const auto&v : bin_list[j]) f2(v[2]);
                            for(const auto&v : bin_list[j-1]) f2(v[2]);
                            for(const auto&v : bin_list[j+1]) f2(v[2]);
                            avg_r = r_sum/cnt;
                        }
                        if(cnt>radius_thres)
                        {
                            centers.push_back(hough_centers[i]);
                            radius.push_back(avg_r);
                            // printf("circle2:%d r:%f n:%d(%d, %d)\n",
                            //         j, radius.back(), cnt, n_pre, n_nex);
                        }
                    }
                }
                n_pre = n_cur;
                n_cur = n_nex;
                n_nex = bin_list[j+2].size();
            }
        }
    }
#if 0 //show hough accumulate map
    cv::Mat img_show;
    accum.convertTo(img_show, CV_8UC1, 0.5);
    // cv::imshow("direction", img_show);
    cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2BGR);
    for(auto c : hough_centers)
    {
        cv::circle(img_show, c, 1, cv::Scalar(0, 0, 255), -1);
        printf("(%f %f)\n", c.x, c.y);
    }
    cv::imshow("hough_cnt", img_show);
    printf("hought_circle:%d\n", (int)centers.size());
#endif
}

int detectCircle(const cv::Mat& gray, std::vector<cv::Point2f>& centers,
                   std::vector<float>& radius, int min_radius,
                   int max_radius)
{
    /*preprocessing*/
    cv::Mat blur, edge, dx, dy;
    cv::blur(gray, blur, cv::Size(3, 3));
    cv::Canny(blur, edge, 50, 150);
    cv::Sobel(blur, dx, CV_16S, 1, 0, 3);
    cv::Sobel(blur, dy, CV_16S, 0, 1, 3);
    /*hough circle*/
    centers.clear();
    radius.clear();
    houghCircles(edge, dx, dy, centers, radius, min_radius, max_radius);
    return centers.size();
}

static int findConcenterCirclesFromCircles(const std::vector<cv::Point2f>& centers,
                                    const std::vector<float>& radius,
                                    std::vector<cv::Point2f>& con_centers,
                                    std::vector<std::vector<float>>& con_radius,
                                    float dist_thres, int k, float min_outr,
                                    float radius_ratio_thres)
{
    con_centers.clear();
    con_radius.clear();
    int n_res = 0;
    int N = centers.size();
    std::vector<bool> flag(N, true);
    for(int i = 0; i < N; i++)
    {
        if(!flag[i]) continue;
        std::vector<int> idx;
        for(int j = 0; j < N; j++)
        {
            if(flag[j] && cv::norm(centers[i]-centers[j]) < dist_thres)
                idx.push_back(j);
        }
        int n_circles = idx.size();
        if(n_circles <= 1)
        {
            flag[i] = false;
            continue;
        }
        cv::Point2f mean_center(0, 0);
        for(auto j : idx) mean_center+=centers[j];
        mean_center /= n_circles;
        idx.clear();
        for(int j = 0; j < N; j++)
        {
            if(flag[j] && cv::norm(mean_center-centers[j]) < dist_thres)
                idx.push_back(j);
        }
        n_circles = idx.size();
        if(n_circles >= k)
        {
            float inner_r = radius[0];
            float outer_r = radius[0];
            for(auto j : idx)
            {
                float r = radius[j];
                if(r < inner_r) inner_r = r;
                else if(r > outer_r) outer_r = r;
            }
            if(outer_r > min_outr && inner_r/outer_r < radius_ratio_thres)
            {
                cv::Point2f psum(0, 0);
                std::vector<float> radius_vec;
                for(auto j : idx)
                {
                    psum += centers[j];
                    radius_vec.push_back(radius[j]);
                }
                con_centers.push_back(psum / n_circles);
                con_radius.push_back(radius_vec);
                n_res++;
            }
        }
        for(auto j : idx)
        {
            flag[j] = false;
        }
    }
    return n_res;
}

int detectCircleConcentric(const cv::Mat& gray, std::vector<cv::Point2f>& con_centers,
                             std::vector<std::vector<float>>& con_radius,
                             int min_radius, int max_radius,
                             float dist_thres, int k, float min_outr,
                             float radius_ratio_thres, bool draw)
{
    std::vector<cv::Point2f> centers;
    std::vector<float> radius;
    detectCircle(gray, centers, radius, min_radius, max_radius);
    int n = findConcenterCirclesFromCircles(centers, radius, con_centers, con_radius,
                                            dist_thres, k, min_outr,
                                            radius_ratio_thres);
    if(draw)
    {
        cv::namedWindow("DetectCircleConcentric");
        cv::moveWindow("DetectCircleConcentric", 100, 500);
        cv::Mat img_show;
        cv::cvtColor(gray, img_show, cv::COLOR_GRAY2BGR);

        for(int i = 0; i < n; i++)
        {
            const cv::Point2f& c = con_centers[i];
            cv::circle(img_show, c, 3, cv::Scalar(0, 0, 255), 2);
            for(auto r : con_radius[i])
            {
                cv::circle(img_show, c, r, cv::Scalar(0, 125, 255), 1);
            }
        }
        cv::imshow("DetectCircleConcentric", img_show);
    }
    return n;
}

static cv::Ptr<cv::LineSegmentDetector> plsd()
{
    static cv::Ptr<cv::LineSegmentDetector>
        lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_STD, 0.8, 0.6, 2, 10);
    return lsd;
}

int detectLine(const cv::Mat& gray, std::vector<cv::Vec4f>& lines,
                 const cv::Rect& roi)
{
    if(roi.width>0 || roi.height>0)
    {
        cv::Mat subimg = gray(roi);
        // line segment detect
        plsd()->detect(subimg, lines);
        if(!lines.empty())
        {
            for(auto& l : lines)
            {
                l[0]+=roi.x;
                l[1]+=roi.y;
                l[2]+=roi.x;
                l[3]+=roi.y;
            }
        }
    }
    else
    {
        plsd()->detect(gray, lines);
    }
    return lines.size();
}

int detectLine(const cv::Mat& gray, std::vector<cv::Vec4f>& lines,
                 const cv::Mat& mask)
{
    // cv::LineSegmentDetector not support mask, replace with LSDDetector
    // plsd()->detect(gray, lines, mask);
    plsd()->detect(gray, lines);
    return lines.size();
}

int detectArrow(const std::vector<cv::Vec4f>& lines, std::vector<Arrow>& arrows,
               float angle_min, float angle_max, float thres_connect)
{
    int nlines = lines.size();
    std::vector<cv::Point2f> dirs;
    dirs.reserve(nlines);
    for(const auto& l : lines)
    {
        dirs.push_back(normalize(cv::Point2f(l[2] - l[0], l[3] - l[1])));
    }

    arrows.clear();
    float cos_min = cos(angle_min);
    float cos_max = cos(angle_max);
    if(cos_min > cos_max) std::swap(cos_min, cos_max);
    bool accross_zero = (cos_min * cos_max < 0);

    for(int i = 0; i < nlines; i++)
    {
        const auto& li = lines[i];
        const auto& diri = dirs[i];
        for(int j = i + 1; j < nlines; j++)
        {
            const auto& lj = lines[j];
            const auto& dirj = dirs[j];
            float cos_angle = diri.dot(dirj);
            if(inRange(cos_angle, cos_min, cos_max))
            {
                float d1 = hypotf(li[0] - lj[0], li[1] - lj[1]);
                float d2 = hypotf(li[2] - lj[2], li[3] - lj[3]);
                if(d1 <= d2 && d1 < thres_connect)
                {
                    arrows.push_back(Arrow(lineIntersect(li, lj), diri, dirj, i, j));
                }
                else if(d2 < thres_connect)
                {
                    arrows.push_back(Arrow(lineIntersect(li, lj), -diri, -dirj, i, j));
                }
                else if(accross_zero)
                {
                    float d3 = hypotf(li[0] - lj[2], li[1] - lj[3]);
                    float d4 = hypotf(li[2] - lj[0], li[3] - lj[1]);
                    if(d3 <= d4 && d3 < thres_connect)
                    {
                        arrows.push_back(Arrow(lineIntersect(li, lj), diri, -dirj, i, j));
                    }
                    else if(d4 < thres_connect)
                    {
                        arrows.push_back(Arrow(lineIntersect(li, lj), -diri, dirj, i, j));
                    }
                }
            }
            else if(inRange(-cos_angle, cos_min, cos_max))
            {
                float d3 = hypotf(li[0] - lj[2], li[1] - lj[3]);
                float d4 = hypotf(li[2] - lj[0], li[3] - lj[1]);
                if(d3 <= d4 && d3 < thres_connect)
                {
                    arrows.push_back(Arrow(lineIntersect(li, lj), diri, -dirj, i, j));
                }
                else if(d4 < thres_connect)
                {
                    arrows.push_back(Arrow(lineIntersect(li, lj), -diri, dirj, i, j));
                }
                else if(accross_zero)
                {
                    float d1 = hypotf(li[0] - lj[0], li[1] - lj[1]);
                    float d2 = hypotf(li[2] - lj[2], li[3] - lj[3]);
                    if(d1 <= d2 && d1 < thres_connect)
                    {
                        arrows.push_back(Arrow(lineIntersect(li, lj), diri, dirj, i, j));
                    }
                    else if(d2 < thres_connect)
                    {
                        arrows.push_back(Arrow(lineIntersect(li, lj), -diri, -dirj, i, j));
                    }
                }
            }
        }
    }
    return arrows.size();
}

int detectArrow(const cv::Mat& gray, std::vector<Arrow>& arrows,
               float angle_min, float angle_max, float thres_connect)
{
    std::vector<cv::Vec4f> lines;
    detectLine(gray, lines);
    return detectArrow(lines, arrows, angle_min, angle_max, thres_connect);
}

// check if two arrows are parallel
static inline bool arrowCluster(const Arrow& a, const Arrow& b)
{
    return isCoincidence(a.top, a.ray, b.top, b.ray, 1);
}

// check if two arrows are connected
// return 0:not connect 1:line connect 2:parallel and coincidence
static inline int arrowConnect(const Arrow& a, const Arrow& b)
{
    if(a.ray.cross(b.ray) > 0)
    {
        if(a.id1 == b.id2) return 1;
        // coincidence with opposite direction
        if(isCoincidence(a.top, a.dir1, b.top, b.dir2, 2)) return 2;
    }
    else
    {
        if(a.id2 == b.id1) return 1;
        // coincidence with opposite direction
        if(isCoincidence(a.top, a.dir2, b.top, b.dir1, 2)) return 2;
    }
    return 0;
}

static void findClusterConnect(const std::vector<Arrow>& arrows,
                               const std::vector<int>& cluster1,
                               const std::vector<int>& cluster2,
                               int& match_cnt, int& match_id_cnt)
{
    match_cnt = 0;
    match_id_cnt = 0;
    std::vector<bool> used_j(cluster2.size(), false);
    for(int i : cluster1)
    {
        const auto& ai = arrows[i];
        for(int j : cluster2)
        {
            if(used_j[j]) continue;
            int ret = arrowConnect(ai, arrows[j]);
            if(ret > 0)
            {
                match_cnt++;
                used_j[j] = true;
                if(ret == 1) match_id_cnt++;
            }
        }
    }
}

static bool clusterConnect(const std::vector<Arrow>& arrows,
                            const std::vector<int>& cluster1,
                            const std::vector<int>& cluster2,
                            int min_match_cnt = 3, int min_match_id_cnt = 0)
{
    int match_cnt = 0;
    int match_id_cnt = 0;
    findClusterConnect(arrows, cluster1, cluster2, match_cnt, match_id_cnt);
    return match_cnt >= min_match_cnt && match_id_cnt >= min_match_id_cnt;
}

static int clusterArrows(const std::vector<Arrow>& arrows,
                         std::vector<std::vector<int>>& cluster_arrow_id,
                         int min_cluster_cnt = 3, bool search_twice = false)
{
    cluster_arrow_id.clear();
    int n_arrows = arrows.size();
    std::vector<bool> used(n_arrows, false);
    for(int i = 0; i < n_arrows; i++)
    {
        if(used[i]) continue;
        std::vector<int> cluster(1, i);
        const auto& ai = arrows[i];
        for(int j = i + 1; j < n_arrows; j++)
        {
            const auto& aj = arrows[j];
            if(!used[j] && arrowCluster(ai, aj))
            {
                for(int k : cluster)
                {
                    if(dist(arrows[k].top, aj.top) < 200)
                    {
                        cluster.push_back(j);
                        break;
                    }
                }
            }
        }
        int nc = cluster.size();
        if(nc < min_cluster_cnt) continue;
        for(auto k : cluster)
        {
            used[k] = true;
        }
        // search again to cluser more arrows
        if(search_twice)
        {
            cv::Point2f sum_ray(0, 0);
            cv::Point2f sum_top(0, 0);
            for(auto k : cluster)
            {
                const auto& a = arrows[k];
                sum_ray += a.ray;
                sum_top += a.top;
            }
            Arrow mean_arrow;
            mean_arrow.ray = normalize(sum_ray);
            mean_arrow.top = sum_top / nc;
            for(int j = i + 1; j < n_arrows; j++)
            {
                if(!used[j] && arrowCluster(mean_arrow, arrows[j]))
                {
                    cluster.push_back(j);
                    used[j] = true;
                }
            }
        }
        cluster_arrow_id.push_back(cluster);
    }
    return cluster_arrow_id.size();
}

static int calcClusterRay(const std::vector<Arrow>& arrows,
                          const std::vector<std::vector<int>>& cluster_arrow_id,
                          std::vector<cv::Point2f>& rays)
{
    rays.clear();
    rays.reserve(cluster_arrow_id.size());
    for(auto& cluster : cluster_arrow_id)
    {
        cv::Point2f r(0, 0);
        for(auto i : cluster)
        {
            r += arrows[i].ray;
        }
        rays.push_back(normalize(r));
    }
    return rays.size();
}

static bool checkShapeRadius(const cv::Point2f& center,
                             const std::vector<Arrow>& arrows,
                             const std::vector<int>& ids1,
                             const std::vector<int>& ids2 = std::vector<int>(0),
                             const std::vector<int>& ids3 = std::vector<int>(0),
                             const std::vector<int>& ids4 = std::vector<int>(0))
{
    float min_radius = FLT_MAX;
    float min_radius2 = FLT_MAX;
    float max_radius = 0;
    float max_radius2 = 0;
    auto foo = [&](int i)
    {
        float d = dist(arrows[i].top, center);
        if(d < min_radius)
        {
            min_radius2 = min_radius;
            min_radius = d;
        }
        else if(d < min_radius2)
        {
            min_radius2 = d;
        }
        if(d > max_radius)
        {
            max_radius2 = max_radius;
            max_radius = d;
        }
        else if(d > max_radius2)
        {
            max_radius2 = d;
        }
    };
    for(auto i : ids1) {foo(i);}
    for(auto i : ids2) {foo(i);}
    for(auto i : ids3) {foo(i);}
    for(auto i : ids4) {foo(i);}
    return max_radius2 > 20 && min_radius2 / max_radius2 < 0.7;
}

static int findTrianglesInClusters(const std::vector<Arrow>& arrows,
                                const std::vector<std::vector<int>>& cluster_arrow_id,
                                std::vector<cv::Point2f>& centers)
{
    centers.clear();
    std::vector<cv::Point2f> rays;
    int n_clusters = calcClusterRay(arrows, cluster_arrow_id, rays);
    std::vector<bool> cluster_used(n_clusters, false);
    const float thres_cos_min = -0.643f; //cos(130deg)
    const float thres_cos_max = -0.342f; //cos(110deg)
    const float thres_center_diff = 3;
    const float thres_parallel = 0.995f;
    for(int i = 0; i < n_clusters; i++)
    {
        const cv::Point2f& rayi = rays[i];
        const auto& idsi = cluster_arrow_id[i];
        for(int j = i + 1; j < n_clusters; j++)
        {
            if(cluster_used[i]) break;
            if(cluster_used[j]) continue;
            const cv::Point2f& rayj = rays[j];
            const auto& idsj = cluster_arrow_id[j];
            // check ray angle, 110~130deg
            if(!inRange(rayi.dot(rayj), thres_cos_min, thres_cos_max)) continue;
            // try to find the 3rd arrow
            for(int k = j + 1; k < n_clusters; k++)
            {
                if(cluster_used[i] || cluster_used[j]) break;
                if(cluster_used[k]) continue;
                const cv::Point2f& rayk = rays[k];
                // check ray angle, 110~130deg
                if(!inRange(rayi.dot(rayk), thres_cos_min, thres_cos_max)
                 || !inRange(rayj.dot(rayk), thres_cos_min, thres_cos_max)) continue;
                // check intersect
                cv::Point2f p[3] = {arrows[idsi[0]].top,
                                    arrows[cluster_arrow_id[j][0]].top,
                                    arrows[cluster_arrow_id[k][0]].top};
                cv::Point2f c[3] = {lineIntersect(p[0], rayi, p[1], rayj),
                                    lineIntersect(p[0], rayi, p[2], rayk),
                                    lineIntersect(p[1], rayj, p[2], rayk)};
                cv::Point2f center = (c[0] + c[1] + c[2]) / 3;
                if(dist(center, c[0]) > thres_center_diff
                 || dist(center, c[1]) > thres_center_diff
                 || dist(center, c[2]) > thres_center_diff) continue;
                if(!(isParallel(normalize(center - p[0]), rayi, 1, thres_parallel)
                  && isParallel(normalize(center - p[1]), rayj, 1, thres_parallel)
                  && isParallel(normalize(center - p[2]), rayk, 1, thres_parallel)))
                    continue;
                if(checkShapeRadius(center, arrows, idsi, idsj, cluster_arrow_id[k]))
                {
                    cluster_used[i] = cluster_used[j] = cluster_used[k] = true;
                    centers.push_back(center);
                }
            }
            if(!cluster_used[i] && clusterConnect(arrows, idsi, idsj, 3, 1))
            {
                cv::Point2d center = lineIntersect(arrows[idsi[0]].top, rayi,
                                                   arrows[idsj[0]].top, rayj);
                if(checkShapeRadius(center, arrows, idsi, idsj))
                {
                    cluster_used[i] = cluster_used[j] = true;
                    centers.push_back(center);
                }
            }
        }
    }
    return centers.size();
}

static int findRectanglesInClusters(const std::vector<Arrow>& arrows,
                                const std::vector<std::vector<int>>& cluster_arrow_id,
                                std::vector<cv::Point2f>& centers)
{
    centers.clear();
    std::vector<cv::Point2f> rays;
    int n_clusters = calcClusterRay(arrows, cluster_arrow_id, rays);
    std::vector<bool> cluster_used(n_clusters, false);
    const float thres_center_diff = 3;
    for(int i = 0; i < n_clusters; i++)
    {
        const cv::Point2f& rayi = rays[i];
        const auto& idsi = cluster_arrow_id[i];
        const auto& topi = arrows[idsi[0]].top;
        // find two arrow cluster in diagonal line of rectangle
        for(int j = i + 1; j < n_clusters; j++)
        {
            if(cluster_used[i]) break;
            if(cluster_used[j]) continue;
            const cv::Point2f& rayj = rays[j];
            const auto& idsj = cluster_arrow_id[j];
            const auto& topj = arrows[idsj[0]].top;
            // cond1: opposite coincidence
            if(!isCoincidence(topi, rayi, topj, rayj, 2)) continue;
            // cond2: ab same direction with rayi
            if(rayi.dot(topj - topi) <= 0) continue;
            // find third corner connect with two clusters
            for(int k = 0; k < n_clusters; k++)
            {
                if(cluster_used[i] || cluster_used[j]) break;
                if(k == i || k == j || cluster_used[k]) continue;
                const cv::Point2f& rayk = rays[k];
                const auto& idsk = cluster_arrow_id[k];
                const auto& topk = arrows[idsk[0]].top;
                // cond1: perpendicular with ij ray
                if(!isPerpendicular(rayi, rayk)) continue;
                // cond2: connect with arrow cluster ij
                if(!(clusterConnect(arrows, idsi, idsk, 1)
                  && clusterConnect(arrows, idsj, idsk, 1))) continue;
                // cond3: intersection is near
                cv::Point2f c[2] = {lineIntersect(topi, rayi, topk, rayk),
                                    lineIntersect(topj, rayj, topk, rayk)};
                if(dist(c[0], c[1]) > thres_center_diff) continue;
                // find rectangle
                cv::Point2f center = (c[0] + c[1]) / 2;
                if(!checkShapeRadius(center, arrows, idsi, idsj, idsk)) continue;
                cluster_used[i] = cluster_used[j] = cluster_used[k] = true;
                centers.push_back(center);
            #if 0 // need to find the 4th corner and set it as used
                for(int l = k + 1; l < n_clusters; l++)
                {
                    const cv::Point2f& rayl = rays[l];
                    const auto& idsl = cluster_arrow_id[l];
                    const auto& topl = arrows[idsl[0]].top;
                    if(!cluster_used[l] && rayk.dot(topl - topk) > 0
                     && isCoincidence(topk, rayk, topl, rayl, 2)
                     && (clusterConnect(arrows, idsl, idsi)
                        || clusterConnect(arrows, idsl, idsj)))
                    {
                        cluster_used[l] = true;
                        break;
                    }
                }
            #endif
            }

        }
    }
    return centers.size();
}

// find shape like '> > > < < < '
static int findArrowPairsInClusters(const std::vector<Arrow>& arrows,
                                const std::vector<std::vector<int>>& cluster_arrow_id,
                                std::vector<cv::Point2f>& centers)
{
    centers.clear();
    std::vector<cv::Point2f> rays;
    int n_clusters = calcClusterRay(arrows, cluster_arrow_id, rays);
    std::vector<bool> cluster_used(n_clusters, false);
    for(int i = 0; i < n_clusters; i++)
    {
        const cv::Point2f& rayi = rays[i];
        const auto& idsi = cluster_arrow_id[i];
        for(int j = i + 1; j < n_clusters; j++)
        {
            if(cluster_used[i] || cluster_used[j]) continue;
            const cv::Point2f& rayj = rays[j];
            // coincidence with different direction
            if(!isCoincidence(arrows[cluster_arrow_id[i][0]].top, rayi,
                              arrows[cluster_arrow_id[j][0]].top, rayj, 2)) continue;
            // find center
            const auto& idsj = cluster_arrow_id[j];
            int nearest_ii = -1;
            int nearest_jj = -1;
            for(int ii : idsi)
            {
                const auto& a = arrows[ii].top;
                int min_jj = 0;
                float min_d = FLT_MAX;
                int cnt = 0;
                for(int jj : idsj)
                {
                    float d = (a - arrows[jj].top).dot(rayi);
                    if(d > 0)
                    {
                        cnt++;
                        if(d < min_d)
                        {
                            min_d = d;
                            min_jj = jj;
                        }
                    }
                }
                if(cnt >= 3)
                {
                    nearest_jj = min_jj;
                    break;
                }
            }
            if(nearest_jj < 0) continue;
            {
                int min_ii = -1;
                float min_d = FLT_MAX;
                cv::Point2f pjj = arrows[nearest_jj].top;
                int cnt = 0;
                for(int ii : idsi)
                {
                    float d = (pjj - arrows[ii].top).dot(rayj);
                    if(d > 0 && d < min_d)
                    {
                        cnt++;
                        min_d = d;
                        min_ii = ii;
                    }
                }
                if(cnt >= 3)
                {
                    nearest_ii = min_ii;
                }
            }
            if(nearest_ii >=0 && nearest_jj >= 0 && nearest_ii != nearest_jj)
            {
                centers.push_back((arrows[nearest_ii].top + arrows[nearest_jj].top) / 2);
                cluster_used[i] = cluster_used[j] = true;
                break;
            }
        }
    }
    return centers.size();
}

static int findRhombusInClusters(const std::vector<Arrow>& arrows,
                                const std::vector<std::vector<int>>& cluster_arrow_id,
                                std::vector<cv::Point2f>& centers)
{
    centers.clear();
    std::vector<cv::Point2f> rays;
    int n_clusters = calcClusterRay(arrows, cluster_arrow_id, rays);
    std::vector<bool> cluster_used(n_clusters, false);
    for(int i = 0; i < n_clusters; i++)
    {
        const cv::Point2f& rayi = rays[i];
        const auto& idsi = cluster_arrow_id[i];
        for(int j = i + 1; j < n_clusters; j++)
        {
            if(cluster_used[i] || cluster_used[j]) continue;
            const cv::Point2f& rayj = rays[j];
            // coincidence with different direction
            if(!isCoincidence(arrows[cluster_arrow_id[i][0]].top, rayi,
                              arrows[cluster_arrow_id[j][0]].top, rayj, 2)) continue;
            // find center
            const auto& idsj = cluster_arrow_id[j];
            int nearest_ii = -1;
            int nearest_jj = -1;
            for(int ii : idsi)
            {
                const auto& a = arrows[ii].top;
                int min_jj = 0;
                float min_d = FLT_MAX;
                int cnt = 0;
                for(int jj : idsj)
                {
                    float d = - (a - arrows[jj].top).dot(rayi);
                    if(d > 0)
                    {
                        cnt++;
                        if(d < min_d)
                        {
                            min_d = d;
                            min_jj = jj;
                        }
                    }
                }
                if(cnt >= 3)
                {
                    nearest_jj = min_jj;
                    break;
                }
            }
            if(nearest_jj < 0) continue;
            {
                int min_ii = -1;
                float min_d = FLT_MAX;
                cv::Point2f pjj = arrows[nearest_jj].top;
                int cnt = 0;
                for(int ii : idsi)
                {
                    float d = - (pjj - arrows[ii].top).dot(rayj);
                    if(d > 0 && d < min_d)
                    {
                        cnt++;
                        min_d = d;
                        min_ii = ii;
                    }
                }
                if(cnt >= 3)
                {
                    nearest_ii = min_ii;
                }
            }
            if(nearest_ii >=0 && nearest_jj >= 0 && nearest_ii != nearest_jj)
            {
                centers.push_back((arrows[nearest_ii].top + arrows[nearest_jj].top) / 2);
                cluster_used[i] = cluster_used[j] = true;
                break;
            }
        }
    }
    return centers.size();
}

int detectNestedShape(const cv::Mat& gray, std::vector<cv::Point2f>& centers,
                        int type, bool draw)
{
    const float arrow_angle_diff = 0.17453f; //deg2rad(10);
    float arrow_angle = deg2rad(60);
    int min_cluster_cnt = 3;
    bool search_twice = true;
    auto foo = findTrianglesInClusters;
    switch(type)
    {
        case 2:
            arrow_angle = deg2rad(60);
            foo = findArrowPairsInClusters;
            break;
        case 3:
            arrow_angle = deg2rad(60);
            foo = findTrianglesInClusters;
            break;
        case 4:
            arrow_angle = deg2rad(90);
            foo = findRectanglesInClusters;
            break;
        case 5:
            arrow_angle = deg2rad(120);
            foo = findRhombusInClusters;
            break;
        default:
            printf("[ERROR]detectNestedShapes: unknown type '%d'\n", type);
            return -1;
    }

    /* detect lines */
    std::vector<cv::Vec4f> lines;
    int nlines = detectLine(gray, lines);

    /* detect arrows from lines*/
    std::vector<Arrow> arrows;
    int n_arrows = nlines < 6 ? 0 : detectArrow(lines, arrows,
                    arrow_angle - arrow_angle_diff, arrow_angle + arrow_angle_diff);

    /* cluster arrows */
    std::vector<std::vector<int>> cluster_arrow_id;
    int n_clusters = n_arrows < 6 ? 0 :
                     clusterArrows(arrows, cluster_arrow_id, min_cluster_cnt, search_twice);

    /* find shape */
    int n_centers = n_clusters < 2 ? 0 : foo(arrows, cluster_arrow_id, centers);

    if(draw)
    {
        cv::namedWindow("DetectNestedShape");
        cv::moveWindow("DetectNestedShape", 100, 500);
        cv::Mat img_show;
        cv::cvtColor(gray, img_show, cv::COLOR_GRAY2BGR);
    #if 0
        for(const auto& l : lines)
        {
            cv::line(img_show, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                     cv::Scalar(255, 20, 0), 1);
        }
    #endif
        for(const auto& a : arrows)
        {
            cv::line(img_show, a.top, a.top + a.dir1 * 15, cv::Scalar(190, 20, 180), 1);
            cv::line(img_show, a.top, a.top + a.dir2 * 15, cv::Scalar(190, 20, 180), 1);
        }
        for(int i = 0; i < n_clusters; i++)
        {
            const static cv::Scalar colors[] = {cv::Scalar(255, 0, 0),
                cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 255),
                cv::Scalar(0, 255, 255), cv::Scalar(255, 255, 0), cv::Scalar(255, 120, 120),
                cv::Scalar(0, 120, 0), cv::Scalar(0, 255, 120), cv::Scalar(120, 120, 255)};
            const static int k = sizeof(colors) / sizeof(colors[0]);
            cv::Scalar color = colors[i % k];
            for(auto k : cluster_arrow_id[i])
            {
                cv::circle(img_show, arrows[k].top, 2, color, -1);
            }
        }
        for(const auto& c : centers)
        {
            cv::circle(img_show, c, 3, cv::Scalar(0, 0, 255), 2);
        }
        cv::imshow("DetectNestedShape", img_show);
    }
    return n_centers;
}

} /* namespace vs */