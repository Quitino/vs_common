#include "vs_color_filter.h"
#include "vs_color_lut.hpp"
#include <map>
#include <cmath>
#include <stack>
#include <fstream>

namespace vs
{

static auto fkernel = [](int k = 3)
        {return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));};

void colorFilter(const cv::Mat& img_bgr, cv::Mat& mask,
                   const ColorModelList& model_list,
                   float resize_rate, int post_process)
{
    if(img_bgr.channels() != 3)
    {
        printf("[ERROR]ColorFilter: Need input bgr image.\n");
        return;
    }

    std::map<int, cv::Mat> inputs;
    auto& bgr = inputs[CMT_BGR];
    if(resize_rate <= 0)
        bgr = img_bgr;
    else
        cv::resize(img_bgr, bgr, cv::Size(), resize_rate, resize_rate);

    // create inputs
    for(const auto& p : model_list)
    {
        auto& input = inputs[p->type];
        if(input.empty())
        {
            switch(p->type)
            {
                case CMT_RGB:
                    cv::cvtColor(bgr, input, cv::COLOR_BGR2RGB);
                    break;
                case CMT_HSV:
                    cv::cvtColor(bgr, input, cv::COLOR_BGR2HSV);
                    break;
                default:
                    printf("[ERROR]Unknown color model type '%d'\n", p->type);
                    break;
            }
        }
    }

    mask = cv::Mat(bgr.size(), CV_8UC1, cv::Scalar(0));
    for(const auto& p : model_list)
    {
        cv::Mat mask_i;
        p->filter(inputs[p->type], mask_i);
        mask |= mask_i;
    }

    if(post_process)
    {
        if(post_process & CFPM_MORPHOLOGY)
        {
            cv::dilate(mask, mask, fkernel(3));
            cv::dilate(mask, mask, fkernel(3));
            cv::erode(mask, mask, fkernel(3));
        }
        if(post_process & CFPM_FLOODFILL)
        {
            cv::Mat temp = ~ mask;
            cv::filterSpeckles(temp, 0, 220, 50);
            mask = ~temp;
        }
        if(post_process & CFPM_SPECKLE)
        {
            cv::filterSpeckles(mask, 0, 100, 50);
        }
    }

    if(mask.size() != img_bgr.size())
    {
        cv::resize(mask, mask, img_bgr.size());
    }
}

struct GaussModel: public ColorModel
{
    GaussModel(int _type = 0, int _sample_cnt = 0,
               cv::Mat _mu = cv::Mat(), cv::Mat _cov_inv = cv::Mat(),
               double _sigma = 4.5)
        : ColorModel(type), sample_cnt(_sample_cnt), mu(_mu)
        , cov_inv(_cov_inv), sigma(_sigma) {}
    int sample_cnt;
    cv::Mat mu;
    cv::Mat cov_inv;
    double sigma;
    virtual uchar judge(const cv::Vec3b& a) const
    {
        if(sample_cnt <= 0) return false;
        cv::Mat err = a - mu;
        cv::Mat res = err * cov_inv * err.t();
        return (res.at<double>(0, 0) < sigma) ? 255 : 0;
    }
};

struct HsvYellow: public ColorModel
{
    HsvYellow(): ColorModel(CMT_HSV) {}
    virtual uchar judge(const cv::Vec3b& a) const
    {
        uchar h = a[0];
        uchar s = a[1];
        uchar v = a[2];
        return (11 < h && h < 34 && 43 < s && s < 255 && 26 < v && v < 255)
                ? 255 : 0;
    }
};

struct ColorLUT: public ColorModel
{
    ColorLUT(const char* color_file)
    : ColorModel(CMT_BGR), size(1<<24), lut(size, 0)
    {
        std::ifstream fin(color_file, std::ios::binary);
        if(!fin.is_open())
        {
            printf("[ERROR] ColorLUT cannot open file '%s'\n", color_file);
            return;
        }

        char bit, ds;
        fin.read(&bit, 1);
        fin.read(&ds, 1);

        std::stack<uchar> c;
        auto fget = [&fin, bit, &c]()
        {
            if(bit == 8)
            {
                uchar a;
                fin.read((char*)&a, 1);
                return a;
            }
            if(c.empty())
            {
                uchar a;
                fin.read((char*)&a, 1);
                int k = 8 / bit;
                int b = 8 - bit;
                int mask = (1 << bit) - 1;
                for(int i = 0; i < k; i++)
                {
                    c.push((a & mask) << b);
                    a = a >> bit;
                }
            }
            uchar res = c.top();
            c.pop();
            return res;
        };

        for(int i = 0; i < 256; i += ds)
            for(int j = 0; j < 256; j += ds)
                for(int k = 0; k < 256; k += ds)
                    lut[idx(i,j,k)] = fget();

        interpolation(ds);
    }

    ColorLUT(const std::vector<uint64_t>& table, int bit, int ds)
        : ColorModel(CMT_BGR), size(1<<24), lut(size, 0)
    {
        std::vector<uint64_t> table_new;
        table_new.reserve(table.size() << 1);
        for(auto it = table.begin(); it != table.end(); it++)
        {
            uint64_t a = *it;
            if(a != 0) table_new.push_back(a);
            else
            {
                it++;
                for(uint64_t i = 0; i <= *it; i++)
                    table_new.push_back(0);
            }
        }
        int id = 0;
        std::stack<uchar> c;
        int k = 64 / bit;
        int b = 8 - bit;
        int m = (1 << bit) - 1;
        auto fget = [&table_new, &id, &c, bit, k, b, m]()
        {
            if(c.empty())
            {
                uint64_t a = table_new[id++];
                for(int i = 0; i < k; i++)
                {
                    c.push((a & m) << b);
                    a = a >> bit;
                }
            }
            uchar res = c.top();
            c.pop();
            return res;
        };

        for(int i = 0; i < 256; i += ds)
            for(int j = 0; j < 256; j += ds)
                for(int k = 0; k < 256; k += ds)
                    lut[idx(i,j,k)] = fget();

        interpolation(ds);
        
    }

    virtual uchar judge(const cv::Vec3b& a) const
    {
        uint32_t idx = ((uint32_t)(a[0]) << 16)
                     | ((uint32_t)(a[1]) << 8)
                     | (uint32_t)(a[2]);
        return (lut[idx] > 80) ? 255 : 0;
    }

    const uint32_t size;
    std::vector<uchar> lut;

    int idx(int i, int j, int k) {return (i << 16) | (j << 8) | k;}

    void interpolation(int ds)
    {
        if(ds != 2) return;
        for(int i = 0; i < 256; i += ds)
            for(int j = 0; j < 256; j += ds)
            {
                for(int k = 1; k < 255; k += ds)
                    lut[idx(i,j,k)] = ((int)lut[idx(i,j,k+1)]
                                    + (int)lut[idx(i,j,k-1)]) >> 1;
                lut[idx(i,j,255)] = lut[idx(i,j,254)];
            }

        for(int i = 0; i < 256; i += ds)
        {
            for(int j = 1; j < 255; j += ds)
                for(int k = 0; k < 256; k++)
                    lut[idx(i,j,k)] = ((int)lut[idx(i,j+1,k)]
                                    + (int)lut[idx(i,j-1,k)]) >> 1;
            for(int k = 0; k < 256; k++)
                lut[idx(i,255,k)] = lut[idx(i,254,k)];
        }

        for(int i = 1; i < 255; i += ds)
        {
            for(int j = 0; j < 256; j++)
                for(int k = 0; k < 256; k++)
                    lut[idx(i,j,k)] = ((int)lut[idx(i+1,j,k)]
                                    + (int)lut[idx(i-1,j,k)]) >> 1;
        }
        for(int j = 0; j < 256; j++)
            for(int k = 0; k < 256; k++)
                lut[idx(255,j,k)] = lut[idx(254,j,k)];
    }
};

ColorModelList loadColorModel(const char* color_file)
{
    return ColorModelList(1, std::shared_ptr<ColorModel>(
            new ColorLUT(color_file)));
}

ColorModelList defaultColorModel(int color)
{
    ColorModelList list;
    switch(color)
    {
        case 0:
        {
            break;
        }
        case 1:
        {
            break;
        }
        case 2:
        {
            break;
        }
        case 3:
        {
            list.push_back(std::shared_ptr<ColorModel>(new HsvYellow));
            break;
        }
        case 100:
        {
            list.push_back(std::shared_ptr<ColorModel>(new
                    ColorLUT(p_lut_red_compress0_bit4_downsample2, 4, 2)));
            break;
        }
        case 103:
        {
            list.push_back(std::shared_ptr<ColorModel>(new
                    ColorLUT(p_lut_yellow_compress0_bit4_downsample2, 4, 2)));
            break;
        }
        case 110:
        {
            list.push_back(std::shared_ptr<ColorModel>(new
                    ColorLUT(p_lut_white_compress0_bit4_downsample2, 4, 2)));
            break;
        }
        default:
            break;
    }
    return list;
}

} /* namespace vs */