#ifndef __VS_COLOR_FILTER_H__
#define __VS_COLOR_FILTER_H__
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

namespace vs
{

enum ColorModelType
{
    CMT_BGR = 0,
    CMT_RGB = 1,
    CMT_HSV = 2,
};

struct ColorModel
{
    int type;
    ColorModel(int _type = 0): type(_type) {}

    virtual ~ColorModel(){}

    virtual uchar judge(const cv::Vec3b& a) const = 0; 

    virtual void filter(const cv::Mat& input, cv::Mat& mask) const
    {
        mask = cv::Mat(input.size(), CV_8UC1, cv::Scalar(0));
        for(int i = 0; i < input.rows; i++)
        {
            const cv::Vec3b* p = input.ptr<cv::Vec3b>(i);
            uchar* m = mask.ptr<uchar>(i);
            for(int j = 0; j < input.cols; j++)
            {
                *m++ = judge(*p++);
            }
        }
    }    
};

typedef std::vector<std::shared_ptr<ColorModel>> ColorModelList;


enum ColorFilterPostMethod
{
    CFPM_MORPHOLOGY = 1 << 0,
    CFPM_FLOODFILL = 1 << 1,
    CFPM_SPECKLE = 1 << 2,
};
void colorFilter(const cv::Mat& img_bgr, cv::Mat& mask, const ColorModelList& model_list,
                   float resize_rate = -1, int post_process = 0);

// color 0:red
//       1: green
//       2: blue
//       3: yellow
//       100: red lut
//       101: green lut
//       102: blue lut
//       103: yellow lut
//       110: white lut
ColorModelList defaultColorModel(int color);

ColorModelList loadColorModel(const char* color_file);

} /* namespace vs */
#endif//__VS_COLOR_FILTER_H__