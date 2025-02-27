#ifndef ARMOR_DETECTOR__TOOLS_HPP_
#define ARMOR_DETECTOR__TOOLS_HPP_
#include <opencv2/opencv.hpp>
#include <string>
namespace tools
{
    class Tools {
    public:
        Tools()=default;
        ~Tools()=default;
        // 通过调节条进行滤波的函数
        void filterImageWithTrackbars(const cv::Mat& img);
    };
}
#endif  // ARMOR_DETECTOR__ARMOR_HPP_