#include "armor_detector/tools.hpp"
#include <iostream>

namespace tools
{
    std::string boolToString(bool value) 
    {
        return value ? "true" : "false";
    }
    // 实现通过调节条进行滤波的函数
    void Tools::filterImageWithTrackbars(const cv::Mat& img) 
    {
        if (img.empty()) {
            std::cerr << "Input image is empty." << std::endl;
            return;
        }

        // 转换图像到HSV颜色空间
        cv::Mat imgHSV, mask;
        cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

        // 初始化颜色阈值
        int hmin = 0, smin = 110, vmin = 153;
        int hmax = 19, smax = 240, vmax = 255;

        // 创建一个名为 "Tracker" 的窗口
        cv::namedWindow("Tracker", cv::WINDOW_NORMAL);

        // 创建调节条
        cv::createTrackbar("Hue Min", "Tracker", &hmin, 179); // 0
        cv::createTrackbar("Hue Max", "Tracker", &hmax, 179); // 179
        cv::createTrackbar("Sat Min", "Tracker", &smin, 255); // 0
        cv::createTrackbar("Sat Max", "Tracker", &smax, 255); // 255
        cv::createTrackbar("Val Min", "Tracker", &vmin, 255); // 255
        cv::createTrackbar("Val Max", "Tracker", &vmax, 255); // 255

        // 循环处理，直到用户按下ESC键
        while (true) {
            // 定义颜色范围
            cv::Scalar lower(hmin, smin, vmin);
            cv::Scalar upper(hmax, smax, vmax);

            // 根据颜色范围创建掩码
            cv::inRange(imgHSV, lower, upper, mask);

            // 显示原始图像、HSV图像和掩码图像
            cv::imshow("Image", img);
            cv::imshow("Image Mask", mask);

            // 等待用户按键
            int key = cv::waitKey(25);
            if (key == 27) {  // 如果按下ESC键，退出循环
                break;
            }
        }
    }
}