// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/tools.hpp"
// #include "auto_aim_interfaces/msg/debug_armor.hpp"
// #include "auto_aim_interfaces/msg/debug_light.hpp"


namespace rm_auto_aim
{
    Detector::Detector(
        const int &bin_thres, const int &color, const LightParams &l, const ArmorParams &a)
        : binary_thres(bin_thres), detect_color(color), l_param(l), a_param(a)
    {
    }

    std::vector<Armor> Detector::detect(const cv::Mat &input)
    {
        cv::Mat gray_img;
        cv::cvtColor(input,gray_img,cv::COLOR_BGR2GRAY);

        cv::Mat binary_img = preprocessImage(input);

        lights_ = findLights(input, binary_img);

        armors_ = matchLights(lights_);

        // 装甲板角点矫正
        for(auto &armor : armors_)
        {
            corrector_.correctCorners(armor, gray_img);
        }

        if (!armors_.empty())
        {
            // std::cout<<"armors_ size is:"<<armors_.size()<<std::endl; // debug

            classifier->extractNumbers(input, armors_);
            classifier->classify(armors_);
        }

        // debug
        // else{
        //     std::cout<<"Private armors_ is empty!"<<std::endl;
        // }
        

        return armors_;
    }
    cv::Mat Detector::preprocessImage(const cv::Mat &rgb_image)
    {
        cv::Mat gray_image;
        cv::cvtColor(rgb_image, gray_image, cv::COLOR_RGB2GRAY);

        cv::Mat binary_image;
        cv::threshold(gray_image, binary_image, binary_thres, 255, cv::THRESH_OTSU);

        return binary_image;


        // // 转换图像到HSV颜色空间
        // cv::Mat imgHSV, mask;
        // cv::cvtColor(rgb_image, imgHSV, cv::COLOR_BGR2HSV);
        // // 初始化颜色阈值
        // int hmin = 0, smin = 0, vmin = 250;
        // int hmax = 179, smax = 255, vmax = 255;
        // // 定义颜色范围
        // cv::Scalar lower(hmin, smin, vmin);
        // cv::Scalar upper(hmax, smax, vmax);
        // // 根据颜色范围创建掩码
        // cv::inRange(imgHSV, lower, upper, mask);
        // return mask;
    }
    std::vector<Light> Detector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
    {
        using std::vector;
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;

        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // // 腐蚀操作
        // cv::Mat eroded_img;
        // cv::erode(binary_img, eroded_img, kernel);

        // // 膨胀操作
        // cv::Mat dilated_img;
        // cv::dilate(binary_img, dilated_img, kernel);

        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // debug
        cv::Mat output_img = rgb_img.clone();
        cv::drawContours(output_img, contours, -1, cv::Scalar(0, 255, 0), 2);

        vector<Light> lights;
        // this->debug_lights.data.clear();

        for (const auto & contour : contours) {
            if (contour.size() < 5) continue;

            auto r_rect = cv::minAreaRect(contour);
            auto light = Light(r_rect);

            if (isLight(light)) {
            auto rect = light.boundingRect();

        
            cv::rectangle(output_img,rect,cv::Scalar(255,255,0),3); // debug

                if (  // Avoid assertion failed
                    0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
                    0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
                    int sum_r = 0, sum_b = 0;
                    auto roi = rgb_img(rect);

                    // Iterate through the ROI

                    // update
                    std::vector<cv::Mat> bgr_planes;
                    cv::split(roi,bgr_planes);
                    // sum函数返回Scalar图像，单通道图像的第一个元素为像素值总和
                    sum_r=cv::sum(bgr_planes[2])[0]; 
                    sum_b=cv::sum(bgr_planes[0])[0];

                    // // 计算 ROI 区域的总像素值
                    // int total_pixels = rect.width * rect.height * 255;

                    // // 计算红色和蓝色像素的占比
                    // double red_ratio = static_cast<double>(sum_r) / total_pixels;
                    // double blue_ratio = static_cast<double>(sum_b) / total_pixels;

                    // // 设定占比阈值
                    // const double ratio_threshold = 0.5;

                    // // 判断占比是否过低
                    // if (red_ratio < ratio_threshold && blue_ratio < ratio_threshold) {
                    //     continue; // 占比过低，跳过该轮廓
                    // }

                    // Sum of red pixels > sum of blue pixels ?
                    light.color = sum_r > sum_b ? RED : BLUE;
                    lights.emplace_back(light);
                }
            }
    }

    // debug
    cv::namedWindow("Detected Contours",cv::WINDOW_NORMAL);
    cv::imshow("Detected Contours", output_img);
    return lights;
    }

    bool Detector::isLight(const Light & light)
    {
        // The ratio of light (short side / long side)
        float ratio = light.width / light.length;
        bool ratio_ok = l_param.min_ratio < ratio && ratio < l_param.max_ratio;

        if(!ratio_ok) std::cout<<"ratio_ok is false."<<std::endl; // debug

        bool angle_ok = light.tilt_angle < l_param.max_angle;

        if(!angle_ok) std::cout<<"angle_ok is false."<<std::endl; // debug

        bool is_light = ratio_ok && angle_ok;

        if(!is_light) std::cout<<"is_light is false."<<std::endl; // debug

        // Fill in debug information
        // auto_aim_interfaces::msg::DebugLight light_data;
        // light_data.center_x = light.center.x;
        // light_data.ratio = ratio;
        // light_data.angle = light.tilt_angle;
        // light_data.is_light = is_light;
        // this->debug_lights.data.emplace_back(light_data);

        return is_light;
    }
    ArmorType Detector::isArmor(const Light& light_1,const Light &light_2)
    {
        // 两灯条颜色不相同
        if(light_1.color != light_2.color)
        {
            return ArmorType::INVALID;
        }
        // 两个灯条的比例大于最小灯条比例
        float light_length_ratio = light_1.length < light_2.length ? 
        light_1.length / light_2.length : light_2.length / light_1.length;

        bool light_ratio_ok = light_length_ratio > a_param.min_light_ratio;
        if(!light_ratio_ok) std::cout<<"Armor/ light_ratio_ok is false."<<std::endl; // debug

        // 灯条中心之间的距离符合大装甲板或者小装甲板的距离(用灯条平均长度作为单位)
        float avg_light_length = (light_1.length + light_2.length) / 2;
        float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length; 
        bool center_distance_ok = (a_param.min_small_center_distance <= center_distance &&
                                center_distance < a_param.max_small_center_distance) ||
                                (a_param.min_large_center_distance <= center_distance &&
                                center_distance < a_param.max_large_center_distance);
        
        if(!center_distance_ok) std::cout<<"Armor/ center_distance_ok is false."<<std::endl; // debug

        cv::Point2f diff=light_1.center-light_2.center;           // 灯条中心差值
        float angle=std::abs(std::atan(diff.y/diff.x))/CV_PI*180; // 弧度制转角度制
        bool angle_ok = angle<a_param.max_angle;

        if(!angle_ok) std::cout<<"Armor/ angle_ok is false."<<std::endl; // debug

        bool is_armor=light_ratio_ok && center_distance_ok && angle_ok;

        if(!is_armor) std::cout<<"Armor/ is_armor is false."<<std::endl; // debug

        // 判断装甲板类型
        ArmorType type;
        if(is_armor)
        {
            type = center_distance>a_param.min_large_center_distance?ArmorType::LARGE : ArmorType::SMALL;
        }
        else{type = ArmorType::INVALID;}
        
        // Fill in debug information
        // auto_aim_interfaces::msg::DebugArmor armor_data;
        // armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
        // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
        // armor_data.light_ratio = light_length_ratio;
        // armor_data.center_distance = center_distance;
        // armor_data.angle = angle;
        // this->debug_armors.data.emplace_back(armor_data);

        // debug
        // std::string typeStr;
        // std::cout<<"The ArmorType is:"<<armorTypeToString(type)<<std::endl;
        
        return type;
    }
    std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
    {
        std::vector<Armor> armors;
        // this->debug_armors.data.clear();
        for(auto light_1 = lights.begin();light_1 != lights.end();light_1++){
            if(light_1->color!=detect_color) continue;

            // 装甲板实际长度=灯条长度*灯条中心间距(用灯条长度作为单位)
            double max_iter_width= light_1->length *a_param.max_large_center_distance; 
            for(auto light_2 = light_1+1;light_2 != lights.end();light_2++){
                if(light_2->color!=detect_color) continue;

                if(containLight(*light_1,*light_2,lights))
                {
                    // std::cout << "Contain light, skipping" << std::endl;  //debug
                    continue;
                }
                if(light_2->center.x-light_1->center.x>max_iter_width)break;

                auto type =isArmor(*light_1,*light_2);
                if(type!=ArmorType::INVALID)
                {   
                    auto armor = Armor(*light_1,*light_2);
                    armor.type = type;
                    armors.emplace_back(armor);
                }
            }
        }
        
        return armors;
    } 
    bool Detector::containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
    {
        if (lights.empty()) 
        {
            return false;
        }

        double avg_length = (light_1.length + light_2.length) / 2.0;
        double avg_width = (light_1.width + light_2.width) / 2.0;
        // 获取灯条之间的矩形
        auto points = std::vector<cv::Point2f>{light_1.top,light_1.bottom,light_2.top,light_2.bottom};
        auto bounding_rect = cv::boundingRect(points);
        for(const auto & test_light : lights)
        {
            // 去除明显不合理的灯条
            if(test_light.width > 2 * avg_width){continue;}
            if(test_light.length < 0.5 * avg_width){continue;}
            // 避免比较自身
            if(test_light.center == light_1.center || test_light.center == light_2.center ){continue;}
            if(bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) 
               || bounding_rect.contains(test_light.center))
               {return true;}
        }
        return false;
    }
    
    void Detector::drawResults(cv::Mat & img)
    {
    // Draw Lights
    // for(const auto & light:lights_)
    // {
    //     // 白色标注角点
    //     cv::circle(img,light.top,2,cv::Scalar(255,255,255),1);
    //     cv::circle(img,light.bottom,3,cv::Scalar(255,255,255),1);
    //     // 红色灯条使用黄色线，蓝色灯条使用粉色线
    //     auto line_color = light.color==RED?cv::Scalar(255,255,0):cv::Scalar(255,0,255);
    //     cv::line(img,light.top,light.bottom,line_color,3);
    // }


    // Draw armors
    for ( auto & armor : armors_) 
    {
        // // 红色灯条使用黄色线，蓝色灯条使用粉色线
        auto line_color = armor.left_light.color == RED?cv::Scalar(255,255,0):cv::Scalar(255,0,255); 
        cv::line(img,armor.left_light.top,armor.left_light.bottom,line_color);
        cv::line(img,armor.right_light.top,armor.right_light.bottom,line_color);

        cv::line(img,armor.left_light.top,armor.right_light.bottom,cv::Scalar(0,255,0),2);
        cv::line(img,armor.left_light.bottom,armor.right_light.top, cv::Scalar(0, 255, 0), 2);
        // std::cout<<"Armors has been drew."<<std::endl; // debug
    }
    // Show numbers and confidence
        for (const auto & armor : armors_) {
            cv::putText(
                img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2);
        }
    }
}
