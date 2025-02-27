// 装甲板和灯条的定义，取自rv
// 宏定义防止重复包含
#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
    //颜色常量：红，蓝
const int RED = 0;
const int BLUE = 1;
    // 枚举类：装甲板类型
enum class ArmorType { SMALL, LARGE, INVALID };
inline std::string armorTypeToString(const ArmorType &type) {
  switch (type) {
    case ArmorType::SMALL:
      return "small";
    case ArmorType::LARGE:
      return "large";
    default:
      return "invalid";
  }
}
    // 结构体：灯条，继承自旋转矩形 RotatedRect
struct Light : public cv::RotatedRect
{
    // 默认构造：无
  Light() = default;
    // 显式构造：防止隐式转换
    // point方法：获得四个角点
    // sort方法：将点的y坐标从小到大排列(lambda表达式)
    // norm方法：得到两点之间距离   
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);  
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;    
    // color：灯条的颜色。
    // top 和 bottom：灯条的顶部和底部中心点。
    // length：灯条的长度。
    // width：灯条的宽度。
    // tilt_angle：灯条的倾斜角度。
  cv::Point2f axis;
    // axis:对称轴
};
// 结构体：装甲板
struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;

    // left_light 和 right_light：装甲板的左灯条和右灯条。
    // center：装甲板的中心点。
    // type：装甲板的类型（ArmorType）。
    // number_img：包含装甲板数字的图像。
    // number：数字的字符串表示。
    // confidence：识别的置信度。
    // classfication_result：分类结果。
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
