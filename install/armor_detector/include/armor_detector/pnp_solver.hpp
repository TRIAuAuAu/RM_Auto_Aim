#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_


#include <geometry_msgs/msg/point.hpp> // ros2消息类型：用于表示三维空间中的点
#include <opencv2/core.hpp>

#include <array>
#include <vector>

#include "armor_detector/armor.hpp"
namespace rm_auto_aim
{
class PnPSolver
{
public:
  // 构造函数：接收相机矩阵 (camera_matrix) 和畸变系数 (distortion_coefficients)
  // 其中 camera_matrix 大小是 3x3，distortion_coefficients 通常包含 5 个畸变参数
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // solvePnP：通过给定的 Armor 信息，计算 3D 位姿 (旋转矩阵rmat 和平移向量 tvec)
  bool solvePnP(const Armor & armor,  cv::Mat & rmat, cv::Mat & tvec);

  // calculateDistanceToCenter：计算某个图像点 (image_point) 与相机成像中心之间的距离
  float calculateDistanceToCenter(const cv::Point2f & image_point);
  
  // debug
  void drawResults(cv::Mat& img, const std::string& text,const cv::Point &textOrg);

private:
  // 相机内参矩阵和畸变系数
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // 以 mm（毫米）为单位的小装甲和大装甲尺寸
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_LENGTH = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float LARGE_ARMOR_LENGTH = 55;

  // 小装甲和大装甲在 3D 空间中的四个顶点坐标
  // 注意，这里的坐标单位为米（具体转换在实现文件中）
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};
} // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_