#ifndef ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_
#define ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_

// opencv
#include <opencv2/opencv.hpp>
// project
#include "armor_detector/armor.hpp"

namespace rm_auto_aim {

struct SymmetryAxis {
  cv::Point2f centroid;  // 质心
  cv::Point2f direction; // 方向
  float mean_val;        // 平均亮度值
};
class LightCornerCorrector {
public:
  explicit LightCornerCorrector() noexcept {}

    void processLight(Light& light, const cv::Mat& gray_img);

  // Correct the corners of the armor's lights
  void correctCorners(Armor &armor, const cv::Mat &gray_img);

private:
  // Find the symmetry axis of the light
  SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const Light &light);

  // Find the corner of the light
  cv::Point2f findCorner(const cv::Mat &gray_img,
                         const Light &light,
                         const SymmetryAxis &axis,
                         std::string order);
};

}  // namespace rm_auto_aim
#endif  // ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_