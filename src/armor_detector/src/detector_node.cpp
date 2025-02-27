#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
    RCLCPP_INFO(this->get_logger(),"Starting DetectorNode!");
}
std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
    // 描述并设置参数
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    // 描述整数范围
    param_desc.integer_range.resize(1);
    // 整数范围成员:仅有一个范围,范围为0~255
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = this->declare_parameter("binary_thres",160,param_desc); // 默认值为160

    param_desc.description = "0-RED,1-BLUE";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    auto detector_color = this->declare_parameter("detector_color",RED,param_desc);

    Detector::LightParams l_params = {
        .min_ratio = this->declare_parameter("light_min_ratio",0.1),
        .max_ratio = this->declare_parameter("light_max_ratio",0.4),
        .max_angle = this->declare_parameter("light_max_angle",40.0)
    };
    Detector::ArmorParams a_params ={
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

    auto detector = std::make_unique<Detector>(binary_thres, detector_color, l_params, a_params);

    // Init classifier
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    auto model_path = pkg_path + "/model/mlp.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    double threshold = this->declare_parameter("classifier_threshold", 0.7);
    std::vector<std::string> ignore_classes =
        this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
    detector->classifier =
        std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
    }
}
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode);