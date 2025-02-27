#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <cv_bridge/cv_bridge.h>
// image_transport:传输图像
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
// rclcpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
// sensor_msgs:传感器
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// visualization_msgs:可视化消息包
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
namespace rm_auto_aim
{
    class ArmorDetectorNode:public rclcpp::Node
    {
    
    public:
        ArmorDetectorNode(const rclcpp::NodeOptions &options);   
    private:
        // 图像订阅回调函数
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        // 初始化节点
        std::unique_ptr<Detector> initDetector();

        std::unique_ptr<Detector> detector_;
        auto_aim_interfaces::msg::Armors armos_msg_;

    };  

}
#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_