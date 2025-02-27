#include <iostream>
#include <fmt/format.h>
// opencv
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include <armor_detector/armor.hpp>
#include <armor_detector/detector.hpp>
#include <armor_detector/number_classifier.hpp>
#include <armor_detector/pnp_solver.hpp>
#include <armor_detector/tools.hpp>

#include <memory>
// for path
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace fs = std::filesystem;
using namespace rm_auto_aim;
using namespace tools;

void test_Video(const std::string &path,Detector &detector,PnPSolver pnp_solver)
{
    
    // 读取视频
    cv::VideoCapture video(path);
    if(!video.isOpened())
    { 
        std::cout<<"video is not open!"<<std::endl;
        return;
    }
    while (true)
    {
        cv::Mat frame;
        video >> frame;
        if(frame.empty())
        { 
            // std::cout<<"the frame is empty!"<<std::endl; 
            break;
        }
        // // 降低分辨率
        // cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        // 滤波 // debug
        // Tools filter;
        // filter.filterImageWithTrackbars(frame.clone());

        // 检测装甲板
        std::vector<Armor> armors00;
        armors00 = detector.detect(frame);

        // 提取数字图像
        detector.classifier->extractNumbers(frame,armors00);

        // 分类数字
        detector.classifier->classify(armors00);

        // 绘制结果
        detector.drawResults(frame);

        // 计算旋转矩阵和欧拉角
        for (const auto& armor : armors00) 
        {
            cv::Mat rotation_matrix, tvec;
            bool success = pnp_solver.solvePnP(armor, rotation_matrix, tvec);
            if (success) {
                double roll, pitch, yaw;

                // 计算 pitch 角
                pitch = -std::asin(rotation_matrix.at<double>(2, 0));

                // 检查是否出现奇异情况（万向节锁）
                if (std::abs(std::cos(pitch)) > 1e-6) {
                    // 正常情况
                    roll = std::atan2(rotation_matrix.at<double>(2, 1) / std::cos(pitch), rotation_matrix.at<double>(2, 2) / std::cos(pitch));
                    yaw = std::atan2(rotation_matrix.at<double>(1, 0) / std::cos(pitch), rotation_matrix.at<double>(0, 0) / std::cos(pitch));
                } else {
                    // 奇异情况（pitch 接近 ±π/2）
                    roll = std::atan2(-rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(1, 1));
                    yaw = 0;
                }

                std::string euler_text = fmt::format("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}", roll*57.3, pitch*57.3, yaw*57.3);
                cv::Point textOrg(10, 30);
                pnp_solver.drawResults(frame, euler_text, textOrg);
            }
        }
        // 显示结果
        cv::imshow("Draw Results", frame);

        // 计算每帧之间的时间间隔并等待，同时检查是否按下 'q' 键
        int delay = 1000 / video.get(cv::CAP_PROP_FPS);
        if (cv::waitKey(delay) == 'q') 
        {
            break;
        }
    }
}
void test_Image(std::string path,Detector &detector)
{
    // 读取图片
    std::string img_path = path;
    cv::Mat input_image = cv::imread(img_path);
    if (input_image.empty()) {
        std::cerr << "Failed to read image: " << img_path<< std::endl;
        return;
    }

    // 检测装甲板
    std::vector<Armor> armors;
    armors = detector.detect(input_image);

    if(!armors.empty()){std::cout<<"armor is not empty:"<<armors.size()<<std::endl;}
    else std::cout<<"armor is empty!"<<std::endl;
    // 提取数字图像
    detector.classifier->extractNumbers(input_image, armors);

    // 分类数字
    detector.classifier->classify(armors);

    // // 绘制结果
    detector.drawResults(input_image);

    // 获取所有数字图像
    // cv::Mat all_numbers_image = detector.getAllNumbersImage();

    // 显示结果
    cv::namedWindow("Draw Results",cv::WINDOW_NORMAL);
    cv::imshow("Draw Results", input_image);
    // cv::imshow("All Numbers Image", all_numbers_image);
    cv::waitKey(0);
}
int main() 
{
    // std::string pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");

    Detector::LightParams l;
    l.min_ratio = 0.08; // default:0.08
    l.max_ratio = 0.4;  // default:0.4
    l.max_angle = 40.0;

    Detector::ArmorParams a;
    a.min_light_ratio = 0.6;
    a.min_small_center_distance = 0.8;
    a.max_small_center_distance = 3.2; // default:3.2
    a.min_large_center_distance = 3.2;
    a.max_large_center_distance = 5.5; // default:5.0
    a.max_angle = 35.0;

    int binary_thres = 160; 
    int detect_color = RED;  // 待检测装甲板颜色

    // 创建Detector对象
    Detector detector(binary_thres,detect_color,l,a);

    // 初始化Detector所需的参数
    // std::string model_path = fs::path(pkg_share_path) / "model/mlp.onnx";
    // std::string label_path = fs::path(pkg_share_path) / "model/label.txt";

    // 创建NumberClassifier对象
    std::string model_path ="/home/auauau/RM_Auto_Aim/src/armor_detector/model/mlp.onnx";
    std::string label_path = "/home/auauau/RM_Auto_Aim/src/armor_detector/model/label.txt";

    double threshold = 0.6; // 分类置信度阈值
    std::vector<std::string> ignore_classes{"negative"};
    detector.classifier = std::make_unique<rm_auto_aim::NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    // 相机内参:参考网上的
    static const cv::Mat camera_matrix =
    (    cv::Mat_<double>(3,3) << 1286.307063384126 , 0                 , 645.34450019155256,
                                 0                 , 1288.1400736562441, 483.6163720300021,
                                 0                 , 0                 , 1                            
    );

    // 畸变参数:参考网上的
    static const cv::Mat distort_coeffs = 
    ( cv::Mat_<double>(1,5) << -0.47562935060124745, 0.12831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0 );
    
    // 创建PnPSolver对象
    std::array<double, 9> camera_matrix_array;
    std::copy(camera_matrix.ptr<double>(0), camera_matrix.ptr<double>(0) + 9, camera_matrix_array.begin());
    std::vector<double> distort_coeffs_vector(distort_coeffs.ptr<double>(0), distort_coeffs.ptr<double>(0) + 5);
    PnPSolver pnp_solver(camera_matrix_array, distort_coeffs_vector);

    // 读取视频
    //test_Video("/home/auauau/RM_Auto_Aim/src/armor_detector/ARMORS/car.mp4",detector,pnp_solver);

    // 读取图片
    test_Image("/home/auauau/RM_Auto_Aim/src/armor_detector/ARMORS/num_raw.png",detector);

    // 等待按键
    cv::waitKey(0);
    return 0;
}
