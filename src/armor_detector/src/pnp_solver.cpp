#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
namespace rm_auto_aim
{
    PnPSolver::PnPSolver(const std::array<double, 9> & camera_matrix,const std::vector<double> & dist_coeffs):
    camera_matrix_(cv::Mat(3,3,CV_64F,const_cast<double *>(camera_matrix.data())).clone()),
    dist_coeffs_(cv::Mat(1,5,CV_64F,const_cast<double *>(dist_coeffs.data())).clone())
    {
        // 单位转化为米
        constexpr double small_half_x = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
        constexpr double small_half_y = SMALL_ARMOR_LENGTH / 2.0 / 1000.0;
        constexpr double large_half_x = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
        constexpr double large_half_y = LARGE_ARMOR_LENGTH / 2.0 / 1000.0;
        // 将装甲顶点的坐标存入相应向量
        // 使用 emplace_back 依次插入四个顶点
        small_armor_points_.emplace_back(cv::Point3f(-small_half_x,-small_half_y,0));
        small_armor_points_.emplace_back(cv::Point3f( small_half_x,-small_half_y,0));
        small_armor_points_.emplace_back(cv::Point3f( small_half_x, small_half_y,0));
        small_armor_points_.emplace_back(cv::Point3f(-small_half_x, small_half_y,0));

        large_armor_points_.emplace_back(cv::Point3f(-large_half_x,-large_half_y,0));
        large_armor_points_.emplace_back(cv::Point3f( large_half_x,-large_half_y,0));
        large_armor_points_.emplace_back(cv::Point3f( large_half_x, large_half_y,0));
        large_armor_points_.emplace_back(cv::Point3f(-large_half_x, large_half_y,0));
    }
    bool PnPSolver::solvePnP(const Armor &armor,cv::Mat &rmat,cv::Mat &tvec)
    {
        // 旋转向量
        cv::Mat rvec;

        // 收集装甲在图像上的四个 2D 顶点（左灯条的底、左灯条的顶、右灯条的顶、右灯条的底）
        std::vector<cv::Point2f> image_armor_points;
        image_armor_points.emplace_back(armor.left_light.bottom);
        image_armor_points.emplace_back(armor.left_light.top);
        image_armor_points.emplace_back(armor.right_light.top);
        image_armor_points.emplace_back(armor.right_light.bottom);

        // 根据装甲类型 (SMALL or LARGE) 选择对应的 3D 顶点坐标集合
        auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;

        // 调用 OpenCV 的 solvePnP 函数，使用 IPPE 方法 (cv::SOLVEPNP_IPPE) 进行位姿求解
        // - object_points：装甲在 3D 空间中的顶点坐标
        // - image_armor_points：装甲在图像上的顶点坐标
        // - camera_matrix_：相机内参矩阵
        // - dist_coeffs_：相机畸变系数
        // - rvec：输出旋转向量
        // - tvec：输出平移向量
        // - false：表示不使用初始估计
        bool success = cv::solvePnP(object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

        if (success) 
        {
            // 将旋转向量转换为旋转矩阵
            cv::Rodrigues(rvec, rmat);
        }

        return success;
    }
    float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
    {
        // camera_matrix_ 中 (0,2) 和 (1,2) 分别存储成像中心 cx, cy
        float cx = camera_matrix_.at<double>(0, 2);
        float cy = camera_matrix_.at<double>(1, 2);

        // 计算给定点 (image_point) 到 (cx, cy) 的欧几里得距离
        return cv::norm(image_point - cv::Point2f(cx, cy));
    }
    void PnPSolver::drawResults(cv::Mat& img,const std::string& text,const cv::Point &textOrg) 
    {
        double fontScale = 1;
        cv::Scalar textColor(255, 255, 255);  // 白色文字
        int thickness = 2;
        // 在图像上绘制文字
        cv::putText(img, text, textOrg,cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);
    }
} // namespace rm_auto_aim