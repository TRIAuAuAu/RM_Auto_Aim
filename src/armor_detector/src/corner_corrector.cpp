#include "armor_detector/corner_corrector.hpp"

namespace rm_auto_aim{

void LightCornerCorrector::processLight(Light& light, const cv::Mat& gray_img) {
    constexpr int PASS_OPTIMIZE_WIDTH = 3;
    if (light.width > PASS_OPTIMIZE_WIDTH) {
        // Find the symmetry axis of the light
        SymmetryAxis axis = findSymmetryAxis(gray_img, light);
        light.center = axis.centroid;
        light.axis = axis.direction;
        // Find the corner of the light
        if (cv::Point2f top = findCorner(gray_img, light, axis, "top"); top.x > 0) {
            light.top = top;
        }
        if (cv::Point2f bottom = findCorner(gray_img, light, axis, "bottom"); bottom.x > 0) {
            light.bottom = bottom;
        }
    }
}

void LightCornerCorrector::correctCorners(Armor& armor, const cv::Mat& gray_img) {
    
    processLight(armor.left_light, gray_img);
    processLight(armor.right_light, gray_img);   
}


SymmetryAxis LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img, const Light &light)
{
    constexpr float MAX_BRIGHTNESS = 25;
    constexpr float SCALE = 0.07;

    // Scale the bounding box
    cv::Rect light_box = light.boundingRect();
    // 移动边界框左上角坐标
    light_box.x -= light_box.width * SCALE;
    light_box.y -= light_box.height * SCALE;
    // 扩大边界框宽高
    light_box.width += light_box.width * SCALE * 2;
    light_box.height += light_box.height * SCALE * 2;

    // Check boundary
    // 对边界值取大或取小，防止访问越界
    light_box.x = std::max(light_box.x, 0);
    light_box.x = std::min(light_box.x, gray_img.cols - 1);
    light_box.y = std::max(light_box.y, 0);
    light_box.y = std::min(light_box.y, gray_img.rows - 1);
    light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
    light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

    // Get normalized light image
    cv::Mat roi = gray_img(light_box);
    float mean_vel = cv::mean(roi)[0]; // mean函数取均值，灰度图0索引处即为总和
    roi.convertTo(roi,CV_32F);
    cv::normalize(roi,roi,0,MAX_BRIGHTNESS,cv::NORM_MINMAX);

    // Calculate the centroid
    cv::Moments moments = cv::moments(roi, false);                 // 计算矩，指定输入图像为非二值图像
    cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
                            cv::Point2f(light_box.x, light_box.y); // 计算质心坐标

    // Initialize the PointCloud：优化前需要遍历所有点
    //   std::vector<cv::Point2f> points;
    //   for (int i = 0; i < roi.rows; i++) {
    //     for (int j = 0; j < roi.cols; j++) {
    //       for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
    //         points.emplace_back(cv::Point2f(j, i));
    //       }
    //     }
    //   }

    // Initialize the PointCloud:仅遍历非0像素

    std::vector<cv::Point2f> points;
    // 创建掩码：0像素取0，非0像素为true
    cv::Mat mask = ( roi > 0);
    // 放大像素值
    cv::multiply(roi,25,roi);
    // 查找非0元素
    std::vector<cv::Point> non_zero;
    cv::findNonZero(mask,non_zero);
    // 生成点云
    for(const auto& point : non_zero)
    {
        int repeat_Times = std::round(roi.at<float>(point));
        for(int k=0;k<repeat_Times;k++)
        {
            points.emplace_back(cv::Point2f(point.x,point.y));
        }
    }
    cv::Mat points_mat = cv::Mat(points).reshape(1); // 可以不写reshape(1)

    // PCA (Principal Component Analysis)
    auto pca = cv::PCA(points_mat,cv::Mat(),cv::PCA::DATA_AS_ROW);

    // Get the symmetry axis
    cv::Point2f axis = cv::Point2f(pca.eigenvectors.at<float>(0,0),
                                   pca.eigenvectors.at<float>(0, 1));
    
    // Normalize the axis
    axis = axis / cv::norm(axis);

    // 默认情况下axis向量有可能朝下，统一朝上
    if (axis.y > 0) 
    {
        axis = -axis;
    }
    return SymmetryAxis{.centroid=centroid,.direction=axis,.mean_val=mean_vel};
}

cv::Point2f LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                             const Light &light,
                                             const SymmetryAxis &axis,
                                             std::string order)
{
    constexpr float START = 0.8 / 2;
    constexpr float END = 1.2 / 2;

    auto inImage = [&gray_img](const cv::Point &point)->bool
    {
        return point.x>=0 && point.x<gray_img.cols && point.y>=0 && point.y<gray_img.rows;
    };
    auto distance = [](const cv::Point2f& p0, const cv::Point2f& p1) -> float 
    {
        return cv::norm(p0-p1);
    };


    int oper = order == "top" ? 1 : -1; // 确认方向，1为向上，-1为向下
    float L = light.length;             // 灯条长度
    float dx = axis.direction.x * oper; 
    float dy = axis.direction.y * oper;

    std::vector<cv::Point2f> candidates; // 候选点的容器
    int n = light.width - 2;    
    int half_n = std::round(n / 2);
    for (int i = -half_n; i <= half_n; i++) 
    {   // i在[-half_n,half_n]之间变化，可以在横向上确认多个起始点
        // L * START * dx,dy为起始偏移量，即搜索起点相对对质心的偏移量
        // (x0,y0)为起始点
        float x0 = axis.centroid.x + L * START * dx + i;
        float y0 = axis.centroid.y + L * START * dy;

        cv::Point2f prev = cv::Point2f(x0, y0);   // 前一个搜索点
        cv::Point2f corner = cv::Point2f(x0, y0); // 候选点
        float max_brightness_diff = 0; 
        bool has_corner = false;     

        cv::Point2f startPoint(x0, y0);
        for (float x = x0 + dx, y = y0 + dy; distance(cv::Point2f(x, y), startPoint) < L * (END - START);
             x += dx, y += dy) 
        {
            cv::Point2f cur = cv::Point2f(x, y);
            if (!inImage(cv::Point(cur))) {
                break;
            }

            float brightness_diff = gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
            if (brightness_diff > max_brightness_diff && gray_img.at<uchar>(prev) > axis.mean_val) 
            {   // gray_img.at<uchar>(prev)>axis.mean_val确保前一点亮度为正
                max_brightness_diff = brightness_diff;
                corner = prev;
                has_corner = true;
            }

            prev = cur;
        }
        // 存放候选点
        if (has_corner) {
            candidates.emplace_back(corner);
        }
    }
    if (!candidates.empty()) {
        cv::Point2f result(0, 0);
        for (const auto& point : candidates) {
        result.x += point.x;
        result.y += point.y;
        }
        return result / static_cast<float>(candidates.size());
    }
    return cv::Point2f(-1, -1);
}

}   // namespace rm_auto_aim