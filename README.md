# armor_detector
识别装甲板，并通过detector_node发送装甲板信息（还没写好）
啥都不会，基本照搬RV，修改挺少的
## detector
### LightParams：灯条参数，用于判断是否为灯条
长宽比：最大值和最小值

倾斜角：最大的倾斜角度
### ArmorParams：装甲板参数，用于判断装甲板
最小灯条比例：  

灯条中心间距：大装甲板，小装甲板，以灯条平均长度作为单位

### preprocessImage：图像预处理
第一种方式：使用threshold函数的THRESH_OTSU方法进行二值化

第二种方式：转化为HSV色彩空间，使用inRange函数进行二值化

### findLights：寻找灯条
使用findContours函数寻找所有轮廓，调用isLight函数判断该轮廓是否为灯条

把判断灯条颜色部分的代码修改为分离颜色通道，比较R和B通道总和
### isLight：判断一个旋转矩形（灯条）是否符合要求
对角度和长宽比进行判断

### matchLights:匹配两个灯条，返回一个armor容器
调用containLight函数，去掉中间含有其他灯条的一对灯条
调用isArmor函数，判断两个灯条是否可以组成一个装甲板
#### containLight：判断两个灯条中间是否包含其他灯条
先根据平均长宽去除明显不合理的灯条，再使用矩形类自带的contains成员函数判断两灯条中间是否包含其他灯条
#### isArmor：判断两个灯条是否构成一个装甲板
两个灯条的比例大于最小灯条比例  
灯条中心之间的距离符合大装甲板或者小装甲板的距离(用灯条平均长度作为单位)  
灯条中心连线的角度符合角度要求  
返回装甲板类型

### drawResults：可视化识别结果

通过上面的检测后，可以找出符合数学条件的装甲板，但是还需要判断装甲板的数字类型进一步确认

（onnx模型是直接下载RV的）

## numberclassify:分类数字

### extractNumbers：预处理，并取ROI区域，进行透视变换

### classify：实现分类

#### 加载深度学习模型：

类的构造函数，调用cv::readFromONNX，用成员net_存储。调用std::getline，用成员class_name_存储

#### 转化图像为模型可以读取的类型

调用cv::dnn::blobFromImage函数

#### 网络向前传递，将神经网络输出结果转化为概率

调用setInput将转化好的图像输入网络，使用softmax函数(源码没封装)

    // 实现 Softmax 函数
    cv::Mat softmax(const cv::Mat& logits) {
      // 找出输入矩阵中的最大值
      double maxVal;
      cv::minMaxLoc(logits, nullptr, &maxVal, nullptr, nullptr);
    
      // 为了避免指数运算时出现数值溢出问题，将输入矩阵中的每个元素减去最大值
      cv::Mat expLogits;
      cv::exp(logits - cv::Scalar::all(maxVal), expLogits);
    
      // 计算指数值的总和
      cv::Scalar sumExp = cv::sum(expLogits);
    
      // 将指数值矩阵中的每个元素除以总和，得到概率分布
      cv::Mat probabilities = expLogits / sumExp[0];
    
      return probabilities;
    }
获取最大概率和类别索引：使用 cv::minMaxLoc 函数在 Softmax 概率分布中查找最大概率值及其对应的位置（类别索引）

使用类内confidence成员存储置信度，使用类内number成员存储类别名称

### remove_if搭配erase:移除分类结果不合理的装甲板，同时可以去掉识别到的错误装甲板(不含数字的)

## pnp_solver:计算相机的位姿
主要调用cv::solvePnP函数进行结算，输入了装甲板的四个角点，返回旋转向量和平移向量

这两个向量可以在detector_node中进一步转变为x,y,z坐标和四元数

## corner_corrector:矫正角点

使用了主成分析法矫正角点(修改了点云的生成逻辑，只生成非0像素点)

主要逻辑：

使用cv::moment函数图像计算矩，通过矩找到质点替换light.center

      // 生成点云
      for(const auto& point : non_zero)
      {
          int repeat_Times = std::round(roi.at<float>(point));
          for(int k=0;k<repeat_Times;k++)
          {
              points.emplace_back(cv::Point2f(point.x,point.y));
          }
      }

该循环根据像素点亮度，多次将像素点添加到容器。容器内部亮度越大的点权重越大。

成员变量axis记录了cv::PCA中占权重最大的点，归一化便于使用

沿着对称轴axis，计算相邻两个点的亮度差，把差值大的点存储进容器。求这些点的均值，作为light.top和light.bottom

---

基本都是抄的，了解了RV源码涉及到的opencv的函数，ROS2部分的detector_node还没写完。









