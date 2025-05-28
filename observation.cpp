/* 
基于单应矩阵的3D定位系统
*/

#define _USE_MATH_DEFINES  // 启用数学常量

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <numeric>
#include <iomanip>       // 输入输出格式控制

// Eigen头文件
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV头文件
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
const double start_x = 0.928;
const double start_y = 0.140;
const double z_height = 1.941;
const Size pattern_size(3, 3);  // 棋盘格内角点数目
const double square_size = 0.03;

// 函数声明
void find_feature_matches_and_compute_homography(
    const Mat &img,
    const vector<Point2f> &plane_points,    
    Mat &R, Mat &T,
    const Mat &K,
    Mat &img_corners);

// 用于处理中文文件名的辅助函数
Mat readImageWithUnicodePath(const string& filename) {
    // 将多字节字符串(UTF-8)转换为宽字符(UTF-16)
    int requiredSize = MultiByteToWideChar(CP_UTF8, 0, filename.c_str(), -1, NULL, 0);
    if (requiredSize <= 0) {
        cerr << "转换路径失败，错误码: " << GetLastError() << endl;
        return Mat();
    }

    vector<wchar_t> wideFilename(requiredSize);
    MultiByteToWideChar(CP_UTF8, 0, filename.c_str(), -1, &wideFilename[0], requiredSize);

    // 使用宽字符路径读取图像
    return imread(String(wideFilename.begin(), wideFilename.end() - 1), IMREAD_GRAYSCALE);
}

// 用于显示中文窗口标题的辅助函数
void showImageWithChineseTitle(const string& title, const Mat& image) {
    // 确保窗口名称使用正确的编码
    namedWindow(title, WINDOW_NORMAL);
    imshow(title, image);
}

// void pcl_point_cloud_process(
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_cloud, // 场景坐标
//     const vector<Point2f> &plane_points,
//     Mat &Rp, 
//     Mat &Tp);



int main(int argc, char **argv) {
    /* 参数说明：
    argv[1] : 输入图像路径
    K : 相机内参矩阵 [fx, 0, ; 0, fy, cy; 0, 0, 1]
    dist_coeffs : 镜头畸变系数 [k1, k2, p1, p2, k3]
    */

    // 设置控制台编码为UTF-8，确保正确显示中文输出
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    setlocale(LC_ALL, ".UTF8");

    // 如果没有提供图像路径（即 argc != 2），程序提示正确的使用方式并退出。
    if (argc != 2) {
        cout << "Usage: ./H_3d path_to_image" << endl;
        return -1;
    }
    
   
/* 参数说明：
   argv[1] : 输入图像路径
   K : 相机内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
   dist_coeffs : 镜头畸变系数 [k1, k2, p1, p2, k3]
   */
   
//!----------------------- 1. 相机参数初始化 -----------------------
    
    // Mat K = (Mat_<double>(3,3) << 
    //     967.450, 0, 631.27,
    //     0, 972.936, 558.1723,
    //     0, 0, 1); 
    
    // Mat dist_coeffs = (Mat_<double>(1,5) << -0.170733, 0.102965, 0.002771, -0.007285, 0.000000);
   
    Mat K = (Mat_<double>(3,3) << 
        1241.76, 0, 673.50,
        0, 1233.33, 499.84,
        0, 0, 1); 
    Mat dist_coeffs = (Mat_<double>(1,5) << 0, 0, 0, 0, 0.000000);


//!----------------------- 2. 图像读取与校正 -----------------------
    // 使用处理Unicode路径的函数替代直接调用imread
    Mat img = readImageWithUnicodePath(argv[1]);
    Mat undistorted_img; //去除镜头畸变的图像
    Mat img_corners;  // 用于存储角点可视化结果
    
    if (img.empty()) {
        cout << "无法打开或找到图像，请检查文件路径是否正确" << endl;
        return -1;
    }
    
    undistort(img, undistorted_img, K, dist_coeffs);  // 去除镜头畸变

    vector<Point2f> plane_points;    // 物理坐标
    
//!----------------------- 3. 定义平面坐标点(square_size控制格子大小) --------------------------
    cout << "单应平面处于Z=0的平面坐标系的棋盘格角点坐标: " << endl;
    for (int i = 0; i < pattern_size.height; ++i) {  // 3行
        for (int j = 0; j < pattern_size.width; ++j) {  // 3列
            plane_points.emplace_back(square_size + j * square_size, square_size + i * square_size);
            cout << "(" << square_size + j * square_size << ", " << square_size + i * square_size << ")";
            if ((j + 1) % pattern_size.width == 0) cout << endl;
            else cout << ", ";
        }
        
    }

//!----------------------- 4. 定义场景点云(2d,z实际并咩有关系) --------------------------
    vector<Point3f> scene_points;    // 场景点云坐标
    vector<Point3f> aligned_scene_points;    // 对齐用场景点云坐标
    // 定义步长和数量
    
    double step = square_size;    // 步长
    int rows = pattern_size.height;          // 行数
    int cols = pattern_size.width;         // 列数

    //? 第二个，三个的坐标：1.022, 1.753;  1.022, 3.235

    // 使用循环生成坐标点
    cout << "生成场景点云坐标: " << endl;
    for (int i = 0; i < rows; ++i) {       // 行循环
        for (int j = 0; j < cols; ++j) {   // 列循环
            double x = start_x + j * step; // 计算X坐标
            double y = start_y + i * step; // 计算Y坐标
            cout << "(" << x << ", " << y << ", " << z_height << ")";
            if ((j + 1) % cols == 0) cout << endl;
            else cout << ", ";
        }
    }
    
    // 生成调整后的场景点云（Y轴镜像）
    // cout << "生成对齐用场景点云坐标: " << endl;
    // for (int i = rows-1; i >= 0; --i) {  // Y轴倒序（从6到0）
    //     for (int j = 0; j < cols; ++j) { // X轴保持顺序
    //         double x = start_x + j * step;
    //         double y = start_y + i * step; 
    //         aligned_scene_cloud->push_back(pcl::PointXYZ(x, y, 0)); // 添加到对齐点云
    //         cout << "(" << x << ", " << y << ", 0)";
    //         if ((j + 1) % cols == 0) cout << endl;
    //         else cout << ", ";
    //     }
    // }


    //----------------------- 5. 视觉定位计算得到R T ------------------------
    Mat T;
    Mat R(3, 3, CV_64F);
    find_feature_matches_and_compute_homography(undistorted_img, plane_points, R, T, K, img_corners);
    // R = (cv::Mat_<double>(3, 3) <<
    // 1.0, 0.0, 0.0,
    // 0.0, -1.0, 0.0,
    // 0.0, 0.0, -1.0
    // );

    //----------------------- 6. 点云配准计算得到Rp,Tp ------------------------
    // Mat Rp, Tp;
    // pcl_point_cloud_process(scene_cloud, plane_points, Rp, Tp);
    cv::Mat Rp = (cv::Mat_<double>(3, 3) <<
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
    );

    cv::Mat Tp = (cv::Mat_<double>(3,1) << start_x, start_y, z_height);
   
    cout << "Rp: " << Rp << endl;
    cout << "Tp: " << Tp << endl;



    //----------------------- 7. 计算相机在世界坐标系中的位置 ------------------------
    // 计算 R 的逆矩阵
    Mat R_inv = R.t();
    
    // 计算 -Rp * R^{-1} * t + tp
    Mat camera_pos_3d = -Rp * (R_inv * T) + Tp;
        
    // 正交投影获取二维坐标 (取前两个分量)
    Point2f camera_pos_2d(camera_pos_3d.at<double>(0), 
                         camera_pos_3d.at<double>(1));
    
    // 输出最终结果    
    cout << "\n相机在世界坐标系中的二维位置: (" 
    << camera_pos_2d.x << ", " << camera_pos_2d.y << ")" << endl;
    // 使用新函数显示图像，确保中文标题正确显示
    showImageWithChineseTitle("conner_test", img_corners);      
    waitKey(0);
    return 0;
}


//===================== 视觉特征处理与单应矩阵计算 =====================
void find_feature_matches_and_compute_homography(
    const Mat &img,
    const vector<Point2f> &plane_points, 
    Mat &R, Mat &T,
    const Mat &K,
    Mat &img_corners)
{
    /* 功能：通过棋盘格特征计算相机位姿
       参数：
       - img: 输入图像
       - plane_points: 物理坐标系点
       - R/T: 输出的旋转矩阵和平移向量
       - K: 相机内参矩阵
    */

    // 1. 棋盘格角点检测
    vector<Point2f> corners,corners_sorted;// 检测到的角点
    bool found = findChessboardCornersSB(img, pattern_size, corners);
    
    // 使用自定义排序逻辑（确保与物理坐标系x从左到右，y从下到上一致）
    if (found) {
    // 复制角点并排序
    corners_sorted = corners;
    std::sort(corners_sorted.begin(), corners_sorted.end(), [](const Point2f& a, const Point2f& b) {
        const float eps = 5;
        if (std::abs(a.y - b.y) < eps) {
            return a.x < b.x;
        }
        return a.y > b.y;

    });

    // 打印排序后的角点（保留两位小数）
    std::cout << "Sorted corners (y, then x):" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < corners_sorted.size(); ++i) {
        std::cout << "[" << i << "] (" << corners_sorted[i].x 
                  << ", " << corners_sorted[i].y << ")" << std::endl;
    }
}

   // 2. 显示检测结果
    // 创建彩色图像用于可视化
    
    cvtColor(img, img_corners, COLOR_GRAY2BGR);

    // 自定义绘制更精细的角点标记（红色小圆圈+中心点）
    const int marker_radius = 2;  // 标记半径减小为3像素
    const Scalar marker_color(0, 0, 255); // BGR红色
    
    for (const auto& corner : corners_sorted) {
        // 绘制中心点
        circle(img_corners, corner, 1, marker_color, -1);
    }
    

    // 3. 匹配点对应验证
    cout << "\n===== 点对应验证 =====" << endl;
    cout << "平面坐标索引 | 坐标 | 图像坐标索引 | 坐标 (u,v) | 相邻点间距" << endl;
    cout << fixed << setprecision(3);
    vector<double> distances;

    for (size_t i = 0; i < plane_points.size(); ++i) {
        // 打印当前点信息
        cout << "[" << setw(2) << i << "]    "
            << "(" << setw(5) << plane_points[i].x << ", " << setw(5) << plane_points[i].y << ")    "
            << "[" << setw(2) << i << "]    "
            << "(" << setw(7) << corners_sorted[i].x << ", " << setw(7) << corners_sorted[i].y << ")";

        // 计算与上一个点的间距
        if (i > 0) {
            double dx = corners_sorted[i].x - corners_sorted[i-1].x;
            double dy = corners_sorted[i].y - corners_sorted[i-1].y;
            double dist = sqrt(dx*dx + dy*dy);
            distances.push_back(dist);
            cout << "    Δ=" << setw(5) << dist << "px";
        }
        cout << endl;
    }

    //  cv::Mat cv::findHomography(InputArray srcPoints, InputArray dstPoints, OutputArray mask,
    //  目标图像中的点 = H * 源图像中的点
    Mat H = findHomography(plane_points, corners_sorted, RANSAC);
    cout << "平面坐标系变换到相机坐标系的单应矩阵 H:\n" << H << endl;

    // 4. 单应矩阵分解
    /* 
    根据公式：t = K^{-1} * H_3 / ||K^{-1} * H_1||
    */
    
    // 带转置，与老师的相同
    // T1 = ((K.inv()).t() * H.col(2)) / norm((K.inv()).t() * H.col(0), NORM_L2);
    // cout << "使用H计算的平移向量(第一列带转置) t:\n" << T1 << endl;

    T = (K.inv() * H.col(2)) / norm(K.inv() * H.col(0), NORM_L2);
    // T.at<double>(1) *= -1; // 反转Y轴方向
    cout << "使用H计算的平移向量 T:\n" << T << endl;

    // 4.6 计算旋转矩阵 R
    /* 
    根据公式：R = [r1 r2 r3]
    其中：
    r1 = K^{-1}H_1 / ||K^{-1}H_1||  （第一列）
    r2 = K^{-1}H_2 / ||K^{-1}H_2||  （第二列）
    r3 = r1 × r2                    （第三列，叉积）
    */

    // 计算r1（第一列）
    Mat r1 = K.inv() * H.col(0) / norm(K.inv() * H.col(0));
    
    // 计算r2（第二列）
    Mat r2 = K.inv() * H.col(1) / norm(K.inv() * H.col(1));
    

    // 计算r3（第三列）：r1和r2的叉积

    Mat r3 = Mat(3, 1, CV_64F);// cv_64f是64位浮点数
    r3.at<double>(0,0) = r1.at<double>(1,0) * r2.at<double>(2,0) - r1.at<double>(2,0) * r2.at<double>(1,0);
    r3.at<double>(1,0) = r1.at<double>(2,0) * r2.at<double>(0,0) - r1.at<double>(0,0) * r2.at<double>(2,0);
    r3.at<double>(2,0) = r1.at<double>(0,0) * r2.at<double>(1,0) - r1.at<double>(1,0) * r2.at<double>(0,0);

    // 组合成完整的旋转矩阵 R = [r1 r2 r3]
    
    r1.copyTo(R.col(0));  // 第一列
    r2.copyTo(R.col(1));  // 第二列
    r3.copyTo(R.col(2));  // 第三列

    // 验证旋转矩阵的性质
    // 1. 检查行列式是否为1
    double det = determinant(R);
    cout << "旋转矩阵的行列式：" << det << " (应该接近1)" << endl;

    // 2. 检查正交性：R * R^T 应该是单位矩阵
    Mat RRt = R * R.t();
    cout << "R * R^T (应该接近单位矩阵):\n" << RRt << endl;

    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0));
    bool singular = sy < 1e-6; // 是否处于万向锁状态
    
    R = (cv::Mat_<double>(3,3) <<
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0
    ); // 

    // 3. 计算欧拉角
    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1), R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    // 4. 输出旋转矩阵
    cout << "计算得到的旋转矩阵 R:\n" << R << endl;
    const double rad2deg = 180.0 / M_PI;
    cout << "欧拉角 (Z-Y-X顺序):\n"
         << "  Yaw(Z)   : " << z * rad2deg << "°\n"
         << "  Pitch(Y) : " << y * rad2deg << "°\n"
         << "  Roll(X)  : " << x * rad2deg << "°\n" 
         << "注：当Pitch接近±90°时存在万向锁现象" << endl;
    
    // 3. 提取x,y坐标（相当于乘以[1 0 0; 0 1 0]矩阵）- 获取2D平面上的位置
    // 使用static_cast明确转换类型，避免警告
    Mat R_inv = R.inv();
    Mat pos_3d = R_inv * T;
    Point2f camera_pos_2d(
        static_cast<float>(pos_3d.at<double>(0)),  // X坐标
        static_cast<float>(pos_3d.at<double>(1))   // Y坐标
    );
    cout << "\n相机在平面坐标系中的二维位置: (" 
    << camera_pos_2d.x << ", " << camera_pos_2d.y << ")" << endl;



   // 5. 添加投影验证
    if (!H.empty()) {
        vector<Point2f> projected_points;
        perspectiveTransform(plane_points, projected_points, H);
        
        cout << "投影误差分析：" << endl;
        vector<double> errors;
        for (size_t i = 0; i < plane_points.size(); ++i) {
            double dx = projected_points[i].x - corners_sorted[i].x;
            double dy = projected_points[i].y - corners_sorted[i].y;
            double error = sqrt(dx*dx + dy*dy);
            errors.push_back(error);
            
            printf("点 %2d | 平面(%4.2f,%4.2f) -> 图像实际(%6.1f,%6.1f) vs 图像投影(%6.1f,%6.1f) | 误差=%-5.2fpx\n",
                   (int)i, plane_points[i].x, plane_points[i].y,                   
                   corners_sorted[i].x, corners_sorted[i].y,
                   projected_points[i].x, projected_points[i].y,
                   error);
        }
        double avg_error = accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        cout << "平均投影误差: " << avg_error << " 像素" << endl;
    }
 

}

