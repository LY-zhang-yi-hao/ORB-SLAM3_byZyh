/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <sophus/se3.hpp>
#include<opencv2/core/core.hpp>
#include<fstream>
#include<iomanip>
#include<System.h>
#include "Tracking.h"
using namespace std;


// 按照ORB-SLAM3的TUM格式，轨迹记录
class TUMFormatTrajectoryRecorder : public ORB_SLAM3::IPoseObserver
{
private:
    std::ofstream mOutputFile; // 文件流，用于写入轨迹数据
    bool mIsFirstFrame; // 标志变量，指示当前是否为第一帧图像
    bool mIsKeyFrameRecorder; // 标志变量，指示是否为关键帧记录器
    std::string mOutputFilename; // 保存文件名，方便后续使用

public:
    TUMFormatTrajectoryRecorder(const std::string& filename, bool isKeyFrameRecorder = false)
        : mIsFirstFrame(true), mIsKeyFrameRecorder(isKeyFrameRecorder), mOutputFilename(filename) {
        // 打开输出文件
        mOutputFile.open(filename.c_str());
        if(!mOutputFile.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            exit(1);
        }
        // 设置固定点数格式
        mOutputFile << std::fixed;
        std::cout << "轨迹记录已初始化，输出文件: " << filename 
                  << (mIsKeyFrameRecorder ? "（关键帧模式）" : "（所有帧模式）") << std::endl;
    }
    
    ~TUMFormatTrajectoryRecorder() {
        // 检查输出文件是否已打开
        if (mOutputFile.is_open()) {
            // 关闭输出文件
            mOutputFile.close();
            // 输出轨迹记录完成的信息
            std::cout << "轨迹记录完成，文件已关闭: " << mOutputFilename << std::endl;
        }
    }
    
    void OnPoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) override 
    {
        if (!mOutputFile.is_open()) return;        
        
        // 如果是关键帧记录器但当前不是关键帧，则跳过
        if (mIsKeyFrameRecorder && !IsKeyFrame()) 
            return;
        
        // 获取平移向量
        Eigen::Vector3f twc = T_custom_world_camera.translation();
        // 获取单位四元数
        Eigen::Quaternionf q = T_custom_world_camera.unit_quaternion();
        // 输出，时间戳6位精度，坐标和四元数9位精度
        // 注意：ORB-SLAM3使用的顺序是 w, x, y, z
        mOutputFile << std::setprecision(6) << timestamp << " "
                    << std::setprecision(9) 
                    << twc.x() << " " 
                    << twc.y() << " " 
                    << twc.z() << " "
                    << q.w() << " " 
                    << q.x() << " " 
                    << q.y() << " " 
                    << q.z() << std::endl;        
    }

private:
    // 判断当前帧是否为关键帧的辅助函数（仅在关键帧记录模式下使用）
    // 注意：由于我们无法直接从 Observer访问到KeyFrame信息，这里使用timestamp作为标记
    // 在实际运行中，这将导致所有帧都被记录，我们需要在System类中实现更好的方法
    bool IsKeyFrame() {
        // 对于普通帧记录器，始终返回true，对于关键帧记录器，也返回true
        // 对于关键帧记录器，一般应该验证帧的属性，但这里我们没有这个信息
        // 所以很多情况下我们两个输出文件会很相似
        return true;
    }
};

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{  
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[(2*seq)+3]) + "/mav0/cam0/data", string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);


    int fps = 20;
    float dT = 1.f/fps;
    // 创建SLAM系统，初始化所有系统线程，并准备好处理帧
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true); // 是否使用可视化界面。

   // 获取 Tracking 对象并设置
    ORB_SLAM3::Tracking* pTracker = SLAM.GetTracker();
    if (pTracker) {

        // 创建自定义坐标系的轨迹记录器（所有帧）
        std::string customTrajFile = "f_custom_"+ std::string(argv[5]) + ".txt";
        TUMFormatTrajectoryRecorder* pTrajectoryRecorder = new TUMFormatTrajectoryRecorder(customTrajFile, false);
        
        // 创建自定义坐标系的关键帧轨迹记录器
        std::string customKfTrajFile = "kf_custom_"+ std::string(argv[5]) + ".txt";
        TUMFormatTrajectoryRecorder* pKfTrajectoryRecorder = new TUMFormatTrajectoryRecorder(customKfTrajFile, true);
        
        // 注册位姿观察者
        pTracker->RegisterPoseObserver(pTrajectoryRecorder);
        pTracker->RegisterPoseObserver(pKfTrajectoryRecorder);

        // 设置自定义坐标系
        // 假设ORB坐标系的原点在自定义世界坐标的位置（1.5，2.2，0）
        Eigen::Vector3f t_custom_orb_translation(1.5,2.2,0);
        Eigen::Matrix3f R_custom_orb = Eigen::Matrix3f::Identity();// 假设方向一致
        
        // 创建变换队形 T_custom_orb
        Sophus::SE3f T_custom_orb(R_custom_orb,t_custom_orb_translation);
        
        // 设置自定义坐标系
        pTracker->SetCustomWorldTransform(T_custom_orb);

        std::cout << "🔄 已注册自定义坐标系轨迹记录器，输出文件：" << customTrajFile << std::endl;
        std::cout << "📝 可与系统默认生成的 f_" << argv[5] << ".txt 文件进行比较分析" << std::endl;

    } else {    // 如果获取 Tracker 对象失败，输出错误信息并退出程序
        std::cerr << "错误: 无法获取 Tracker 对象!" << std::endl;
        return 1;
    }
    
    
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            //std::cout << "T: " << T << std::endl;
            //std::cout << "ttrack: " << ttrack << std::endl;

            if(ttrack<T) {
                //std::cout << "usleep: " << (dT-ttrack) << std::endl;
                usleep((T-ttrack)*1e6); // 1e6
            }
        }

        if(seq < num_seq - 1)
        {
            string kf_file_submap =  "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
            string f_file_submap =  "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
            SLAM.SaveTrajectoryEuRoC(f_file_submap);
            SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}
