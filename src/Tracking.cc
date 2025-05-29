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


#include "Tracking.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include <iostream>

#include <mutex>
#include <chrono>


using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyFrame数据类型

namespace ORB_SLAM3
{

/**
 * @brief 跟踪线程构造函数
 * @param pSys 系统类指针
 * @param pVoc 词典
 * @param pFrameDrawer 画图像的
 * @param pMapDrawer 画地图的
 * @param pAtlas atlas
 * @param pKFDB 关键帧词典数据库
 * @param strSettingPath 参数文件路径
 * @param sensor 传感器类型
 * @param settings 参数类
 * @param _strSeqName 序列名字，没用到
 */
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
    Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq)
    : mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mbReadyToInitializate(false), mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr), mpLastKeyFrame(static_cast<KeyFrame*>(NULL))
{
    // Load camera parameters from settings file
    // Step 1 从配置文件中加载相机参数
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if(!b_parse_cam)
        {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load ORB parameters
        bool b_parse_orb = ParseORBParamFile(fSettings);
        if(!b_parse_orb)
        {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        bool b_parse_imu = true;
        if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO || sensor==System::IMU_RGBD)
        {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if(!b_parse_imu)
            {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    initID = 0; lastID = 0;
    mbInitWith3KFs = false;
    mnNumDataset = 0;

    // 遍历下地图中的相机，然后打印出来了
    vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for(GeometricCamera* pCam : vpCams)
    {
        std::cout << "Camera " << pCam->GetId();
        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            std::cout << " is pinhole" << std::endl;
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            std::cout << " is fisheye" << std::endl;
        }
        else
        {
            std::cout << " is unknown" << std::endl;
        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif

    //! 📝 从配置文件中读取棋盘格参数
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // 🔘 读取是否使用棋盘格初始化的开关
    bool useChessboard = false; // 默认不开启棋盘格初始化
    fSettings["UseChessboardInit"] >> useChessboard; // 读取配置文件中的参数
    mbUseChessboardInit = useChessboard; // 将读取的值赋给成员变量
    mbChessboardInitialized = false; // 初始化为false

    // 如果启用了棋盘格初始化，读取相关参数
    if (mbUseChessboardInit) {
        // 🧩 读取棋盘格的宽度和高度（内角点数量）
        int width = fSettings["ChessboardWidth"];
        int height = fSettings["ChessboardHeight"];
        mChessboardSize = cv::Size(width, height);

        // 📏 读取棋盘格方格的实际大小（米）
        mSquareSize = fSettings["SquareSize"];

        // 🌐 读取棋盘格在世界坐标系中的起始位置和高度
        mStartX = fSettings["StartX"];
        mStartY = fSettings["StartY"];
        mZHeight = fSettings["ZHeight"];

        // 📢 打印棋盘格初始化的配置信息
        std::cout << "Using chessboard for initialization: "
                  << width << "x" << height << " with square size "
                  << mSquareSize << "m" << std::endl;
    }
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::LocalMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void Tracking::TrackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if(!vdResizeImage_ms.empty())
        {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if(!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();
}

void Tracking::PrintTimeStats()
{
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();


    ofstream f;
    f.open("ExecMean.txt");
    f << fixed;
    //Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if(!vdRectStereo_ms.empty())
    {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdResizeImage_ms.empty())
    {
        average = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if(!vdStereoMatch_ms.empty())
    {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdIMUInteg_ms.empty())
    {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBestMap = vpMaps[0];
    for(int i=1; i<vpMaps.size(); ++i)
    {
        if(pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
        {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdDataQuery_ms);
    deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdEstSim3_ms);
    deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdPRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
    average = calcAverage(mpLoopClosing->vnLoopKFs);
    deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeKFs);
    deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeMPs);
    deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdGBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
    deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAKFs);
    deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAMPs);
    deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f.close();

}

#endif

Tracking::~Tracking()
{
    //f_track_stats.close();

}

/**
 * @brief 根据参数类读取参数，可快速略过不看
 * @param settings 参数类
 */
void Tracking::newParameterLoader(Settings *settings) {
    // 1. 读取相机1
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if(settings->needToUndistort()){
        mDistCoef = settings->camera1DistortionCoef();
    }
    else{
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    }

    //TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = mpCamera->getParameter(0);
    mK.at<float>(1,1) = mpCamera->getParameter(1);
    mK.at<float>(0,2) = mpCamera->getParameter(2);
    mK.at<float>(1,2) = mpCamera->getParameter(3);

    mK_.setIdentity();
    mK_(0,0) = mpCamera->getParameter(0);
    mK_(1,1) = mpCamera->getParameter(1);
    mK_(0,2) = mpCamera->getParameter(2);
    mK_(1,2) = mpCamera->getParameter(3);

    // 读取相机2
    if((mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD) &&
        settings->cameraType() == Settings::KannalaBrandt){
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }

    // 读取双目
    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD ){
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    // 读取rgbd
    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD){
        mDepthMapFactor = settings->depthMapFactor();
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    // 2. 读取特征点参数
    int nFeatures = settings->nFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    //IMU parameters
    // 3. 读取imu参数
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001; //1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
}

/**
 * @brief 根据文件读取相机参数，可快速略过不看
 * @param fSettings 配置文件
 */
bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        if(mImageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

        mK_.setIdentity();
        mK_(0,0) = fx;
        mK_(1,1) = fy;
        mK_(0,2) = cx;
        mK_(1,2) = cy;
    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(!b_miss_params)
        {
            if(mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;

            mK_.setIdentity();
            mK_(0,0) = fx;
            mK_(1,1) = fy;
            mK_(0,2) = cx;
            mK_(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if(!node.empty())
            {
                cvTlr = node.mat();
                if(cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                if(mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin = leftLappingBegin * mImageScale;
                    leftLappingEnd = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd = rightLappingEnd * mImageScale;
                }

                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toSophus(cvTlr);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD )
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
            if(mImageScale != 1.f)
            {
                mbf *= mImageScale;
            }
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

/**
 * @brief 根据文件读取特征点参数，可快速略过不看
 * @param fSettings 配置文件
 */
bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

/**
 * @brief 根据文件读取IMU参数，可快速略过不看
 * @param fSettings 配置文件
 */
bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        cvTbc = node.mat();
        if(cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    cout << endl;
    cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
    Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    node = fSettings["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if(!node.empty() && node.isInt())
    {
        mInsertKFsLost = (bool) node.operator int();
    }

    if(!mInsertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;



    float Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        mImuFreq = node.operator int();
        mImuPer = 0.001; //1.0 / (double) mImuFreq;
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.fastInit"];
    mFastInit = false;
    if(!node.empty())
    {
        mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
    }

    if(mFastInit)
        cout << "Fast IMU initialization. Acceleration is not checked \n";

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    cout << endl;
    cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

/**
 * @brief 设置局部建图器
 * @param pLocalMapper 局部地图类
 */
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

/**
 * @brief 设置回环器
 * @param pLoopClosing 回环类
 */
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

/**
 * @brief 设置显示器
 * @param pViewer 显示类
 */
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

// 一步一步进行
void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}

bool Tracking::GetStepByStep()
{
    return bStepByStep;
}


/**
 * @brief 输入左右目图像，可以为RGB、BGR、RGBA、GRAY
 * 1、将图像转为mImGray和imGrayRight并初始化mCurrentFrame
 * 2、进行tracking过程
 * 输出世界坐标系到该帧相机坐标系的变换矩阵
 * @param imRectLeft 左图
 * @param imRectRight 右图
 * @param timestamp 时间戳
 * @param filename 文件名字，貌似调试用的
 */
Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
    //cout << "GrabImageStereo" << endl;

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    mImRight = imRectRight;

    // step 1 ：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        //cout << "Image with 3 channels" << endl;
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGR2GRAY);
        }
    }
    // 这里考虑得十分周全,甚至连四通道的图像都考虑到了
    else if(mImGray.channels()==4)
    {
        //cout << "Image with 4 channels" << endl;
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGRA2GRAY);
        }
    }

    //cout << "Incoming frame creation" << endl;
    // 双目模式，注意跟两个相机模式区分开
    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(
            mImGray,                // 左目图像
            imGrayRight,            // 右目图像
            timestamp,              // 时间戳
            mpORBextractorLeft,     // 左目特征提取器
            mpORBextractorRight,    // 右目特征提取器
            mpORBVocabulary,        // 字典
            mK,                     // 内参矩阵
            mDistCoef,              // 去畸变参数
            mbf,                    // 基线长度
            mThDepth,				// 远点,近点的区分阈值
            mpCamera);				// 相机模型
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib);

    //cout << "Incoming frame ended" << endl;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    //cout << "Tracking start" << endl;
    // Step 2 ：跟踪
    Track();
    //cout << "Tracking end" << endl;

    // 返回位姿
    return mCurrentFrame.GetPose();
}

/**
 * @brief 输入左目RGB或RGBA图像和深度图
 * 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
 * 2、进行tracking过程
 * 输出世界坐标系到该帧相机坐标系的变换矩阵
 * @param imRGB 彩色图
 * @param imD 深度图
 * @param timestamp 时间戳
 * @param filename 文件名字，貌似调试用的
 */
Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    // step 1：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    // Step 2 ：将深度相机的disparity转为Depth , 也就是转换成为真正尺度下的深度
    if((fabs(mDepthMapFactor-1.0f)>1e-5) && imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    // Step 3：构造Frame
    if (mSensor == System::RGBD)
        mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    else if(mSensor == System::IMU_RGBD)
        mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif
    // Step 4：跟踪
    Track();

    // 返回当前帧的位姿
    return mCurrentFrame.GetPose();
}

/**
 * @brief 输入左目RGB或RGBA图像，输出世界坐标系到该帧相机坐标系的变换矩阵
 *
 * @param im 图像
 * @param timestamp 时间戳
 * @param filename 文件名字，貌似调试用的
 *
 * Step 1 ：将彩色图像转为灰度图像
 * Step 2 ：构造Frame
 * Step 3 ：跟踪
 */
Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;
    // Step 1 ：将彩色图像转为灰度图像
    // 若图片是3、4通道的彩色图，还需要转化成单通道灰度图 输出 mImGray是灰度图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    // Step 2 ：构造Frame类
    if (mSensor == System::MONOCULAR) //如果是单目相机
    {
        // 判断该帧是不是初始化
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)  //没有成功初始化的前一个状态就是NO_IMAGES_YET
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);// 初始化时使用初始化的特征提取器

            // 如果已经初始化了，则使用正常的特征提取器
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }

    // 设置当前帧的文件名和数据集ID
    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    //! 🔍 启用棋盘格检测计算位姿，保存位姿信息-----------------------------------------------
    if (mbUseChessboardInit && !mbHasChessboardPosed && (mState==NOT_INITIALIZED || mState==NO_IMAGES_YET))
    {

        // 尝试使用棋盘格初始化（使用灰度图）
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCornersSB(mImGray, chessboardSize, corners);        
        if(found)
        {
            std::cout << "Chessboard detected, computing pose..." << std::endl;
            //计算相机位姿
            cv::Mat Tcw;
            std::vector<cv::Point3f> worldPoints;
            bool poseComputed = ComputePoseFromChessboard(corners, Tcw, worldPoints);
            if (poseComputed)            
            {                
                // 设置当前帧的位姿
                mTcw_Chessboard = Sophus::SE3f(
                    Converter::toMatrix3f(Tcw.rowRange(0, 3).colRange(0, 3)),
                    Converter::toVector3f(Tcw.rowRange(0, 3).col(3))
                );
                mTwc_chessboard = mTcw_Chessboard.inverse();
                mbHasChessboardPosed = true;

                std::cout << "Chessboard pose computed: " << mTcw_Chessboard.matrix() << std::endl;
                std::cout << " will apply coordinate system conversion after initialization" << std::endl;
            }
        }
    }               
    else if(mSensor == System::IMU_MONOCULAR) //如果是单目IMU相机
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR) //如果是单目IMU相机
    {
        // 判断该帧是不是初始化
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)  //没有成功初始化的前一个状态就是NO_IMAGES_YET
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    // t0存储未初始化时的第1帧图像时间戳
    if (mState==NO_IMAGES_YET)
        t0=timestamp; // 记录第一帧的时间戳


#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    // Step 3 ：跟踪
    Track();

    // 返回当前帧的位姿
    return mCurrentFrame.GetPose();
}

/**
 * @brief 将imu数据存放在mlQueueImuData的list链表里
 * @param[in] imuMeasurement
 */
void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

/**
 * @brief 预积分，对于一个帧有两种预积分，一种是相对于上一帧，一种是相对于上一个关键帧
 */
void Tracking::PreintegrateIMU()
{
    // Step 1.拿到两两帧之间待处理的预积分数据，组成一个集合
    // 上一帧不存在,说明两帧之间没有imu数据，不进行预积分
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    // 没有imu数据,不进行预积分
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true)
    {
        // 数据还没有时,会等待一段时间,直到mlQueueImuData中有imu数据.一开始不需要等待
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                // 拿到第一个imu数据作为起始数据
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                // imu起始数据会比当前帧的前一帧时间戳早,如果相差0.001则舍弃这个imu数据
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-mImuPer)
                {
                    mlQueueImuData.pop_front();
                }
                // 同样最后一个的imu数据时间戳也不能理当前帧时间间隔多余0.001
                else if(m->t<mCurrentFrame.mTimeStamp-mImuPer)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    // 得到两帧间的imu数据放入mvImuFromLastFrame中,得到后面预积分的处理数据
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }

    // Step 2.对两帧之间进行中值积分处理
    // m个imu组数据会有m-1个预积分量
    const int n = mvImuFromLastFrame.size()-1;
    if(n==0){
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    // 构造imu预处理器,并初始化标定数据
    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);
    // 针对预积分位置的不同做不同中值积分的处理
    /**
     *  根据上面imu帧的筛选，IMU与图像帧的时序如下：
     *  Frame---IMU0---IMU1---IMU2---IMU3---IMU4---------------IMUx---Frame---IMUx+1
     *  T_------T0-----T1-----T2-----T3-----T4-----------------Tx-----_T------Tx+1
     *  A_------A0-----A1-----A2-----A3-----A4-----------------Ax-----_T------Ax+1
     *  W_------W0-----W1-----W2-----W3-----W4-----------------Wx-----_T------Wx+1
     *  T_和_T分别表示上一图像帧和当前图像帧的时间戳，A(加速度数据)，W(陀螺仪数据)，同理
     */
    for(int i=0; i<n; i++)
    {
        float tstep;
        Eigen::Vector3f acc, angVel;
        // 第一帧数据但不是最后两帧,imu总帧数大于2
        if((i==0) && (i<(n-1)))
        {
            // 获取相邻两段imu的时间间隔
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            // 获取当前imu到上一帧的时间间隔
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            // 设当前时刻imu的加速度a0，下一时刻加速度a1，时间间隔tab 为t10，tini t0p
            // 正常情况下时为了求上一帧到当前时刻imu的一个平均加速度，但是imu时间不会正好落在上一帧的时刻，需要做补偿，要求得a0时刻到上一帧这段时间加速度的改变量
            // 有了这个改变量将其加到a0上之后就可以表示上一帧时的加速度了。其中a0 - (a1-a0)*(tini/tab) 为上一帧时刻的加速度再加上a1 之后除以2就为这段时间的加速度平均值
            // 其中tstep表示a1到上一帧的时间间隔，a0 - (a1-a0)*(tini/tab)这个式子中tini可以是正也可以是负表示时间上的先后，(a1-a0)也是一样，多种情况下这个式子依然成立
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            // 计算过程类似加速度
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            // 中间的数据不存在帧的干扰，正常计算
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        // 直到倒数第二个imu时刻时，计算过程跟第一时刻类似，都需要考虑帧与imu时刻的关系
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
         // 就两个数据时使用第一个时刻的，这种情况应该没有吧，，回头应该试试看
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        // Step 3.依次进行预积分计算
        // 应该是必存在的吧，一个是相对上一关键帧，一个是相对上一帧
        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }

    // 记录当前预积分的图像帧
    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}

/**
 * @brief 跟踪不成功的时候，用初始化好的imu数据做跟踪处理，通过IMU预测状态
 * 两个地方用到：
 * 1. 匀速模型计算速度,但并没有给当前帧位姿赋值；
 * 2. 跟踪丢失时不直接判定丢失，通过这个函数预测当前帧位姿看看能不能拽回来，代替纯视觉中的重定位
 *
 * @return true
 * @return false
 */
bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    // 总结下都在什么时候地图更新，也就是mbMapUpdated为true
    // 1. 回环或融合
    // 2. 局部地图LocalBundleAdjustment
    // 3. IMU三阶段的初始化
    // 下面的代码流程一模一样，只不过计算时相对的帧不同，地图有更新后相对于上一关键帧做的，反之相对于上一帧
    // 地图更新后会更新关键帧与MP，所以相对于关键帧更准
    // 而没更新的话，距离上一帧更近，计算起来误差更小
    // 地图更新时，并且上一个图像关键帧存在
    if(mbMapUpdated && mpLastKeyFrame)
    {
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        // 计算当前帧在世界坐标系的位姿,原理都是用预积分的位姿（预积分的值不会变化）与上一帧的位姿（会迭代变化）进行更新
        // 旋转 R_wb2 = R_wb1 * R_b1b2
        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        // 位移
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        // 速度
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        // 设置当前帧的世界坐标系的相机位姿
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        // 记录bias
        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    // 地图未更新时
    else if(!mbMapUpdated)
    {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        // mpImuPreintegratedFrame是当前帧上一帧，不一定是关键帧
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}

//! 检测图像中的棋盘格角点 ------------------------------------------------------------

/**
 *  @brief 检测图像中的棋盘格角点
 *  @param mImGray 输入灰度图像
 *  @param corners 输出的角点坐标
 *  @return bool 返回是否成功检测到角点
 */

bool Tracking::DetectChessboard(const cv::Mat &mImGray, std::vector<cv::Point2f> &corners, cv::Size chessboardSize)
{
    // 清空之前的角点
    corners.clear();

    // 输出调试信息
    std::cout << "🔍 正在检测棋盘格..." << std::endl;
    std::cout << "   图像尺寸: " << mImGray.cols << "x" << mImGray.rows << std::endl;
    std::cout << "   棋盘格尺寸: " << chessboardSize.width << "x" << chessboardSize.height << std::endl;
    std::cout << "   期望角点数: " << (chessboardSize.width * chessboardSize.height) << std::endl;

    // 使用更高级的棋盘格检测函数
    bool found = cv::findChessboardCornersSB(mImGray, chessboardSize, corners);

    // 输出检测结果
    std::cout << "   检测结果: " << (found ? "✅ 成功" : "❌ 失败") << std::endl;
    std::cout << "   实际检测到的角点数: " << corners.size() << std::endl;

    if (found)
    {
        // 打印检测到的角点信息
        std::cout << "🎯 棋盘格角点检测成功！检测到 " << corners.size() << " 个角点" << std::endl;
        // 对检测到的角点进行排序，确保与物理坐标系一致
        // 排序规则：先按y坐标从大到小（图像中y轴向下），再按x坐标从小到大
        std::sort(corners.begin(), corners.end(), [](const cv::Point2f& a, const cv::Point2f& b)
        {
            const float eps = 5.0f; // 定义一个小的误差范围
            if (std::abs(a.y - b.y) < eps) {
                return a.x < b.x;  // 如果y坐标接近，按x从小到大排序
            }
            return a.y > b.y;  // 按y从大到小排序（图像坐标系中y轴向下）
        });
        // 可视化：
        cv::Mat imWithCorners = mImGray.clone();
        // 在图像上绘制角点
        for (const auto& corner : corners) {
            cv::circle(imWithCorners, corner, 5, cv::Scalar(0, 255, 0), -1); // 绘制绿色圆点
        }
        // 显示图像
        cv::imshow("Chessboard Corners", imWithCorners);
        // cv::waitKey(0); // 等待按键
        // cv::destroyAllWindows(); // 关闭窗口

        // 打印排序后的角点坐标
        std::cout << "Sorted chessboard corners:" << std::endl;
        for (size_t i = 0; i < corners.size(); ++i) {
            std::cout << "[" << i << "] (" << corners[i].x
                      << ", " << corners[i].y << ")" << std::endl;
        }
    }
    else
    {
        std::cout << "❌ 棋盘格角点检测失败" << std::endl;
        std::cout << "   可能的原因:" << std::endl;
        std::cout << "   1. 图像中没有棋盘格" << std::endl;
        std::cout << "   2. 棋盘格尺寸配置错误（当前配置: " << chessboardSize.width << "x" << chessboardSize.height << "）" << std::endl;
        std::cout << "   3. 图像质量不佳（模糊、光照不均等）" << std::endl;
        std::cout << "   4. 棋盘格角度过大或部分被遮挡" << std::endl;
    }
    return found;
}


//! 位姿计算 ------------------------------------------------------------

/**
 * @brief 根据检测到的棋盘格角点计算相机位姿
 * @param corners 棋盘格角点坐标
 * @param Tcw 输出的相机位姿变换矩阵（世界坐标系到相机坐标系的变换）
 * @return bool 返回是否成功计算位姿
 */
bool Tracking::ComputePoseFromChessboard(const std::vector<cv::Point2f> &corners, cv::Mat &Tcw,std::vector<cv::Point3f> &worldPoints)
{
    // 检查角点数量是否正确
    if (corners.size() != static_cast<size_t>(mChessboardSize.width * mChessboardSize.height))
    {
        std::cerr << "Error: Incorrect number of corners detected." << std::endl;
        return false;
    }

    // 创建棋盘格的平面坐标系的点
    std::vector<cv::Point2f> planePoints;
    for (int i = 0; i < mChessboardSize.height; ++i)
    {
        for (int j = 0; j < mChessboardSize.width; ++j)
        {
            // 使用配置文件中的参数创建2D点
            planePoints.push_back(cv::Point2f(
                mSquareSize + j * mSquareSize,  // X坐标
                mSquareSize + i * mSquareSize   // Y坐标
            ));
        }
    }
    // 打印平面坐标系中的点（调试用）
    std::cout << "Plane points:" << std::endl;
    for (size_t i = 0; i < planePoints.size(); ++i) {
        std::cout << "[" << i << "] (" << planePoints[i].x << ", "
                  << planePoints[i].y << ")" << std::endl;
    }

    // 创建棋盘格的世界坐标系点
    worldPoints.clear(); // 确保向量为空
    for (int i = 0; i < mChessboardSize.height; ++i)
    {
        for (int j = 0; j < mChessboardSize.width; ++j)
        {
            // 使用配置文件中的参数创建3D点
            // 注意：这里的坐标系与observation.cpp中的一致
            worldPoints.push_back(cv::Point3f(
                mStartX + j * mSquareSize,  // X坐标
                mStartY + i * mSquareSize,  // Y坐标
                mZHeight                    // Z坐标
            ));
        }
    }

    // 打印世界坐标系中的点（调试用）
    std::cout << "World points:" << std::endl;
    for (size_t i = 0; i < worldPoints.size(); ++i) {
        std::cout << "[" << i << "] (" << worldPoints[i].x << ", "
                  << worldPoints[i].y << ", " << worldPoints[i].z << ")" << std::endl;
    }

    //todo 计算单应性矩阵H(从平面坐标系到图像坐标系)   Tcp = [R|t]
    cv::Mat H = cv::findHomography(planePoints, corners, cv::RANSAC);
    if (H.empty())
    {
        std::cerr << "Error: Homography matrix could not be computed." << std::endl;
        return false;
    }
    // 打印单应性矩阵H（调试用）
    std::cout << "Homography matrix H:" << std::endl;
    std::cout << H << std::endl;

    //todo  从单应性矩阵H中提取旋转和平移
    // 获取相机内参矩阵K，确保数据类型为CV_64F
    cv::Mat K;
    mK.convertTo(K, CV_64F);

    // 确保单应性矩阵H也是CV_64F类型
    cv::Mat H_double;
    H.convertTo(H_double, CV_64F);

    // 计算平移向量t
    // 根据公式：T = (K^{-1} * H_3) / ||K^{-1} * H_1||
    cv::Mat K_inv = K.inv();
    cv::Mat T = (K_inv * H_double.col(2)) / cv::norm(K_inv * H_double.col(0), cv::NORM_L2);
    std::cout << "Translation vector T:\n" << T << std::endl;

    // 计算旋转矩阵R
    // 根据公式：R = [r1 r2 r3]
    // 其中：
    // r1 = K^{-1}H_1 / ||K^{-1}H_1||  （第一列）
    // r2 = K^{-1}H_2 / ||K^{-1}H_2||  （第二列）
    // r3 = r1 × r2                    （第三列，叉积）

    // 计算r1（第一列）
    cv::Mat r1 = K_inv * H_double.col(0) / cv::norm(K_inv * H_double.col(0));

    // 计算r2（第二列）
    cv::Mat r2 = K_inv * H_double.col(1) / cv::norm(K_inv * H_double.col(1));

    // 计算r3（第三列）：r1和r2的叉积
    cv::Mat r3 = cv::Mat(3, 1, CV_64F);
    r3.at<double>(0,0) = r1.at<double>(1,0) * r2.at<double>(2,0) - r1.at<double>(2,0) * r2.at<double>(1,0);
    r3.at<double>(1,0) = r1.at<double>(2,0) * r2.at<double>(0,0) - r1.at<double>(0,0) * r2.at<double>(2,0);
    r3.at<double>(2,0) = r1.at<double>(0,0) * r2.at<double>(1,0) - r1.at<double>(1,0) * r2.at<double>(0,0);

    // 组合成完整的旋转矩阵 R = [r1 r2 r3]
    cv::Mat R(3, 3, CV_64F);
    r1.copyTo(R.col(0));  // 第一列
    r2.copyTo(R.col(1));  // 第二列
    r3.copyTo(R.col(2));  // 第三列

    // 确保旋转矩阵是正交的（通过SVD分解）
    cv::Mat U, S, Vt;
    cv::SVD::compute(R, S, U, Vt);
    R = U * Vt;  // 重新构造正交旋转矩阵

    std::cout << "Orthogonalized rotation matrix R:\n" << R << std::endl;

    //todo 从平面坐标系到相机坐标系的变换 Tcp = [R|t]
    cv::Mat Tcp = cv::Mat::eye(4, 4, CV_64F); // 创建一个4x4的单位矩阵
    R.copyTo(Tcp(cv::Rect(0, 0, 3, 3))); // 将旋转矩阵R放入左上角
    T.copyTo(Tcp(cv::Rect(3, 0, 1, 3))); // 将平移向量T放入最后一列

    // 计算欧拉角
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
    bool singular = sy < 1e-6; // 是否处于万向锁状态

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

    const double rad2deg = 180.0 / M_PI;
    std::cout << "Euler angles (Z-Y-X):\n"
              << "  Yaw(Z)   : " << z * rad2deg << "°\n"
              << "  Pitch(Y) : " << y * rad2deg << "°\n"
              << "  Roll(X)  : " << x * rad2deg << "°\n"
              << std::endl;

    //todo 平面坐标系到世界坐标系的变换矩阵 Twp
    cv::Mat Rwp = (cv::Mat_<double>(3, 3) <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    );

    cv::Mat twp = (cv::Mat_<double>(3,1) << mStartX, mStartY, mZHeight);  // 修改变量名避免重复定义

    std::cout << "Rwp: " << Rwp << std::endl;
    std::cout << "twp: " << twp << std::endl;

    cv::Mat Twp = cv::Mat::eye(4, 4, CV_64F);
    Rwp.copyTo(Twp(cv::Rect(0, 0, 3, 3)));
    twp.copyTo(Twp(cv::Rect(3, 0, 1, 3)));

    std::cout << "Twp: " << Twp << std::endl;

    //todo 计算相机在自定义世界坐标系中的位置
    // 计算 R 的逆矩阵
    cv::Mat R_inv = R.t();

    // 计算 -Rwp * R^{-1} * T + twp
    cv::Mat camera_pos_3d = -Rwp * (R_inv * T) + twp;  // 修改变量名

    // 正交投影获取二维坐标 (取前两个分量)
    cv::Point2f camera_pos_2d(camera_pos_3d.at<double>(0),
                             camera_pos_3d.at<double>(1));

    // 输出最终结果
    std::cout << "\n camera position in custom world coordinate system: ("
              << camera_pos_2d.x << ", " << camera_pos_2d.y << ")" << std::endl;

    //todo 将世界坐标系到相机坐标系的变换 Tcw = Tcp * Twp^(-1)
    cv::Mat Twp_inv;
    // 确保数据类型一致
    cv::Mat Twp_double;
    Twp.convertTo(Twp_double, CV_64F);
    cv::invert(Twp_double, Twp_inv);
    cv::Mat Tcw_double = Tcp * Twp_inv;
    // 转换为float类型（ORB-SLAM3使用float类型）
    Tcw_double.convertTo(Tcw, CV_32F);

    std::cout << "世界坐标系到相机坐标系的变换 Tcw:\n" << Tcw << std::endl;

    return true;  // 添加返回值
}

//! 应用棋盘格坐标系转换到所有地图元素
/**
 * @brief 应用棋盘格坐标系转换到所有地图元素
 * @param Tcw_chess 棋盘格位姿
 * @param Twc_chess 相机坐标系到棋盘格世界坐标系的变换（逆变换）
 */
void TransformAllMapElements(const Sophus::SE3f &Tcw_chess, const Sophus::SE3f &Twc_chess)
{
    //todo 1. 获取当前地图 ，关键帧 ，地图点
    Map* pMap = mpAtlas->GetCurrentMap();
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    //todo 2. 计算从原始世界坐标系到棋盘格坐标系的变换
    //假设第一个关键帧定义了世界坐标系
    if (vpKFs.empty())
    {
        std::cerr << "Error:❌ No keyframes in the map." << std::endl;
        return;
    }

    // 获取第一个关键帧的位姿
    KeyFrame* pFirstKF = vpKFs[0];
    Sophus::SE3f Tcw_first = pFirstKF->GetPose();
    Sophus::SE3f Twc_first = pFirstKF->GetPoseInverse();

    // 计算从原始世界坐标系到棋盘格坐标系的变换
    // Tcb = Tcw_chess * Twc_first 表示从原始世界坐标系到棋盘格坐标系的变换
    Sophus::SE3f Tcb = Tcw_chess * Twc_first;
    Sophus::SE3f Twc_chess = Tcb.inverse();
    std::cout << "🔄 原始世界坐标系到棋盘格坐标系的变换矩阵：\n" << Tcb.matrix() << std::endl;

    //todo 3. 应用变换到所有地图元素
    
    //todo 3.1 更新关键帧的位姿
    for (KeyFrame* pKF : vpKFs)
    {
        // 获取关键帧的位姿
        Sophus::SE3f Tcw_orig = pKF->GetPose();
        // 计算关键帧在棋盘格坐标系的位姿
        // Tcw_new = Tcw_orig * Tbc
        Sophus::SE3f Tcw_new = Tcw_orig * Tbc;
        // 更新关键帧的位姿
        pKF->SetPose(Tcw_new);
    }
    
    //todo 3.2 更新地图点
    for (MapPoint* pMP : vpMPs)
    {
        if(!pMP || pMP->isBad()) // 如果地图点不存在或已失效
            continue;

        // 获取地图点的在原始世界坐标系下的位置
        Eigen::Vector3f Xw = pMP->GetWorldPos();

        // 计算地图点在棋盘格坐标系下的坐标
        // Pw_new = Tcb.rotationMatrix() * Pw_orig + Tcb.translation()
        Eigen::Vector3f Pw_new = Tcb.rotationMatrix() * Pw_orig + Tcb.translation();
        
        // 更新地图点位置
        pMP->SetWorldPos(Pw_new);
        
        // 更新地图点的法线方向和深度范围
        pMP->UpdateNormalAndDepth();
    }
    

}







//! 创建初始化地图------------------------------------------------------------
/**
 * @brief 创建初始地图，使用棋盘格角点作为特征点
 * @param worldPoints 世界坐标系中的3D点
 * @param imagePoints 图像坐标系中的2D点
 */
void Tracking::CreateInitialMapWithChessboard (const std::vector<cv::Point3f> &worldPoints , const std::vector<cv::Point2f> &imagePoints)
{
    // todo0 检查参数和状态
    if(worldPoints.size() != imagePoints.size())
    {
        std::cerr << "Error: 世界坐标系中的点数与图像坐标系中的点数不匹配。" << std::endl;
        return;
    }
    // 🔧 确保当前帧有足够的特征点和描述子
    std::cout << "📊 当前帧信息检查:" << std::endl;
    std::cout << "   当前帧特征点数量 N: " << mCurrentFrame.N << std::endl;
    std::cout << "   需要创建的地图点数量: " << worldPoints.size() << std::endl;
    std::cout << "   mvpMapPoints 当前大小: " << mCurrentFrame.mvpMapPoints.size() << std::endl;
    std::cout << "   mvKeys 大小: " << mCurrentFrame.mvKeys.size() << std::endl;
    std::cout << "   mDescriptors 行数: " << mCurrentFrame.mDescriptors.rows << std::endl;

    // 确保当前帧有足够的特征点
    if (mCurrentFrame.N < worldPoints.size())
    {
        std::cout << "⚠️ 当前帧特征点数量不足，需要调整..." << std::endl;

        // 调整特征点相关的容器大小
        mCurrentFrame.mvpMapPoints.resize(worldPoints.size(), nullptr);
        mCurrentFrame.mvKeys.resize(worldPoints.size());
        mCurrentFrame.mvKeysUn.resize(worldPoints.size());
        mCurrentFrame.mvuRight.resize(worldPoints.size(), -1.0f);  // 🔧 单目相机，右图坐标设为-1
        mCurrentFrame.mvDepth.resize(worldPoints.size(), -1.0f);   // 🔧 单目相机，深度设为-1
        mCurrentFrame.mvbOutlier.resize(worldPoints.size(), false); // 🔧 初始化外点标志

        // 为新增的特征点设置图像坐标（使用棋盘格角点）
        for (size_t i = mCurrentFrame.N; i < worldPoints.size(); i++)
        {
            if (i < imagePoints.size())
            {
                cv::KeyPoint kp;
                kp.pt = imagePoints[i];
                kp.octave = 0;
                kp.angle = 0;
                kp.response = 1.0;
                mCurrentFrame.mvKeys[i] = kp;
                mCurrentFrame.mvKeysUn[i] = kp;  // 假设已经去畸变
            }
        }

        // 🔧 创建虚拟描述子（因为棋盘格角点没有真实的ORB描述子）
        if (mCurrentFrame.mDescriptors.rows < worldPoints.size())
        {
            std::cout << "🔧 创建虚拟描述子..." << std::endl;
            // 创建一个足够大的描述子矩阵（32字节 = 256位）
            cv::Mat descriptors = cv::Mat::zeros(worldPoints.size(), 32, CV_8UC1);

            // 为每个角点创建一个简单的虚拟描述子
            for (size_t i = 0; i < worldPoints.size(); i++)
            {
                // 使用角点坐标生成简单的描述子
                cv::Point2f pt = imagePoints[i];
                uchar* desc = descriptors.ptr<uchar>(i);

                // 简单的描述子生成：基于坐标的哈希
                uint32_t hash = static_cast<uint32_t>(pt.x * 1000 + pt.y);
                for (int j = 0; j < 8; j++) {
                    desc[j] = (hash >> (j * 4)) & 0xFF;
                }
                // 其余字节保持为0
            }

            mCurrentFrame.mDescriptors = descriptors.clone();
            std::cout << "✅ 虚拟描述子创建完成，大小: " << mCurrentFrame.mDescriptors.rows << "x" << mCurrentFrame.mDescriptors.cols << std::endl;
        }

        // 更新特征点数量
        mCurrentFrame.N = worldPoints.size();

        std::cout << "✅ 特征点数量已调整为: " << mCurrentFrame.N << std::endl;
    }

    // todo1 创建关键帧：使用当前帧构建关键帧
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    // todo2 将关键帧添加到地图中
    mpAtlas->AddKeyFrame(pKFini);

    // todo3 创建地图点
    std::cout << "🔄 开始创建地图点，总数: " << worldPoints.size() << std::endl;

    for (size_t i = 0; i < worldPoints.size(); i++)
    {
        try
        {
            std::cout << "📍 正在创建第 " << (i+1) << "/" << worldPoints.size() << " 个地图点..." << std::endl;

            // 将世界坐标系中的3D点转换为Eigen::Vector3f格式
            Eigen::Vector3f worldPos;
            worldPos << worldPoints[i].x, worldPoints[i].y, worldPoints[i].z;
            std::cout << "   世界坐标: (" << worldPos.x() << ", " << worldPos.y() << ", " << worldPos.z() << ")" << std::endl;

            // 为每个3D点创建一个MapPoint对象
            std::cout << "   创建MapPoint对象..." << std::endl;
            MapPoint* pMP = new MapPoint(worldPos, pKFini, mpAtlas->GetCurrentMap());
            std::cout << "   ✅ MapPoint对象创建成功" << std::endl;

            // 为MapPoint添加观测、计算描述子、更新法向量和深度范围
            std::cout << "   添加MapPoint到关键帧..." << std::endl;
            pKFini->AddMapPoint(pMP, i);
            std::cout << "   添加观测..." << std::endl;
            pMP->AddObservation(pKFini, i);

            std::cout << "   计算描述子..." << std::endl;
            pMP->ComputeDistinctiveDescriptors();

            std::cout << "   更新法向量和深度..." << std::endl;
            // 🔧 暂时跳过UpdateNormalAndDepth，因为它需要更复杂的观测信息
            // pMP->UpdateNormalAndDepth();
            std::cout << "   ⚠️ 跳过法向量和深度更新（棋盘格初始化模式）" << std::endl;

            // 将MapPoint添加到地图中
            std::cout << "   添加到地图..." << std::endl;
            mpAtlas->AddMapPoint(pMP);

            // 安全地将MapPoint添加到当前帧中
            if (i < mCurrentFrame.mvpMapPoints.size()) {
                mCurrentFrame.mvpMapPoints[i] = pMP;
                std::cout << "   ✅ 第 " << (i+1) << " 个地图点创建完成" << std::endl;
            } else {
                std::cerr << "❌ 错误：索引 " << i << " 超出 mvpMapPoints 范围 "
                          << mCurrentFrame.mvpMapPoints.size() << std::endl;
            }
        }
        catch(const std::exception& e)
        {
                std::cerr << "❌ 创建第 " << (i+1) << " 个地图点时发生异常: " << e.what() << std::endl;
                break; // 遇到异常就退出循环
        }
    }

    // todo4 更新局部地图
    std::cout << "新地图创建完成，包含 " << mpAtlas->MapPointsInMap() << " 个棋盘格地图点" << std::endl;

    // 🚀 创建额外的ORB特征地图点以提高跟踪稳定性
    std::cout << "🔄 创建额外的ORB特征地图点..." << std::endl;
    int additionalPoints = 0;

    // 遍历当前帧的所有特征点，为没有对应地图点的特征点创建地图点
    for(int i = 0; i < mCurrentFrame.N && additionalPoints < 100; i++) {
        if(mCurrentFrame.mvpMapPoints[i] == nullptr) {
            // 为这个特征点创建一个临时地图点（深度设为平均深度）
            cv::KeyPoint kp = mCurrentFrame.mvKeys[i];

            // 使用平均深度（假设在棋盘格平面附近）
            float depth = 3.0f; // 大约3米深度

            // 将像素坐标转换为相机坐标系下的3D点
            float x = (kp.pt.x - mCurrentFrame.cx) * depth / mCurrentFrame.fx;
            float y = (kp.pt.y - mCurrentFrame.cy) * depth / mCurrentFrame.fy;
            float z = depth;

            // 转换到世界坐标系
            Eigen::Vector3f x3Dc(x, y, z);
            Sophus::SE3f Twc = pKFini->GetPoseInverse();
            Eigen::Vector3f x3Dw = Twc * x3Dc;

            // 创建地图点
            MapPoint* pNewMP = new MapPoint(x3Dw, pKFini, mpAtlas->GetCurrentMap());

            // 添加观测
            pNewMP->AddObservation(pKFini, i);
            pKFini->AddMapPoint(pNewMP, i);

            // 计算描述子
            pNewMP->ComputeDistinctiveDescriptors();

            // 添加到地图
            mpAtlas->AddMapPoint(pNewMP);
            mCurrentFrame.mvpMapPoints[i] = pNewMP;

            additionalPoints++;
        }
    }

    std::cout << "✅ 创建了 " << additionalPoints << " 个额外的ORB特征地图点" << std::endl;
    std::cout << "📊 总地图点数: " << mpAtlas->MapPointsInMap() << " 个" << std::endl;

    // 更新当前帧的参考关键帧
    mCurrentFrame.mpReferenceKF = pKFini;

    // 将当前帧的位姿设置为关键帧的位姿
    mCurrentFrame.SetPose(pKFini->GetPose());

    // 更新上一帧
    mLastFrame = Frame(mCurrentFrame);

    // 🔧 安全地更新局部关键帧和局部地图点
    try {
        std::cout << "🔄 更新LocalMapper..." << std::endl;
        if (mpLocalMapper) {
            mpLocalMapper->InsertKeyFrame(pKFini);
            std::cout << "✅ LocalMapper更新成功" << std::endl;
        }

        std::cout << "🔄 更新参考关键帧和地图点..." << std::endl;
        // 更新参考关键帧、参考地图点等
        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.push_back(pKFini);

        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;

        // 设置参考地图点
        if (!mvpLocalMapPoints.empty()) {
            mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
        }

        std::cout << "🔄 更新地图原点..." << std::endl;
        // 添加初始关键帧到地图原点
        if (mpAtlas->GetCurrentMap()) {
            mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);
        }

        std::cout << "🔄 更新地图绘制器..." << std::endl;
        // 更新地图绘制器
        if (mpMapDrawer) {
            mpMapDrawer->SetCurrentCameraPose(pKFini->GetPose());
        }

        std::cout << "🔄 更新系统状态..." << std::endl;
        // todo5 更新系统状态
        mState = OK;
        std::cout << "✅ 棋盘格初始化完全成功！系统状态已设置为OK" << std::endl;
    }
    catch(const std::exception& e) {
        std::cerr << "❌ 初始化后续处理发生异常: " << e.what() << std::endl;
        mState = NOT_INITIALIZED;
    }
}

//! 棋盘格初始化主函数
/**
 * @brief 使用棋盘格初始化SLAM系统
 * @param mImGray 输入的灰度图像
 * @return bool 返回是否成功初始化
 */
bool Tracking::InitializeWithChessboard(const cv::Mat &mImGray)
{
    // 步骤1：检测棋盘格角点
    std::vector<cv::Point2f> corners;
    bool found = DetectChessboard(mImGray, corners, mChessboardSize);

    if (!found)
    {
        std::cout << "未检测到棋盘格，初始化失败" << std::endl;
        return false;
    }

    // 步骤2：计算相机位姿
    cv::Mat Tcw;
    std::vector<cv::Point3f> worldPoints; // 用于存储世界坐标系中的3D点
    bool poseComputed = ComputePoseFromChessboard(corners, Tcw, worldPoints);

    if (!poseComputed)
    {
        std::cout << "相机位姿计算失败，初始化失败" << std::endl;
        return false;
    }

    // 步骤3：设置当前帧的位姿
    mCurrentFrame.SetPose(Sophus::SE3f(
        Converter::toMatrix3f(Tcw.rowRange(0, 3).colRange(0, 3)),
        Converter::toVector3f(Tcw.rowRange(0, 3).col(3))
    ));

    // 🎯 计算并显示相机在自定义世界坐标系中的位置（这将是轨迹的起始点）
    cv::Mat Twc = Tcw.inv();
    cv::Mat camera_position = Twc.rowRange(0,3).col(3);

    std::cout << "🎯 轨迹起始位置（相机在自定义世界坐标系中的位置）: ("
              << camera_position.at<float>(0) << ", "
              << camera_position.at<float>(1) << ", "
              << camera_position.at<float>(2) << ")" << std::endl;

    // 步骤4：创建混合初始地图（棋盘格角点 + ORB特征点）
    CreateInitialMapWithChessboard(worldPoints, corners);

    // 🔧 设置运动模型，避免跟踪失败
    mbVelocity = false; // 初始化时没有速度模型

    // 🎯 关键：禁用棋盘格初始化，防止重复初始化
    mbUseChessboardInit = false;
    mbChessboardInitialized = true;  // 标记已完成棋盘格初始化
    std::cout << "🔒 棋盘格初始化完成，已禁用重复初始化" << std::endl;

    // 步骤5：输出初始化信息
    std::cout << "✅ 使用棋盘格成功初始化SLAM系统" << std::endl;
    std::cout << "   棋盘格大小: " << mChessboardSize.width << "x" << mChessboardSize.height << std::endl;
    std::cout << "   方格尺寸: " << mSquareSize << "米" << std::endl;
    std::cout << "   自定义坐标系原点: (" << mStartX << ", " << mStartY << ", " << mZHeight << ")" << std::endl;
    std::cout << "🚀 系统将继续使用标准ORB-SLAM3跟踪模式" << std::endl;

    return true;
}




/**
 * @brief 跟踪过程，包括恒速模型跟踪、参考关键帧跟踪、局部地图跟踪
 * track包含两部分：估计运动、跟踪局部地图
 *
 * Step 1：初始化
 * Step 2：跟踪
 * Step 3：记录位姿信息，用于轨迹复现
 */
void Tracking::Track()
{

    if (bStepByStep)
    {
        std::cout << "Tracking: Waiting to the next step" << std::endl;
        while(!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    // Step 1 如局部建图里认为IMU有问题，重置当前活跃地图
    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    // 从Atlas中取出当前active的地图
    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if(!pCurrentMap)
    {
        cout << "ERROR: There is not an active map in the atlas" << endl;
    }

    // Step 2 处理时间戳异常的情况
    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            // 如果当前图像时间戳比前一帧图像时间戳小，说明出错了，清除imu数据，创建新的子地图
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            // mlQueueImuData.clear();
            // 创建新地图
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
            // cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            // 如果当前图像时间戳和前一帧图像时间戳大于1s，说明时间戳明显跳变了，重置地图后直接返回
            //根据是否是imu模式,进行imu的补偿
            if(mpAtlas->isInertial())
            {
                // 如果当前地图imu成功初始化
                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    // IMU完成第3次初始化（在localmapping线程里）
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        // 如果当前子图中imu没有经过BA2，重置active地图，也就是之前的数据不要了
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        // 如果当前子图中imu进行了BA2，重新创建新的子图，保存当前地图
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    // 如果当前子图中imu还没有初始化，重置active地图
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }

        }
    }

    // Step 3 IMU模式下设置IMU的Bias参数,还要保证上一帧存在
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());  // 使用上一帧的bias作为当前帧的初值

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
    // Step 4 IMU模式且没有创建地图的情况下对IMU数据进行预积分
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
        // IMU数据进行预积分
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif

    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    // 地图更新时加锁。保证地图不会发生变化
    // 疑问:这样子会不会影响地图的实时更新?
    // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    // 判断地图id是否更新了
    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // 检测到地图更新了
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    // Step 5 初始化
    if(mState==NOT_INITIALIZED) // 如果没有初始化，开始初始化
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
        {
            // 双目RGBD相机的初始化共用一个函数
            StereoInitialization();
        }
        else
        {
            // 单目初始化
            MonocularInitialization();
        }

        //mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            // 如果没有成功初始化，直接返回
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            // 如果当前地图是第一个地图，记录当前帧id为第一帧
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        // Step 6 系统成功初始化，下面是具体跟踪过程
        bool bOK; // 跟踪是否成功

#ifdef REGISTER_TIMES
        // 用于记录初始姿态预测的开始时间，仅在REGISTER_TIMES宏被定义时有效，便于性能监控。
        std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // mbOnlyTracking等于false表示正常SLAM模式（定位+地图更新），mbOnlyTracking等于true表示仅定位模式
        // tracking 类构造时默认为false。在viewer中有个开关ActivateLocalizationMode，可以控制是否开启mbOnlyTracking
        if(!mbOnlyTracking)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            // 跟踪进入正常SLAM模式，有地图更新

            // 如果正常跟踪
            if(mState==OK)
            {

                // Local Mapping might have changed some MapPoints tracked in last frame
                // Step 6.1 检查并更新上一帧被替换的MapPoints
                // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                CheckReplacedInLastFrame();

                // Step 6.2 运动模型是空的并且imu未初始化或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                // 第一个条件,如果运动模型为空并且imu未初始化,说明是刚开始第一帧跟踪，或者已经跟丢了。
                // 第二个条件,如果当前帧紧紧地跟着在重定位的帧的后面，我们用重定位帧来恢复位姿
                // mnLastRelocFrameId 上一次重定位的那一帧
                if((!mbVelocity && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    // 用恒速模型跟踪。所谓的恒速就是假设上上帧到上一帧的位姿=上一帧的位姿到当前帧位姿
                    // 根据恒速模型设定当前帧的初始位姿，用最近的普通帧来跟踪当前的普通帧
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点，优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();  // 根据恒速模型失败了，只能根据参考关键帧来跟踪
                }

                // 新增了一个状态RECENTLY_LOST，主要是结合IMU看看能不能拽回来
                // Step 6.3 如果经过跟踪参考关键帧、恒速模型跟踪都失败的话，并满足一定条件就要标记为RECENTLY_LOST或LOST
                if (!bOK)
                {
                    // 条件1：如果当前帧距离上次重定位成功不到1s
                    //        mnFramesToResetIMU 表示经过多少帧后可以重置IMU，一般设置为和帧率相同，对应的时间是1s
                    // 条件2：单目+IMU 或者 双目+IMU模式
                    // 同时满足条件1，2，标记为LOST
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        // 条件1：当前地图中关键帧数目较多（大于10）
                        // 条件2（隐藏条件）：当前帧距离上次重定位帧超过1s（说明还比较争气，值的救）或者非IMU模式
                        // 同时满足条件1，2，则将状态标记为RECENTLY_LOST，后面会结合IMU预测的位姿看看能不能拽回来
                        mState = RECENTLY_LOST;
                        // 记录丢失时间
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else  // 跟踪不正常按照下面处理
            {
                // 如果是RECENTLY_LOST状态
                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);
                    // bOK先置为true
                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        // IMU模式下可以用IMU来预测位姿，看能否拽回来
                        // Step 6.4 如果当前地图中IMU已经成功初始化，就用IMU数据预测位姿
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        // 如果IMU模式下当前帧距离跟丢帧超过5s还没有找回（time_recently_lost默认为5s）
                        // 放弃了，将RECENTLY_LOST状态改为LOST
                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // Step 6.5 纯视觉模式则进行重定位。主要是BOW搜索，EPnP求解位姿
                        // Relocalization
                        bOK = Relocalization();
                        //std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                        //std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK)
                        {
                            // 纯视觉模式下重定位失败，状态为LOST
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)  // 上一帧为最近丢失且重定位失败时
                {
                    //! Step 6.6 如果是LOST状态-----------------------------
                    // 开启一个新地图
                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        // 🔄 如果使用了棋盘格初始化，重置地图并重新启用棋盘格检测
                        if (mbChessboardInitialized) {
                            std::cout << "🔄 棋盘格初始化的地图跟踪失败，重置地图并重新启用棋盘格检测..." << std::endl;
                            // 重新启用棋盘格初始化，以便重新检测
                            mbUseChessboardInit = true;
                            mbChessboardInitialized = false;
                            std::cout << "✅ 已重新启用棋盘格初始化，系统将重新检测棋盘格" << std::endl;
                        }
                        // 当前地图中关键帧数目小于10，重置当前地图
                        mpSystem->ResetActiveMap();
                        Verbose::PrintMess("Reseting current map...", Verbose::VERBOSITY_NORMAL);
                    }else
                        CreateMapInAtlas();  // 当前地图中关键帧数目超过10，创建新地图
                    // 干掉上一个关键帧
                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }

        }
        else  // 纯定位模式
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            // 只进行跟踪tracking，局部地图不工作
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                // Step 6.1 LOST状态进行重定位
                bOK = Relocalization();
            }
            else
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变量
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常 (注意有点反直觉)
                // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跟丢
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    // Step 6.2 如果跟踪状态正常，使用恒速模型或参考关键帧跟踪
                    if(mbVelocity)
                    {
                        // 优先使用恒速模型跟踪
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        // 如果恒速模型不被满足,那么就只能够通过参考关键帧来跟踪
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跟丢的节奏，既做跟踪又做重定位

                    // MM=Motion Model,通过运动模型进行跟踪的结果
                    bool bOKMM = false;
                    // 通过重定位方法来跟踪的结果
                    bool bOKReloc = false;
                    // 运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    // 在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;
                    // 运动模型得到的位姿
                    Sophus::SE3f TcwMM;
                    // Step 6.3 当运动模型有效的时候,根据运动模型计算位姿
                    if(mbVelocity)
                    {
                        bOKMM = TrackWithMotionModel();
                        // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetPose();
                    }
                    // Step 6.4 使用重定位的方法来得到当前帧的位姿
                    bOKReloc = Relocalization();

                    // Step 6.5 根据前面的恒速模型、重定位结果来更新状态
                    if(bOKMM && !bOKReloc)
                    {
                        // 恒速模型成功、重定位失败，重新使用之前暂存的恒速模型结果
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        // 如果当前帧匹配的3D点很少，增加当前可视地图点的被观测次数
                        if(mbVO)
                        {
                            // 更新当前帧的地图点被找到次数，注意跟观测次数区分，这是两个概念
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                // 如果这个特征点形成了地图点,并且也不是外点的时候
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    // 增加被找到次数
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        // 只要重定位成功整个跟踪过程正常进行（重定位与跟踪，更相信重定位）
                        mbVO = false;
                    }
                    // 有一个成功我们就认为执行成功了
                    bOK = bOKReloc || bOKMM;
                }
            }
        }
        // 将最新的关键帧作为当前帧的参考关键帧
        // mpReferenceKF先是上一时刻的参考关键帧，如果当前为新关键帧则变成当前关键帧，如果不是新的关键帧则先为上一帧的参考关键帧，而后经过更新局部关键帧重新确定
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif

        // Step 7 在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        // 在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
        // 前面主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
        // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                // 局部地图跟踪
                bOK = TrackLocalMap();
            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }
        // 到此为止跟踪确定位姿阶段结束，下面开始做收尾工作和为下一帧做准备

        // 查看到此为止时的两个状态变化
        // bOK的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true                     -->OK   1 跟踪局部地图成功
        //          \               \              \---局部地图跟踪失败---false
        //           \               \---当前帧跟踪失败---false
        //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true                       -->OK  2 重定位
        //                          \           \---局部地图跟踪失败---false
        //                           \---重定位失败---false

        //
        // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK                  -->OK  1 跟踪局部地图成功
        //            \               \              \---局部地图跟踪失败---OK                  -->OK  3 正常跟踪
        //             \               \---当前帧跟踪失败---非OK
        //              \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---非OK
        //                            \           \---局部地图跟踪失败---非OK
        //                             \---重定位失败---非OK（传不到这里，因为直接return了）
        // 由上图可知当前帧的状态OK的条件是跟踪局部地图成功，重定位或正常跟踪都可
        // Step 8 根据上面的操作来判断是否追踪成功
        if(bOK)
            // 此时还OK才说明跟踪成功了
            mState = OK;
        else if (mState == OK)  // 由上图可知只有当第一阶段跟踪成功，但第二阶段局部地图跟踪失败时执行
        {
            // 状态变为最近丢失
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    // IMU模式下IMU没有成功初始化或者没有完成IMU BA，则重置当前地图
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState=RECENTLY_LOST;
            }
            else
                mState=RECENTLY_LOST; // 上一个版本这里直接判定丢失 LOST

            // 被注释掉了，记录丢失时间
            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        // 这段貌似没啥作用
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) &&
           (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }
        // 下面代码没有用
        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                // 当前帧距离上次重定位帧刚好等于1s，重置（还未实现 TODO）
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;  // 没啥用，后面会重新赋值后传给普通帧
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        // 更新显示线程中的图像、特征点、地图点等信息
        mpFrameDrawer->Update(this);
        if(mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        // 查看到此为止时的两个状态变化
        // bOK的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true
        //          \               \              \---局部地图跟踪失败---false
        //           \               \---当前帧跟踪失败---false
        //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true
        //                          \           \---局部地图跟踪失败---false
        //                           \---重定位失败---false

        // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
        //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
        //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
        //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
        //               \                           \           \---局部地图跟踪失败---LOST
        //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
        //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）

        // Step 9 如果跟踪成功 或 最近刚刚跟丢，更新速度，清除无效地图点，按需创建关键帧
        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            // Step 9.1 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
            if(mLastFrame.isSet() && mCurrentFrame.isSet())
            {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            }
            else {
                // 否则没有速度
                mbVelocity = false;
            }

            // 使用IMU积分的位姿显示
            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            // Clean VO matches
            // Step 9.2 清除观测不到的地图点
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // Step 9.3 清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
            // 上个步骤中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
            // 不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
            // 判断是否需要插入关键帧
            bool bNeedKF = NeedNewKeyFrame();

            // Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)

            // Step 9.4 根据条件来判断是否插入关键帧
            // 需要同时满足下面条件1和2
            // 条件1：bNeedKF=true，需要插入关键帧
            // 条件2：bOK=true跟踪成功 或 IMU模式下的RECENTLY_LOST模式且mInsertKFsLost为true
            if(bNeedKF && (bOK || (mInsertKFsLost && mState==RECENTLY_LOST &&
                                   (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
                CreateNewKeyFrame();  // 创建关键帧，对于双目或RGB-D会产生新的地图点

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
            // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

            //  Step 9.5 删除那些在BA中检测为外点的地图点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }

            // 🌟 在这里添加检查是否需要应用棋盘格坐标系转换的代码
            //! 检查是否需要应用棋盘格坐标系转换----------------------------------------
            if (mbUseChessboardInit && mbHasChessboardPosed && !mbChessboardInitialized && mState == OK)
            {
                // 检查地图是否已经初始化
                Map* pCurrentMap = mpAtlas->GetCurrentMap();
                if (pCurrentMap && pCurrentMap->KeyFramesInMap() >= 2)
                {
                    std::cout << "🌍 检测到系统已完成初始化，应用棋盘格坐标系转换..." << std::endl;
                    
                    // 应用坐标系转换
                    TransformAllMapElements(mTcw_Chessboard, mTwc_chessboard);
                    
                    // 标记已完成棋盘格初始化
                    mbChessboardInitialized = true;
                    std::cout << "✅ 棋盘格坐标系转换完成！" << std::endl;
                }
            }
        }




        // Reset if the camera get lost soon after initialization
        // Step 10 如果第二阶段跟踪失败，跟踪状态为LOST
        if(mState==LOST)
        {
            // 如果地图中关键帧小于10，重置当前地图，退出当前跟踪
            if(pCurrentMap->KeyFramesInMap()<=10)  // 上一个版本这里是5
            {
                // 🔄 如果使用了棋盘格初始化，重置地图并重新启用棋盘格检测
                if (mbChessboardInitialized) {
                    std::cout << "🔄 棋盘格初始化的地图跟踪失败，重置地图并重新启用棋盘格检测..." << std::endl;
                    // 重新启用棋盘格初始化，以便重新检测
                    mbUseChessboardInit = true;
                    mbChessboardInitialized = false;
                    std::cout << "✅ 已重新启用棋盘格初始化，系统将重新检测棋盘格" << std::endl;
                }
                mpSystem->ResetActiveMap();
                return;
            }
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                if (!pCurrentMap->isImuInitialized())
                {
                    // 如果是IMU模式并且还未进行IMU初始化，重置当前地图，退出当前跟踪
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }
            // 如果地图中关键帧超过10 并且 纯视觉模式 或 虽然是IMU模式但是已经完成IMU初始化了，保存当前地图，创建新的地图
            CreateMapInAtlas();

            // 新增加了个return
            return;
        }
        // 确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        // 保存上一帧的数据,当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
    }

    // 查看到此为止
    // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
    //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
    //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
    //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
    //               \                           \           \---局部地图跟踪失败---LOST
    //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
    //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）
    // last.记录位姿信息，用于轨迹复现
    // Step 11 记录位姿信息，用于最后保存所有的轨迹
    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // Step 11：记录位姿信息，用于最后保存所有的轨迹
        if(mCurrentFrame.isSet())
        {
            // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
            Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            // 如果跟踪失败，则相对位姿使用上一次值
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }

#ifdef REGISTER_LOOP
    if (Stop()) {

        // Safe area to stop
        while(isStopped())
        {
            usleep(3000);
        }
    }
#endif
}

/*
 * @brief 双目和rgbd的地图初始化，比单目简单很多
 *
 * 由于具有深度信息，直接生成MapPoints
 */
void Tracking::StereoInitialization()
{
    // 初始化要求当前帧的特征点超过500
    if(mCurrentFrame.N>500)
    {
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA).norm()<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        // 设定初始位姿为单位旋转，0平移，imu模式下设置的是相机位姿
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        }
        else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // Create KeyFrame
        // 将当前帧构造为初始关键帧
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // Insert KeyFrame in the map
        // 在地图中添加该初始关键帧
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){
            // 为每个特征点构造MapPoint
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                // 只有具有正深度的点才会被构造地图点
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    // 通过反投影得到该特征点的世界坐标系下3D坐标
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    // 为该MapPoint添加属性：
	                // a.观测到该MapPoint的关键帧
	                // b.该MapPoint的描述子
	                // c.该MapPoint的平均观测方向和深度范围
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;
        // 在局部地图中添加该初始关键帧
        mpLocalMapper->InsertKeyFrame(pKFini);

        // 更新当前帧为上一帧
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        //mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        // 追踪成功
        mState=OK;
    }
}

/*
 * @brief 单目的地图初始化
 *
 * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 *
 * Step 1：（未创建）得到用于初始化的第一帧，初始化需要两帧
 * Step 2：（已创建）如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
 * Step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
 * Step 4：如果初始化的两帧之间的匹配点太少，重新初始化
 * Step 5：通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
 * Step 6：删除那些无法进行三角化的匹配点
 * Step 7：将三角化得到的3D点包装成MapPoints
 */
void Tracking::MonocularInitialization()
{
    //? Step 1 如果单目初始器还没有被创建，则创建。后面如果重新初始化时会清掉这个
    if(!mbReadyToInitializate)
    {
        // Set Reference Frame
        // 单目初始帧的特征点数必须大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            // 初始化需要两帧，分别是mInitialFrame，mCurrentFrame
            mInitialFrame = Frame(mCurrentFrame);
            // 用当前帧更新上一帧
            mLastFrame = Frame(mCurrentFrame);
            // mvbPrevMatched  记录"上一帧"所有特征点
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            // 初始化为-1 表示没有任何匹配。这里面存储的是匹配的点的id
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            // 初始化预积分
            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }
            // 下一帧准备做单目初始化了
            mbReadyToInitializate = true;

            return;
        }
    }
    else  // 第二帧来了
    {
        // Step 2 如果当前帧特征点数太少（不超过100），则重新构造初始器
        if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences
        // Step 3 在mInitialFrame与mCurrentFrame中找匹配的特征点对
        // 0.9 表示最佳的和次佳特征点评分的比值阈值，这里是比较宽松的，跟踪时一般是0.7
        // true 表示检查特征点的方向
        ORBmatcher matcher(0.9,true);

        // 对 mInitialFrame,mCurrentFrame 进行特征点匹配
        // mvbPrevMatched为参考帧的特征点坐标，初始化存储的是mInitialFrame中特征点坐标，匹配后存储的是匹配好的当前帧的特征点坐标
        // mvIniMatches 保存参考帧F1中特征点是否匹配上，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        // Step 4 验证匹配结果，如果初始化的两帧之间的匹配点太少，重新初始化
        if(nmatches<100)
        {
            mbReadyToInitializate = false;
            return;
        }

        // Step 5 通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
        {
            // Step 6 初始化成功后，删除那些无法进行三角化的匹配点
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // Step 7 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(Sophus::SE3f());
            // 由Rcw和tcw构造Tcw,并赋值给mTcw，mTcw为世界坐标系到相机坐标系的变换矩阵
            mCurrentFrame.SetPose(Tcw);

            // Step 8 创建初始化地图点MapPoints
            // Initialize函数会得到mvIniP3D，
            // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量，
            // CreateInitialMapMonocular将3D点包装成MapPoint类型存入KeyFrame和Map中
            CreateInitialMapMonocular();
        }
    }
}

/**
 * @brief 单目相机成功初始化后用三角化得到的点生成MapPoints
 *
 */
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames 认为单目初始化时候的参考帧和当前帧都是关键帧
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

    // Step 1 将初始关键帧,当前关键帧的描述子转为BoW
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    // Step 2 将关键帧插入到地图
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    // Step 3 用初始化得到的3D点来生成地图点MapPoints
    //  mvIniMatches[i] 表示初始化两帧特征点匹配关系。
    //  具体解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值,没有匹配关系的话，vMatches12[i]值为 -1
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        // 没有匹配，跳过
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        // 用三角化点初始化为空间点的世界坐标
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        // Step 3.1 用3D点构造地图点
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        // Step 3.2 为该MapPoint添加属性：
        // a.观测到该MapPoint的关键帧
        // b.该MapPoint的描述子
        // c.该MapPoint的平均观测方向和深度范围

        // 表示该KeyFrame的2D特征点和对应的3D地图点
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        // b.从众多观测到该MapPoint的特征点中挑选最有代表性的描述子
        pMP->ComputeDistinctiveDescriptors();

        // c.更新该MapPoint平均观测方向以及观测距离的范围
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        // mvIniMatches下标i表示在初始化参考帧中的特征点的序号
        // mvIniMatches[i]是初始化当前帧中的特征点的序号
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    // Step 3.3 更新关键帧间的连接关系
    // 在3D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧公共3D点的个数
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    // Step 4 全局BA优化，同时优化所有位姿和三维点
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    // Step 5 取场景的中值深度，用于尺度归一化
    // 为什么是 pKFini 而不是 pKCur ? 答：都可以的，内部做了位姿变换了
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    // 两个条件,一个是平均深度要大于0,另外一个是在当前帧中被观测到的地图点的数目应该大于50
    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
        mpSystem->ResetActiveMap();
        return;
    }

    // Step 6 将两帧之间的变换归一化到平均深度1的尺度下
    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    // x/z y/z 将z归一化到1
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    // Step 7 把3D点的尺度也归一化到1
    // 为什么是pKFini? 是不是就算是使用 pKFcur 得到的结果也是相同的? 答：是的，因为是同样的三维点
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }

    // Step 8 将关键帧插入局部地图，更新归一化后的位姿、局部地图点
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    //mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    // 单目初始化之后，得到的初始地图中的所有点都是局部地图点
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    // 初始化成功，至此，初始化过程完成
    mState=OK;

    initID = pKFcur->mnId;
}

/**
 * @brief 在Atlas中保存当前地图，创建新地图，所有跟状态相关的变量全部重置
 * 1. 前后两帧对应的时间戳反了
 * 2. imu模式下前后帧超过1s
 * 3. 上一帧为最近丢失且重定位失败时
 * 4. 重定位成功，局部地图跟踪失败
 */
void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();  // mpAtlas中map的mbIsInertial=true
    mbSetInit=false;  // 好像没什么用

    mnInitialFrameId = mCurrentFrame.mnId+1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    //mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mbReadyToInitializate = false;
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();
    mlQueueImuData.clear();

    mbCreatedMap = true;
}

/*
 * @brief 检查上一帧中的地图点是否需要被替换
 *
 * Local Mapping线程可能会将关键帧中某些地图点进行替换，由于tracking中需要用到上一帧地图点，所以这里检查并更新上一帧中被替换的地图点
 * @see LocalMapping::SearchInNeighbors()
 */
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        // 如果这个地图点存在
        if(pMP)
        {
            // 获取其是否被替换,以及替换后的点
            // 这也是程序不直接删除这个地图点删除的原因
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                // 然后替换一下
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/*
 * @brief 用参考关键帧的地图点来对当前普通帧进行跟踪
 *
 * Step 1：将当前普通帧的描述子转化为BoW向量
 * Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
 * Step 3: 将上一帧的位姿态作为当前帧位姿的初始值
 * Step 4: 通过优化3D-2D的重投影误差来获得位姿
 * Step 5：剔除优化后的匹配点中的外点
 * @return 如果匹配数超10，返回true
 *
 */
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    // Step 1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    // Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    // 匹配数目小于15，认为跟踪失败
    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    // Step 3:将上一帧的位姿态作为当前帧位姿的初始值
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());  // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

    //mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    // Step 4:通过优化3D-2D的重投影误差来获得位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // Step 5：剔除优化后的匹配点中的外点
    //之所以在优化之后才剔除外点，是因为在优化的过程中就有了对这些外点的标记
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        //if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i])
        {
            // 如果对应到的某个特征点是外点
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 清除它在当前帧中存在过的痕迹
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                // 匹配的内点计数++
                nmatchesMap++;
        }
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap>=10;  // 跟踪成功的数目超过10才认为跟踪成功，否则跟踪失败
}

/**
 * @brief 更新上一帧位姿，在上一帧中生成临时地图点
 * 单目情况：只计算了上一帧的世界坐标系位姿
 * 双目和rgbd情况：选取有有深度值的并且没有被选为地图点的点生成新的临时地图点，提高跟踪鲁棒性
 */
void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    // Step 1：利用参考关键帧更新上一帧在世界坐标系下的位姿
    // 上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    // ref_keyframe 到 lastframe的位姿变换
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    // 将上一帧的世界坐标系下的位姿计算出来
    // l:last, r:reference, w:world
    // Tlw = Tlr*Trw
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    // 如果上一帧为关键帧，或者单目/单目惯性，SLAM模式的情况，则退出
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Step 2：对于双目或rgbd相机，为上一帧生成新的临时地图点
    // 注意这些地图点只是用来跟踪，不加入到地图中，跟踪完后会删除
    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // Step 2.1：得到上一帧中具有有效深度值的特征点（不一定是地图点）
    vector<pair<float,int> > vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for(int i=0; i<Nfeat;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            // vDepthIdx第一个元素是某个点的深度,第二个元素是对应的特征点id
            vDepthIdx.push_back(make_pair(z,i));
        }
    }
    // 如果上一帧中没有有效深度的点,那么就直接退出
    if(vDepthIdx.empty())
        return;

    // 按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // Step 2.2：从中找出不是地图点的部分
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
            // 地图点被创建后就没有被观测，认为不靠谱，也需要重新创建
            bCreateNew = true;

        if(bCreateNew)
        {
            // Step 2.3：需要创建的点，包装为地图点。只是为了提高双目和RGBD的跟踪成功率，并没有添加复杂属性，因为后面会扔掉
            // 反投影到世界坐标系中
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1){
                mLastFrame.UnprojectStereo(i, x3D);
            }
            else{
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            // 加入上一帧的地图点中
            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            // 因为从近到远排序，记录其中不需要创建地图点的个数
            nPoints++;
        }

        // Step 2.4：如果地图点质量不好，停止创建地图点
        // 停止新增临时地图点必须同时满足以下条件：
        // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
        // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;

    }
}

/**
 * @brief 根据恒定速度模型用上一帧地图点来对当前帧进行跟踪
 * Step 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
 * Step 2：根据上一帧特征点对应地图点进行投影匹配
 * Step 3：优化当前帧位姿
 * Step 4：剔除地图点中外点
 * @return 如果匹配数大于10，认为跟踪成功，返回true
 */
bool Tracking::TrackWithMotionModel()
{
    // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // Step 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
    UpdateLastFrame();

    // Step 2：根据IMU或者恒速模型得到当前帧的初始位姿。
    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        // IMU完成初始化 并且 距离重定位挺久不需要重置IMU，用IMU来估计位姿，没有后面的这那那这的
        PredictStateIMU();
        return true;
    }
    else
    {
        // 根据之前估计的速度，用恒速模型得到当前帧的初始位姿。
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }

    // 清空当前帧的地图点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    // 设置特征匹配过程中的搜索半径
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    // Step 3：用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    // 如果匹配点太少，则扩大搜索半径再来一次
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    // 这里不同于ORB-SLAM2的方式
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    // Step 4：利用3D-2D投影关系，优化当前帧位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // Step 5：剔除地图点中外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 如果优化后判断某个地图点是外点，清除它的所有关系
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                // 累加成功匹配到的地图点数目
                nmatchesMap++;
        }
    }

    // 纯定位模式下：如果成功追踪的地图点非常少,那么这里的mbVO标志就会置位
    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap>=10;  // 匹配超过10个点就认为跟踪成功
}

/**
 * @brief 用局部地图进行跟踪，进一步优化位姿
 *
 * 1. 更新局部地图，包括局部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return true if success
 *
 * Step 1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
 * Step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
 * Step 3：更新局部所有MapPoints后对位姿再次优化
 * Step 4：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
 * Step 5：决定是否跟踪成功
 */
bool Tracking::TrackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    // Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
    UpdateLocalMap();
    // Step 2：筛选局部地图中新增的在视野范围内的地图点，投影到当前帧搜索匹配，得到更多的匹配关系
    SearchLocalPoints();

    // TOO check outliers before PO
    // 查看内外点数目，调试用
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    // 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化
    // Step 3：前面新增了更多的匹配关系，BA优化得到更准确的位姿
    int inliers;
    // IMU未初始化，仅优化位姿
    if (!mpAtlas->isImuInitialized())
        Optimizer::PoseOptimization(&mCurrentFrame);
    else
    {
        // 初始化，重定位，重新开启一个地图都会使mnLastRelocFrameId变化
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else  // 如果积累的IMU数据量比较多，考虑使用IMU数据优化
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            // mbMapUpdated变化见Tracking::PredictStateIMU()
            // 未更新地图
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                // 使用上一普通帧以及当前帧的视觉信息和IMU信息联合优化当前帧位姿、速度和IMU零偏
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                // 使用上一关键帧以及当前帧的视觉信息和IMU信息联合优化当前帧位姿、速度和IMU零偏
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }
    // 查看内外点数目，调试用
    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    // Step 4：更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            // 由于当前帧的地图点可以被当前帧观测到，其被观测统计量加1
            if(!mCurrentFrame.mvbOutlier[i])
            {
                // 找到该点的帧数mnFound 加 1
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                // 查看当前是否是在纯定位过程
                if(!mbOnlyTracking)
                {
                    // 如果该地图点被相机观测数目nObs大于0，匹配内点计数+1
                    // nObs： 被观测到的相机数目，单目+1，双目或RGB-D则+2
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    // 记录当前帧跟踪到的地图点数目，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            // 如果这个地图点是外点,并且当前相机输入还是双目的时候,就删除这个点
            // 原因分析：因为双目本身可以左右互匹配，删掉无所谓
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    // Step 5：根据跟踪匹配数目及重定位情况决定是否跟踪成功
    // 如果最近刚刚发生了重定位,那么至少成功匹配50个点才认为是成功跟踪
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    // RECENTLY_LOST状态下，至少成功跟踪10个才算成功
    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;

    // 单目IMU模式下做完初始化至少成功跟踪15个才算成功，没做初始化需要50个
    if (mSensor == System::IMU_MONOCULAR)
    {
        if((mnMatchesInliers<15 && mpAtlas->isImuInitialized())||(mnMatchesInliers<50 && !mpAtlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        // 🎯 针对棋盘格初始化的特殊处理：如果地图点总数很少（比如棋盘格初始化），降低跟踪成功的阈值
        int totalMapPoints = mpAtlas->MapPointsInMap();
        int minMatches = 30; // 默认阈值
        
        if(mnMatchesInliers < minMatches)
            return false;
        else
            return true;
    }
}

/**
 * @brief 判断当前帧是否需要插入关键帧
 *
 * Step 1：纯VO模式下不插入关键帧，如果局部地图被闭环检测使用，则不插入关键帧
 * Step 2：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
 * Step 3：得到参考关键帧跟踪到的地图点数量
 * Step 4：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
 * Step 5：对于双目或RGBD摄像头，统计可以添加的有效地图点总数 和 跟踪到的地图点数量
 * Step 6：决策是否需要插入关键帧
 * @return true         需要
 * @return false        不需要
 */
bool Tracking::NeedNewKeyFrame()
{
    // 如果是IMU模式并且当前地图中未完成IMU初始化
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        // 如果是IMU模式，当前帧距离上一关键帧时间戳超过0.25s，则说明需要插入关键帧，不再进行后续判断
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    // Step 1：纯VO模式下不插入关键帧
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // Step 2：如果局部地图线程被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    // 获取当前地图中的关键帧数目
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    // mCurrentFrame.mnId是当前帧的ID
    // mnLastRelocFrameId是最近一次重定位帧的ID
    // mMaxFrames等于图像输入的帧率
    //  Step 3：如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    // Step 4：得到参考关键帧跟踪到的地图点数量
    // UpdateLocalKeyFrames 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧

    // 地图点的最小观测次数
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    // 参考关键帧地图点中观测的数目>= nMinObs的地图点数目
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    // Step 5：查询局部地图线程是否繁忙，当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    // Step 6：对于双目或RGBD摄像头，统计成功跟踪的近点的数量，如果跟踪到的近点太少，没有跟踪到的近点较多，可以插入关键帧
    int nNonTrackedClose = 0;  // 双目或RGB-D中没有跟踪到的近点
    int nTrackedClose= 0;  // 双目或RGB-D中成功跟踪的近点（三维点）

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            // 深度值在有效范围内
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
        //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
    }
    // 双目或RGBD情况下：跟踪到的地图点中近点太少 同时 没有跟踪到的三维点太多，可以插入关键帧了
    // 单目时，为false
    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Step 7：决策是否需要插入关键帧
    // Thresholds
    // Step 7.1：设定比例阈值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
    float thRefRatio = 0.75f;
    // 关键帧只有一帧，那么插入关键帧的阈值设置的低一点，插入频率较低
    if(nKFs<2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
        thRefRatio = 0.9f;
    }*/

    // 单目情况下插入关键帧的频率很高
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    // 单目+IMU情况下如果，匹配内点数目超过350，插入关键帧的频率可以适当降低
    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // Step 7.2：很长时间没有插入关键帧，可以插入
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // Step 7.3：满足插入关键帧的最小间隔并且localMapper处于空闲状态，可以插入
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle); //mpLocalMapper->KeyframesInQueue() < 2);
    //Condition 1c: tracking is weak
    // Step 7.4：在双目，RGB-D的情况下当前帧跟踪到的点比参考关键帧的0.25倍还少，或者满足bNeedToInsertClose
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // Step 7.5：和参考帧相比当前跟踪到的点太少 或者满足bNeedToInsertClose；同时跟踪到的内点还不能太少
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
    // Temporal condition for Inertial cases
    // 新增的条件c3：单目/双目+IMU模式下，并且IMU完成了初始化（隐藏条件），当前帧和上一关键帧之间时间超过0.5秒，则c3=true
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    // 新增的条件c4：单目+IMU模式下，当前帧匹配内点数在15~75之间或者是RECENTLY_LOST状态，c4=true
    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    // 相比ORB-SLAM2多了c3,c4
    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        // Step 7.6：local mapping空闲时或者正在做imu初始化时可以直接插入，不空闲的时候要根据情况插入
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
        {
            // 可以插入关键帧
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                // 双目或双目+IMU或RGB-D模式下，如队列里没有阻塞太多关键帧，可以插入
                // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                // 然后localmapper再逐个pop出来插入到mspKeyFrames
                if(mpLocalMapper->KeyframesInQueue()<3)
                    // 队列中的关键帧数目不是很多,可以插入
                    return true;
                else
                    // 队列中缓冲的关键帧数目太多,暂时不能插入
                    return false;
            }
            else
            {
                //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                //对于单目情况,就直接无法插入关键帧了
                //? 为什么这里对单目情况的处理不一样?
                //回答：可能是单目关键帧相对比较密集
                return false;
            }
        }
    }
    else
        // 不满足上面的条件,自然不能插入关键帧
        return false;
}

/**
 * @brief 创建新的关键帧
 * 对于非单目的情况，同时创建新的MapPoints
 *
 * Step 1：将当前帧构造成关键帧
 * Step 2：将当前关键帧设置为当前帧的参考关键帧
 * Step 3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
 */
void Tracking::CreateNewKeyFrame()
{
    // 如果局部建图线程正在初始化且没做完或关闭了,就无法插入关键帧
    if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;

    // Step 1：将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    // Step 2：将当前关键帧设置为当前帧的参考关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    // 这段代码和 Tracking::UpdateLastFrame 中的那一部分代码功能相同
    // Step 3：对于双目或rgbd摄像头，为当前帧生成新的地图点；单目无操作
    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        // 根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        // Step 3.1：得到当前帧有深度值的特征点（不一定是地图点）
        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                // 第一个元素是深度,第二个元素是对应的特征点的id
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            // Step 3.2：按照深度从小到大排序
            sort(vDepthIdx.begin(),vDepthIdx.end());

            // Step 3.3：从中找出不是地图点的生成临时地图点
            // 处理的近点的个数
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                // 如果需要就新建地图点，这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
                if(bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    if(mCurrentFrame.Nleft == -1){
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    // 这些添加属性的操作是每次创建MapPoint后都要做的
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    // 因为从近到远排序，记录其中不需要创建地图点的个数
                    nPoints++;
                }

                // Step 3.4：停止新建地图点必须同时满足以下条件：
                // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
                // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }
            //Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
        }
    }

    // Step 4：插入关键帧
    // 关键帧插入到列表 mlNewKeyFrames中，等待local mapping线程临幸
    mpLocalMapper->InsertKeyFrame(pKF);

    // 插入好了，允许局部建图停止
    mpLocalMapper->SetNotStop(false);

    // 当前帧成为新的关键帧，更新
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

/**
 * @brief 用局部地图点进行投影匹配，得到更多的匹配关系
 * 注意：局部地图点中已经是当前帧地图点的不需要再投影，只需要将此外的并且在视野范围内的点和当前帧进行投影匹配
 */
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    // Step 1：遍历当前帧的地图点，标记这些地图点不参与之后的投影搜索匹配
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1(被当前帧观测了)
                pMP->IncreaseVisible();
                // 标记该点被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // 标记该点在后面搜索匹配时不被投影，因为已经有匹配了
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    // 准备进行投影匹配的点的数目
    int nToMatch=0;

    // Project points in frame and check its visibility
    // Step 2：判断所有局部地图点中除当前帧地图点外的点，是否在当前帧视野范围内
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        // 已经被当前帧观测到的地图点肯定在视野范围内，跳过
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        // 跳过坏点
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        // 判断地图点是否在在当前帧视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数加1
            pMP->IncreaseVisible();
            // 只有在视野范围内的地图点才参与之后的投影匹配
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    // Step 3：如果需要进行投影匹配的点的数目大于0，就进行投影匹配，增加更多的匹配关系
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)  // RGBD相机输入的时候,搜索的阈值会变得稍微大一些
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=6;  // 0.4版本这里是3
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15
        // 投影匹配得到更多的匹配关系
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

/**
 * @brief 更新LocalMap
 *
 * 局部地图包括：
 * 1、K1个关键帧、K2个临近关键帧和参考关键帧
 * 2、由这些关键帧观测到的MapPoints
 */
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    // 设置参考地图点用于绘图显示局部地图点（红色）
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    // 用共视图来更新局部关键帧和局部地图点
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

/*
 * @brief 更新局部关键点。先把局部地图清空，然后将局部关键帧的有效地图点添加到局部地图中
 */
void Tracking::UpdateLocalPoints()
{
    // Step 1：清空局部地图点
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    // Step 2：遍历局部关键帧 mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // step 2：将局部关键帧的地图点添加到mvpLocalMapPoints
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            // 用该地图点的成员变量mnTrackReferenceForFrame 记录当前帧的id
            // 表示它已经是当前帧的局部地图点了，可以防止重复添加局部地图点
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

/**
 * @brief 跟踪局部地图函数里，更新局部关键帧
 * 方法是遍历当前帧的地图点，将观测到这些地图点的关键帧和相邻的关键帧及其父子关键帧，作为mvpLocalKeyFrames
 * Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
 * Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧包括以下3种类型
 *      类型1：能观测到当前帧地图点的关键帧，也称一级共视关键帧
 *      类型2：一级共视关键帧的共视关键帧，称为二级共视关键帧
 *      类型3：一级共视关键帧的子关键帧、父关键帧
 * Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
 */
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    // Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
    map<KeyFrame*,int> keyframeCounter;
    // 如果IMU未初始化 或者 刚刚完成重定位
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    // 得到观测到该地图点的关键帧和该地图点在关键帧中的索引
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    // 由于一个地图点可以被多个关键帧观测到,因此对于每一次观测,都对观测到这个地图点的关键帧进行累计投票
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        // 这里的操作非常精彩！
	                    // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对
	                    // it->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
	                    // 所以最后keyframeCounter 第一个参数表示某个关键帧，第2个参数表示该关键帧看到了多少当前帧(mCurrentFrame)的地图点，也就是共视程度
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    else
    {
        // ?为什么IMU初始化后用mLastFrame？mLastFrame存储的是上一帧跟踪成功后帧数据。
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }

    // 存储具有最多观测次数（max）的关键帧
    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有3种类型
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    // 先申请3倍内存，不够后面再加
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // Step 2.1 类型1：能观测到当前帧地图点的关键帧作为局部关键帧 （将邻居拉拢入伙）（一级共视关键帧）
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        // 如果设定为要删除的，跳过
        if(pKF->isBad())
            continue;

        // 寻找具有最大观测数目的关键帧
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        // 添加到局部关键帧的列表里
        mvpLocalKeyFrames.push_back(pKF);
        // 用该关键帧的成员变量mnTrackReferenceForFrame 记录当前帧的id
        // 表示它已经是当前帧的局部关键帧了，可以防止重复添加局部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // Step 2.2 遍历一级共视关键帧，寻找更多的局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        // 处理的局部关键帧不超过80帧
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        // 类型2:一级共视关键帧的共视（前10个）关键帧，称为二级共视关键帧（将邻居的邻居拉拢入伙）
        // 如果共视帧不足10帧,那么就返回所有具有共视关系的关键帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        // vNeighs 是按照共视程度从大到小排列
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 类型3:将一级共视关键帧的子关键帧作为局部关键帧（将邻居的孩子们拉拢入伙）
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 类型3:将一级共视关键帧的父关键帧（将邻居的父母们拉拢入伙）
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            // mnTrackReferenceForFrame防止重复添加局部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    // IMU模式下增加了临时的关键帧
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&mvpLocalKeyFrames.size()<80)
    {
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    // Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/**
 * @details 重定位过程
 * @return true
 * @return false
 *
 * Step 1：计算当前帧特征点的词袋向量
 * Step 2：找到与当前帧相似的候选关键帧
 * Step 3：通过BoW进行匹配
 * Step 4：通过EPnP算法估计姿态
 * Step 5：通过PoseOptimization对姿态进行优化求解
 * Step 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
 */
bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    // Step 1: 计算当前帧特征点的Bow映射
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // Step 2：找到与当前帧相似的候选关键帧组
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    // 每个关键帧的解算器
    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    // 每个关键帧和当前帧中特征点的匹配关系
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    // 放弃某个关键帧的标记
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    // 有效的候选关键帧数目
    int nCandidates=0;

    // Step 3：遍历所有的候选关键帧，通过BoW进行快速匹配，用匹配结果初始化PnP Solver
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 当前帧和候选关键帧用BoW进行快速匹配，匹配结果记录在vvpMapPointMatches，nmatches表示匹配的数目
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            // 如果和当前帧的匹配数小于15,那么只能放弃这个关键帧
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // 如果匹配数目够用，用匹配结果初始化MLPnPsolver
                // ? 为什么用MLPnP? 因为考虑了鱼眼相机模型，解耦某些关系？
                // 参考论文《MLPNP-A REAL-TIME MAXIMUM LIKELIHOOD SOLUTION TO THE PERSPECTIVE-N-POINT PROBLEM》
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                // 构造函数调用了一遍，这里重新设置参数
                pSolver->SetRansacParameters(
                    0.99,                    // 模型最大概率值，默认0.9
                    10,                      // 内点的最小阈值，默认8
                    300,                     // 最大迭代次数，默认300
                    6,                       // 最小集，每次采样六个点，即最小集应该设置为6，论文里面写着I > 5
                    0.5,                     // 理论最少内点个数，这里是按照总数的比例计算，所以epsilon是比例，默认是0.4
                    5.991);                  // 卡方检验阈值 //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;  // 1.0版本新加的
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    // 足够的内点才能匹配使用PNP算法，MLPnP需要至少6个点
    // 是否已经找到相匹配的关键帧的标志
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    // Step 4: 通过一系列操作,直到找到能够匹配上的关键帧
    // 为什么搞这么复杂？答：是担心误闭环
    while(nCandidates>0 && !bMatch)
    {
        // 遍历当前所有的候选关键帧
        for(int i=0; i<nKFs; i++)
        {
            // 忽略放弃的
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            // 内点标记
            vector<bool> vbInliers;
            // 内点数
            int nInliers;
            // 表示RANSAC已经没有更多的迭代次数可用 -- 也就是说数据不够好，RANSAC也已经尽力了。。。
            bool bNoMore;

            // Step 4.1：通过MLPnP算法估计姿态，迭代5次
            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            // PnP算法的入口函数
            bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            // bNoMore 为true 表示已经超过了RANSAC最大迭代次数，就放弃当前关键帧
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(bTcw)
            {
                // Step 4.2：如果MLPnP 计算出了位姿，对内点进行BA优化
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);
                // MLPnP 里RANSAC后的内点的集合
                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                // 遍历所有内点
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                // 只优化位姿,不优化地图点的坐标，返回的是内点的数量
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                // 如果优化之后的内点数目不多，跳过了当前候选关键帧,但是却没有放弃当前帧的重定位
                if(nGood<10)
                    continue;

                // 删除外点对应的地图点,这里直接设为空指针
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                // Step 4.3：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                // 前面的匹配关系是用词袋匹配过程得到的
                if(nGood<50)
                {
                    // 通过投影的方式将关键帧中未匹配的地图点投影到当前帧中, 生成新的匹配
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    // 如果通过投影过程新增了比较多的匹配特征点对
                    if(nadditional+nGood>=50)
                    {
                        // 根据投影匹配的结果，再次采用3D-2D pnp BA优化位姿
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        // Step 4.4：如果BA后内点数还是比较少(<50)但是还不至于太少(>30)，可以挽救一下, 最后垂死挣扎
                        // 重新执行上一步 4.3的过程，只不过使用更小的搜索窗口
                        // 这里的位姿已经使用了更多的点进行了优化,应该更准，所以使用更小的窗口搜索
                        if(nGood>30 && nGood<50)
                        {
                            // 用更小窗口、更严格的描述子阈值，重新进行投影搜索匹配
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            // 如果成功挽救回来，匹配数目达到要求，最后BA优化一下
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                                // 更新地图点
                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                            // 如果还是不能够满足就放弃了
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                // 如果对于当前的候选关键帧已经有足够的内点(50个)了,那么就认为重定位成功
                if(nGood>=50)
                {
                    bMatch = true;
                    // 只要有一个候选关键帧重定位成功，就退出循环，不考虑其他候选关键帧了
                    break;
                }
            }
        }// 一直运行,知道已经没有足够的关键帧,或者是已经有成功匹配上的关键帧
    }

    // 折腾了这么久还是没有匹配上，重定位失败
    if(!bMatch)
    {
        return false;
    }
    else
    {
        // 如果匹配上了,说明当前帧重定位成功了(当前帧已经有了自己的位姿)
        // 记录成功重定位帧的id，防止短时间多次重定位
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

/**
 * @brief 整个追踪线程执行复位操作
 */
void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);
    // 基本上是挨个请求各个线程终止
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    // 然后复位各种变量
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

/**
 * @brief 重置当前活动地图
 */
void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    //mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    mbVelocity = false;

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

/**
 * @brief 显示用的
 */
vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

/**
 * @brief 没用
 */
void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mK_.setIdentity();
    mK_(0,0) = fx;
    mK_(1,1) = fy;
    mK_(0,2) = cx;
    mK_(1,2) = cy;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

/**
 * @brief 更新了关键帧的位姿，但需要修改普通帧的位姿，因为正常跟踪需要普通帧
 * localmapping中初始化imu中使用，速度的走向（仅在imu模式使用），最开始速度定义于imu初始化时，每个关键帧都根据位移除以时间得到，经过非线性优化保存于KF中.
 * 之后使用本函数，让上一帧与当前帧分别与他们对应的上一关键帧做速度叠加得到，后面新的frame速度由上一个帧速度决定，如果使用匀速模型（大多数情况下），通过imu积分更新速度。
 * 新的关键帧继承于对应帧
 * @param  s 尺度
 * @param  b 初始化后第一帧的偏置
 * @param  pCurrentKeyFrame 当前关键帧
 */
void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;

    // 每一帧的参考关键帧
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();  // 对应帧是否跟踪丢失

    // mlRelativeFramePoses 存放的是Tcr
    // 三个变量一一对应
    // mlRelativeFramePoses用于输出位姿，因此初始化之前里面数据没有尺度，所以要更新下尺度
    for(auto lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }
    // 设置偏置
    mLastBias = b;
    // 设置上一关键帧，如果说mpLastKeyFrame已经是经过添加的新的kf，而pCurrentKeyFrame还是上一个kf，mpLastKeyFrame直接指向之前的kf
    mpLastKeyFrame = pCurrentKeyFrame;
    // 更新偏置
    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        // 当前帧需要预积分完毕，这段函数实在localmapping里调用的
        usleep(500);
    }

    // TODO 如果上一帧正好是上一帧的上一关键帧（mLastFrame.mpLastKeyFrame与mLastFrame不可能是一个，可以验证一下）
    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT;
        // 根据mLastFrame的上一个关键帧的信息（此时已经经过imu初始化了，所以关键帧的信息都是校正后的）以及imu的预积分重新计算上一帧的位姿
        mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    // 当前帧是否做了预积分
    if (mCurrentFrame.mpImuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder)
{
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    //mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap)
{
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if(!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale()
{
    return mImageScale;
}

#ifdef REGISTER_LOOP
void Tracking::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool Tracking::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool Tracking::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void Tracking::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}
#endif

} //namespace ORB_SLAM
