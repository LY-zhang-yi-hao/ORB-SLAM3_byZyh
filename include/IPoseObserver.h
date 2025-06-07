#ifndef IPOSESUBSCRIBER_H  // 防止重复包含
#define IPOSESUBSCRIBER_H 

#include <sophus/se3.hpp>  //Sophus库
#include <string>
#include <vector>
#include <mutex>

namespace ORB_SLAM3
 {

// 姿态观察者接口
// 用于观察者模式，当姿态发生变化时，通知观察者
class IPoseObserver
{
public:
    /**
     * @brief 虚析构函数
     * 确保当通过基类指针删除派生类对象时，派生类的析构函数能被正确调用。
     */
    virtual ~IPoseObserver() {}
    
    /**
     * @brief 当 Tracking 线程更新其实时估计位姿时调用此函数。
     * 这种位姿通常具有较低的延迟，但可能不如经过后端优化的位姿精确。
     * @param T_custom_world_camera 自定义世界坐标系下的相机位姿 (T_custom_c)。
     *                              这是从 ORB-SLAM 的世界坐标系转换到用户定义的自定义坐标系后的位姿。
     * @param timestamp 该位姿对应的时间戳，通常来自当前处理的帧。
     */
     virtual void OnRealTimePoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) = 0;

     /**
      * @brief 当 LocalMapping 线程（或其他后端优化模块）更新了优化后的位姿时调用此函数。
      * 这种位姿通常具有较高的精度，但可能伴随一定的延迟。
      * @param T_custom_world_camera 自定义世界坐标系下的相机位姿 (T_custom_c)。
      *                              这是经过优化后，并从 ORB-SLAM 的世界坐标系转换到用户定义的自定义坐标系后的位姿。
      * @param timestamp 该位姿对应的时间戳，通常来自相关的关键帧。
      */
     virtual void OnOptimizedPoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) = 0;
        
};       
    
}
#endif

