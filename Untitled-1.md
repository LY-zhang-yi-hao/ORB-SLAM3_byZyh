## 6. 🗺️ [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程：优化位姿并融入自定义坐标系

在 ORB-SLAM3 中，[LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程扮演着地图构建和优化的核心角色。它接收来自 `Tracking` 线程的关键帧，通过一系列复杂的处理流程，不仅丰富了地图信息，更重要的是通过**局部光束法平差 (Local Bundle Adjustment, LBA)** 来优化这些关键帧的位姿以及相关的地图点。本章节将深入探讨 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程如何处理和优化位姿，并阐述如何将我们的自定义坐标系逻辑融入其中，以获取经过优化的、在自定义坐标系下的相机位姿。

### 6.1. 🧠 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程核心流程与位姿优化机制

[LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程在其主循环 `LocalMapping::Run()` 中执行一系列关键操作。以下是其核心步骤，特别是与位姿处理和优化相关的部分：

1. **接收新关键帧 ([ProcessNewKeyFrame](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:377:0-435:1))**:
  - [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 从一个队列中取出由 `Tracking` 线程送来的新关键帧 (`mpCurrentKeyFrame`)。
  - 此时，关键帧的位姿是 `Tracking` 线程给出的初始估计。
  - 此步骤会计算关键帧的词袋表示、更新共视图和本质图等。

2. **地图点剔除 ([MapPointCulling](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:446:0-500:1))**:
  - 移除在跟踪过程中被判断为质量不佳或冗余的地图点。此步骤不直接修改关键帧位姿，但通过净化地图点间接影响后续优化的质量。

3. **创建新地图点 ([CreateNewMapPoints](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:502:0-911:1))**:
  - 通过与共视关键帧进行三角化，创建新的三维地图点。这些新点会为后续的位姿优化提供更多约束。

4. **邻域搜索与融合 ([SearchInNeighbors](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:913:0-1065:1))**:
  - 在当前关键帧的邻域内（共视关键帧）搜索匹配，并融合重复的地图点。这增强了局部地图的一致性。

5. **✨ 局部光束法平差 (LBA)**:
  - **这是 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程中对位姿进行精炼的最关键步骤。**
  - 当队列中没有新的关键帧需要立即处理，并且系统满足特定条件时（例如地图中关键帧数量足够），[LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 会执行 LBA。
  - **调用点**: 在 `LocalMapping::Run()` 内部，会根据是否启用 IMU 调用：
    - `Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, ...)`: 用于纯视觉或 IMU 未初始化的情况。
    - `Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, ...)`: 用于视觉惯性模式，联合优化相机位姿、速度、IMU偏置以及地图点。
  - **优化目标**: LBA 会优化一个由 `mpCurrentKeyFrame` 及其在共视图中的邻近关键帧（通常是一级和二级邻居）构成的局部窗口。同时，这些关键帧观测到的地图点也会被优化。
  - **结果**: LBA 完成后，`mpCurrentKeyFrame` 以及局部窗口内其他参与优化的关键帧的位姿 (`mTcw`) 会得到更新，变得更加精确。**此时的位姿是 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程针对该局部区域能提供的最佳估计。**

6. **IMU 初始化与进一步优化 (若启用 IMU)**:
  - [InitializeIMU()](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:1509:0-1840:1): 如果系统使用 IMU 且尚未初始化，此函数会被调用以估计初始的 IMU 参数（尺度、重力方向、速度、偏置）。这会显著影响关键帧位姿的绝对尺度和姿态。
  - [ScaleRefinement()](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:1842:0-1923:1): 在 IMU 初始化早期，可能会调用此函数进一步优化尺度和重力方向。

7. **关键帧剔除 ([KeyFrameCulling](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:1172:0-1366:1))**:
  - 移除那些信息冗余的关键帧，以保持地图的紧凑性。

**结论**: [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程通过 LBA (以及 IMU 相关优化) 对关键帧位姿进行显著优化。**LBA 成功执行之后，`mpCurrentKeyFrame` (以及其局部邻域内的其他关键帧) 的位姿代表了 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 在该次迭代中对这些关键帧位置和姿态的最佳估计。**

### 6.2. 🛠️ 融入自定义坐标系：原理与具体方式

我们的目标是在 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程完成位姿优化后，获取这些优化后的位姿，并将其转换到用户自定义的世界坐标系下，然后通过已建立的观察者模式通知出去。

#### 6.2.1. 原理概述

1. **共享资源**: 复用 `Tracking` 线程中已经实现的机制：
  - 自定义坐标变换矩阵 `mT_custom_orb`。
  - 观察者列表 `mvpPoseObservers` 及相应的互斥锁 `mMutexTransform` 和 `mMutexPoseAccess`。
  - 为此，[LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 类需要持有指向 `Tracking` 对象的指针 (`mpTracker`)。

2. **关键时机**: 在 `LocalMapping::Run()` 中，紧随 `Optimizer::LocalBundleAdjustment` 或 `Optimizer::LocalInertialBA` **成功执行之后**。

3. **操作流程**:
  - 获取 LBA 优化后的关键帧 (主要关注 `mpCurrentKeyFrame`)。
  - 读取其更新后的位姿 `T_orb_c = pKFOptimized->GetPoseInverse()`。
  - 使用 `mpTracker->mT_custom_orb` 进行坐标变换: `T_custom_c = mpTracker->mT_custom_orb * T_orb_c`。
  - 通过 `mpTracker->mvpPoseObservers` 通知所有观察者。

#### 6.2.2. 代码修改的详细思路 (概念性，用于指导实际编码)

**A. 准备工作：使 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 能访问 `Tracking` 的资源**

-  **`LocalMapping.h`**:
    ```cpp
    // ... (其他 #include)
    #include "Tracking.h" // 确保 Tracking 类的定义可见

    // class Tracking; // 或者使用前向声明，如果 Tracking.h 引入了循环依赖问题，但通常直接包含更简单

    class LocalMapping 
    {
    public:
        // 修改构造函数声明以接收 Tracking 指针
        LocalMapping(System* pSys, Atlas *pAtlas, Tracking* pTracker, const float bMonocular, bool bInertial, const std::string &_strSeqName);
        
        // ... (其他公共成员)

    private:
        // ... (其他私有成员)
        Tracking* mpTracker; // 指向 Tracking 对象的指针
        
        // 用于 LBA 后通知的辅助函数 (可选，但推荐封装)
        void NotifyOptimizedPose(KeyFrame* pKF); 
    };
    ```

-  **[LocalMapping.cc](cci:7://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:0:0-0:0)**:
    *   **构造函数实现**:
        ```cpp
        LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, Tracking* pTracker, const float bMonocular, bool bInertial, const std::string &_strSeqName) :
            mpSystem(pSys), 
            mpAtlas(pAtlas), 
            mpTracker(pTracker), // 初始化 mpTracker 成员
            mbMonocular(bMonocular), 
            mbInertial(bInertial), 
            // ... (其他成员的初始化)
        {
            // 构造函数体
        }
        ```
    *   **`System.cc` (或等效的初始化位置)**:
        在创建 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 对象实例时，将 `Tracking` 对象的指针 (`this`，如果在 `Tracking` 内部创建；或者 `mpTracker`，如果在 `System` 中创建) 传递给其构造函数。
        ```cpp
        // 示例: 在 System::Initialize() 或构造函数中
        // mpTracker = new Tracking(...);
        // mpLocalMapper = new LocalMapping(this, mpAtlas, mpTracker, ...); // 将 mpTracker 传递过去
        ```

**B. 在 `LocalMapping::Run()` 中 LBA 之后插入通知逻辑**

-  **定位修改点**: 在 `LocalMapping::Run()` 函数中，找到调用 `Optimizer::LocalBundleAdjustment(...)` 和 `Optimizer::LocalInertialBA(...)` 的代码块。通常在这之后会有一个标志（如 `b_doneLBA`）表示 LBA 是否执行。

-  **插入代码的上下文**:
    ```cpp
    // 在 LocalMapping::Run() 中
    // ...
    bool b_doneLBA = false;
    // ...
    if (!CheckNewKeyFrames() && !stopRequested()) {
        if (mpAtlas->KeyFramesInMap() > 2) {
            if (mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized()) {
                // ...
                Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), /*...*/);
                b_doneLBA = true;
            } else {
                // ...
                Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), /*...*/);
                b_doneLBA = true;
            }
        } // 结束 if(mpAtlas->KeyFramesInMap()>2)

        // ***** 在此处插入我们的自定义逻辑 *****
        if (b_doneLBA && !mbAbortBA) { // 确保 LBA 完成且未被中止
            if (mpCurrentKeyFrame && !mpCurrentKeyFrame->isBad()) {
                 NotifyOptimizedPose(mpCurrentKeyFrame); // 调用辅助函数
            }
            // 可选：如果LBA优化了多个关键帧 (lLocalKeyFrames)，并且您需要它们的位姿，
            // 则需要获取这些关键帧的列表并遍历调用 NotifyOptimizedPose。
            // 例如，Optimizer::LocalBundleAdjustment 内部会处理 vpOptimizableKFs。
            // 如果这个列表没有直接暴露，可以考虑优化 mpCurrentKeyFrame 的共视关键帧。
        }
        // ***** 自定义逻辑结束 *****
    }
    // ...
    ```

**C. 实现 `NotifyOptimizedPose` 辅助函数**

-  **[LocalMapping.cc](cci:7://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:0:0-0:0)**:
    ```cpp
    void LocalMapping::NotifyOptimizedPose(KeyFrame* pKF)
    {
        // 检查 mpTracker 和观察者列表是否有效
        if (!mpTracker || !pKF || pKF->isBad() || mpTracker->mvpPoseObservers.empty()) {
            return;
        }

        // 1. 获取优化后的位姿 (T_orb_c, 即 mTwc)
        const Sophus::SE3f T_orb_c = pKF->GetPoseInverse();
        
        Sophus::SE3f T_custom_c;

        // 2. 加锁并进行坐标变换 (使用 Tracking 线程的互斥锁)
        {
            std::unique_lock<std::mutex> lock_transform(mpTracker->mMutexTransform);
            // 确保 mT_custom_orb 已经被正确设置
            if (mpTracker->mbSetCustomTransform) { // 假设 Tracking 中有这样一个标志
                 T_custom_c = mpTracker->mT_custom_orb * T_orb_c;
            } else {
                // 如果自定义变换未设置，可以选择不通知或发送原始位姿（取决于需求）
                // 或者直接返回，避免发送无意义数据
                return; 
            }
        }
        
        // 3. 加锁并通知所有观察者 (使用 Tracking 线程的互斥锁)
        {
            std::unique_lock<std::mutex> lock_observers(mpTracker->mMutexPoseAccess);
            for (IPoseObserver* pObserver : mpTracker->mvpPoseObservers) {
                if (pObserver) {
                    // 假设 IPoseObserver::OnRealTimePoseUpdated 接口形式为:
                    // void OnRealTimePoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp);
                    // 如果需要区分位姿来源 (Tracking vs LocalMapping), 接口需要调整，
                    // 例如增加一个 PoseSource 枚举参数。
                    // pObserver->OnRealTimePoseUpdated(T_custom_c, pKF->mTimeStamp, OPTIMIZED_LOCAL_MAPPING_POSE_SOURCE_FLAG);
                    pObserver->OnRealTimePoseUpdated(T_custom_c, pKF->mTimeStamp); // 使用现有接口
                }
            }
        }
    }
    ```
    *   **关于 `mpTracker->mbSetCustomTransform`**: 这是一个建议的标志。在 `Tracking::SetCustomWorldTransform` 中，当用户成功设置了 `mT_custom_orb` 后，可以将此标志设为 `true`。这样可以避免在 `mT_custom_orb` 未初始化时进行无效的乘法操作。

#### 6.2.3. 考虑位姿来源的区分

## 7. 🏗️ 代码重构：创建专门的 `IPoseObserver` 接口文件

为了提升代码的模块化和可维护性，遵循行业规范，我们将姿态观察者 (`IPoseObserver`) 相关的定义从 `Tracking.h` 中分离出来，放入一个专门的头文件中。这使得接口定义更加清晰，也便于其他模块（如 `LocalMapping`）直接引用，而无需包含整个 `Tracking.h`。

### 7.1. 创建新头文件 `IPoseObserver.h`

在 ORB-SLAM3 的 `include` 目录下创建一个新的头文件，命名为 `IPoseObserver.h`。

**文件路径**: `/home/zyh/Desktop/ORB-SLAM3_byZyh/include/IPoseObserver.h`

**`IPoseObserver.h` 的内容**:

```cpp
#ifndef IPOSESUBSCRIBER_H // 使用 IPOSESUBSCRIBER_H 或 IPOSESOBSERVER_H 均可，保持一致即可
#define IPOSESUBSCRIBER_H

#include <sophus/se3.hpp> // 用于 Sophus::SE3f

// 可选：如果观察者实现类需要这些，可以包含
// #include <string>
// #include <vector>
// #include <mutex>

namespace ORB_SLAM3
{

// 姿态观察者接口 (Interface for Pose Observer/Subscriber)
// 该接口定义了不同来源位姿更新的回调函数
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
     *                             这是从 ORB-SLAM 的世界坐标系转换到用户定义的自定义坐标系后的位姿。
     * @param timestamp 该位姿对应的时间戳，通常来自当前处理的帧。
     */
    virtual void OnRealTimePoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) = 0;

    /**
     * @brief 当 LocalMapping 线程（或其他后端优化模块）更新了优化后的位姿时调用此函数。
     * 这种位姿通常具有较高的精度，但可能伴随一定的延迟。
     * @param T_custom_world_camera 自定义世界坐标系下的相机位姿 (T_custom_c)。
     *                             这是经过优化后，并从 ORB-SLAM 的世界坐标系转换到用户定义的自定义坐标系后的位姿。
     * @param timestamp 该位姿对应的时间戳，通常来自相关的关键帧。
     */
    virtual void OnOptimizedPoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) = 0;
};

} // namespace ORB_SLAM3

#endif // IPOSESUBSCRIBER_H
```

### 7.2. 修改 `Tracking.h`

1. **移除旧的 `IPoseObserver` 定义**:
    *   如果您之前在 `Tracking.h` 中定义了 `IPoseObserver` 接口（或类似的 `PoseObserver`）以及可能的 `PoseSource` 枚举，请将它们从 `Tracking.h` 中完全删除。

2.  **包含新的头文件**:
    在 `Tracking.h` 的顶部（或其他合适的位置，通常在命名空间声明之前或之后，具体取决于项目风格）添加 `#include` 指令：
    ```cpp
    // In /home/zyh/Desktop/ORB-SLAM3_byZyh/include/Tracking.h
    // ... (其他 #includes)
    #include "IPoseObserver.h" // <--- 包含新的接口定义

    namespace ORB_SLAM3
    {
    // ... (class Tracking 定义)
    }
    ```

3.  **确认成员变量类型**:
    确保 `Tracking` 类中用于存储观察者指针的列表（例如 `mvpPoseObservers`）的类型是 `std::vector<IPoseObserver*>`。
    ```cpp
    // In class Tracking:
    // private:
    //     std::vector<IPoseObserver*> mvpPoseObservers; // 确保类型是 IPoseObserver*
    //     std::mutex mMutexPoseAccess; // 用于保护 mvpPoseObservers
    //     // ... 其他成员 ...
    ```

### 7.3. 修改 `Tracking.cc`

在 `Tracking::Track()` 方法中（或者您进行实时位姿通知的具体位置），修改对观察者回调的调用，使其调用 `OnRealTimePoseUpdated`：

```cpp
// In /home/zyh/Desktop/ORB-SLAM3_byZyh/src/Tracking.cc
// Inside Tracking::Track() method, where pose notification happens:

// ... (计算得到 T_custom_c) ...

// 通知所有观察者 (使用新的接口方法)
{
    std::unique_lock<std::mutex> lock(mMutexPoseAccess); // 假设 mMutexPoseAccess 保护 mvpPoseObservers
    for(IPoseObserver* pObserver : mvpPoseObservers)
    {
        if(pObserver)
        {
            // 调用新的、专门用于实时位姿的方法
            pObserver->OnRealTimePoseUpdated(T_custom_c, mCurrentFrame.mTimeStamp);
        }
    }
}
// ...
```

### 7.4. 修改 `LocalMapping.h`

1.  **包含新的头文件**:
    与 `Tracking.h` 类似，在 `LocalMapping.h` 的顶部添加 `#include` 指令：
    ```cpp
    // In /home/zyh/Desktop/ORB-SLAM3_byZyh/include/LocalMapping.h
    // ... (其他 #includes)
    #include "IPoseObserver.h" // <--- 包含新的接口定义

    // 如果 LocalMapping.h 已经包含了 Tracking.h, 且 Tracking.h 包含了 IPoseObserver.h,
    // 理论上这里可以不显式包含。但为了代码清晰和减少隐式依赖，建议显式包含。

    namespace ORB_SLAM3
    {
    // ... (class LocalMapping 定义)
    }
    ```

### 7.5. 修改 `LocalMapping.cc`

在 `LocalMapping::NotifyOptimizedPose()` 函数（或您用于通知优化后位姿的等效函数）中，修改对观察者回调的调用，使其调用 `OnOptimizedPoseUpdated`：

```cpp
// In /home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc
// Inside LocalMapping::NotifyOptimizedPose() (or similar function):

// ... (获取到优化后的 pKF 和转换后的 T_custom_c) ...

// 通知所有观察者 (使用新的接口方法)
// 假设 mpTracker 是 LocalMapping 持有的 Tracking 对象指针
if (mpTracker && !mpTracker->mvpPoseObservers.empty()) {
    std::unique_lock<std::mutex> lock_observers(mpTracker->mMutexPoseAccess); // 使用 Tracking 的互斥锁
    for (IPoseObserver* pObserver : mpTracker->mvpPoseObservers)
    {
        if (pObserver)
        {
            // 调用新的、专门用于优化位姿的方法
            pObserver->OnOptimizedPoseUpdated(T_custom_c, pKF->mTimeStamp);
        }
    }
}
// ...
```

### 7.6. 修改具体的观察者实现类

任何实现了 `IPoseObserver` 接口的类（例如，您在 `Examples/Monocular/mono_euroc.cc` 中可能有的 `TUMFormatTrajectoryRecorder` 或类似的类）都需要更新以匹配新的接口定义。这意味着它们现在必须实现 `OnRealTimePoseUpdated` 和 `OnOptimizedPoseUpdated` 这两个纯虚函数。

**示例：修改 `TUMFormatTrajectoryRecorder` (假设存在这样的类)**

```cpp
// In your example file, e.g., /home/zyh/Desktop/ORB-SLAM3_byZyh/Examples/Monocular/mono_euroc.cc
// or a dedicated observer implementation file.

#include "IPoseObserver.h" // <--- 确保包含新的接口头文件
#include <fstream>        // For std::ofstream
#include <iomanip>        // For std::fixed, std::setprecision

// 假设您有一个这样的类
class TUMFormatTrajectoryRecorder : public ORB_SLAM3::IPoseObserver {
public:
    TUMFormatTrajectoryRecorder(const std::string& realTimeFile, const std::string& optimizedFile) {
        // 打开用于记录实时位姿的文件
        mRealTimePoseFile.open(realTimeFile.c_str());
        if (!mRealTimePoseFile.is_open()) {
            std::cerr << "Error: Could not open real-time pose file: " << realTimeFile << std::endl;
        } else {
            mRealTimePoseFile << std::fixed << std::setprecision(6); // 设置输出格式
        }

        // 打开用于记录优化位姿的文件
        mOptimizedPoseFile.open(optimizedFile.c_str());
        if (!mOptimizedPoseFile.is_open()) {
            std::cerr << "Error: Could not open optimized pose file: " << optimizedFile << std::endl;
        } else {
            mOptimizedPoseFile << std::fixed << std::setprecision(6); // 设置输出格式
        }
    }

    ~TUMFormatTrajectoryRecorder() {
        if (mRealTimePoseFile.is_open()) {
            mRealTimePoseFile.close();
        }
        if (mOptimizedPoseFile.is_open()) {
            mOptimizedPoseFile.close();
        }
    }

    // 实现来自 Tracking 的实时位姿回调
    void OnRealTimePoseUpdated(const Sophus::SE3f& T_custom_c, double timestamp) override {
        if (mRealTimePoseFile.is_open()) {
            // 将 Sophus::SE3f 转换为 TUM 格式: timestamp tx ty tz qx qy qz qw
            Eigen::Vector3f translation = T_custom_c.translation();
            Eigen::Quaternionf quaternion = T_custom_c.unit_quaternion();

            mRealTimePoseFile << timestamp << " "
                              << translation.x() << " " << translation.y() << " " << translation.z() << " "
                              << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w()
                              << std::endl;
        }
    }

    // 实现来自 LocalMapping 的优化位姿回调
    void OnOptimizedPoseUpdated(const Sophus::SE3f& T_custom_c, double timestamp) override {
        if (mOptimizedPoseFile.is_open()) {
            // 将 Sophus::SE3f 转换为 TUM 格式
            Eigen::Vector3f translation = T_custom_c.translation();
            Eigen::Quaternionf quaternion = T_custom_c.unit_quaternion();

            mOptimizedPoseFile << timestamp << " "
                               << translation.x() << " " << translation.y() << " " << translation.z() << " "
                               << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w()
                               << std::endl;
        }
    }

private:
    std::ofstream mRealTimePoseFile;
    std::ofstream mOptimizedPoseFile;
};
```
在您的主程序（如 `mono_euroc.cc`）中，创建并注册这个观察者实例：
```cpp
// // In main function of mono_euroc.cc (or similar)
ORB_SLAM3::System SLAM(...);
TUMFormatTrajectoryRecorder* trajectoryRecorder = new TUMFormatTrajectoryRecorder("realtime_poses.txt", "optimized_poses.txt");
SLAM.GetTracker()->RegisterPoseObserver(trajectoryRecorder);
// ... rest of the main function ...
```

### 7.7. 编译与测试

完成上述所有代码修改后：

1.  **重新编译**整个 ORB-SLAM3 项目。
    ```bash
cd /home/zyh/Desktop/ORB-SLAM3_byZyh
mkdir -p build # if not exists
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release # Or Debug
make -j$(nproc)
```
2.  **仔细测试**：运行您的 SLAM 系统，检查是否仍然能够正确地发布和接收两种来源的位姿，并且您的观察者类（如 `TUMFormatTrajectoryRecorder`）是否按预期工作，将数据写入相应的文件。
3.  **检查编译器输出**: 留意编译过程中的任何警告或错误，并根据提示进行修正。由于接口发生了变化，这是非常关键的一步。

通过这些步骤，您的代码将更加整洁和模块化。🎉

如前所述（章节 5.2 和 6.2.2 的注释中），如果您的应用需要明确区分从 `Tracking` 线程获取的实时位姿和从 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 获取的优化位姿，您应该调整 `IPoseObserver` 接口。例如：

-  **方案1: 修改 `OnRealTimePoseUpdated`**
    ```cpp
// In IPoseObserver.h
enum class PoseSource { FROM_TRACKING, FROM_LOCALMAPPING };
virtual void OnRealTimePoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp, PoseSource source) = 0;
```
    调用时：
    *   `Tracking.cc`: `pObserver->OnRealTimePoseUpdated(T_custom_c, mCurrentFrame.mTimeStamp, PoseSource::FROM_TRACKING);`
    *   [LocalMapping.cc](cci:7://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:0:0-0:0): `pObserver->OnRealTimePoseUpdated(T_custom_c, pKF->mTimeStamp, PoseSource::FROM_LOCALMAPPING);`

-  **方案2: 新增接口方法**
    ```cpp
// In IPoseObserver.h
virtual void OnRealTimePoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) = 0;
virtual void OnOptimizedPoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp) = 0;
```
-  **方案3: 使用单一接口方法**
    ```cpp
// In IPoseObserver.h
virtual void OnPoseUpdated(const Sophus::SE3f& T_custom_world_camera, double timestamp, PoseSource source) = 0;
```
    调用时：
    *   `Tracking.cc`: `pObserver->OnPoseUpdated(T_custom_c, mCurrentFrame.mTimeStamp, PoseSource::FROM_TRACKING);`
    *   [LocalMapping.cc](cci:7://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:0:0-0:0): `pObserver->OnPoseUpdated(T_custom_c, pKF->mTimeStamp, PoseSource::FROM_LOCALMAPPING);`

选择哪种方案取决于您的偏好和现有代码的兼容性。

### 6.3. 预期效果与总结

通过上述修改思路，[LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1) 线程将在其完成对关键帧（特别是 `mpCurrentKeyFrame`）的位姿进行局部BA优化后，获取该优化位姿，将其转换到您指定的自定义世界坐标系，并利用您在 `Tracking` 线程中已建立的观察者模式，将此高精度位姿通知给所有感兴趣的模块。

这使得您的系统不仅能获得实时的相机位姿（来自 `Tracking`），也能在稍有延迟后获得经过更精细优化的位姿（来自 [LocalMapping](cci:1://file:///home/zyh/Desktop/ORB-SLAM3_byZyh/src/LocalMapping.cc:32:0-70:1)），为不同需求的下游应用提供了灵活的数据源。

**请注意**: 上述代码修改是概念性的，旨在阐述原理和方法。在实际集成到 ORB-SLAM3 源码时，需要仔细处理头文件包含、命名空间、具体的变量名以及与现有代码的融合。

---