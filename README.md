# ORB-SLAM3 超详细注释

![DEMO](https://github.com/electech6/ORB_SLAM3_detailed_comments/blob/master/demo.gif)



----
# ORB-SLAM3

### V1.0, December 22th, 2021
**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

The [Changelog](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) describes the features of each version.

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 

We provide examples to run ORB-SLAM3 in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular, with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular, with or without IMU. Videos of some example executions can be found at [ORB-SLAM3 channel](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q).

This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a>

### Related Publications:

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM3 in an academic work, please cite:
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04** and **18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

## ROS (optional)

We provide some examples to process input of a monocular, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these examples is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

# 3. Building ORB-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

# 4. Running ORB-SLAM3 with your camera

Directory `Examples` contains several demo programs and calibration files to run ORB-SLAM3 in all sensor configurations with Intel Realsense cameras T265 and D435i. The steps needed to use your own camera are: 

1. Calibrate your camera following `Calibration_Tutorial.pdf` and write your calibration file `your_camera.yaml`

2. Modify one of the provided demos to suit your specific camera model, and build it

3. Connect the camera to your computer using USB3 or the appropriate interface

4. Run ORB-SLAM3. For example, for our D435i camera, we would execute:

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

# 5. EuRoC Examples
[EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) was recorded with two pinhole cameras and an inertial sensor. We provide an example script to launch EuRoC sequences in all the sensor configurations.

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Open the script "euroc_examples.sh" in the root of the project. Change **pathDatasetEuroc** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./euroc_examples
```

## Evaluation
EuRoC provides ground truth for each sequence in the IMU body reference. As pure visual executions report trajectories centered in the left camera, we provide in the "evaluation" folder the transformation of the ground truth to the left camera reference. Visual-inertial trajectories use the ground truth from the dataset.

Execute the following script to process sequences and compute the RMS ATE:
```
./euroc_eval_examples
```

# 6. TUM-VI Examples
[TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) was recorded with two fisheye cameras and an inertial sensor.

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.

2. Open the script "tum_vi_examples.sh" in the root of the project. Change **pathDatasetTUM_VI** variable to point to the directory where the dataset has been uncompressed. 

3. Execute the following script to process all the sequences with all sensor configurations:
```
./tum_vi_examples
```

## Evaluation
In TUM-VI ground truth is only available in the room where all sequences start and end. As a result the error measures the drift at the end of the sequence. 

Execute the following script to process sequences and compute the RMS ATE:
```
./tum_vi_eval_examples
```

# 7. ROS Examples

### Building the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D
Tested with ROS Melodic and ubuntu 18.04.

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
  ```
  gedit ~/.bashrc
  ```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### Running Monocular-Inertial Node
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]	
  ```

### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary file and a settings file. For Pinhole camera model, if you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**. For FishEye camera model, rectification is not required since system works with original images:

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```

### Running Stereo-Inertial Node
For a stereo input from topics `/camera/left/image_raw` and `/camera/right/image_raw`, and an inertial input from topic `/imu`, run node ORB_SLAM3/Stereo_Inertial. You will need to provide the vocabulary file and a settings file, including rectification matrices if required in a similar way to Stereo case:

  ```
  rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]	
  ```
  
### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

**Running ROS example:** Download a rosbag (e.g. V1_02_medium.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
  ```
  
Once ORB-SLAM3 has loaded the vocabulary, press space in the rosbag tab.

**Remark:** For rosbags from TUM-VI dataset, some play issue may appear due to chunk size. One possible solution is to rebag them with the default chunk size, for example:
  ```
  rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
  ```

# 8. Running time analysis
A flag in `include\Config.h` activates time measurements. It is necessary to uncomment the line `#define REGISTER_TIMES` to obtain the time stats of one execution which is shown at the terminal and stored in a text file(`ExecTimeMean.txt`).

# 9. Calibration
You can find a tutorial for visual-inertial calibration and a detailed description of the contents of valid configuration files at  `Calibration_Tutorial.pdf`

# SLAM轨迹可视化工具

这个工具包用于可视化SLAM系统输出的轨迹数据，支持完整轨迹和关键帧轨迹的对比分析。

## 文件说明

### 数据文件
- `f_dataset-Basler_mono_final_23.txt`: 完整轨迹数据（977个数据点）
- `kf_dataset-Basler_mono_final_23.txt`: 关键帧轨迹数据（34个数据点）

### 脚本文件
- `visualize_trajectory_enhanced.py`: 增强版轨迹可视化脚本（推荐使用）
- `trajectory_info.py`: 轨迹数据信息查看器
- `requirements.txt`: Python依赖包列表

### 输出文件夹结构
运行脚本后会自动创建以下文件夹结构：
```
trajectory_visualization_output/
├── plots/                          # 生成的可视化图表
│   ├── trajectory_2d_enhanced.png
│   ├── trajectory_3d_enhanced.png
│   └── motion_analysis.png
└── data/                           # 轨迹数据文件备份
    ├── f_dataset-Basler_mono_final_23.txt
    └── kf_dataset-Basler_mono_final_23.txt
```

## 数据格式

每行数据格式为：
```
时间戳 x y z qx qy qz qw
```

其中：
- `时间戳`: 纳秒级时间戳
- `x, y, z`: 位置坐标（米）
- `qx, qy, qz, qw`: 四元数表示的旋转

## 安装依赖

```bash
pip install -r requirements.txt
```

或者手动安装：
```bash
pip install numpy matplotlib
```

## 使用方法

### 快速查看轨迹信息
```bash
python3 trajectory_info.py
```

### 生成可视化图表（推荐）
```bash
python3 visualize_trajectory_enhanced.py
```

生成的图片将保存在 `trajectory_visualization_output/plots/` 目录中：
- `trajectory_2d_enhanced.png`: 增强版2D轨迹对比图
- `trajectory_3d_enhanced.png`: 增强版3D轨迹图（包含时间进展颜色映射）
- `motion_analysis.png`: 运动分析图表

## 字号调节功能

代码中已添加详细的字号调节注释，您可以根据需要修改以下字号：

### 2D轨迹图 (plot_trajectory_2d_enhanced函数)
- **主标题**: `fontsize=16` → 可改为18, 20等
- **子图标题**: `fontsize=14` → 可改为12, 16等  
- **坐标轴标签**: `fontsize=12` → 可改为10, 14等
- **图例**: `fontsize=10` → 可改为8, 12等
- **统计信息文字**: `fontsize=11` → 可改为9, 13等（已调大2号）

### 3D轨迹图 (plot_trajectory_3d_enhanced函数)
- **标题**: `fontsize=14` → 可改为12, 16等
- **坐标轴标签**: `fontsize=12` → 可改为10, 14等
- **图例**: `fontsize=10` → 可改为8, 12等
- **颜色条标签**: `fontsize=10` → 可改为8, 12等

### 运动分析图 (plot_motion_analysis函数)
- **主标题**: `fontsize=16` → 可改为14, 18等
- **子图标题**: `fontsize=14` → 可改为12, 16等
- **坐标轴标签**: `fontsize=12` → 可改为10, 14等
- **图例**: `fontsize=9/10` → 可改为8, 11等

## 功能特性

### 基础功能
- 2D多视角轨迹对比（XY、XZ、YZ平面）
- 3D轨迹可视化
- 轨迹统计信息（长度、范围等）
- 坐标随时间变化分析

### 增强功能
- 起点和终点标记
- 时间进展颜色映射
- 详细的运动统计分析
- 步长分析
- 轨迹密度热图
- 高度变化分析
- 自动文件夹管理
- 字体处理优化

## 输出图表说明

### 2D轨迹对比图
- **XY平面（俯视图）**: 从上往下看的轨迹投影
- **XZ平面（侧视图）**: 从侧面看的轨迹投影
- **YZ平面（正视图）**: 从正面看的轨迹投影
- **统计信息**: 轨迹长度、数据点数、坐标范围等（右下角，字号已调大）

### 3D轨迹图
- 完整的三维轨迹可视化
- 起点（绿色方块）和终点（红色三角）标记
- 时间进展颜色映射

### 运动分析图
- **位置vs时间**: 各坐标轴随时间的变化
- **步长分析**: 相邻点之间的距离分布
- **轨迹密度**: XY平面的轨迹密度热图
- **高度变化**: Z坐标随时间的变化

## 轨迹统计信息

脚本会自动计算并显示以下统计信息：
- 数据点数量
- 轨迹总长度（路径长度）
- 总位移（起点到终点的直线距离）
- 平均步长和最大步长
- 各坐标轴的范围

## 注意事项

1. 脚本会自动创建输出文件夹
2. 轨迹数据会备份到data文件夹中
3. 如果系统没有中文字体，图表标签会自动切换为英文
4. 生成的图片为高分辨率PNG格式（300 DPI）
5. 脚本会自动处理坐标轴比例，确保轨迹显示不变形

## 故障排除

### 常见问题
1. **文件未找到错误**: 确保数据文件在正确位置
2. **中文字体警告**: 这不影响功能，只是显示效果
3. **内存不足**: 对于大型数据集，可能需要调整采样率

### 性能优化
- 对于超大数据集，可以修改脚本中的采样间隔
- 可以选择只生成特定类型的图表来节省时间

## 扩展功能

脚本设计为模块化，可以轻松添加新的分析功能：
- 速度和加速度分析
- 轨迹平滑处理
- 误差分析
- 与真值轨迹的对比

## 示例输出

运行脚本后，会在控制台看到类似输出：
```
Loading trajectory data...
Full trajectory data points: 977
Keyframe data points: 34
Trajectory files copied to: trajectory_visualization_output/data
Generating enhanced 2D trajectory plots...
Generating enhanced 3D trajectory plot...
Generating motion analysis plots...
Saving plots...
Plots saved to:
- trajectory_visualization_output/plots/trajectory_2d_enhanced.png: Enhanced 2D trajectory comparison
- trajectory_visualization_output/plots/trajectory_3d_enhanced.png: Enhanced 3D trajectory with time progression
- trajectory_visualization_output/plots/motion_analysis.png: Motion analysis plots

All outputs saved in: trajectory_visualization_output/
```
