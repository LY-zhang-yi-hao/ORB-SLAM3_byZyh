#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EuRoC数据集格式处理脚本
用于将采集的图片处理成标准的EuRoC数据集格式

功能:
1. 扫描指定目录下的图片文件
2. 根据文件名提取时间戳信息
3. 生成标准的EuRoC格式data.csv文件
4. 支持图片重命名和排序
5. 数据质量检查和验证

使用方法:
python process_euroc_dataset.py <数据集路径>
"""

import os
import sys
import cv2
import argparse
from pathlib import Path
import shutil

def print_banner():
    """打印脚本横幅"""
    print("=" * 60)
    print("           EuRoC数据集格式处理工具")
    print("           ORB-SLAM3 数据预处理脚本")
    print("=" * 60)

def check_directory_structure(dataset_path):
    """检查和创建EuRoC目录结构"""
    print(f"\n📁 检查目录结构: {dataset_path}")
    
    # 期望的EuRoC目录结构
    mav0_path = os.path.join(dataset_path, "mav0")
    cam0_path = os.path.join(mav0_path, "cam0")
    data_path = os.path.join(cam0_path, "data")
    
    # 创建必要的目录
    os.makedirs(data_path, exist_ok=True)
    
    print(f"✅ EuRoC目录结构就绪: {data_path}")
    return mav0_path, cam0_path, data_path

def scan_images(data_path):
    """扫描并分析图片文件"""
    print(f"\n🔍 扫描图片文件: {data_path}")
    
    image_files = []
    supported_formats = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']
    
    for filename in os.listdir(data_path):
        file_ext = os.path.splitext(filename.lower())[1]
        if file_ext in supported_formats:
            filepath = os.path.join(data_path, filename)
            # 尝试读取图片验证
            img = cv2.imread(filepath)
            if img is not None:
                image_files.append(filename)
            else:
                print(f"⚠️ 无法读取图片: {filename}")
    
    print(f"📊 找到 {len(image_files)} 个有效图片文件")
    return image_files

def extract_timestamps_from_filenames(image_files):
    """从文件名提取时间戳"""
    print(f"\n⏰ 分析时间戳信息...")
    
    timestamp_data = []
    failed_files = []
    
    for filename in image_files:
        try:
            # 假设文件名格式为: timestamp.png (纳秒时间戳)
            name_without_ext = os.path.splitext(filename)[0]
            timestamp_ns = int(name_without_ext)
            timestamp_data.append({
                'filename': filename,
                'timestamp_ns': timestamp_ns,
                'timestamp_s': timestamp_ns / 1e9
            })
        except ValueError:
            failed_files.append(filename)
            print(f"⚠️ 无法从文件名提取时间戳: {filename}")
    
    if failed_files:
        print(f"❌ {len(failed_files)} 个文件名格式不正确")
        print("提示: 文件名应为纳秒时间戳格式，如: 1621583823123456789.png")
    
    # 按时间戳排序
    timestamp_data.sort(key=lambda x: x['timestamp_ns'])
    
    if timestamp_data:
        duration = (timestamp_data[-1]['timestamp_s'] - timestamp_data[0]['timestamp_s'])
        print(f"✅ 成功处理 {len(timestamp_data)} 个文件")
        print(f"📈 时间跨度: {duration:.2f} 秒")
        print(f"🕐 开始时间: {timestamp_data[0]['timestamp_ns']}")
        print(f"🕐 结束时间: {timestamp_data[-1]['timestamp_ns']}")
    
    return timestamp_data, failed_files

def generate_data_csv(timestamp_data, output_path):
    """生成EuRoC格式的data.csv文件"""
    print(f"\n📝 生成data.csv文件: {output_path}")
    
    try:
        with open(output_path, 'w') as f:
            # EuRoC格式的CSV文件头部（注释掉，因为ORB-SLAM3读取时会有问题）
            # f.write("#timestamp [ns],filename\n")
            
            for data in timestamp_data:
                f.write(f"{data['timestamp_ns']},{data['filename']}\n")
        
        print(f"✅ 成功生成data.csv，包含 {len(timestamp_data)} 条记录")
        return True
    except Exception as e:
        print(f"❌ 生成data.csv失败: {e}")
        return False

def generate_timestamp_txt(timestamp_data, dataset_name):
    """生成外部时间戳txt文件（与grab_data.py格式一致）"""
    # 创建时间戳文件保存目录
    timestamps_dir = "Examples/Monocular/Basler_TimeStamps"
    os.makedirs(timestamps_dir, exist_ok=True)
    
    # 生成时间戳文件路径
    timestamp_file_path = os.path.join(timestamps_dir, f"{dataset_name}.txt")
    
    print(f"\n📝 生成时间戳文件: {timestamp_file_path}")
    
    try:
        with open(timestamp_file_path, 'w') as f:
            for data in timestamp_data:
                f.write(f"{data['timestamp_ns']}\n")
        
        print(f"✅ 成功生成时间戳文件，包含 {len(timestamp_data)} 条记录")
        return timestamp_file_path
    except Exception as e:
        print(f"❌ 生成时间戳文件失败: {e}")
        return None

def validate_dataset(data_path, csv_path):
    """验证数据集完整性"""
    print(f"\n🔍 验证数据集完整性...")
    
    # 检查CSV文件
    if not os.path.exists(csv_path):
        print("❌ data.csv文件不存在")
        return False
    
    # 读取CSV文件
    csv_files = []
    try:
        with open(csv_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    parts = line.split(',')
                    if len(parts) >= 2:
                        csv_files.append(parts[1])
    except Exception as e:
        print(f"❌ 读取data.csv失败: {e}")
        return False
    
    # 检查文件是否存在
    missing_files = []
    for filename in csv_files:
        filepath = os.path.join(data_path, filename)
        if not os.path.exists(filepath):
            missing_files.append(filename)
    
    if missing_files:
        print(f"❌ 发现 {len(missing_files)} 个缺失文件:")
        for filename in missing_files[:5]:  # 只显示前5个
            print(f"   - {filename}")
        if len(missing_files) > 5:
            print(f"   ... 还有 {len(missing_files) - 5} 个文件")
        return False
    
    print(f"✅ 数据集验证通过，共 {len(csv_files)} 个文件")
    return True

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='EuRoC数据集格式处理工具')
    parser.add_argument('dataset_path', nargs='?', help='数据集路径 (例如: Datasets/Mydataset/5_23_desktop)')
    parser.add_argument('--validate-only', action='store_true', help='仅验证数据集，不处理')
    
    args = parser.parse_args()
    
    print_banner()
    
    # 获取数据集路径
    if args.dataset_path:
        dataset_path = args.dataset_path
    else:
        # 交互式输入
        print("请输入数据集路径 (例如: Datasets/Mydataset/5_23_desktop):")
        dataset_path = input(">>> ").strip()
    
    if not dataset_path:
        print("❌ 未指定数据集路径")
        sys.exit(1)
    
    # 检查路径是否存在
    if not os.path.exists(dataset_path):
        print(f"❌ 路径不存在: {dataset_path}")
        sys.exit(1)
    
    try:
        # 检查目录结构
        mav0_path, cam0_path, data_path = check_directory_structure(dataset_path)
        
        # CSV文件路径
        csv_path = os.path.join(cam0_path, "data.csv")
        
        # 如果仅验证模式
        if args.validate_only:
            success = validate_dataset(data_path, csv_path)
            sys.exit(0 if success else 1)
        
        # 扫描图片文件
        image_files = scan_images(data_path)
        if not image_files:
            print("❌ 未找到图片文件")
            sys.exit(1)
        
        # 提取时间戳
        timestamp_data, failed_files = extract_timestamps_from_filenames(image_files)
        if not timestamp_data:
            print("❌ 没有有效的时间戳数据")
            sys.exit(1)
        
        # 用户确认
        print(f"\n📋 处理摘要:")
        print(f"   - 有效图片: {len(timestamp_data)} 个")
        print(f"   - 失败文件: {len(failed_files)} 个")
        print(f"   - 输出路径: {csv_path}")
        
        confirm = input("\n是否继续生成data.csv文件? (yes/no): ").strip().lower()
        if confirm != 'yes':
            print("操作已取消")
            sys.exit(0)
        
        # 生成CSV文件
        if generate_data_csv(timestamp_data, csv_path):
            # 生成时间戳txt文件
            dataset_name = os.path.basename(os.path.normpath(dataset_path))
            timestamp_file_path = generate_timestamp_txt(timestamp_data, dataset_name)
            
            # 验证生成的数据集
            if validate_dataset(data_path, csv_path):
                print(f"\n🎉 EuRoC数据集处理完成!")
                print(f"📁 数据位置: {data_path}")
                print(f"📄 CSV文件: {csv_path}")
                if timestamp_file_path:
                    print(f"⏰ 时间戳文件: {timestamp_file_path}")
                
            else:
                print("❌ 数据集验证失败")
                sys.exit(1)
        else:
            print("❌ 生成CSV文件失败")
            sys.exit(1)
            
    except Exception as e:
        print(f"❌ 处理过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main() 