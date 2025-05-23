#!/bin/bash
# 快速处理最新数据集的脚本

echo "=== EuRoC数据集快速处理工具 ==="

# 查找最新的数据集目录
DATASET_BASE="Datasets/Mydataset"
if [ -d "$DATASET_BASE" ]; then
    # 按修改时间排序，找到最新的目录
    LATEST_DIR=$(find "$DATASET_BASE" -maxdepth 1 -type d -name "*_desktop" -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
    
    if [ -n "$LATEST_DIR" ]; then
        echo "找到最新数据集: $LATEST_DIR"
        
        # 检查是否有图片文件
        IMG_COUNT=$(find "$LATEST_DIR/mav0/cam0/data" -name "*.png" 2>/dev/null | wc -l)
        if [ $IMG_COUNT -gt 0 ]; then
            echo "发现 $IMG_COUNT 张图片"
            echo "开始处理..."
            
            # 查找相机配置文件
            CONFIG_FILE=""
            if [ -f "$LATEST_DIR/mav0/cam0/Basler.yaml" ]; then
                CONFIG_FILE="$LATEST_DIR/mav0/cam0/Basler.yaml"
            elif [ -f "Datasets/Mydataset/5_22_desktop/mav0/cam0/Basler.yaml" ]; then
                CONFIG_FILE="Datasets/Mydataset/5_22_desktop/mav0/cam0/Basler.yaml"
            fi
            
            # 运行处理脚本
            if [ -n "$CONFIG_FILE" ]; then
                python3 Scripts/process_euroc_dataset.py "$LATEST_DIR" --config "$CONFIG_FILE"
            else
                python3 Scripts/process_euroc_dataset.py "$LATEST_DIR"
            fi
        else
            echo "未在 $LATEST_DIR 中找到图片文件"
        fi
    else
        echo "未找到数据集目录"
    fi
else
    echo "数据集基础目录不存在: $DATASET_BASE"
fi 