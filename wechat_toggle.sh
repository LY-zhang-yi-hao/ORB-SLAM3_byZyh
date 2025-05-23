#!/bin/bash

# 设置错误处理
set -e

# 添加调试模式（可选）
DEBUG=false

debug_print() {
    if [ "$DEBUG" = true ]; then
        echo "[DEBUG] $1"
    fi
}

# 获取微信窗口的所有ID，存储到数组中
debug_print "正在搜索微信窗口..."

# 使用更宽松的搜索条件，避免某些X11相关错误
wechat_search_result=$(xdotool search --name '微信' 2>/dev/null || true)

if [ -z "$wechat_search_result" ]; then
    echo "未找到微信窗口"
    exit 1
fi

# 过滤出真正的微信应用窗口（排除浏览器）
declare -a real_wechat_ids
while read -r id; do
    if [ -n "$id" ]; then
        window_name=$(xdotool getwindowname "$id" 2>/dev/null || echo "")
        debug_print "检查窗口ID $id: $window_name"
        
        # 排除包含浏览器标识的窗口
        if [[ ! "$window_name" =~ (Microsoft.*Edge|Chrome|Firefox|Safari|Browser) ]]; then
            # 如果窗口名称就是"微信"或包含微信但不包含浏览器相关词汇
            if [[ "$window_name" == "微信" ]] || [[ "$window_name" =~ 微信 && ! "$window_name" =~ (博客|网页|浏览器) ]]; then
                real_wechat_ids+=("$id")
                debug_print "找到真正的微信窗口: $id - $window_name"
            fi
        fi
    fi
done <<< "$wechat_search_result"

# 如果没有找到真正的微信窗口
if [ ${#real_wechat_ids[@]} -eq 0 ]; then
    echo "未找到微信桌面应用窗口"
    exit 1
fi

debug_print "找到 ${#real_wechat_ids[@]} 个真正的微信窗口"
for id in "${real_wechat_ids[@]}"; do
    debug_print "微信窗口ID: $id"
done

# 获取当前活动窗口的ID
debug_print "获取当前活动窗口..."
current_window_id=$(xdotool getactivewindow 2>/dev/null || echo "")

if [ -z "$current_window_id" ]; then
    echo "无法获取当前活动窗口，直接激活微信窗口"
    # 尝试多种方法来显示窗口
    for wechat_id in "${real_wechat_ids[@]}"; do
        debug_print "显示窗口ID: $wechat_id"
        xdotool windowmap "$wechat_id" 2>/dev/null || true
        xdotool windowraise "$wechat_id" 2>/dev/null || true
        xdotool windowfocus "$wechat_id" 2>/dev/null || true
        xdotool windowactivate "$wechat_id" 2>/dev/null || true
    done
    echo "微信窗口已激活"
    exit 0
fi

debug_print "当前活动窗口ID: $current_window_id"

# 标记是否找到匹配的窗口
found_match=false

# 遍历微信窗口ID数组
for wechat_id in "${real_wechat_ids[@]}"; do
    debug_print "比较: $wechat_id vs $current_window_id"
    if [ "$wechat_id" = "$current_window_id" ]; then
        found_match=true
        debug_print "找到匹配的窗口"
        break
    fi
done

# 根据匹配结果执行对应操作
if [ "$found_match" = true ]; then
    # 如果当前窗口是微信窗口，则最小化所有微信窗口
    debug_print "最小化微信窗口..."
    for wechat_id in "${real_wechat_ids[@]}"; do
        xdotool windowminimize "$wechat_id" 2>/dev/null || true
        debug_print "最小化窗口ID: $wechat_id"
    done
    echo "微信窗口已最小化"
else
    # 如果当前窗口不是微信窗口，则激活第一个微信窗口
    debug_print "激活微信窗口..."
    target_id="${real_wechat_ids[0]}"
    debug_print "目标窗口ID: $target_id"
    
    # 使用多种方法确保窗口显示
    debug_print "尝试取消最小化..."
    xdotool windowmap "$target_id" 2>/dev/null || true
    
    debug_print "尝试移动到当前桌面..."
    # 获取当前桌面编号
    current_desktop=$(xdotool get_desktop 2>/dev/null || echo "0")
    xdotool set_desktop_for_window "$target_id" "$current_desktop" 2>/dev/null || true
    
    debug_print "尝试将窗口置于前台..."
    xdotool windowraise "$target_id" 2>/dev/null || true
    
    debug_print "尝试设置焦点..."
    xdotool windowfocus "$target_id" 2>/dev/null || true
    
    debug_print "尝试激活窗口..."
    xdotool windowactivate "$target_id" 2>/dev/null || true
    
    # 额外等待一下
    sleep 0.1
    
    # 再次尝试激活
    debug_print "再次尝试激活..."
    xdotool windowactivate "$target_id" 2>/dev/null || true
    
    # 最后尝试强制显示窗口（移动到屏幕可见位置）
    debug_print "强制移动窗口到可见位置..."
    
    # 首先尝试还原窗口（如果被最大化或最小化）
    xdotool key --window "$target_id" "alt+F5" 2>/dev/null || true
    sleep 0.2
    
    # 获取屏幕分辨率
    screen_info=$(xdpyinfo | grep dimensions | awk '{print $2}' 2>/dev/null || echo "1920x1080")
    screen_width=$(echo $screen_info | cut -d'x' -f1)
    screen_height=$(echo $screen_info | cut -d'x' -f2)
    
    debug_print "屏幕分辨率: ${screen_width}x${screen_height}"
    
    # 设置合理的窗口大小（屏幕的70%）
    window_width=$((screen_width * 70 / 100))
    window_height=$((screen_height * 70 / 100))
    
    debug_print "设置窗口大小为: ${window_width}x${window_height}"
    
    # 强制移动和调整窗口
    xdotool windowmove "$target_id" 50 50 2>/dev/null || true
    sleep 0.1
    xdotool windowsize "$target_id" "$window_width" "$window_height" 2>/dev/null || true
    sleep 0.1
    
    # 多次尝试确保窗口在前台
    for i in {1..3}; do
        xdotool windowraise "$target_id" 2>/dev/null || true
        xdotool windowactivate "$target_id" 2>/dev/null || true
        sleep 0.1
    done
    
    # 尝试使用wmctrl作为备选方案
    wmctrl -i -a "$target_id" 2>/dev/null || true
    
    echo "微信窗口已激活"
fi 