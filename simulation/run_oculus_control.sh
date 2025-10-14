#!/bin/bash
# Franka Oculus 遥操作控制 - 快速启动脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=====================================================================${NC}"
echo -e "${BLUE}        🤖 Franka Oculus 遥操作控制系统 - 快速启动        ${NC}"
echo -e "${BLUE}=====================================================================${NC}"

# 进入正确的目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "\n${GREEN}📁 当前目录: ${SCRIPT_DIR}${NC}"

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ 错误: 未找到 python3${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Python: $(python3 --version)${NC}"

# 检查依赖
echo -e "\n${YELLOW}🔍 检查依赖...${NC}"

DEPS=("numpy" "scipy" "pyyaml")
MISSING_DEPS=()

for dep in "${DEPS[@]}"; do
    if ! python3 -c "import $dep" 2>/dev/null; then
        MISSING_DEPS+=("$dep")
    fi
done

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${YELLOW}⚠️  缺少以下依赖: ${MISSING_DEPS[*]}${NC}"
    echo -e "${YELLOW}正在安装...${NC}"
    pip install "${MISSING_DEPS[@]}"
fi

echo -e "${GREEN}✅ 所有依赖已安装${NC}"

# 检查设备连接
echo -e "\n${YELLOW}🔍 检查设备连接...${NC}"

# 检查 Oculus
if command -v adb &> /dev/null; then
    OCULUS_CONNECTED=$(adb devices | grep -v "List" | grep "device" | wc -l)
    if [ "$OCULUS_CONNECTED" -gt 0 ]; then
        echo -e "${GREEN}✅ Oculus 已连接${NC}"
        adb devices | grep "device" | grep -v "List"
    else
        echo -e "${YELLOW}⚠️  未检测到 Oculus 连接${NC}"
        echo -e "${YELLOW}   请确保 Oculus 通过 USB 或 WiFi 连接${NC}"
        echo -e "${YELLOW}   运行: adb connect <oculus-ip>:5555${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  未安装 adb，无法检查 Oculus 连接${NC}"
fi

# 显示菜单
echo -e "\n${BLUE}=====================================================================${NC}"
echo -e "${BLUE}请选择控制模式:${NC}"
echo -e "  ${GREEN}1${NC}. 相对控制模式 (推荐) - 增量控制，更安全"
echo -e "  ${GREEN}2${NC}. 绝对控制模式 - 直接位姿映射"
echo -e "  ${GREEN}3${NC}. 关节角度控制模式 - 通过逆运动学"
echo -e "  ${GREEN}4${NC}. 自定义参数启动"
echo -e "  ${GREEN}5${NC}. 仅测试（不启动控制）"
echo -e "  ${GREEN}0${NC}. 退出"
echo -e "${BLUE}=====================================================================${NC}"

read -p "请输入选项 [1-5, 0]: " choice

case $choice in
    1)
        echo -e "\n${GREEN}🚀 启动相对控制模式 (30Hz)${NC}"
        python3 franka_oculus.py --mode relative --frequency 30
        ;;
    2)
        echo -e "\n${GREEN}🚀 启动绝对控制模式 (30Hz)${NC}"
        python3 franka_oculus.py --mode absolute --frequency 30
        ;;
    3)
        echo -e "\n${GREEN}🚀 启动关节角度控制模式 (30Hz)${NC}"
        python3 franka_oculus.py --mode joint --frequency 30
        ;;
    4)
        echo -e "\n${YELLOW}📝 自定义参数${NC}"
        echo -e "控制模式 [relative/absolute/joint]:"
        read -p "> " mode
        mode=${mode:-relative}

        echo -e "控制频率 (Hz) [默认: 30]:"
        read -p "> " freq
        freq=${freq:-30}

        echo -e "是否启用数据记录? [y/n, 默认: y]:"
        read -p "> " record
        record=${record:-y}

        echo -e "机械臂IP地址 [可选，留空使用配置文件]:"
        read -p "> " robot_ip

        echo -e "Oculus IP地址 [可选，留空自动检测]:"
        read -p "> " oculus_ip

        cmd="python3 franka_oculus.py --mode $mode --frequency $freq"

        if [ "$record" = "n" ] || [ "$record" = "N" ]; then
            cmd="$cmd --no-recording"
        fi

        if [ -n "$robot_ip" ]; then
            cmd="$cmd --robot-ip $robot_ip"
        fi

        if [ -n "$oculus_ip" ]; then
            cmd="$cmd --oculus-ip $oculus_ip"
        fi

        echo -e "\n${GREEN}🚀 启动命令: $cmd${NC}"
        $cmd
        ;;
    5)
        echo -e "\n${GREEN}🧪 测试模式${NC}"
        echo -e "\n${BLUE}测试 1: 导入检查${NC}"
        python3 -c "
import sys
from pathlib import Path
sys.path.insert(0, str(Path('..') / 'droid' / 'oculus_reader' / 'oculus_reader'))

print('  ✓ 导入 numpy...')
import numpy
print('  ✓ 导入 scipy...')
import scipy
print('  ✓ 导入 franka_r7arm...')
from franka_r7arm import FrankaR7arm
print('\n  ✅ 所有导入成功!')
" || echo -e "${RED}❌ 导入测试失败${NC}"

        echo -e "\n${BLUE}测试 2: 文件存在性${NC}"
        FILES=(
            "franka_oculus.py"
            "franka_r7arm.py"
            "../droid/oculus_reader/oculus_reader/reader.py"
        )

        for file in "${FILES[@]}"; do
            if [ -f "$file" ]; then
                echo -e "  ${GREEN}✓${NC} $file"
            else
                echo -e "  ${RED}✗${NC} $file ${RED}(未找到)${NC}"
            fi
        done

        echo -e "\n${GREEN}✅ 测试完成${NC}"
        ;;
    0)
        echo -e "\n${BLUE}👋 再见!${NC}"
        exit 0
        ;;
    *)
        echo -e "\n${RED}❌ 无效选项${NC}"
        exit 1
        ;;
esac

# 退出信息
echo -e "\n${BLUE}=====================================================================${NC}"
echo -e "${GREEN}程序已结束${NC}"
echo -e "${BLUE}=====================================================================${NC}"
