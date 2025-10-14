# Franka Oculus 遥操作控制系统

## 📋 功能概述

此系统实现了使用 Oculus Quest 手柄实时控制 Franka R7 机械臂的功能，支持：

- ✅ **3种控制模式**：绝对位姿、相对位姿、关节角度
- ✅ **多线程实时控制**：独立控制线程，默认30Hz刷新率
- ✅ **按钮映射**：夹爪控制、急停、精确模式
- ✅ **数据记录**：完整的轨迹数据保存为JSON
- ✅ **安全限制**：工作空间限制、速度限制、死区过滤
- ✅ **平滑滤波**：移动平滑，减少抖动

## 🗂️ 文件说明

### 核心文件

1. **franka_oculus.py** (新创建)
   - 完整的集成控制系统
   - 包含 SimpleController 类和 FrankaOculusController 类
   - 所有功能集成在一个文件中，避免导入问题

2. **franka_r7arm.py** (已存在)
   - Franka R7 机械臂底层控制接口
   - 提供连接、运动控制、状态读取等功能

3. **oculus_reader/reader.py** (已存在)
   - Oculus Quest 手柄数据读取
   - 通过 ADB 读取手柄的位姿和按钮数据

## 🚀 快速开始

### 1. 环境准备

确保已安装所有依赖：

```bash
cd /home/joker/Code_Work/Code/droid

# 安装 Python 依赖
pip install numpy scipy pyyaml pure-python-adb

# 确保 franky 库已安装（如果使用真实机械臂）
pip install franky

# 确保 robosuite 已安装（如果使用仿真）
pip install robosuite
```

### 2. 连接设备

#### Oculus Quest 手柄
```bash
# 确保 Oculus Quest 通过 USB 或无线连接
adb devices

# 应该看到设备列表
# List of devices attached
# 192.168.x.x:5555    device
```

#### Franka 机械臂
- 真实机械臂：确保网络连接正常，配置文件中设置正确的IP
- 仿真模式：确保 robosuite 环境正常

### 3. 运行控制程序

#### 基本使用（相对控制模式，30Hz）

```bash
cd /home/joker/Code_Work/Code/droid/simulation
python franka_oculus.py
```

#### 自定义参数

```bash
# 使用绝对位姿控制模式
python franka_oculus.py --mode absolute

# 设置控制频率为 50Hz
python franka_oculus.py --frequency 50

# 禁用数据记录
python franka_oculus.py --no-recording

# 指定设备IP
python franka_oculus.py --robot-ip 192.168.1.100 --oculus-ip 192.168.1.200

# 组合参数
python franka_oculus.py --mode relative --frequency 30 --robot-ip 192.168.1.100
```

### 4. 操作说明

启动后，程序会显示：

```
======================================================================
🤖 Franka Oculus 遥操作控制器
======================================================================

📱 初始化Oculus手柄读取器...
🦾 初始化Franka机械臂...
🔄 初始化控制转换器...

✅ 初始化完成！
   控制模式: relative
   控制频率: 30.0 Hz
   数据记录: 启用

======================================================================
🎮 控制说明:
   • 移动右手柄控制机械臂末端位置
   • A按钮 - 夹爪关闭
   • B按钮 - 夹爪打开
   • 右触发器 - 精确模式（降低速度）
   • 右摇杆按下 - 急停
======================================================================

🏠 移动到初始位置...
✅ 控制已启动！

💡 按 Ctrl+C 停止控制

🚀 启动控制循环 (频率: 30.0 Hz)
⏱️  运行: 10.2s | 频率: 30.1Hz | 命令: 305 | 成功: 303 | 失败: 2
```

## 🎮 控制模式详解

### 1. 相对控制模式 (relative) - 推荐

**特点**：增量控制，更安全、更直观

```bash
python franka_oculus.py --mode relative
```

- 手柄移动产生速度命令
- 机械臂跟随手柄移动方向和速度
- 有死区保护（小于2mm的移动被忽略）
- 自动平滑滤波（5帧窗口）
- 支持精确模式（按住右触发器，速度降至30%）

**适用场景**：
- 精细操作
- 需要实时反馈的任务
- 初学者使用

### 2. 绝对控制模式 (absolute)

**特点**：手柄位姿直接映射到机械臂位姿

```bash
python franka_oculus.py --mode absolute
```

- 手柄位置 = 机械臂末端位置（经过坐标变换）
- 需要预先校准坐标系
- 响应快速但需要操作者适应

**适用场景**：
- 快速演示
- 已知坐标系映射关系
- 轨迹录制

### 3. 关节角度控制模式 (joint)

**特点**：通过逆运动学将手柄位姿转换为关节角度

```bash
python franka_oculus.py --mode joint
```

- 需要逆运动学求解器
- 可以避免奇异点
- 更符合机器人运动学

**注意**：需要在 franka_r7arm.py 中实现逆运动学接口

## 📊 数据记录

### 自动记录

启用记录后（默认），程序会自动保存所有控制命令：

```json
{
  "trajectory": [
    {
      "position": [0.45, 0.02, 0.35],
      "rotation": [0.0, 0.707, 0.707, 0.0],
      "control_mode": "relative",
      "delta_position": [0.001, 0.0, 0.002],
      "gripper_position": null,
      "precision_mode": false,
      "emergency_stop": false,
      "timestamp": 1705234567.123
    },
    ...
  ],
  "metadata": {
    "num_points": 1234,
    "scale_factor": 1.0,
    "workspace_limits": {...},
    "timestamp": "2025-01-14 15:30:00"
  }
}
```

### 文件命名

自动生成时间戳文件名：
```
franka_oculus_trajectory_20250114_153000.json
```

### 禁用记录

```bash
python franka_oculus.py --no-recording
```

## ⚙️ 参数配置

### 在代码中修改

编辑 `franka_oculus.py`，在 `FrankaOculusController.__init__` 中：

```python
self.controller = SimpleController(
    scale_factor=1.0,           # 位置缩放因子
    offset=[0.4, 0.0, 0.3],     # 工作空间偏移 [x, y, z]
    workspace_limits={          # 工作空间限制（米）
        'x': [0.2, 0.8],
        'y': [-0.4, 0.4],
        'z': [0.1, 0.8]
    },
    velocity_limits={           # 速度限制
        'linear': 0.2,          # 最大线速度 m/s
        'angular': 0.5          # 最大角速度 rad/s
    },
    enable_smoothing=True,      # 启用平滑
    smoothing_window=5,         # 平滑窗口大小（帧数）
    deadzone=0.002,            # 死区阈值（米）
    enable_recording=True       # 启用记录
)
```

### 关键参数说明

- **scale_factor**: Oculus空间到机械臂空间的缩放
  - `1.0` = 1:1 映射（手柄移动1m，末端移动1m）
  - 典型值：0.5 - 2.0

- **offset**: 坐标原点偏移
  - 将 Oculus 坐标系原点映射到机械臂工作空间
  - 典型值：`[0.4, 0.0, 0.3]` (机械臂前方、中央、稍高)

- **workspace_limits**: 安全限制
  - 防止机械臂移动到不可达或危险区域
  - 根据实际机械臂调整

- **velocity_limits**: 速度限制
  - 防止过快运动导致不安全
  - `linear`: 0.1-0.5 m/s（推荐0.2）
  - `angular`: 0.3-1.0 rad/s（推荐0.5）

- **smoothing_window**: 平滑窗口
  - 值越大越平滑，但延迟越大
  - 推荐：5-10帧

- **deadzone**: 死区阈值
  - 过滤手柄抖动
  - 推荐：0.001-0.005 米

## 🔧 故障排查

### 1. 无法导入 OculusReader

**错误**：
```
ModuleNotFoundError: No module named 'reader'
```

**解决**：
- 确保在正确的目录运行：`cd /home/joker/Code_Work/Code/droid/simulation`
- 检查路径设置（franka_oculus.py 的第33-35行）

### 2. 无法连接Franka机械臂

**错误**：
```
RuntimeError: 无法连接到Franka机械臂
```

**解决**：
- 检查机械臂电源和网络连接
- 检查 `franka_r7arm.py` 中的配置文件路径和IP设置
- 确保 franky 库已正确安装

### 3. Oculus手柄无响应

**问题**：程序运行但机械臂不动

**解决**：
- 检查 Oculus 连接：`adb devices`
- 重启 Oculus Reader
- 检查日志中是否收到手柄数据
- 尝试重新连接手柄

### 4. 控制频率低于预期

**问题**：实际频率远低于30Hz

**可能原因**：
- 网络延迟（无线连接Oculus）
- 机械臂响应慢
- 计算负载过高

**解决**：
- 使用 USB 连接 Oculus（更低延迟）
- 减少平滑窗口大小
- 降低目标频率：`--frequency 20`

### 5. 机械臂运动不平滑

**解决**：
- 增加平滑窗口：`smoothing_window=10`
- 检查速度限制是否过低
- 确保死区阈值合适

## 📈 性能优化

### 提高控制频率

1. **减少计算负载**
   ```python
   smoothing_window=3  # 减少平滑窗口
   enable_recording=False  # 禁用记录
   ```

2. **优化网络**
   - 使用有线连接（机械臂和电脑）
   - USB连接Oculus（而非WiFi）

3. **调整系统**
   ```bash
   # 设置进程优先级
   sudo nice -n -10 python franka_oculus.py
   ```

### 降低延迟

1. **使用相对控制模式**（响应最快）
2. **减少死区**：`deadzone=0.001`
3. **禁用不必要的功能**

## 🔐 安全建议

1. **初次使用**
   - 在仿真中测试
   - 降低速度限制：`linear: 0.1`
   - 缩小工作空间范围

2. **正式使用前**
   - 确保急停按钮可用（右摇杆按下）
   - 确认工作空间限制正确
   - 测试夹爪控制

3. **运行时**
   - 随时准备按 Ctrl+C 或急停
   - 观察机械臂运动是否异常
   - 注意障碍物

4. **测试急停**
   ```bash
   # 测试时按下右摇杆，应该立即暂停控制
   ```

## 📝 开发和扩展

### 添加新的控制模式

编辑 `FrankaOculusController._execute_command()`:

```python
elif self.control_mode == 'my_mode' and 'my_data' in command:
    return self._execute_my_mode(command)
```

### 自定义按钮映射

编辑 `SimpleController._process_buttons()`:

```python
# X按钮 - 自定义功能
if buttons.get('X', False):
    actions['custom_action'] = True
```

### 添加传感器反馈

在控制循环中读取机械臂状态：

```python
status = self.robot.get_current_status()
ee_pose = status['ee_pose']
# 使用反馈调整控制
```

## 📞 支持

如有问题，请检查：
1. 此 README 的故障排查部分
2. 代码中的注释和文档字符串
3. 日志输出中的错误信息

## 🎯 下一步

- [ ] 添加力控制模式
- [ ] 实现双手协调控制
- [ ] 添加虚拟夹具（约束）
- [ ] 集成视觉反馈
- [ ] 添加轨迹回放功能
