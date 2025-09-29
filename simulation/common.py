
import contextlib
import logging
import signal
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

# ———————————————————————————————————————————————————————————————————————————————————————————————————————————————————

def compute_target_pose_from_relative_action(current_pose: np.ndarray, relative_action: np.ndarray) -> np.ndarray:
    """根据当前位置和相对动作计算目标位姿

    Args:
        current_pose: 当前位姿 [x, y, z, rx, ry, rz]
        relative_action: 相对动作 [dx, dy, dz, drx, dry, drz]

    Returns:
        target_pose: 目标位姿 [x, y, z, rx, ry, rz]
    """
    target_pose = np.zeros(6)

    # 位置增量：直接相加
    target_pose[:3] = relative_action[:3] + current_pose[:3]

    # 旋转增量：使用四元数进行正确的旋转组合
    try:
        # 当前位姿的欧拉角转四元数
        current_rotation = R.from_euler('xyz', current_pose[3:6], degrees=False)
        current_quat = current_rotation.as_quat()

        # 相对旋转的欧拉角转四元数
        relative_rotation = R.from_euler('xyz', relative_action[3:6], degrees=False)
        relative_quat = relative_rotation.as_quat()

        # 四元数组合运算：当前旋转 * 相对旋转
        # 这表示在当前旋转基础上应用相对旋转
        combined_quat = R.from_quat(current_quat) * R.from_quat(relative_quat)

        # 转换回欧拉角
        target_pose[3:6] = combined_quat.as_euler('xyz', degrees=False)

    except Exception as e:
        print(f"旋转计算失败，使用简单加法: {e}")
        # 备用方案：简单加法（不推荐，但作为后备）
        target_pose[3:6] = current_pose[3:6] + relative_action[3:6]

    return target_pose

@contextlib.contextmanager
def prevent_keyboard_interrupt():
    """临时阻止键盘中断，直到保护的代码执行完毕，Ctrl+C中断保护上下文管理器"""
    interrupted = False
    original_handler = signal.getsignal(signal.SIGINT)

    def handler(signum, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, handler)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, original_handler)
        if interrupted:
            raise KeyboardInterrupt

def setup_logger(name: str,
                level: int = logging.INFO,
                output: str = "both",
                log_dir: str = Path(__file__).parent / "log",
                mode: str = "append") -> logging.Logger:
    """设置双输出日志系统（文件 + 控制台），支持选择输出目标和日志模式

    Args:
        name: 日志记录器名称
        level: 日志级别
        output: 输出目标 ("file", "console", "both")
        log_dir: 日志文件目录
        mode: 日志文件模式 ("append" 追加, "overwrite" 覆盖)
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    if logger.handlers:
        return logger

    # 文件处理器：输出到 当前目录的
    if output in ("file", "both"):
        log_dir.mkdir(exist_ok=True)
        # 根据模式选择文件打开方式
        if mode == "overwrite":
            file_mode = "w"  # 覆盖模式
        else:  # 默认追加模式
            file_mode = "a"  # 追加模式
        file_handler = logging.FileHandler(log_dir / f"{name}.log", mode=file_mode, encoding="utf-8")
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    # 控制台处理器：输出到终端
    if output in ("console", "both"):
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    return logger