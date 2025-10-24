
import contextlib
import signal
import sys
from pathlib import Path

import numpy as np
from loguru import logger
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
                level: str = "INFO",
                output: str = "both",
                log_dir: Path = Path(__file__).parent / "log",
                mode: str = "append"):
    """设置双输出日志系统（文件 + 控制台），使用 loguru
    Args:
        name: 日志记录器名称
        level: 日志级别 ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")
        output: 输出目标 ("file", "console", "both")
        log_dir: 日志文件目录
        mode: 日志文件模式 ("append" 追加, "overwrite" 覆盖)
    """
    # 移除默认的 handler
    logger.remove()

    # 日志格式 - 使用固定宽度以对齐
    log_format = "<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level:<5}</level> | <cyan>{name:<10}</cyan>:<cyan>{line:<3}</cyan> - <level>{message}</level>"

    # 控制台输出
    if output in ("console", "both"):
        logger.add(
            sys.stdout,
            format=log_format,
            level=level,
            colorize=True
        )

    # 文件输出
    if output in ("file", "both"):
        log_dir.mkdir(exist_ok=True)
        rotation = None if mode == "overwrite" else "10 MB"  # 覆盖模式不轮转
        logger.add(
            log_dir / f"{name}.log",
            format=log_format,
            level=level,
            rotation=rotation,
            retention="10 days",
            compression="zip",
            encoding="utf-8",
            enqueue=True  # 异步写入
        )

    return logger