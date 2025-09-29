# cyto_camera.py - 简化优化的相机控制代码

import queue
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from common import setup_logger

logger = setup_logger("cyto_camera", output="file")
logger.info("cyto_camera.py 日志系统已初始化")

def kill_camera_processes():
    """关闭所有使用相机的进程"""
    try:
        logger.info("正在关闭所有使用相机的进程...")

        # 查找使用RealSense相机的进程
        camera_processes = []

        # 查找使用realsense-viewer的进程
        try:
            result = subprocess.run(
                ["pgrep", "-f", "realsense-viewer"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.stdout.strip():
                camera_processes.extend(result.stdout.strip().split('\n'))
        except Exception as e:
            logger.debug(f"查找realsense-viewer进程时出错: {e}")

        # 查找使用rs-enumerate-devices的进程
        try:
            result = subprocess.run(
                ["pgrep", "-f", "rs-enumerate-devices"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.stdout.strip():
                camera_processes.extend(result.stdout.strip().split('\n'))
        except Exception as e:
            logger.debug(f"查找rs-enumerate-devices进程时出错: {e}")

        # 查找使用rs-record的进程
        try:
            result = subprocess.run(
                ["pgrep", "-f", "rs-record"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.stdout.strip():
                camera_processes.extend(result.stdout.strip().split('\n'))
        except Exception as e:
            logger.debug(f"查找rs-record进程时出错: {e}")

        # 查找使用rs-playback的进程
        try:
            result = subprocess.run(
                ["pgrep", "-f", "rs-playback"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.stdout.strip():
                camera_processes.extend(result.stdout.strip().split('\n'))
        except Exception as e:
            logger.debug(f"查找rs-playback进程时出错: {e}")

        # 查找使用python且包含realsense的进程（排除当前进程）
        try:
            result = subprocess.run(
                ["pgrep", "-f", "python.*realsense"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.stdout.strip():
                pids = result.stdout.strip().split('\n')
                current_pid = str(subprocess.getpid())
                for pid in pids:
                    if pid.strip() != current_pid:
                        camera_processes.append(pid.strip())
        except Exception as e:
            logger.debug(f"查找python realsense进程时出错: {e}")

        # 去重并关闭进程
        unique_processes = list(set(camera_processes))
        if unique_processes:
            logger.info(f"发现 {len(unique_processes)} 个使用相机的进程: {unique_processes}")
            for pid in unique_processes:
                if pid.strip():
                    try:
                        logger.info(f"正在关闭进程 {pid}")
                        subprocess.run(["kill", "-9", pid.strip()], timeout=5)
                        time.sleep(0.5)  # 短暂等待进程关闭
                    except Exception as e:
                        logger.warning(f"关闭进程 {pid} 时出错: {e}")
            logger.info("所有相机进程已关闭")
        else:
            logger.info("未发现使用相机的进程")

        # 等待一段时间确保进程完全关闭
        time.sleep(1)

    except Exception as e:
        logger.error(f"关闭相机进程时出错: {e}")


class D405Recorder:
    """Intel RealSense D405 相机录制器"""

    def __init__(self, width=640, height=480, fps=30, serial_number=None):
        self.width = width
        self.height = height
        self.fps = fps
        self.serial_number = serial_number

        # RealSense 对象
        self.pipeline = None
        self.align = None
        self.depth_scale = None

        # 录制状态
        self.is_recording = False
        self.frame_count = 0
        self.recording_thread = None
        self.frame_queue = queue.Queue(maxsize=50)  # 减小队列大小

    def init_camera(self):
        """优化的相机初始化 - 去除冗余步骤"""
        try:
            # 简化的设备检查
            ctx = rs.context()
            devices = ctx.query_devices()

            if len(devices) == 0:
                logger.error("没有检测到RealSense设备")
                return False

            # 快速找到目标设备
            target_serial = None
            if self.serial_number:
                for device in devices:
                    try:
                        if device.get_info(rs.camera_info.serial_number) == self.serial_number:
                            target_serial = self.serial_number
                            break
                    except:
                        continue
                if not target_serial:
                    logger.error(f"指定的相机 {self.serial_number} 不存在")
                    return False
            else:
                # 使用第一个可用设备
                try:
                    target_serial = devices[0].get_info(rs.camera_info.serial_number)
                except:
                    logger.error("无法获取设备串口号")
                    return False

            # 直接配置和启动Pipeline - 只启用彩色流
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(target_serial)
            # 只启用彩色流，不启用深度流
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

            # 启动Pipeline
            profile = self.pipeline.start(config)
            logger.info(f"相机启动成功: {target_serial}")

            # 获取设备信息（不强制获取传感器）
            try:
                device = profile.get_device()
                device_name = device.get_info(rs.camera_info.name)
                device_serial = device.get_info(rs.camera_info.serial_number)
                logger.debug(f"设备信息 - {device_name} (序列号: {device_serial})")
            except Exception as e:
                logger.debug(f"设备信息获取失败: {e}")

            # 不需要对齐对象，因为我们不处理深度
            self.align = None

            # 简化的预热 - 只获取3帧
            try:
                for i in range(3):
                    frames = self.pipeline.wait_for_frames(timeout_ms=3000)
                    if not frames:
                        logger.warning(f"预热第{i+1}次失败")
                    else:
                        color_frame = frames.get_color_frame()
                        if not color_frame:
                            logger.warning(f"预热第{i+1}次失败，无法获取彩色帧")
            except Exception as e:
                logger.warning(f"预热失败: {e}")

            return True

        except Exception as e:
            logger.error(f"相机初始化失败: {e}")
            if self.pipeline:
                try:
                    self.pipeline.stop()
                except:
                    pass
                self.pipeline = None
            return False

    def get_current_frame(self):
        """优化的帧获取 - 只获取RGB图像"""
        if self.pipeline is None:
            return None

        try:
            # 增加超时时间，确保能获取到帧
            frame_set = self.pipeline.wait_for_frames(timeout_ms=2000)
            if not frame_set:
                logger.warning("无法获取帧集")
                return None

            # 只获取彩色图像，跳过深度图像和对齐处理
            color_frame = frame_set.get_color_frame()
            if not color_frame:
                logger.warning("无法获取彩色帧")
                return None

            color_image = np.asanyarray(color_frame.get_data())

            # 检查图像是否为空或全黑
            if color_image is None or color_image.size == 0:
                logger.warning("图像为空")
                return None

            # 检查图像是否全黑
            if np.all(color_image == 0):
                logger.warning("图像全黑")
                return None

            # 只在第一次获取时显示调试信息
            if not hasattr(self, '_debug_shown'):
                logger.debug(f"图像形状={color_image.shape}, 数据类型={color_image.dtype}, 像素范围=[{color_image.min()}-{color_image.max()}]")
                self._debug_shown = True
            return color_image

        except Exception as e:
            logger.error(f"获取帧失败: {e}")
            return None

    def close(self):
        """优化的关闭方法"""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
            self.pipeline = None
            self.align = None


class CytobotCamera:
    """双RealSense相机接口"""

    def __init__(self, front_camera_id: str = None, wrist_camera_id: str = None):
        """简化的初始化"""
        # 首先关闭所有使用相机的进程
        kill_camera_processes()

        self.front_camera_id = front_camera_id
        self.wrist_camera_id = wrist_camera_id

        self.front_camera = None
        self.wrist_camera = None
        self._front_connected = False
        self._wrist_connected = False

        # 视频显示相关
        self._display_thread = None
        self._display_running = False
        self._display_lock = threading.Lock()

    def connect(self):
        """优化的连接方法 - 去除冗余检查"""
        logger.info("开始连接相机...")

        success_front = False
        success_wrist = False

        # 连接前置相机
        if self.front_camera_id:
            try:
                self.front_camera = D405Recorder(
                    width=640, height=480, fps=30,
                    serial_number=self.front_camera_id
                )
                success_front = self.front_camera.init_camera()
                self._front_connected = success_front
                if success_front:
                    logger.info("✅ 前置相机连接成功")
                else:
                    logger.error("❌ 前置相机连接失败")
                    self.front_camera = None
            except Exception as e:
                logger.error(f"❌ 前置相机连接异常: {e}")
                self.front_camera = None

        # 连接腕部相机
        if self.wrist_camera_id:
            try:
                self.wrist_camera = D405Recorder(
                    width=640, height=480, fps=30,
                    serial_number=self.wrist_camera_id
                )
                success_wrist = self.wrist_camera.init_camera()
                self._wrist_connected = success_wrist
                if success_wrist:
                    logger.info("✅ 腕部相机连接成功")
                else:
                    logger.error("❌ 腕部相机连接失败")
                    self.wrist_camera = None
            except Exception as e:
                logger.error(f"❌ 腕部相机连接异常: {e}")
                self.wrist_camera = None

        total_success = success_front or success_wrist
        if total_success:
            logger.info(f"✅ 相机连接完成 - 前置: {success_front}, 腕部: {success_wrist}")
        else:
            logger.error("❌ 所有相机连接都失败了")

        return total_success

    def get_images(self):
        """优化的图像获取 - 只获取RGB图像"""
        # 创建虚拟图像作为后备
        dummy_rgb = np.zeros((480, 640, 3), dtype=np.uint8)

        # 获取前置相机图像
        front_rgb = dummy_rgb.copy()
        if self.front_camera and self._front_connected:
            try:
                front_color = self.front_camera.get_current_frame()
                if front_color is not None:
                    front_rgb = cv2.cvtColor(front_color, cv2.COLOR_BGR2RGB)
            except Exception as e:
                logger.debug(f"前置相机读取失败: {e}")

        # 获取腕部相机图像
        wrist_rgb = dummy_rgb.copy()
        if self.wrist_camera and self._wrist_connected:
            try:
                wrist_color = self.wrist_camera.get_current_frame()
                if wrist_color is not None:
                    wrist_rgb = cv2.cvtColor(wrist_color, cv2.COLOR_BGR2RGB)
            except Exception as e:
                logger.debug(f"腕部相机读取失败: {e}")

        return {
            'front_rgb': front_rgb,
            'wrist_rgb': wrist_rgb
        }

    def stop(self):
        """优化的停止方法"""
        logger.info("正在停止相机系统...")

        # 首先停止视频显示
        self.stop_video_display()

        if self.front_camera:
            try:
                self.front_camera.close()
                self.front_camera = None
                self._front_connected = False
            except Exception as e:
                logger.error(f"停止前置相机时出错: {e}")

        if self.wrist_camera:
            try:
                self.wrist_camera.close()
                self.wrist_camera = None
                self._wrist_connected = False
            except Exception as e:
                logger.error(f"停止腕部相机时出错: {e}")

        logger.info("相机系统已停止")

    def is_connected(self):
        """检查连接状态"""
        return {
            'front_connected': self._front_connected,
            'wrist_connected': self._wrist_connected,
            'any_connected': self._front_connected or self._wrist_connected
        }

    def _display_video_loop(self):
        """视频显示循环（在后台线程中运行）- 合并双摄像头画面"""
        logger.info("视频显示线程启动")

        while self._display_running:
            try:
                # 获取图像
                images = self.get_images()

                if images:
                    front_rgb = images.get('front_rgb')
                    wrist_rgb = images.get('wrist_rgb')

                    # 处理前置相机图像
                    if front_rgb is not None and not np.all(front_rgb == 0):
                        front_bgr = cv2.cvtColor(front_rgb, cv2.COLOR_RGB2BGR)
                    else:
                        # 如果前置相机没有图像，显示黑屏
                        front_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
                        # 在黑屏上添加文字提示
                        cv2.putText(front_bgr, 'Front Camera: No Signal', (50, 240),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    # 处理腕部相机图像
                    if wrist_rgb is not None and not np.all(wrist_rgb == 0):
                        wrist_bgr = cv2.cvtColor(wrist_rgb, cv2.COLOR_RGB2BGR)
                    else:
                        # 如果腕部相机没有图像，显示黑屏
                        wrist_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
                        # 在黑屏上添加文字提示
                        cv2.putText(wrist_bgr, 'Wrist Camera: No Signal', (50, 240),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                    # 水平合并两个画面
                    combined_frame = np.hstack((front_bgr, wrist_bgr))

                    # 添加分割线和标签
                    cv2.line(combined_frame, (640, 0), (640, 480), (0, 255, 0), 2)
                    cv2.putText(combined_frame, 'Front Camera', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(combined_frame, 'Wrist Camera', (650, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # 显示合并后的画面
                    cv2.imshow('Cytobot Dual Camera View', combined_frame)

                    # 检查按键，按 'q' 或 ESC 退出
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' 或 ESC 键
                        logger.info("检测到退出按键，停止视频显示")
                        self._display_running = False
                        break

                # 控制帧率，约30FPS
                time.sleep(0.033)

            except Exception as e:
                logger.error(f"视频显示循环出错: {e}")
                time.sleep(0.1)

        # 清理窗口
        try:
            cv2.destroyAllWindows()
            logger.info("视频显示窗口已关闭")
        except:
            pass

        logger.info("视频显示线程结束")

    def start_video_display(self):
        """启动实时视频显示"""
        with self._display_lock:
            if self._display_running:
                logger.warning("视频显示已在运行")
                return False

            # 无论相机是否连接都可以启动显示（未连接时显示黑屏）
            self._display_running = True
            self._display_thread = threading.Thread(target=self._display_video_loop, daemon=True)
            self._display_thread.start()
            logger.info("实时视频显示已启动")
            return True
            return True

    def stop_video_display(self):
        """停止实时视频显示"""
        with self._display_lock:
            if not self._display_running:
                logger.info("视频显示未运行")
                return

            logger.info("正在停止视频显示...")
            self._display_running = False

            # 等待线程结束
            if self._display_thread and self._display_thread.is_alive():
                self._display_thread.join(timeout=2.0)
                if self._display_thread.is_alive():
                    logger.warning("视频显示线程未能正常结束")
                else:
                    logger.info("视频显示线程已结束")

            self._display_thread = None

    def is_display_running(self):
        """检查视频显示是否正在运行"""
        return self._display_running


def test():
    """测试相机功能并演示合并视频显示"""
    print("开始测试相机...")
    print("📺 新功能：双摄像头画面将合并在一个窗口中显示")
    print("   - 左侧：前置相机画面")
    print("   - 右侧：腕部相机画面")
    print("   - 未连接的相机将显示黑屏和提示文字")
    print()

    camera = CytobotCamera(
        front_camera_id="218622272613",
        wrist_camera_id="230322270249"
    )

    try:
        start_time = time.time()
        if camera.connect():
            connect_time = time.time() - start_time
            print(f"相机连接成功，耗时: {connect_time:.2f}秒")

            # 创建保存目录
            save_dir = Path("log/camera_test")
            save_dir.mkdir(parents=True, exist_ok=True)

            print("\n=== 测试1: 单独获取图像（不显示）===")
            # 获取并保存图像
            for i in range(3):
                print(f"\n获取图像 {i+1}/3")
                start_time = time.time()
                images = camera.get_images()
                get_time = time.time() - start_time

                print(f"获取图像耗时: {get_time:.3f}秒")

                # 保存前置相机图像
                if images['front_rgb'] is not None:
                    front_rgb_path = save_dir / f"front_rgb_{i+1:02d}.png"

                    # 检查图像是否全黑
                    if np.all(images['front_rgb'] == 0):
                        print("  警告: 前置相机图像全黑，跳过保存")
                    else:
                        # 转换RGB到BGR并保存
                        front_rgb_bgr = cv2.cvtColor(images['front_rgb'], cv2.COLOR_RGB2BGR)
                        cv2.imwrite(str(front_rgb_path), front_rgb_bgr)
                        print(f"  保存前置相机图像: {front_rgb_path.name}")
                else:
                    print("  警告: 前置相机图像为None")

                # 保存腕部相机图像
                if images['wrist_rgb'] is not None:
                    wrist_rgb_path = save_dir / f"wrist_rgb_{i+1:02d}.png"

                    # 检查图像是否全黑
                    if np.all(images['wrist_rgb'] == 0):
                        print("  警告: 腕部相机图像全黑，跳过保存")
                    else:
                        # 转换RGB到BGR并保存
                        wrist_rgb_bgr = cv2.cvtColor(images['wrist_rgb'], cv2.COLOR_RGB2BGR)
                        cv2.imwrite(str(wrist_rgb_path), wrist_rgb_bgr)
                        print(f"  保存腕部相机图像: {wrist_rgb_path.name}")
                else:
                    print("  警告: 腕部相机图像为None")

                # 显示图像信息
                print(f"  前置RGB: {images['front_rgb'].shape if images['front_rgb'] is not None else 'None'}")
                print(f"  腕部RGB: {images['wrist_rgb'].shape if images['wrist_rgb'] is not None else 'None'}")

                time.sleep(0.5)

            print(f"\n✅ 图像保存完成，保存目录: {save_dir}")

            print("\n=== 测试2: 启动合并视频显示 ===")
            print("启动合并视频显示...")
            if camera.start_video_display():
                print("✅ 合并视频显示已启动")
                print("📺 单窗口双摄像头画面应该已经打开")
                print("   - 左侧显示前置相机")
                print("   - 右侧显示腕部相机")
                print("   - 未连接的相机显示黑屏")
                print("💡 按 'q' 键或 ESC 键可以关闭视频显示")

                # 在视频显示运行的同时，演示可以单独调用 get_images
                print("\n=== 测试3: 视频显示运行时单独调用 get_images ===")
                for i in range(5):
                    print(f"\n在视频显示运行时获取图像 {i+1}/5")

                    # 单独调用 get_images
                    start_time = time.time()
                    images = camera.get_images()
                    get_time = time.time() - start_time

                    print(f"  获取耗时: {get_time:.3f}秒")
                    print(f"  前置RGB形状: {images['front_rgb'].shape if images['front_rgb'] is not None else 'None'}")
                    print(f"  腕部RGB形状: {images['wrist_rgb'].shape if images['wrist_rgb'] is not None else 'None'}")
                    print(f"  视频显示状态: {'运行中' if camera.is_display_running() else '已停止'}")

                    # 可以对获取的图像进行处理（例如保存到不同目录）
                    if images['front_rgb'] is not None and not np.all(images['front_rgb'] == 0):
                        individual_path = save_dir / f"individual_front_{i+1:02d}.png"
                        front_rgb_bgr = cv2.cvtColor(images['front_rgb'], cv2.COLOR_RGB2BGR)
                        cv2.imwrite(str(individual_path), front_rgb_bgr)
                        print(f"  单独保存前置图像: {individual_path.name}")

                    time.sleep(2)  # 等待2秒再获取下一帧

                    # 检查视频显示是否仍在运行（用户可能已按键退出）
                    if not camera.is_display_running():
                        print("  视频显示已被用户停止")
                        break

                # 如果视频显示仍在运行，等待用户操作
                if camera.is_display_running():
                    print("\n⏳ 视频显示仍在运行，您可以：")
                    print("   - 在视频窗口中按 'q' 或 ESC 键退出")
                    print("   - 或者等待10秒后自动停止")

                    # 等待最多10秒或用户退出
                    wait_time = 0
                    while camera.is_display_running() and wait_time < 10:
                        time.sleep(1)
                        wait_time += 1
                        if wait_time % 2 == 0:  # 每2秒打印一次状态
                            print(f"   等待中... {wait_time}/10秒")

                    if camera.is_display_running():
                        print("   自动停止视频显示")
                        camera.stop_video_display()

                print("✅ 实时视频显示测试完成")
            else:
                print("❌ 启动实时视频显示失败")
        else:
            print("相机连接失败")

    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        camera.stop()
        print("测试结束")


if __name__ == "__main__":
    test()
