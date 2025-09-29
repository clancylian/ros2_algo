import time
import rclpy
from std_msgs.msg import String
import cv2
from .base import AlgoBase

VIDEO_PATH = "/public/lgl/20250916_152719.mp4"
#VIDEO_PATH = "rtsp://192.168.234.1:8554/test"

class TrashDetect(AlgoBase):
    """
    垃圾检测算法，用于检测环境中的垃圾
    """
    def __init__(self):
        """
        初始化垃圾检测算法
        """
        super().__init__('TrashDetect', '/algo_result/trash_detect')
        # 初始化资源标志
        self.initialized = False

    def _initialize_resources(self):
        """
        初始化YOLOv8模型和视频捕获设备
        
        Returns:
            bool: 初始化是否成功
        """
        if self.initialized:
            return True
            
        try:
            # 导入必要的库
            from ultralytics import YOLO
            
            # 从项目的models目录加载模型
            import os
            import platform
            from ament_index_python.packages import get_package_share_directory
            
            # 获取package共享目录
            package_share_dir = get_package_share_directory('multi_algo_manager')
            
            # 根据系统架构选择不同的模型
            system_architecture = platform.machine().lower()
            
            if 'x86' in system_architecture:
                # x86架构，使用pytorch模型
                model_path = os.path.join(package_share_dir, 'models', 'pytorch_model', 'best.pt')
                self.elogger.info(f"Using PyTorch model on x86 architecture: {model_path}")
            elif 'arm' in system_architecture:
                # ARM架构，使用rknn模型
                model_path = os.path.join(package_share_dir, 'models', 'rknn_model')
                self.elogger.info(f"Using RKNN model on ARM architecture: {model_path}")
            else:
                # 未知架构，默认使用pytorch模型
                model_path = os.path.join(package_share_dir, 'models', 'pytorch_model', 'best.pt')
                self.elogger.warning(f"Unknown architecture {system_architecture}, defaulting to PyTorch model: {model_path}")
            
            self.model = YOLO(model_path)
            
            # 打开视频文件或RTSP流
            # 可以根据需要选择视频文件或RTSP流
            self.cap = cv2.VideoCapture(VIDEO_PATH)
            
            # 初始化帧计数器
            self.frame_counter = 0
            
            self.initialized = True
            self.elogger.info("YOLOv8 model and video capture initialized successfully")
            return True
        except Exception as e:
            self.elogger.error(f"Failed to initialize YOLOv8 model or video capture: {str(e)}")
            self.initialized = False
            return False

    def run_analysis(self):
        """
        垃圾检测算法主循环，定期发布检测结果
        """
        try:
            # 在循环开始前初始化资源
            if not self._initialize_resources():
                # 初始化失败，等待一段时间后重试
                self.elogger.warning("Initialization failed, retrying in 5 seconds...")
                time.sleep(5)
                if not self._initialize_resources():
                    self.elogger.error("Failed to initialize resources after multiple attempts")
                    return
                    
            while not self._should_stop():
                try:
                    # 执行具体的垃圾检测算法
                    detection_result = self.perform_trash_detection()
                    
                    # 创建并发布消息
                    msg = String()
                    msg.data = f"TrashDetect result: {detection_result} at {time.time()}"
                    self.pub.publish(msg)
                    
                    # 记录检测结果日志
                    self.elogger.info(f"Published detection result: {detection_result}")
                    
                    # 更新心跳，表示算法正常运行
                    self._last_heartbeat = time.time()
                    time.sleep(self._polling_interval)
                except Exception as e:
                    self.elogger.error(f"Error in TrashDetect analysis: {str(e)}")
                    # 短暂暂停后继续，避免快速失败
                    time.sleep(0.1)
        finally:
            # 清理资源
            self._cleanup_resources()
    
    def _cleanup_resources(self):
        """
        清理算法使用的所有资源
        """
        # 释放视频捕获对象
        if hasattr(self, 'cap') and self.cap is not None:
            try:
                self.cap.release()
                self.elogger.info("Video capture released successfully")
                # 释放后将cap设置为None，避免重复释放
                self.cap = None
            except Exception as e:
                self.elogger.error(f"Failed to release video capture: {str(e)}")
        
        # 释放模型资源
        if hasattr(self, 'model') and self.model is not None:
            try:
                # 对于YOLO模型，通常不需要显式释放，Python的垃圾回收会处理
                # 但为了彻底清理，我们可以将其设置为None
                self.model = None
                self.elogger.info("Model resources released successfully")
            except Exception as e:
                self.elogger.error(f"Failed to release model resources: {str(e)}")
        
        # 重置初始化标志，确保重新启动时能正确初始化
        self.initialized = False
                
    def perform_trash_detection(self):
        """
        具体的垃圾检测算法实现，使用YOLOv8进行目标检测
        
        Returns:
            str: 检测结果描述
        """
        # 确保资源已初始化
        if not self.initialized:
            if not self._initialize_resources():
                return "Initialization failed"
        
        try:
            # 读取一帧视频
            success, frame = self.cap.read()
            
            if not success:
                # 如果读取失败，尝试重新打开视频
                self.elogger.warning("Failed to read frame, trying to reopen video capture")
                self.cap.release()
                self.cap = cv2.VideoCapture(VIDEO_PATH)
                return "Failed to read frame"
            
            # 增加帧计数器
            self.frame_counter += 1
            
            # 每20帧处理一次，减少计算量
            if self.frame_counter % 20 != 0:
                return "Skipping frame"
            
            # 运行YOLOv8推理
            results = self.model(frame, conf=0.1)
            
            # 获取检测结果
            detected_objects = []
            for result in results:
                for box in result.boxes:
                    # 获取类别ID和置信度
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    
                    # 获取边界框坐标
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # 获取类别名称
                    class_name = result.names[class_id]
                    
                    # 只记录置信度大于0.5的结果
                    if confidence > 0.5:
                        detected_objects.append(
                            f"{class_name} at ({center_x}, {center_y}) with confidence {confidence:.2f}")
            
            # 如果有检测到物体，返回格式化的结果
            if detected_objects:
                return f"Detected {len(detected_objects)} objects: {', '.join(detected_objects)}"
            else:
                return "No objects detected"
            
        except Exception as e:
            self.elogger.error(f"Error during trash detection: {str(e)}")
            return f"Detection error: {str(e)}"

    def process(self):
        """
        获取垃圾检测算法的当前状态
        
        Returns:
            str: 算法运行状态信息
        """
        state = self.get_state()
        if state['running']:
            return f"TrashDetect running at {time.time()}"
        return None

def main():
    rclpy.init()
    node = TrashDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
