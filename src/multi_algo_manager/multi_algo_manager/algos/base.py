import time
import threading
import traceback
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from .logger import EnhancedLogger


class AlgoBase(Node):
    """
    算法基类，提供线程安全的启动/停止机制和通用功能
    """
    def __init__(self, node_name, topic_name=None):
        """
        初始化算法基类
        
        Args:
            node_name (str): 节点名称
            topic_name (str): 结果发布的话题名称，如果为None则使用默认格式
        """
        super().__init__(node_name)
        
        # 创建增强型日志工具
        self.elogger = EnhancedLogger(self)
        
        # 设置默认参数
        self.declare_parameter('polling_interval', 1.0)
        self.declare_parameter('thread_timeout', 2.0)
        
        # 获取参数值
        self._polling_interval = self.get_parameter('polling_interval').get_parameter_value().double_value
        self._thread_timeout = self.get_parameter('thread_timeout').get_parameter_value().double_value
        
        # 线程安全控制变量
        self._lock = threading.RLock()  # 使用可重入锁
        self._running = False
        self._thread = None
        self._last_exception = None
        self._last_heartbeat = 0
        
        # 设置结果发布话题
        topic_name = topic_name or f'/algo_result/{node_name.lower()}'
        self.pub = self.create_publisher(String, topic_name, 10)
        
        # 创建心跳定时器
        self._heartbeat_timer = self.create_timer(5.0, self._update_heartbeat)

    def start(self):
        """
        线程安全地启动算法
        """
        with self._lock:
            if not self._running:
                self._running = True
                self._last_exception = None
                self._thread = threading.Thread(target=self._run_wrapper, daemon=True)
                self._thread.start()
                self.elogger.info(f"started with polling interval: {self._polling_interval}s")
                return True
            else:
                self.elogger.debug("is already running")
                return False

    def stop(self):
        """
        线程安全地停止算法
        """
        with self._lock:
            if self._running:
                self._running = False
                if self._thread:
                    self._thread.join(timeout=self._thread_timeout)
                    if self._thread.is_alive():
                        self.elogger.warning("thread did not terminate within timeout")
                    else:
                        self._thread = None
                self.elogger.info("stopped")
                return True
            else:
                self.elogger.debug("is not running")
                return False

    def _should_stop(self):
        """
        检查算法是否应该停止运行
        线程安全地访问_running标志
        """
        with self._lock:
            return not self._running

    def _run_wrapper(self):
        """
        运行包装器，用于捕获和记录异常
        """
        try:
            self.run_analysis()
        except Exception as e:
            self.elogger.error(f"encountered an error: {str(e)}")
            with self._lock:
                self._running = False
                self._last_exception = e

    def run_analysis(self):
        """
        算法分析主循环，子类需要实现此方法
        """
        while not self._should_stop():
            try:
                # 子类应该重写此方法以实现具体的算法逻辑
                self.process()
                time.sleep(self._polling_interval)
            except Exception as e:
                self.elogger.error(f"Error in analysis: {str(e)}")
                # 短暂暂停后继续，避免快速失败
                time.sleep(0.1)

    def process(self):
        """
        算法处理逻辑，返回处理结果
        子类应该重写此方法
        """
        return None

    def get_state(self):
        """
        获取算法当前状态
        
        Returns:
            dict: 包含算法状态的字典
        """
        with self._lock:
            return {
                'running': self._running,
                'has_exception': self._last_exception is not None,
                'last_exception': str(self._last_exception) if self._last_exception else None,
                'last_heartbeat': self._last_heartbeat,
                'thread_alive': self._thread.is_alive() if self._thread else False
            }

    def _update_heartbeat(self):
        """
        更新心跳时间戳，用于监控算法是否正常运行
        """
        with self._lock:
            if self._running:
                self._last_heartbeat = time.time()
                # 如果线程已死但状态仍为running，自动重置状态
                if self._thread and not self._thread.is_alive():
                    self.elogger.warning("thread is dead but state is running, resetting state")
                    self._running = False
                    self._thread = None

    def __del__(self):
        """
        析构函数，确保线程正确停止
        """
        self.stop()

    def shutdown(self):
        """
        关闭算法，释放资源
        """
        self.stop()
        self.destroy_node()