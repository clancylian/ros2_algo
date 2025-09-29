import time
from std_msgs.msg import String
from .base import AlgoBase

class DeviceCheck(AlgoBase):
    """
    设备检查算法，用于检查设备状态
    """
    def __init__(self):
        """
        初始化设备检查算法
        """
        super().__init__('DeviceCheck', '/algo_result/device_check')

    def run_analysis(self):
        """
        设备检查算法主循环，定期发布检查结果
        """
        while not self._should_stop():
            try:
                msg = String()
                msg.data = f"DeviceCheck result at {time.time()}"
                self.pub.publish(msg)
                # 更新心跳，表示算法正常运行
                self._last_heartbeat = time.time()
                time.sleep(self._polling_interval)
            except Exception as e:
                self.elogger.error(f"Error in DeviceCheck analysis: {str(e)}")
                # 短暂暂停后继续，避免快速失败
                time.sleep(0.1)

    def process(self):
        """
        获取设备检查算法的当前状态
        
        Returns:
            str: 算法运行状态信息
        """
        state = self.get_state()
        if state['running']:
            return f"DeviceCheck running at {time.time()}"
        return None

def main():
    rclpy.init()
    node = DeviceCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
