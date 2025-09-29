import time
import threading
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class LineIntegrity(Node):
    def __init__(self):
        super().__init__('LineIntegrity')
        self.pub = self.create_publisher(String, '/algo_result/line_integrity', 10)
        self.running = False
        self.thread = None

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run_analysis, daemon=True)
            self.thread.start()
            self.get_logger().info("LineIntegrity started")

    def stop(self):
        if self.running:
            self.running = False
            if self.thread:
                self.thread.join(timeout=2.0)
            self.get_logger().info("LineIntegrity stopped")

    def run_analysis(self):
        while self.running:
            msg = String()
            msg.data = f"LineIntegrity result at {time.time()}"
            self.pub.publish(msg)
            time.sleep(1)

    def process(self):
        if self.running:
            return f"LineIntegrity running at {time.time()}"
        return None

def main():
    rclpy.init()
    node = LineIntegrity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
