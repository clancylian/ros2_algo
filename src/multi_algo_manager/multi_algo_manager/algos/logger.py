import time
import traceback
import os

class EnhancedLogger:
    """
    增强型日志工具，提供带文件行数和详细时间戳的日志功能
    """
    def __init__(self, node):
        """
        初始化增强型日志工具
        
        Args:
            node (Node): ROS2节点实例
        """
        self.node = node
        self.logger = node.get_logger()
    
    def _format_message(self, level, message):
        """
        格式化日志消息，添加时间戳和文件行数
        
        Args:
            level (str): 日志级别
            message (str): 日志消息
        
        Returns:
            str: 格式化后的日志消息
        """
        # 获取调用者的文件和行号信息
        stack = traceback.extract_stack()
        # 找到调用logger方法的位置（跳过enhanced_logger内部调用）
        for frame in reversed(stack):
            if frame.filename != __file__:
                file_path = frame.filename
                line_number = frame.lineno
                break
        else:
            file_path = "unknown"
            line_number = 0
        
        # 格式化时间戳
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S.%f', time.localtime(time.time()))[:-3]
        
        # 只保留文件名而不是完整路径
        file_name = os.path.basename(file_path)
        
        # 返回格式化的日志消息
        node_name = self.node.get_name()
        return f"[{timestamp}] [{node_name}] [{level}] [{file_name}:{line_number}] {message}"
    
    def debug(self, message):
        """记录调试级别的日志"""
        self.logger.debug(self._format_message("DEBUG", message))
    
    def info(self, message):
        """记录信息级别的日志"""
        self.logger.info(self._format_message("INFO", message))
    
    def warning(self, message):
        """记录警告级别的日志"""
        self.logger.warning(self._format_message("WARNING", message))
    
    def error(self, message):
        """记录错误级别的日志"""
        self.logger.error(self._format_message("ERROR", message))
    
    def fatal(self, message):
        """记录致命级别的日志"""
        self.logger.fatal(self._format_message("FATAL", message))