import rclpy
from rclpy.node import Node
import re
import serial
import serial.tools.list_ports
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class RCControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')
        
        # 参数配置
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        # 创建发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # 初始化状态变量
        self.channels = [0] * 16  # 存储所有通道的值
        self.enabled = False  # 使能状态
        self.manual_control = False  # 手动控制模式
        self.emergency_stop = True  # 急停状态 (默认为急停)
        self.speed_level = 1  # 速度档位 (0: 低 50%, 1: 中 75%, 2: 高 100%)
        self.max_linear_speed = 1.0  # CH9 旋钮调节的最大线速度
        self.max_angular_speed = 1.0  # CH10 旋钮调节的最大角速度
        
        # 速度档位映射 (低/中/高)
        self.speed_multipliers = [0.5, 0.75, 1.0]
        
        # 上一次CH5和CH6的值，用于检测变化
        self.prev_ch5 = 1500
        self.prev_ch6 = 1000
        
        # 串口连接
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info(f'Connected to {self.port} at {self.baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {str(e)}')
            return
        
        # 创建定时器读取串口数据
        self.timer = self.create_timer(0.02, self.read_serial_data)  # 50Hz
        
        # 创建定时器发布控制指令
        self.cmd_timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz
        
        self.get_logger().info('RC Control Node initialized')
    
    def read_serial_data(self):
        """读取串口数据并解析SBUS信号"""
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # 解析通道数据
                    self.parse_sbus_data(line)
                    # 处理开关状态变化
                    self.handle_switch_changes()
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {str(e)}')
    
    def parse_sbus_data(self, data):
        """解析SBUS数据"""
        # 使用正则表达式匹配 "CHx:nnnn" 格式的字符串
        pattern = r"CH(\d+):(\d+)"
        matches = re.findall(pattern, data)
        
        # 创建通道值字典
        for ch_num, value in matches:
            try:
                ch_idx = int(ch_num) - 1  # 转换为数组索引 (0-9)
                ch_val = int(value)
                if 0 <= ch_idx < 16:  # SBUS最多16个通道
                    self.channels[ch_idx] = ch_val
            except ValueError:
                continue
    
    def handle_switch_changes(self):
        """处理开关状态变化"""
        # 处理CH5: 急停 / 使能 (自动回中, 1000=急停, 2000=使能, 1500=自动回中)
        ch5 = self.channels[4] if len(self.channels) > 4 else 1500
        
        # 检测CH5状态变化 (只有从1000/2000变为1500时才触发)
        if self.prev_ch5 != ch5 and self.prev_ch5 in [1000, 2000]:
            if self.prev_ch5 == 1000 and ch5 == 1500:  # 从急停切换到中间位置
                self.emergency_stop = not self.emergency_stop
                self.get_logger().info(f'Emergency stop toggled: {self.emergency_stop}')
            elif self.prev_ch5 == 2000 and ch5 == 1500:  # 从使能切换到中间位置
                self.emergency_stop = not self.emergency_stop
                self.get_logger().info(f'Emergency stop toggled: {self.emergency_stop}')
        
        # 处理CH6: 手动/自动控制权 (1000=自动, 2000=手动)
        ch6 = self.channels[5] if len(self.channels) > 5 else 1000
        
        # 检测CH6状态变化 (只有从1000/2000变为1500时才触发)
        if self.prev_ch6 != ch6 and self.prev_ch6 in [1000, 2000]:
            if self.prev_ch6 == 1000 and ch6 == 1500:  # 从自动切换到中间位置
                self.manual_control = True
                self.get_logger().info('Manual control enabled')
            elif self.prev_ch6 == 2000 and ch6 == 1500:  # 从手动切换到中间位置
                self.manual_control = False
                self.get_logger().info('Manual control disabled')
        
        # 更新速度档位 (CH8: 三段开关)
        ch8 = self.channels[7] if len(self.channels) > 7 else 1500
        if ch8 < 1333:  # 低档
            self.speed_level = 0
        elif ch8 > 1666:  # 高档
            self.speed_level = 2
        else:  # 中档
            self.speed_level = 1
        
        # 更新最大线速度 (CH9)
        ch9 = self.channels[8] if len(self.channels) > 8 else 1500
        self.max_linear_speed = self.map_value_to_range(ch9, 1000, 2000, 0.0, 2.0)
        
        # 更新最大角速度 (CH10)
        ch10 = self.channels[9] if len(self.channels) > 9 else 1500
        self.max_angular_speed = self.map_value_to_range(ch10, 1000, 2000, 0.0, 2.0)
        
        # 记录当前值
        self.prev_ch5 = ch5
        self.prev_ch6 = ch6
    
    def map_value_to_range(self, value, from_min, from_max, to_min, to_max):
        """将值从一个范围映射到另一个范围"""
        if value < from_min:
            value = from_min
        if value > from_max:
            value = from_max
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
    
    def publish_cmd_vel(self):
        """发布控制指令"""
        # 只在手动控制模式下发布命令
        if not self.manual_control or self.emergency_stop:
            # 如果处于自动模式或急停状态，发布零速度
            if self.manual_control and self.emergency_stop:
                self.get_logger().warn('Emergency stop is active, not publishing commands')
            return
        
        # 创建Twist消息
        twist = Twist()
        
        # CH1: 右摇杆X (转向) - 映射到角速度
        ch1 = self.channels[0] if len(self.channels) > 0 else 1500
        angular_speed = self.map_value_to_range(ch1, 1000, 2000, -self.max_angular_speed, self.max_angular_speed)
        twist.angular.z = angular_speed * self.speed_multipliers[self.speed_level]
        
        # CH2: 右摇杆Y (前进/后退) - 映射到线速度
        ch2 = self.channels[1] if len(self.channels) > 1 else 1500
        linear_speed = self.map_value_to_range(ch2, 1000, 2000, -self.max_linear_speed, self.max_linear_speed)
        twist.linear.x = linear_speed * self.speed_multipliers[self.speed_level]
        
        # 发布cmd_vel消息
        self.cmd_vel_publisher.publish(twist)
        
        # 输出日志信息
        self.get_logger().info(
            f'CMD: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f} | '
            f'Status: Manual={self.manual_control}, Emergency={self.emergency_stop}, '
            f'Speed Level={self.speed_level}({self.speed_multipliers[self.speed_level]*100}%),' 
            f'Max Lin={self.max_linear_speed:.2f}, Max Ang={self.max_angular_speed:.2f}',
            throttle_duration_sec=1
        )
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RCControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()