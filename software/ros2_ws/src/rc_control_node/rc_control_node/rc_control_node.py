import rclpy
from rclpy.node import Node
import re
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RCControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')
        
        # 参数配置
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        # 创建发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # 状态变量
        self.channels = [1500] * 16
        self.enabled = False
        self.manual_control = False
        self.emergency_stop = True
        self.speed_level = 1
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0
        self.speed_multipliers = [0.5, 0.75, 1.0]
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
                timeout=0.05
            )
            self.get_logger().info(f'Connected to {self.port} at {self.baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {str(e)}')
            return
        
        # 定时器
        self.create_timer(0.02, self.read_serial_data)  # 50Hz
        self.create_timer(0.1, self.publish_cmd_vel)    # 10Hz
        
        self.get_logger().info('RC Control Node initialized')
        self.get_logger().info(f'Initial state - Emergency Stop: {self.emergency_stop}, Manual Control: {self.manual_control}')

    def read_serial_data(self):
        """读取串口数据并解析"""
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_sbus_data(line)
                    self.handle_switch_changes()
                    self.get_logger().debug(f'Raw channels: {self.channels[:10]}')
        except Exception as e:
            self.get_logger().warn(f'Error reading serial data: {str(e)}')

    def parse_sbus_data(self, sbus_line):
        """解析SBUS数据行"""
        try:
            matches = re.findall(r'CH(\d+):(\d+)', sbus_line)
            for ch_str, val_str in matches:
                ch_index = int(ch_str) - 1
                if 0 <= ch_index < 16:
                    self.channels[ch_index] = int(val_str)
        except Exception as e:
            self.get_logger().warn(f'Error parsing SBUS data: {sbus_line}, Error: {str(e)}')

    def map_value_to_range(self, value, from_min, from_max, to_min, to_max):
        """值映射"""
        value = max(from_min, min(from_max, value))
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def handle_switch_changes(self):
        """处理开关状态"""
        ch5 = self.channels[4]
        if self.prev_ch5 != ch5 and ch5 in [1000, 2000]:
            self.emergency_stop = (ch5 == 1000)
            self.get_logger().info(f'Emergency stop {"enabled" if self.emergency_stop else "disabled"}')
        ch6 = self.channels[5]
        if self.prev_ch6 != ch6 and ch6 in [1000, 2000]:
            self.manual_control = (ch6 == 2000)
            self.get_logger().info(f'{"Manual" if self.manual_control else "Auto"} control mode enabled')
        
        ch8 = self.channels[7]
        if ch8 < 1333:
            self.speed_level = 0
        elif ch8 > 1666:
            self.speed_level = 2
        else:
            self.speed_level = 1
        
        ch9 = self.channels[8]
        self.max_linear_speed = self.map_value_to_range(ch9, 1000, 2000, 0.0, 2.0)
        
        ch10 = self.channels[9]
        self.max_angular_speed = self.map_value_to_range(ch10, 1000, 2000, 0.0, 2.0)
        
        self.prev_ch5 = ch5
        self.prev_ch6 = ch6

    def publish_cmd_vel(self):
        """发布控制指令"""
        if not self.manual_control or self.emergency_stop:
            if self.manual_control and self.emergency_stop:
                self.get_logger().warn('Emergency stop is active, not publishing commands')
            return
        
        twist = Twist()
        ch1 = self.channels[0]
        twist.angular.z = self.map_value_to_range(ch1, 1000, 2000, -self.max_angular_speed, self.max_angular_speed) \
                          * self.speed_multipliers[self.speed_level]
        ch2 = self.channels[1]
        twist.linear.x = self.map_value_to_range(ch2, 1000, 2000, -self.max_linear_speed, self.max_linear_speed) \
                         * self.speed_multipliers[self.speed_level]
        
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(
            f'CMD: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f} | '
            f'Status: Manual={self.manual_control}, Emergency={self.emergency_stop}, '
            f'Speed Level={self.speed_level}({self.speed_multipliers[self.speed_level]*100}%)'
        )

    def destroy_node(self):
        """关闭串口"""
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
