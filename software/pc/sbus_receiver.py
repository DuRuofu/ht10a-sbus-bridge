import sys
import re
import serial
import serial.tools.list_ports
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QComboBox, QGroupBox, 
                             QGridLayout, QTextEdit, QProgressBar)
from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtGui import QFont, QIntValidator, QTextCursor

class SerialReader(QThread):
    """串口读取线程"""
    data_received = Signal(str)
    error_occurred = Signal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        
    def start_connection(self, port_name, baudrate=115200):
        """启动串口连接"""
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.running = True
            self.start()  # 启动线程
        except Exception as e:
            self.error_occurred.emit(f"串口连接失败: {str(e)}")
    
    def stop_connection(self):
        """停止串口连接"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def run(self):
        """线程主循环 - 读取串口数据"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    # 读取一行数据
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
            except Exception as e:
                self.error_occurred.emit(f"读取数据错误: {str(e)}")
                break

class SBUSReceiverGUI(QMainWindow):
    """SBUS接收器主界面"""
    
    def __init__(self):
        super().__init__()
        self.serial_reader = SerialReader()
        self.serial_reader.data_received.connect(self.update_display)
        self.serial_reader.error_occurred.connect(self.show_error)
        
        self.init_ui()
        self.refresh_ports()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("HT-10A SBUS Receiver")
        self.setGeometry(100, 100, 900, 700)
        
        # 主中央窗口
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # 顶部连接控制区域
        connection_layout = QHBoxLayout()
        
        # 串口选择
        port_label = QLabel("Serial Port:")
        self.port_combo = QComboBox()
        self.refresh_ports_btn = QPushButton("Refresh Ports")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        
        # 波特率选择
        baudrate_label = QLabel("Baudrate:")
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["115200", "57600", "38400", "19200", "9600"])
        self.baudrate_combo.setCurrentText("115200")
        
        # 连接/断开按钮
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        # 添加到连接布局
        connection_layout.addWidget(port_label)
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.refresh_ports_btn)
        connection_layout.addWidget(baudrate_label)
        connection_layout.addWidget(self.baudrate_combo)
        connection_layout.addWidget(self.connect_btn)
        connection_layout.addStretch()
        
        main_layout.addLayout(connection_layout)
        
        # 通道数据显示区域
        channels_group = QGroupBox("Channel Values")
        channels_layout = QGridLayout(channels_group)
        
        # 创建10个通道的显示组件
        self.channel_labels = []
        self.channel_values = []
        self.channel_progress = []
        
        # 每行显示5个通道
        for i in range(10):
            ch_num = i + 1
            row = i // 5
            col = (i % 5) * 2  # 每个通道占用两列（标签+进度条）
            
            # 通道标签
            label = QLabel(f"CH{ch_num}:")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-weight: bold;")
            channels_layout.addWidget(label, row * 2, col)
            self.channel_labels.append(label)
            
            # 通道值显示
            value_label = QLabel("--")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setStyleSheet("background-color: #f0f0f0; padding: 5px; border: 1px solid gray;")
            channels_layout.addWidget(value_label, row * 2, col + 1)
            self.channel_values.append(value_label)
            
            # 通道进度条
            progress = QProgressBar()
            progress.setRange(1000, 2000)
            progress.setValue(0)
            progress.setFormat("%v")
            channels_layout.addWidget(progress, row * 2 + 1, col, 1, 2)
            self.channel_progress.append(progress)
        
        main_layout.addWidget(channels_group)
        
        # 通道说明区域
        info_group = QGroupBox("Channel Information")
        info_layout = QGridLayout(info_group)
        
        # 通道说明表格
        channel_info = [
            ("CH1", "Right Stick X (Yaw/Turn)"),
            ("CH2", "Right Stick Y (Throttle/Speed)"),
            ("CH3", "Left Stick Y (Forward/Throttle)"),
            ("CH4", "Left Stick X (Lateral/Rotate)"),
            ("CH5", "SWA (2-pos Switch, Auto-center)"),
            ("CH6", "SWB (2-pos Switch)"),
            ("CH7", "SWC (2-pos Switch)"),
            ("CH8", "SWD (3-pos Switch)"),
            ("CH9", "VRA (Knob)"),
            ("CH10", "VRB (Knob)")
        ]
        
        for i, (ch, desc) in enumerate(channel_info):
            row = i // 2
            col = (i % 2) * 2
            
            ch_label = QLabel(ch)
            ch_label.setStyleSheet("font-weight: bold;")
            info_layout.addWidget(ch_label, row, col)
            
            desc_label = QLabel(desc)
            info_layout.addWidget(desc_label, row, col + 1)
        
        main_layout.addWidget(info_group)
        
        # 串口输出日志区域
        log_group = QGroupBox("Serial Output Log")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        main_layout.addWidget(log_group)
        
        # 状态栏
        self.statusBar().showMessage("Disconnected")
        
    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        
        # 如果没有找到串口，显示提示
        if not ports:
            self.port_combo.addItem("No ports found")
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.connect_btn.text() == "Connect":
            self.connect_to_serial()
        else:
            self.disconnect_serial()
    
    def connect_to_serial(self):
        """连接到串口"""
        port_name = self.port_combo.currentText()
        if not port_name or port_name == "No ports found":
            self.statusBar().showMessage("Please select a valid serial port", 3000)
            return
            
        try:
            baudrate = int(self.baudrate_combo.currentText())
            self.serial_reader.start_connection(port_name, baudrate)
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(f"Connected to {port_name} at {baudrate} baud")
        except Exception as e:
            self.show_error(f"Connection failed: {str(e)}")
    
    def disconnect_serial(self):
        """断开串口连接"""
        self.serial_reader.stop_connection()
        self.connect_btn.setText("Connect")
        self.statusBar().showMessage("Disconnected")
        
        # 重置所有通道显示
        for i in range(10):
            self.channel_values[i].setText("--")
            self.channel_progress[i].setValue(0)
    
    def update_display(self, data):
        """更新显示数据"""
        # 将数据添加到日志
        self.log_text.append(data)
        # 限制日志行数
        if self.log_text.document().lineCount() > 100:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 1)
            cursor.removeSelectedText()
        
        # 解析数据
        channels = self.parse_sbus_data(data)
        if channels:
            for i in range(min(len(channels), 10)):
                value = channels[i]
                self.channel_values[i].setText(str(value))
                
                # 根据通道类型设置进度条
                if i in [4, 5, 6]:  # 开关通道 (CH5, CH6, CH7)
                    # 二位开关: 1000/2000 (自动回中: 1500)
                    self.channel_progress[i].setRange(1000, 2000)
                    self.channel_progress[i].setValue(value)
                elif i == 7:  # 三档开关 (CH8)
                    # 三位开关: 1000, 1500, 2000
                    self.channel_progress[i].setRange(1000, 2000)
                    self.channel_progress[i].setValue(value)
                else:  # 摇杆通道 (CH1-CH4, CH9-CH10)
                    # 摇杆: 1000-2000
                    self.channel_progress[i].setRange(1000, 2000)
                    self.channel_progress[i].setValue(value)
    
    def parse_sbus_data(self, data):
        """解析SBUS数据"""
        # 使用正则表达式匹配 "CHx:nnnn" 格式的字符串
        pattern = r"CH(\d+):(\d+)"
        matches = re.findall(pattern, data)
        
        # 创建通道值字典
        channels = {}
        for ch_num, value in matches:
            try:
                ch_idx = int(ch_num) - 1  # 转换为数组索引 (0-9)
                ch_val = int(value)
                if 0 <= ch_idx < 16:  # SBUS最多16个通道
                    channels[ch_idx] = ch_val
            except ValueError:
                continue
        
        # 返回按顺序排列的通道值列表
        result = []
        for i in range(10):
            result.append(channels.get(i, 0))
        
        return result
    
    def show_error(self, message):
        """显示错误信息"""
        self.statusBar().showMessage(message, 5000)
        self.log_text.append(f"ERROR: {message}")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self.serial_reader.stop_connection()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = SBUSReceiverGUI()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
