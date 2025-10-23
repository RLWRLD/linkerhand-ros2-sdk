import sys
import time
import threading
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject, QEvent
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QSlider, QLabel, QPushButton, QGroupBox, QScrollArea, QTabWidget, 
    QFrame, QSplitter, QMessageBox, QTextEdit
)
from PyQt5.QtGui import QFont

from .utils.mapping import *

# 配置和常量定义
@dataclass
class HandConfig:
    """手部配置数据类"""
    joint_names: List[str]
    joint_names_en: Optional[List[str]] = None
    init_pos: Optional[List[float]] = None
    preset_actions: Optional[Dict[str, List[float]]] = None
    is_arc: bool = False
    slider_min: Optional[List[float]] = None
    slider_max: Optional[List[float]] = None

    @classmethod
    def from_hand_type(cls, hand_type: str, hand_side: str = "left", is_arc: bool = False) -> 'HandConfig':
        """根据手部类型创建配置"""
        hand_type_upper = hand_type.upper()

        base_configs = {
            "L25": {
                "joint_names": ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部",
                                "大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆",
                                "大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部",
                                "中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖",
                                "中指指尖","无名指指尖","小拇指指尖"],
                "init_pos": [255] * 25,
                "preset_actions": {
                    "握拳": [0]*25,
                    "张开": [255]*25,
                    "OK": [255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                           255, 255, 255, 255, 255, 255, 0, 0, 255, 255,
                           0, 0, 0, 255, 255],
                },
            },
            "L21": {
                "joint_names": ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部",
                                "大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆",
                                "大拇指横滚","预留","预留","预留","预留","大拇指中部","预留",
                                "预留","预留","预留","大拇指指尖","食指指尖","中指指尖",
                                "无名指指尖","小拇指指尖"],
                "init_pos": [255] * 25,
            },
            "L20": {
                "joint_names": ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部",
                                "拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆",
                                "拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端",
                                "中指末端","无名指末端","小指末端"],
                "init_pos": [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255],
                "preset_actions": {
                    "握拳": [40, 0, 0, 0, 0, 131, 10, 100, 180, 240, 19, 255, 255, 255, 255, 135, 0, 0, 0, 0],
                    "张开": [255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
                    "OK": [191, 95, 255, 255, 255, 136, 107, 100, 180, 240, 72, 255, 255, 255, 255, 116, 99, 255, 255, 255],
                    "点赞": [255, 0, 0, 0, 0, 127, 10, 100, 180, 240, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0],
                },
            },
            "L10": {
                "joint_names_en": ["thumb_cmc_pitch", "thumb_cmc_roll","index_mcp_pitch","middle_mcp_pitch",
                                   "ring_mcp_pitch","pinky_mcp_pitch","index_mcp_roll","ring_mcp_roll",
                                   "pinky_mcp_roll","thumb_cmc_yaw"],
                "joint_names": ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部",
                                "小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"],
                "init_pos": [255] * 10,
                "preset_actions": {
                    "握拳": [75, 128, 0, 0, 0, 0, 128, 128, 128, 57],
                    "张开": [255, 128, 255, 255, 255, 255, 128, 128, 128, 128],
                    "OK": [110, 128, 75, 255, 255, 255, 128, 128, 128, 68],
                    "点赞": [255, 145, 0, 0, 0, 0, 0, 255, 255, 65],
                },
            },
            "L7": {
                "joint_names": ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲",
                                "小拇指弯曲","拇指旋转"],
                "init_pos": [250] * 7,
                "preset_actions": {
                    "握拳": [71, 79, 0, 0, 0, 0, 64],
                    "张开": [255, 111, 250, 250, 250, 250, 55],
                    "OK": [141, 111, 168, 250, 250, 250, 86],
                    "点赞": [255, 111, 0, 0, 0, 0, 86],
                },
            },
            "O6": {
                "joint_names_en": ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
                                   "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
                "joint_names": ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"],
                "init_pos": [250] * 6,
                "preset_actions": {
                    "张开": [250, 250, 250, 250, 250, 250],
                    "壹": [125, 18, 255, 0, 0, 0],
                    "贰": [92, 87, 255, 255, 0, 0],
                    "叁": [92, 87, 255, 255, 255, 0],
                    "肆": [92, 87, 255, 255, 255, 255],
                    "伍": [255, 255, 255, 255, 255, 255],
                    "OK": [96, 100, 118, 250, 250, 250],
                    "点赞": [250, 79, 0, 0, 0, 0],
                    "握拳": [102, 18, 0, 0, 0, 0],
                },
            },
            "L6": {
                "joint_names_en": ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
                                   "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
                "joint_names": ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"],
                "init_pos": [250] * 6,
                "preset_actions": {
                    "张开": [250, 250, 250, 250, 250, 250],
                    "壹": [125, 18, 255, 0, 0, 0],
                    "贰": [92, 87, 255, 255, 0, 0],
                    "叁": [92, 87, 255, 255, 255, 0],
                    "肆": [92, 87, 255, 255, 255, 255],
                    "伍": [255, 255, 255, 255, 255, 255],
                    "OK": [96, 100, 118, 250, 250, 250],
                    "点赞": [250, 79, 0, 0, 0, 0],
                    "握拳": [102, 18, 0, 0, 0, 0],
                },
            },
            "L6P": {
                "joint_names_en": ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
                                   "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
                "joint_names": ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"],
                "init_pos": [250] * 6,
                "preset_actions": {
                    "张开": [250, 250, 250, 250, 250, 250],
                    "壹": [0, 31, 255, 0, 0, 0],
                    "贰": [0, 31, 255, 255, 0, 0],
                    "叁": [0, 30, 255, 255, 255, 0],
                    "肆": [0, 30, 255, 255, 255, 255],
                    "伍": [250, 250, 250, 250, 250, 250],
                    "OK": [54, 41, 164, 250, 250, 250],
                    "点赞": [255, 31, 0, 0, 0, 0],
                    "握拳": [49, 61, 0, 0, 0, 0],
                },
            },
        }

        base = base_configs.get(hand_type_upper, base_configs["L10"])

        joint_names = list(base["joint_names"])  # type: ignore[index]
        joint_names_en_data = base.get("joint_names_en")
        joint_names_en = list(joint_names_en_data) if joint_names_en_data else None
        init_pos = list(base["init_pos"])  # type: ignore[index]
        preset_actions_data = base.get("preset_actions", {})
        preset_actions = dict(preset_actions_data) if preset_actions_data else {}

        config = cls(
            joint_names=joint_names,
            joint_names_en=joint_names_en,
            init_pos=init_pos,
            preset_actions=preset_actions,
            is_arc=is_arc,
            slider_min=[0.0] * len(joint_names),
            slider_max=[255.0] * len(joint_names),
        )

        if is_arc:
            arc_limits = cls._get_arc_limits(hand_side, hand_type_upper)
            if arc_limits:
                min_vals, max_vals = arc_limits
                config.slider_min = list(min_vals)
                config.slider_max = list(max_vals)
                config.init_pos = cls._range_to_arc(init_pos, hand_side, hand_type_upper)
            else:
                config.slider_min = None
                config.slider_max = None
            config.preset_actions = {}

        return config

    @staticmethod
    def _get_arc_limits(hand_side: str, hand_type: str) -> Optional[Tuple[List[float], List[float]]]:
        lookup: Dict[Tuple[str, str], Tuple[List[float], List[float]]] = {
            ("left", "O6"): (o6_l_min, o6_l_max),
            ("right", "O6"): (o6_r_min, o6_r_max),
            ("left", "L7"): (l7_l_min, l7_l_max),
            ("right", "L7"): (l7_r_min, l7_r_max),
            ("left", "L10"): (l10_l_min, l10_l_max),
            ("right", "L10"): (l10_r_min, l10_r_max),
            ("left", "L20"): (l20_l_min, l20_l_max),
            ("right", "L20"): (l20_r_min, l20_r_max),
            ("left", "L21"): (l21_l_min, l21_l_max),
            ("right", "L21"): (l21_r_min, l21_r_max),
            ("left", "L25"): (l25_l_min, l25_l_max),
            ("right", "L25"): (l25_r_min, l25_r_max),
        }
        key = (hand_side.lower(), hand_type.upper())
        return lookup.get(key)

    @staticmethod
    def _range_to_arc(positions: List[float], hand_side: str, hand_type: str) -> List[float]:
        try:
            if hand_side.lower() == "left":
                arc_positions = range_to_arc_left(positions, hand_joint=hand_type)
            else:
                arc_positions = range_to_arc_right(positions, hand_joint=hand_type)
        except Exception:
            arc_positions = []

        if not arc_positions:
            return [0.0] * len(positions)
        return arc_positions


ARC_SLIDER_SCALE = 1000

class ROS2NodeManager(QObject):
    """ROS2节点管理器，处理ROS通信"""
    status_updated = pyqtSignal(str, str)  # 状态类型, 消息内容

    def __init__(self, node_name: str = "hand_control_node"):
        super().__init__()
        self.node = None
        self.publisher = None
        self.joint_state = JointState()
        self.joint_state.header = Header()
        
        # 初始化ROS2节点
        self.init_node(node_name)

    def init_node(self, node_name: str):
        """初始化ROS2节点"""
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = Node(node_name)
            
            # 声明参数
            self.node.declare_parameter('hand_type', 'left')
            self.node.declare_parameter('hand_joint', 'L20')
            self.node.declare_parameter('topic_hz', 30)
            self.node.declare_parameter('is_arc', False)
            
            # 获取参数
            self.hand_type = self.node.get_parameter('hand_type').value
            self.hand_joint = self.node.get_parameter('hand_joint').value
            self.hz = self.node.get_parameter('topic_hz').value
            self.is_arc = self.node.get_parameter('is_arc').value
            
            if self.is_arc is True:
                # 创建发布者
                self.publisher = self.node.create_publisher(
                    JointState, f'/linkerhand_{self.hand_type}/reference', 10
                )
            else:
                # 创建发布者
                self.publisher = self.node.create_publisher(
                    JointState, f'/cb_{self.hand_type}_hand_control_cmd', 10
                )
            
            self.status_updated.emit("info", f"ROS2节点初始化成功: {self.hand_type} {self.hand_joint}")
            
            # 启动ROS2自旋线程
            self.spin_thread = threading.Thread(target=self.spin_node, daemon=True)
            self.spin_thread.start()
        except Exception as e:
            self.status_updated.emit("error", f"ROS2初始化失败: {str(e)}")
            raise

    def spin_node(self):
        """运行ROS2节点自旋循环"""
        while rclpy.ok() and self.node:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def publish_joint_state(self, positions: List[float]):
        """发布关节状态消息"""
        if not self.publisher or not self.node:
            self.status_updated.emit("error", "ROS2发布者未初始化")
            return
            
        try:
            self.joint_state.header.stamp = self.node.get_clock().now().to_msg()
            self.joint_state.position = [float(pos) for pos in positions]
            # self.joint_state.velocity = [0.1] * len(positions)
            # self.joint_state.effort = [0.01] * len(positions)
            # 如果有关节名称，添加到消息中
            hand_config = HandConfig.from_hand_type(
                self.hand_joint, self.hand_type, self.is_arc
            )
            if hand_config.joint_names and len(hand_config.joint_names) == len(positions):
                if hand_config.joint_names_en:
                    self.joint_state.name = hand_config.joint_names_en
                else:
                    self.joint_state.name = hand_config.joint_names
                
            # if self.is_arc == True:
            #     if self.hand_joint == "O6":
            #         if self.hand_type == "left":
            #             pose = range_to_arc_left(positions,self.hand_joint)
            #         elif self.hand_type == "right":
            #             pose = range_to_arc_right(positions,self.hand_joint)
            #     elif self.hand_joint == "L7" or self.hand_joint == "L21" or self.hand_joint == "L25":
            #         if self.hand_type == "left":
            #             pose = range_to_arc_left(positions,self.hand_joint)
            #         elif self.hand_type == "right":
            #             pose = range_to_arc_right(positions,self.hand_joint)
            #     elif self.hand_joint == "L10":
            #         if self.hand_type == "left":
            #             pose = range_to_arc_left_10(positions)
            #         elif self.hand_type == "right":
            #             pose = range_to_arc_right_10(positions)
            #     elif self.hand_joint == "L20":
            #         if self.hand_type == "left":
            #             pose = range_to_arc_left_l20(positions)
            #         elif self.hand_type == "right":
            #             pose = range_to_arc_right_l20(positions)
            #     else:
            #         print(f"当前{self.hand_joint} {self.hand_type}不支持弧度转换", flush=True)
            #     self.joint_state.position = [float(pos) for pos in pose]
                
            self.publisher.publish(self.joint_state)
            self.status_updated.emit("info", "关节状态已发布")
        except Exception as e:
            self.status_updated.emit("error", f"发布失败: {str(e)}")

    def shutdown(self):
        """关闭ROS2节点"""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

class HandControlGUI(QWidget):
    """灵巧手控制界面"""
    status_updated = pyqtSignal(str, str)  # 状态类型, 消息内容

    def __init__(self, ros_manager: ROS2NodeManager):
        super().__init__()
        
        # 循环控制变量
        self.cycle_timer = None  # 循环定时器
        self.current_action_index = -1  # 当前动作索引
        self.preset_buttons = []  # 存储预设动作按钮引用
        
        # 设置ROS管理器
        self.ros_manager = ros_manager
        self.ros_manager.status_updated.connect(self.update_status)
        
        # 获取手部配置
        self.hand_joint = self.ros_manager.hand_joint
        self.hand_type = self.ros_manager.hand_type
        self.is_arc = self.ros_manager.is_arc
        self.hand_config = HandConfig.from_hand_type(
            self.hand_joint, self.hand_type, self.is_arc
        )
        self.slider_is_arc: List[bool] = []
        self.slider_scales: List[float] = []
        
        # 初始化UI
        self.init_ui()
        
        # 设置定时器发布关节状态
        self.publish_timer = QTimer(self)
        self.publish_timer.setInterval(int(1000 / self.ros_manager.hz))
        self.publish_timer.timeout.connect(self.publish_joint_state)
        self.publish_timer.start()

    def init_ui(self):
        """初始化用户界面"""
        # 设置窗口属性
        self.setWindowTitle(f'灵巧手控制界面 - {self.hand_type} {self.hand_joint}')
        self.setMinimumSize(1200, 900)
        
        # 设置样式
        self.setStyleSheet("""
            QWidget {
                font-family: 'Microsoft YaHei', 'SimHei', sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #CCCCCC;
                border-radius: 6px;
                margin-top: 6px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #165DFF;
                font-weight: bold;
            }
            QSlider::groove:horizontal {
                border: 1px solid #999999;
                height: 8px;
                border-radius: 4px;
                background: #CCCCCC;
                margin: 2px 0;
            }
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #165DFF, stop:1 #0E42D2);
                border: 1px solid #5C8AFF;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 5px 10px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #F0F0F0;
            }
            QPushButton:pressed {
                background-color: #D0D0D0;
            }
            QPushButton[category="preset"] {
                background-color: #E6F7FF;
                color: #1890FF;
                border-color: #91D5FF;
            }
            QPushButton[category="preset"]:hover {
                background-color: #B3E0FF;
            }
            QPushButton[category="action"] {
                background-color: #FFF7E6;
                color: #FA8C16;
                border-color: #FFD591;
            }
            QPushButton[category="action"]:hover {
                background-color: #FFE6B3;
            }
            QPushButton[category="danger"] {
                background-color: #FFF1F0;
                color: #F5222D;
                border-color: #FFCCC7;
            }
            QPushButton[category="danger"]:hover {
                background-color: #FFE8E6;
            }
            QLabel#StatusLabel {
                padding: 5px;
                border-radius: 4px;
            }
            QLabel#StatusInfo {
                background-color: #F0F7FF;
                color: #0066CC;
            }
            QLabel#StatusError {
                background-color: #FFF0F0;
                color: #CC0000;
            }
            /* 数值显示面板样式 */
            QTextEdit#ValueDisplay {
                background-color: #F8F8F8;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 10px;
                font-family: Consolas, monospace;
                font-size: 12px;
            }
        """)
        
        # 创建主垂直布局
        main_layout = QVBoxLayout(self)
        
        # 创建水平分割器（原有三个面板）
        splitter = QSplitter(Qt.Horizontal)
        
        # 创建左侧关节控制面板
        self.joint_control_panel = self.create_joint_control_panel()
        splitter.addWidget(self.joint_control_panel)
        
        # 创建中间预设动作面板
        self.preset_actions_panel = self.create_preset_actions_panel()
        splitter.addWidget(self.preset_actions_panel)
        
        # 创建右侧状态监控面板
        self.status_monitor_panel = self.create_status_monitor_panel()
        splitter.addWidget(self.status_monitor_panel)
        
        # 设置分割器比例
        splitter.setSizes([500, 300, 400])
        
        # 添加分割器到主布局，并设置拉伸因子为1（可伸缩）
        main_layout.addWidget(splitter, stretch=1)
        
        # 创建并添加数值显示面板，设置拉伸因子为0（不可伸缩）
        self.value_display_panel = self.create_value_display_panel()
        main_layout.addWidget(self.value_display_panel, stretch=0)
        
        # 初始更新数值显示
        self.update_value_display()

    def create_joint_control_panel(self):
        """创建关节控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 创建标题
        title_label = QLabel(f"关节控制 - {self.hand_joint}")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # 创建滑动条滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        self.sliders_layout = QGridLayout(scroll_content)
        self.sliders_layout.setSpacing(10)
        
        # 创建滑动条
        self.create_joint_sliders()
        
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        return panel

    

    def create_joint_sliders(self):
        """创建关节滑动条"""
        # 清除现有滑动条
        for i in reversed(range(self.sliders_layout.count())):
            item = self.sliders_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
        
        # 创建新滑动条
        self.sliders = []
        self.slider_labels = []
        self.slider_is_arc = []
        self.slider_scales = []

        joint_names = self.hand_config.joint_names or []
        init_positions = self.hand_config.init_pos or [0.0] * len(joint_names)
        slider_min = self.hand_config.slider_min or []
        slider_max = self.hand_config.slider_max or []

        arc_mode = (
            self.is_arc
            and slider_min
            and slider_max
            and len(slider_min) >= len(joint_names)
            and len(slider_max) >= len(joint_names)
        )
        
        for idx, name in enumerate(joint_names):
            initial_value = float(init_positions[idx]) if idx < len(init_positions) else 0.0

            label = QLabel()
            label.setMinimumWidth(120)
            
            slider = QSlider(Qt.Horizontal)

            if arc_mode:
                min_val = float(slider_min[idx])
                max_val = float(slider_max[idx])
                if min_val > max_val:
                    min_val, max_val = max_val, min_val

                scale = ARC_SLIDER_SCALE
                scaled_min = int(round(min_val * scale))
                scaled_max = int(round(max_val * scale))
                if scaled_min == scaled_max:
                    scaled_max = scaled_min + 1

                slider.setMinimum(scaled_min)
                slider.setMaximum(scaled_max)
                slider.setSingleStep(1)

                slider_value = int(round(initial_value * scale))
                slider_value = max(scaled_min, min(slider_value, scaled_max))
                slider.setValue(slider_value)

                self.slider_is_arc.append(True)
                self.slider_scales.append(float(scale))
            else:
                slider.setMinimum(0)
                slider.setMaximum(255)
                slider.setSingleStep(1)

                slider_value = int(round(initial_value))
                slider_value = max(0, min(slider_value, 255))
                slider.setValue(slider_value)

                self.slider_is_arc.append(False)
                self.slider_scales.append(1.0)

            slider.valueChanged.connect(
                lambda val, slider_index=idx: self.on_slider_value_changed(slider_index, val)
            )

            actual_value = self._slider_raw_to_actual(idx, slider.value())
            label.setText(f"{name}: {self._format_slider_value(actual_value, self.slider_is_arc[idx])}")

            row = idx
            self.sliders_layout.addWidget(label, row, 0)
            self.sliders_layout.addWidget(slider, row, 1)

            self.sliders.append(slider)
            self.slider_labels.append(label)

    def _is_arc_slider(self, index: int) -> bool:
        return 0 <= index < len(self.slider_is_arc) and self.slider_is_arc[index]

    def _slider_raw_to_actual(self, index: int, raw_value: int) -> float:
        if self._is_arc_slider(index):
            scale = self.slider_scales[index] if index < len(self.slider_scales) else ARC_SLIDER_SCALE
            return raw_value / scale
        return float(raw_value)

    def _actual_to_slider_raw(self, index: int, actual_value: float) -> int:
        if self._is_arc_slider(index):
            scale = self.slider_scales[index]
            return int(round(actual_value * scale))
        return int(round(actual_value))

    @staticmethod
    def _format_slider_value(value: float, is_arc: bool) -> str:
        return f"{value:.3f}" if is_arc else f"{int(round(value))}"

    def create_preset_actions_panel(self):
        """创建预设动作面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 系统预设动作
        sys_preset_group = QGroupBox("系统预设")
        sys_preset_layout = QGridLayout(sys_preset_group)
        sys_preset_layout.setSpacing(8)
        
        # 添加系统预设动作按钮
        self.create_system_preset_buttons(sys_preset_layout)
        layout.addWidget(sys_preset_group)
        
        # 添加动作按钮
        actions_layout = QHBoxLayout()
        
        # 添加循环运行按钮
        self.cycle_button = QPushButton("循环预设动作")
        self.cycle_button.setProperty("category", "action")
        self.cycle_button.clicked.connect(self.on_cycle_clicked)
        actions_layout.addWidget(self.cycle_button)
        
        self.home_button = QPushButton("回到初始位置")
        self.home_button.setProperty("category", "action")
        self.home_button.clicked.connect(self.on_home_clicked)
        actions_layout.addWidget(self.home_button)
        
        self.stop_button = QPushButton("停止所有动作")
        self.stop_button.setProperty("category", "danger")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        actions_layout.addWidget(self.stop_button)
        
        layout.addLayout(actions_layout)
        
        return panel

    def create_system_preset_buttons(self, parent_layout):
        """创建系统预设动作按钮"""
        self.preset_buttons = []  # 清空按钮列表
        if self.hand_config.preset_actions:
            buttons = []
            for idx, (name, positions) in enumerate(self.hand_config.preset_actions.items()):
                button = QPushButton(name)
                button.setProperty("category", "preset")
                button.clicked.connect(
                    lambda checked, pos=positions: self.on_preset_action_clicked(pos)
                )
                buttons.append(button)
                self.preset_buttons.append(button)  # 保存按钮引用
                
            # 添加到网格布局
            cols = 2
            for i, button in enumerate(buttons):
                row, col = divmod(i, cols)
                parent_layout.addWidget(button, row, col)

    def create_status_monitor_panel(self):
        """创建状态监控面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 创建标题
        title_label = QLabel("状态监控")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)
        
        # 创建标签页
        tab_widget = QTabWidget()
        
        # 系统信息标签页
        sys_info_widget = QWidget()
        sys_info_layout = QVBoxLayout(sys_info_widget)
        
        # 连接状态
        conn_group = QGroupBox("连接状态")
        conn_layout = QVBoxLayout(conn_group)
        
        if self.ros_manager.publisher.get_subscription_count() > 0:
            self.connection_status = QLabel("ROS2节点已连接")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusInfo")
        else:
            self.connection_status = QLabel("ROS2节点未连接")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusError")
            
        conn_layout.addWidget(self.connection_status)
        
        # 手部信息
        hand_info_group = QGroupBox("手部信息")
        hand_info_layout = QVBoxLayout(hand_info_group)
        
        info_text = f"""手部类型: {self.hand_type}
关节型号: {self.hand_joint}
关节数量: {len(self.hand_config.joint_names)}
发布频率: {self.ros_manager.hz} Hz"""
        self.hand_info_label = QLabel(info_text)
        self.hand_info_label.setWordWrap(True)
        hand_info_layout.addWidget(self.hand_info_label)
        
        sys_info_layout.addWidget(conn_group)
        sys_info_layout.addWidget(hand_info_group)
        sys_info_layout.addStretch()
        
        tab_widget.addTab(sys_info_widget, "系统信息")
        
        # 状态日志标签页
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        self.status_log = QLabel("等待系统启动...")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")
        self.status_log.setWordWrap(True)
        self.status_log.setMinimumHeight(300)
        
        log_layout.addWidget(self.status_log)
        
        # 清除日志按钮
        clear_log_btn = QPushButton("清除日志")
        clear_log_btn.clicked.connect(self.clear_status_log)
        log_layout.addWidget(clear_log_btn)
        
        tab_widget.addTab(log_widget, "状态日志")
        
        layout.addWidget(tab_widget)
        
        return panel

    def create_value_display_panel(self):
        """创建滑动条数值显示面板"""
        panel = QGroupBox("关节数值列表")
        layout = QVBoxLayout(panel)
        
        # 设置布局上下间隔为20像素
        layout.setContentsMargins(10, 20, 10, 20)
        
        self.value_display = QTextEdit()
        self.value_display.setObjectName("ValueDisplay")
        self.value_display.setReadOnly(True)  # 设置只读模式，允许复制
        self.value_display.setMinimumHeight(60)  # 调整最小高度
        self.value_display.setMaximumHeight(80)  # 限制最大高度
        self.value_display.setText("[]")
        
        layout.addWidget(self.value_display)
        
        return panel

    def on_slider_value_changed(self, index: int, value: int):
        """滑动条值改变事件处理"""
        if 0 <= index < len(self.slider_labels):
            joint_names = self.hand_config.joint_names or []
            joint_name = joint_names[index] if index < len(joint_names) else f"Joint {index}"
            actual_value = self._slider_raw_to_actual(index, value)
            display_value = self._format_slider_value(actual_value, self._is_arc_slider(index))
            self.slider_labels[index].setText(f"{joint_name}: {display_value}")
            
        # 更新数值显示
        self.update_value_display()

    def update_value_display(self):
        """更新数值显示面板内容"""
        # 获取所有滑动条的当前值
        formatted_values = []
        for i, slider in enumerate(self.sliders):
            actual_value = self._slider_raw_to_actual(i, slider.value())
            if self._is_arc_slider(i):
                formatted_values.append(round(actual_value, 3))
            else:
                formatted_values.append(int(round(actual_value)))
        
        # 格式化显示为列表形式
        self.value_display.setText(f"{formatted_values}")

    def on_preset_action_clicked(self, positions: List[float]):
        """预设动作按钮点击事件处理"""
        if len(positions) != len(self.sliders):
            QMessageBox.warning(
                self, "动作不匹配", 
                f"预设动作关节数量({len(positions)})与当前关节数量({len(self.sliders)})不匹配"
            )
            return
            
        # 更新滑动条
        for i, (slider, pos) in enumerate(zip(self.sliders, positions)):
            target_actual = float(pos)
            raw_value = self._actual_to_slider_raw(i, target_actual)
            raw_value = max(slider.minimum(), min(raw_value, slider.maximum()))
            slider.setValue(raw_value)
            self.on_slider_value_changed(i, slider.value())
            
        # 发布关节状态
        self.publish_joint_state()

    def on_home_clicked(self):
        """回到初始位置按钮点击事件处理"""
        init_positions = self.hand_config.init_pos or []
        for i, slider in enumerate(self.sliders):
            target_actual = float(init_positions[i]) if i < len(init_positions) else 0.0
            raw_value = self._actual_to_slider_raw(i, target_actual)
            raw_value = max(slider.minimum(), min(raw_value, slider.maximum()))
            slider.setValue(raw_value)
            
        self.publish_joint_state()
        self.status_updated.emit("info", "回到初始位置")
        
        # 更新数值显示
        self.update_value_display()

    def on_stop_clicked(self):
        """停止所有动作按钮点击事件处理"""
        # 停止循环定时器
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("循环运行预设动作")
            self.reset_preset_buttons_color()
            
        self.status_updated.emit("warning", "已停止所有动作")

    def on_cycle_clicked(self):
        """循环运行预设动作按钮点击事件处理"""
        if not self.hand_config.preset_actions:
            QMessageBox.warning(self, "无预设动作", "当前手部型号没有预设动作可循环运行")
            return
            
        if self.cycle_timer and self.cycle_timer.isActive():
            # 停止循环
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("循环运行预设动作")
            self.reset_preset_buttons_color()
            self.status_updated.emit("info", "已停止循环运行预设动作")
        else:
            # 开始循环
            self.current_action_index = -1  # 重置索引
            self.cycle_timer = QTimer(self)
            self.cycle_timer.timeout.connect(self.run_next_action)
            self.cycle_timer.start(2000)  # 2秒间隔
            self.cycle_button.setText("停止循环运行")
            self.status_updated.emit("info", "开始循环运行预设动作")
            self.run_next_action()  # 立即运行第一个动作

    def run_next_action(self):
        """运行下一个预设动作"""
        if not self.hand_config.preset_actions:
            return
            
        # 重置所有按钮颜色
        self.reset_preset_buttons_color()
        
        # 计算下一个动作索引
        self.current_action_index = (self.current_action_index + 1) % len(self.hand_config.preset_actions)
        
        # 获取下一个动作
        action_names = list(self.hand_config.preset_actions.keys())
        action_name = action_names[self.current_action_index]
        action_positions = self.hand_config.preset_actions[action_name]
        
        # 执行动作
        self.on_preset_action_clicked(action_positions)
        
        # 高亮当前动作按钮
        if 0 <= self.current_action_index < len(self.preset_buttons):
            button = self.preset_buttons[self.current_action_index]
            button.setStyleSheet("background-color: green; color: white; border-color: #91D5FF;")
            
        self.status_updated.emit("info", f"运行预设动作: {action_name}")

    def reset_preset_buttons_color(self):
        """重置所有预设按钮颜色"""
        for button in self.preset_buttons:
            button.setStyleSheet("")  # 恢复默认样式
            button.setProperty("category", "preset")  # 恢复类别属性
            # 强制样式刷新
            button.style().unpolish(button)
            button.style().polish(button)

    def on_joint_type_changed(self, joint_type: str):
        """关节类型改变事件处理"""
        self.hand_joint = joint_type
        self.hand_config = HandConfig.from_hand_type(
            joint_type, self.hand_type, self.is_arc
        )
        
        # 更新手部信息
        info_text = f"""手部类型: {self.hand_type}
关节型号: {self.hand_joint}
关节数量: {len(self.hand_config.joint_names)}
发布频率: {self.ros_manager.hz} Hz"""
        self.hand_info_label.setText(info_text)
        
        # 重新创建滑动条和预设按钮
        self.create_joint_sliders()
        self.create_system_preset_buttons(self.sys_preset_layout)  # 假设sys_preset_layout是类变量
        
        # 更新数值显示
        self.update_value_display()
        self.status_updated.emit("info", f"已切换到手部型号: {joint_type}")

    def publish_joint_state(self):
        """发布当前关节状态"""
        positions = [
            self._slider_raw_to_actual(i, slider.value())
            for i, slider in enumerate(self.sliders)
        ]
        self.ros_manager.publish_joint_state(positions)

    def update_status(self, status_type: str, message: str):
        """更新状态显示"""
        # 更新连接状态
        if status_type == "info" and "ROS2节点初始化成功" in message:
            self.connection_status.setText("ROS2节点已连接")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusInfo")
            
        # 更新日志
        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        current_log = self.status_log.text()
        
        if len(current_log) > 10000:  # 限制日志长度
            current_log = current_log[-10000:]
            
        self.status_log.setText(log_entry + current_log)
        
        # 设置日志样式
        self.status_log.setObjectName("StatusLabel")
        if status_type == "error":
            self.status_log.setObjectName("StatusError")
        else:
            self.status_log.setObjectName("StatusInfo")

    def clear_status_log(self):
        """清除状态日志"""
        self.status_log.setText("日志已清除")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")

    def closeEvent(self, event):
        """窗口关闭事件处理"""
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
        super().closeEvent(event)

def main(args=None):
    """主函数"""
    try:
        # 创建ROS2节点管理器
        ros_manager = ROS2NodeManager()
        
        # 创建Qt应用
        app = QApplication(sys.argv)
        
        # 创建GUI
        window = HandControlGUI(ros_manager)
        
        # 连接状态更新信号
        ros_manager.status_updated.connect(window.update_status)
        window.status_updated = ros_manager.status_updated
        
        # 显示窗口
        window.show()
        
        # 运行应用
        exit_code = app.exec_()
        
        # 清理ROS2
        if rclpy.ok():
            ros_manager.node.destroy_node()
            rclpy.shutdown()
            
        sys.exit(exit_code)
    except Exception as e:
        print(f"应用程序启动失败: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main()
