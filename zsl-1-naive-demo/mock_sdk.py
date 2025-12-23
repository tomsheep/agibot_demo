import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MotorState:
    def __init__(self):
        # 关节顺序通常遵循 Unitree 约定：FR, FL, RR, RL
        self.q_abad = [0.0] * 4
        self.q_hip = [0.0] * 4
        self.q_knee = [0.0] * 4

class HighLevel(Node):
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        super().__init__('mock_sdk_node')

        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.current_motor_state = MotorState()
        self.first_run = True # 用于只打印一次调试信息

        # 启动线程处理回调
        self._thread = threading.Thread(target=self._spin_node, daemon=True)
        self._thread.start()
        self.get_logger().info("Mock SDK (Read-Only) 已启动...")

    def _spin_node(self):
        rclpy.spin(self)

    def initRobot(self, *args):
        # 模拟初始化接口
        self.get_logger().info("Robot initialized (Simulation).")

    def getMotorState(self):
        return self.current_motor_state

    def joint_callback(self, msg):
        """
        关键逻辑：将 Gazebo 的关节名称映射到 SDK 的数据结构
        """
        # 如果是第一次收到消息，打印一下看到的所有关节名，方便调试
        if self.first_run:
            self.get_logger().info(f">>> 调试信息：ROS2 收到的关节名称列表: {msg.name}")
            self.first_run = False

        # SDK 标准顺序：0:FR, 1:FL, 2:RR, 3:RL
        # 请确保这里的 list 顺序与你的实际腿部顺序一致
        leg_prefixes = ['FR', 'FL', 'RR', 'RL']

        try:
            for i, prefix in enumerate(leg_prefixes):
                # 【关键修复】这里必须与你的 URDF 中的关节名称完全一致（大小写敏感）
                # 根据你之前的 URDF，格式是 "XX_ABAD_JOINT" (全大写)

                # 1. 侧摆关节 (Abad)
                abad_name = f"{prefix}_ABAD_JOINT"
                if abad_name in msg.name:
                    idx = msg.name.index(abad_name)
                    self.current_motor_state.q_abad[i] = msg.position[idx]

                # 2. 髋关节 (Hip)
                hip_name = f"{prefix}_HIP_JOINT"
                if hip_name in msg.name:
                    idx = msg.name.index(hip_name)
                    self.current_motor_state.q_hip[i] = msg.position[idx]

                # 3. 膝关节 (Knee)
                knee_name = f"{prefix}_KNEE_JOINT"
                if knee_name in msg.name:
                    idx = msg.name.index(knee_name)
                    self.current_motor_state.q_knee[i] = msg.position[idx]

        except Exception as e:
            self.get_logger().error(f"解析关节数据出错: {e}")
