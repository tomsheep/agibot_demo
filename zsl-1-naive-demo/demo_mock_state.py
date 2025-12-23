import time
import os
import sys
import mock_sdk as sdk

def clear_screen():
    # 简单的清屏命令，让数据显示更像仪表盘
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    print("正在初始化 SDK...")
    dog = sdk.HighLevel()
    dog.initRobot()

    print("等待 ROS2 数据...")
    time.sleep(2) # 等待一会，确保 ROS 连接建立

    try:
        while True:
            # 1. 获取最新状态
            state = dog.getMotorState()

            # 2. 格式化打印
            # clear_screen() # 如果你想刷屏显示，取消这行注释

            print("-" * 50)
            print(f"Time: {time.time():.2f}")
            print("-" * 50)
            print("Leg  |  Abad(侧摆)  |   Hip(大腿)  |  Knee(膝盖)")
            print("-" * 50)

            legs = ["FR (右前)", "FL (左前)", "RR (右后)", "RL (左后)"]

            for i, name in enumerate(legs):
                q_a = state.q_abad[i]
                q_h = state.q_hip[i]
                q_k = state.q_knee[i]

                # 保留3位小数打印
                print(f"{name} | {q_a:8.3f}    | {q_h:8.3f}    | {q_k:8.3f}")

            print("-" * 50)
            print("提示：在 Gazebo/RViz 中拖动机器人关节，观察数值变化")

            time.sleep(0.5) # 2Hz 刷新率

    except KeyboardInterrupt:
        print("\n停止监控。")

if __name__ == '__main__':
    main()
