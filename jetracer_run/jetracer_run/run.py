import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 重い処理のためにマルチスレッド化
from nav_msgs.msg import Odometry

import time
import json

class PwmMotorControlNode(Node):
    """
    オドメトリを購読し、タイマーに基づく制御ループでPWMモーターを直接制御するノード
    """

    def __init__(self):
        super().__init__('pwm_motor_control_node')
                
        # 最新のオドメトリデータを保存する変数
        self.latest_odom = Odometry()
        
        # Subscriberの設定
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback, 
            10)
            
        # 1. 状態管理変数
        self.current_state = 'STATE_1_START'
        
        # チェックポイント座標の定義
        # スタート3の中央からスタートしたと仮定したときの座標
        # スタート位置補正など入れるか、スタート位置を必ず同じにするようにする
        self.CHECKPOINTS = {
            'CP_A': {'x1': 3.10, 'y1': -4.0, 'x2': 2.80, 'y2': -6.0}, # 状態1 -> 状態2
            'CP_B': {'x1': 1.5, 'y1': -2.6, 'x2': 0.5, 'y2': -4.0}, # 状態2 -> 状態3
            'CP_C': {'x1': 0.0, 'y1': 0.7, 'x2': -0.5, 'y2': -0.7}  # 状態3 -> 状態1 (ループ)
        }
        
        # 2. 状態ごとの実行関数を格納した辞書 (Pythonにおけるswitchの代わり)
        self.state_handlers = {
            'STATE_1_START': self.handle_state_1_start,
            'STATE_2_PASS_A': self.handle_state_2_pass_a,
            'STATE_3_PASS_B': self.handle_state_3_pass_b,
        }
        
        # 3. タイマー設定 (制御ループ)
        timer_period = 1.0 / 30.0
        self.create_timer(timer_period, self.control_loop_callback)
        self.get_logger().info(f'Current State: {self.current_state}') 
        # self.get_logger().info('Odometry PWM Control Node initialized. Control frequency: 10 Hz')
        
    def is_goal_reached(self, current_x, current_y, checkpoint_key):
            """
            現在の座標が指定されたチェックポイントの許容誤差内にあるか判定する
            """
            # ゴールフラグ
            reach_goal = False
            
            # チェックポイントの座標を取得
            cp = self.CHECKPOINTS[checkpoint_key]
            
            # xの範囲、yの範囲に入っているか判定
            is_x_within = (cp['x2'] <= current_x <= cp['x1'])
            is_y_within = (cp['y2'] <= current_y <= cp['y1'])
            
            # x,yともに範囲に入っていればフラグをTrueに
            if is_x_within and is_y_within:
                reach_goal = True
            
            return reach_goal
    
    def handle_state_1_start(self):
        """ 
        状態 1: スタート (CP_Aに向かって走行) 
        """
        # 現在のオドメトリを取得
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
                
        # 2. 遷移判定: CP_Aに到達したか
        if self.is_goal_reached(current_x, current_y, 'CP_A'):
            self.transition_to('STATE_2_PASS_A') # 状態2へ

    def handle_state_2_pass_a(self):
        """ 
        状態 2: CP_A通過後 (CP_Bに向かって走行) 
        """
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
                
        # 2. 遷移判定: CP_Bに到達したか
        if self.is_goal_reached(current_x, current_y, 'CP_B'):
            self.transition_to('STATE_3_PASS_B') # 状態3へ

    def handle_state_3_pass_b(self):
        """ 
        状態 3: CP_B通過後 (CP_Cに向かって走行し、初期状態に戻る) 
        """
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
                
        # 2. 遷移判定: CP_Cに到達したか
        if self.is_goal_reached(current_x, current_y, 'CP_C'):
            self.transition_to('STATE_1_START') # 状態1に戻る
               
    def transition_to(self, new_state):
            """ 状態を遷移させ、ログを出力する """
            self.get_logger().info(f'Transition: {self.current_state} -> {new_state}')
            self.current_state = new_state

    # --- コールバック関数: オドメトリ受信 ---
    def odometry_callback(self, msg):
        """
        /odometry/filtered トピックからデータを受け取り、最新の値を保存する
        """
        # データを受信したら、最新のデータをクラス変数に上書き保存する
        self.latest_odom = msg
        # self.get_logger().info("odom_callback")
    # --- コールバック関数: 制御ループ (タイマー駆動) ---
    def control_loop_callback(self):
        """
        タイマーによって定周期で呼び出される制御ロジック
        """        
        # 現在の状態に対応するハンドラー関数を実行
        print(self.current_state)
        handler = self.state_handlers.get(self.current_state)
        
        if handler:
            handler()
        else:
            self.get_logger().error(f"Unknown state: {self.current_state}")
            # エラー状態処理 (例: 緊急停止など)
            # self.do_emergency_stop()
            
    def on_shutdown(self):
        """
        ノード終了時に実行されるクリーンアップ処理
        """

def main(args=None):
    rclpy.init(args=args)
    node = PwmMotorControlNode()
    
    # 重い制御（推論やPWM書き込み）とデータ受信を並列処理するためにMultiThreadedExecutorを使用
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(node)

    try:
        # executor.spin()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理と安全なシャットダウン
        node.on_shutdown()
        # executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()