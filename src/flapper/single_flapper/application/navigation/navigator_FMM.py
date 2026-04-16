#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion
from std_msgs.msg import Empty, Float64, Int64
from scipy.spatial.transform import Rotation as R

class Navigator3D:
    def __init__(self, theta_scale=0.1, r_min=0.7, r_max=1.5, chest2eye_h=0.2):
        # 軌道設計パラメータ
        self.theta_scale = theta_scale
        self.r_min = r_min
        self.r_max = r_max
        self.chest2eye_h = chest2eye_h
        self.eye_h = 1.5

        # FMM用パラメータ
        self.res = 21           # 計算解像度（リアルタイム性重視）
        self.window_size = 1.2  # ローカル窓の半径(m)
        self.v_prev = np.zeros(3)
        self.inertia = 0.7      # 指令値のなめらかさ

        # 待機位置
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.takeoff_z = 2.0

        # 状態管理
        self.phase = "takeoff_hover"
        self.running = False
        self.dock_start_time = None
        self.dock_hold_time = 2.0
        self.dock_hand_start_pos = None
        self.dock_hand_move_thresh = 0.3

        # --- Subscribers & Publishers ---
        self.drone_pose_sub = rospy.Subscriber("mocap_node/mocap/flapper/pose", PoseStamped, self.drone_cb)
        self.chest_pose_sub = rospy.Subscriber("mocap_node/mocap/chest/pose", PoseStamped, self.chest_cb)
        self.hand_pose_sub = rospy.Subscriber("mocap_node/mocap/hand/pose", PoseStamped, self.hand_cb)
        self.approach_start_sub = rospy.Subscriber("approach_start", Empty, self.start_cb)
        self.approach_stop_sub = rospy.Subscriber("approach_stop", Empty, self.stop_cb)

        self.approach_pub = rospy.Publisher("approach", Pose, queue_size=1)
        self.phase_pub = rospy.Publisher("navigator/phase", Int64, queue_size=1)
        self.ps_radius_pub = rospy.Publisher("navigator/ps_radius", Float64, queue_size=1)

        # 内部キャッシュ
        self.drone_p = None
        self.chest_p = None
        self.hand_p = None
        self.drone_raw_pose = None

    # --- Callbacks ---
    def drone_cb(self, msg):
        self.drone_raw_pose = msg.pose
        self.drone_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def chest_cb(self, msg):
        self.chest_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.eye_h = self.chest_p[2] + self.chest2eye_h

    def hand_cb(self, msg):
        self.hand_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def start_cb(self, msg):
        if not self.running: self.run()

    def stop_cb(self, msg):
        self.running = False

    # --- Core Logic: FMM Geodesic Velocity ---
    def compute_geodesic_velocity(self, q_final_goal):
        """ローカル窓内でのFMMに基づき、次の移動方向（測地線方向）を計算"""
        # 1. ローカル格子生成
        grid_range = np.linspace(-self.window_size, self.window_size, self.res)
        dX, dY, dZ = np.meshgrid(grid_range, grid_range, grid_range, indexing='ij')
        
        abs_X = dX + self.drone_p[0]
        abs_Y = dY + self.drone_p[1]
        abs_Z = dZ + self.drone_p[2]

        # 2. サブゴールの決定 (窓の境界にクリップ)
        vec_to_final = q_final_goal - self.drone_p
        dist_to_final = np.linalg.norm(vec_to_final)
        if dist_to_final < self.window_size * 0.8:
            q_sub = q_final_goal
        else:
            q_sub = self.drone_p + (vec_to_final / dist_to_final) * (self.window_size * 0.9)

        # 3. 屈折率（コスト場）: シグモイド型パーソナルスペース
        # 高さに応じた半径 R(z)
        R_z = self.r_min + (self.r_max - self.r_min) / (1.0 + np.exp(-20.0 * (abs_Z - self.eye_h)))
        
        # 胸からの水平距離
        dist_h = np.sqrt((abs_X - self.chest_p[0])**2 + (abs_Y - self.chest_p[1])**2)
        
        # コスト関数: パーソナルスペース内を「通りにくく」する
        cost = 1.0 + 150.0 * np.exp(-(dist_h**2) / (2 * (R_z/2)**2))

        # 4. 到達時間ポテンシャルの計算 (Eikonal近似)
        dist_to_sub = np.sqrt((abs_X - q_sub[0])**2 + (abs_Y - q_sub[1])**2 + (abs_Z - q_sub[2])**2)
        arrival_time = dist_to_sub * cost

        # 5. 勾配（進むべき方向）の抽出
        mid = self.res // 2
        gx, gy, gz = np.gradient(arrival_time, grid_range, grid_range, grid_range)
        v_grad = -np.array([gx[mid, mid, mid], gy[mid, mid, mid], gz[mid, mid, mid]])
        
        if np.linalg.norm(v_grad) > 1e-6:
            v_grad /= np.linalg.norm(v_grad)
            
        return v_grad

    def look_at_quaternion(self):
        if self.drone_raw_pose is None or self.chest_p is None: return Quaternion(0,0,0,1)
        vec = self.chest_p[:2] - self.drone_p[:2]
        target_theta = np.arctan2(vec[1], vec[0])
        current_theta = R.from_quat([self.drone_raw_pose.orientation.x, self.drone_raw_pose.orientation.y, 
                                     self.drone_raw_pose.orientation.z, self.drone_raw_pose.orientation.w]).as_euler("xyz")[2]
        diff = (target_theta - current_theta + np.pi) % (2 * np.pi) - np.pi
        q = R.from_euler("xyz", [0, 0, current_theta + diff * self.theta_scale]).as_quat()
        return Quaternion(*q)

    # --- Main Loop ---
    def run(self):
        rospy.loginfo("Navigator Geodesic Loop Started")
        self.running = True
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and self.running:
            if self.drone_p is None or self.chest_p is None or self.hand_p is None:
                rate.sleep(); continue

            # 状態・距離の更新
            dist_c_d = np.linalg.norm(self.drone_p[:2] - self.chest_p[:2])
            dist_h_d = np.linalg.norm(self.drone_p[:2] - self.hand_p[:2])
            dist_to_hand_z = abs(self.drone_p[2] - (self.hand_p[2] + 0.3))
            
            # フェーズ遷移 (ロジックは維持)
            if self.phase == "takeoff_hover":
                if np.linalg.norm(self.drone_p - np.array([self.takeoff_x, self.takeoff_y, self.takeoff_z])) < 0.2:
                    self.phase = "preparing"
            elif self.phase == "preparing":
                if dist_c_d >= (self.r_max - 0.1): self.phase = "leading"
            elif self.phase == "leading":
                if dist_c_d < self.r_max: self.phase = "circling"
            
            # ドッキング判定
            if dist_h_d < 0.15 and dist_to_hand_z < 0.1:
                if self.dock_start_time is None:
                    self.dock_start_time = rospy.get_time()
                elif rospy.get_time() - self.dock_start_time >= self.dock_hold_time:
                    self.phase = "docked"
            else:
                self.dock_start_time = None

            # --- 測地線移動計算 ---
            if self.phase == "takeoff_hover":
                goal_p = np.array([self.takeoff_x, self.takeoff_y, self.takeoff_z])
            elif self.phase == "docked":
                goal_p = self.hand_p + np.array([0, 0, 0.3])
            else:
                # 1. FMMで「方向」を出す
                q_final = self.hand_p + np.array([0, 0, 0.3])
                v_fmm = self.compute_geodesic_velocity(q_final)
                
                # 2. 慣性をのせる（方向の安定化）
                v_move_dir = self.inertia * self.v_prev + (1 - self.inertia) * v_fmm
                if np.linalg.norm(v_move_dir) > 1e-6: 
                    v_move_dir /= np.linalg.norm(v_move_dir)
                self.v_prev = v_move_dir

                # 3. 【重要】元のコードのスケール感に合わせる
                # 方向はFMMで決めるが、移動距離の計算を元の「v_move」のロジックに寄せる
                
                # 「本来行きたい場所（手）」までの距離を測る
                dist_to_final = np.linalg.norm(q_final - self.drone_p)
                
                # 元のコードの「0.3m制限」と同じスケールで「次の一歩」の距離を決める
                # 距離が0.3m以上あれば0.3m、それ以下ならその距離分だけ進む
                move_dist = min(dist_to_final, 0.3)
                
                # 現在位置から、FMMが決めた方向に move_dist 分だけ進んだ点を goal_p にする
                goal_p = self.drone_p + v_move_dir * move_dist

            # 指令値送信
            self.approach_pub.publish(Pose(Vector3(*goal_p), self.look_at_quaternion()))
            self.ps_radius_pub.publish(self.get_radius_single(self.drone_p[2])) # 既存のradius関数
            rate.sleep()

    def get_radius_single(self, z):
        return self.r_min + (self.r_max - self.r_min) / (1 + np.exp(-20 * (z - self.eye_h)))

if __name__ == "__main__":
    try:
        rospy.init_node("navigator_fmm")
        nav = Navigator3D()
        rospy.spin()
    except rospy.ROSInterruptException: pass
