import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Final3DSim:
    def __init__(self):
        # 初期位置：PSの内側上空 (x, y, z)
        self.drone_pos = np.array([-2.5, -2.0, 2.5])
        self.chest_pos = np.array([0.0, 0.0, 1.2])
        self.hand_pos = np.array([0.7, 0.5, 1.0])
        
        # 状態管理
        self.phase = 'preparing' 
        self.r_min, self.r_max, self.eye_h = 0.8, 2.5, 1.6
        
        # 軌跡保存
        self.history_x, self.history_y, self.history_z = [], [], []

    def get_radius(self, z):
        # 高度に応じたパーソナルスペースの半径計算
        return self.r_min + (self.r_max - self.r_min) / (1 + np.exp(-30 * (z - self.eye_h)))

    def step(self):
        r_curr = self.get_radius(self.drone_pos[2])
        v_c_d = self.drone_pos[:2] - self.chest_pos[:2] # 中心からドローンへのXYベクトル
        dist_d = np.linalg.norm(v_c_d)
        
        # --- フェーズ制御 ---
        if self.phase == 'preparing':
            safe_z = 3.0
            safe_r = r_curr + 0.1
            if abs(self.drone_pos[2] - safe_z) < 0.5 and dist_d >= safe_r - 0.2:
                self.phase = 'leading'
        
        if self.phase == 'leading':
            if dist_d < r_curr + 0.1:
                self.phase = 'circling'
        
        if self.phase == 'circling':
            dist_to_hand_xy = np.linalg.norm(self.drone_pos[:2] - self.hand_pos[:2])
            dist_to_hand_z = abs(self.drone_pos[2] - (self.hand_pos[2] + 0.3))
            if dist_to_hand_xy < 0.15 and dist_to_hand_z < 0.1:
                self.phase = 'docked'
                print("Docked!")

        # --- 移動計算 ---
        v_move = np.array([0.0, 0.0])
        goal_z = self.drone_pos[2]

        if self.phase == 'preparing':
            target_r_pos = (v_c_d / (dist_d + 1e-5)) * (r_curr + 0.1)
            v_move = target_r_pos - v_c_d
            goal_z = 3.0

        elif self.phase == 'leading':
            v_move = self.hand_pos[:2] - self.drone_pos[:2]
            goal_z = self.hand_pos[2] + 0.3

        elif self.phase == 'circling':
            goal_z = self.hand_pos[2] + 0.3
            dist_to_hand = np.linalg.norm(self.hand_pos[:2] - self.drone_pos[:2])
            
            # --- 吸着係数の導入：手に近づくほど旋回を抑制 ---
            attraction_factor = np.clip(dist_to_hand / 0.5, 0.2, 1.0)
            
            theta = 0.08 # 回転速度
            r_mat = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            
            r_target = np.linalg.norm(self.hand_pos[:2] - self.chest_pos[:2])
            r_limit = r_curr + 0.05 
            r_next = dist_d + (r_target - dist_d) * 0.1
            r_next = max(r_next, r_limit)
            
            v_next_dir = r_mat @ (v_c_d / dist_d)
            v_next_pos = v_next_dir * r_next
            
            # 旋回ベクトルに吸着係数を掛ける
            v_move = ((self.chest_pos[:2] + v_next_pos) - self.drone_pos[:2]) * attraction_factor
            
            # 手が近い場合は手への直接ベクトルを少し混ぜる
            if dist_to_hand < 0.5:
                v_pull = (self.hand_pos[:2] - self.drone_pos[:2]) * (1.0 - attraction_factor)
                v_move += v_pull

        else: # docked
            v_move = np.array([0.0, 0.0])
            goal_z = self.hand_pos[2] + 0.3

        # XY移動更新
        move_norm = np.linalg.norm(v_move)
        max_step = 0.1
        if move_norm > max_step:
            v_move = (v_move / move_norm) * max_step
        self.drone_pos[:2] += v_move
        
        # Z軸降下更新
        z_step = (goal_z - self.drone_pos[2]) * 0.05
        if abs(z_step) > 0.8: z_step = np.sign(z_step) * 0.8
        self.drone_pos[2] += z_step
        
        self.history_x.append(self.drone_pos[0])
        self.history_y.append(self.drone_pos[1])
        self.history_z.append(self.drone_pos[2])
        
        return self.drone_pos.copy(), r_curr

# --- 描画設定 ---
sim = Final3DSim()
fig = plt.figure(figsize=(14, 6))
ax1 = fig.add_subplot(1, 2, 1) # XY
ax2 = fig.add_subplot(1, 2, 2) # RZ

def update(i):
    p, r = sim.step()
    ax1.clear()
    ax1.set_xlim(-5, 5); ax1.set_ylim(-5, 5); ax1.set_aspect('equal')
    ax1.plot(sim.history_x, sim.history_y, 'r-', alpha=0.3, label='Flight Path')
    ax1.plot(p[0], p[1], 'ro', markersize=8)
    ax1.plot(sim.hand_pos[0], sim.hand_pos[1], 'y*', markersize=12, label='Hand')
    circle = plt.Circle((0,0), r, fill=False, color='b', linestyle='--', alpha=0.6)
    ax1.add_patch(circle)
    ax1.set_title(f"Top-view (Phase: {sim.phase})")
    ax1.legend(loc='upper right')

    ax2.clear()
    ax2.set_xlim(0, 5); ax2.set_ylim(0, 5)
    z_g = np.linspace(0, 5, 50); rb = [sim.get_radius(z) for z in z_g]
    ax2.plot(rb, z_g, 'b--', alpha=0.3, label='PS Wall')
    dist_h = [np.linalg.norm([x, y]) for x, y in zip(sim.history_x, sim.history_y)]
    ax2.plot(dist_h, sim.history_z, 'r-', alpha=0.3)
    ax2.plot(np.linalg.norm(p[:2]), p[2], 'ro')
    ax2.set_xlabel("R (Distance from center)")
    ax2.set_ylabel("Z (Height)")
    ax2.set_title("RZ Path Profile")

ani = FuncAnimation(fig, update, frames=500, interval=20, repeat=False)
plt.show()
