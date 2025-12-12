import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class DroneAgent:
    def __init__(self, agent_id, side, start_pos, hand_pos, target_z=3.0):
        self.id = agent_id
        self.side = side
        self.pos = np.array(start_pos, dtype=float)
        self.target_hand = np.array(hand_pos, dtype=float)
        self.phase = "preparing"
        self.history = {"x": [], "y": [], "z": [], "r": []}
        self.target_z = target_z

    def update_phase(self, r_curr, dist_d):
        if self.phase == "preparing":
            if self.pos[2] - self.target_z < 0.3 and dist_d >= r_curr - 0.1:
                self.phase = "leading"
        if self.phase == "leading":
            if dist_d < r_curr + 0.1:
                self.phase = "circling"
        if self.phase == "circling" or self.phase == "leading":
            dist_to_hand_xy = np.linalg.norm(self.pos[:2] - self.target_hand[:2])
            dist_to_hand_z = abs(self.pos[2] - (self.target_hand[2] + 0.3))
            if dist_to_hand_xy < 0.15 and dist_to_hand_z < 0.1:
                self.phase = "docked"

    def compute_move(self, chest_pos, r_curr, other_drone_pos):
        v_c_d = self.pos[:2] - chest_pos[:2]
        dist_d = np.linalg.norm(v_c_d)
        self.update_phase(r_curr, dist_d)

        v_move = np.array([0.0, 0.0])
        goal_z = self.pos[2]

        if self.phase == "preparing":
            target_r_pos = (v_c_d / (dist_d + 1e-5)) * (r_curr + 0.5)
            v_move = target_r_pos - v_c_d
            goal_z = self.target_z
        elif self.phase == "leading":
            v_move = self.target_hand[:2] - self.pos[:2]
            goal_z = self.target_hand[2] + 0.3
        elif self.phase in ["circling", "docked"]:
            goal_z = self.target_hand[2] + 0.3
            if self.phase == "circling":
                theta = 0.08
                r_mat = np.array(
                    [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
                )
                r_target = np.linalg.norm(self.target_hand[:2] - chest_pos[:2])
                r_next = max(dist_d + (r_target - dist_d) * 0.1, r_curr + 0.05)
                v_next_pos = (r_mat @ (v_c_d / dist_d)) * r_next
                v_move = (chest_pos[:2] + v_next_pos) - self.pos[:2]
                if np.linalg.norm(self.target_hand[:2] - self.pos[:2]) < 0.5:
                    v_move += (self.target_hand[:2] - self.pos[:2]) * 0.3
            else:
                v_move = self.target_hand[:2] - self.pos[:2]

        # 衝突回避ロジック
        dist_between = np.linalg.norm(self.pos[:2] - other_drone_pos[:2])
        repulsion_weight = 0.80 if self.phase != "docked" else 0.02
        repulsion_influence = (
            1.0 if np.linalg.norm(self.target_hand[:2] - self.pos[:2]) > 0.2 else 0.0
        )

        if dist_between < 1.0:
            repulsion_vec = (self.pos[:2] - other_drone_pos[:2]) / (dist_between + 1e-5)
            strength = np.clip((1.0 - dist_between) / 1.0, 0, 1)
            v_move += repulsion_vec * strength * repulsion_weight * repulsion_influence

        move_norm = np.linalg.norm(v_move)
        if move_norm > 0.1:
            v_move = (v_move / move_norm) * 0.1

        self.pos[:2] += v_move
        self.pos[2] += (goal_z - self.pos[2]) * 0.05

        self.history["x"].append(self.pos[0])
        self.history["y"].append(self.pos[1])
        self.history["z"].append(self.pos[2])
        self.history["r"].append(np.linalg.norm(self.pos[:2] - chest_pos[:2]))


class SwarmSimulator:
    def __init__(self):
        self.chest_pos = np.array([0.0, 0.0, 1.2])
        self.hand_l = np.array([-0.8, 0.5, 1.0])
        self.hand_r = np.array([0.8, 0.5, 1.0])
        self.drone_l = DroneAgent(0, "left", [3.0, -1.0, 2.0], self.hand_l)
        self.drone_r = DroneAgent(1, "right", [4.0, 1.0, 2.0], self.hand_r)
        self.r_min, self.r_max, self.eye_h = 0.8, 1.5, 1.6

    def get_radius(self, z):
        return self.r_min + (self.r_max - self.r_min) / (
            1 + np.exp(-30 * (z - self.eye_h))
        )

    def step(self):
        r_l = self.get_radius(self.drone_l.pos[2])
        r_r = self.get_radius(self.drone_r.pos[2])
        self.drone_l.compute_move(self.chest_pos, r_l, self.drone_r.pos)
        self.drone_r.compute_move(self.chest_pos, r_r, self.drone_l.pos)
        return self.drone_l, self.drone_r


sim = SwarmSimulator()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))


def update(i):
    dl, dr = sim.step()

    # --- Top View ---
    ax1.clear()
    ax1.set_xlim(-4, 4)
    ax1.set_ylim(-4, 4)
    ax1.set_aspect("equal")
    r_now = sim.get_radius(max(dl.pos[2], dr.pos[2]))
    ax1.add_patch(
        plt.Circle((0, 0), r_now, fill=False, color="gray", linestyle="--", alpha=0.5)
    )
    ax1.plot(dl.history["x"], dl.history["y"], "b-", alpha=0.2)
    ax1.plot(dl.pos[0], dl.pos[1], "bo", label=f"L: {dl.phase}")
    ax1.plot(sim.hand_l[0], sim.hand_l[1], "bx", markersize=10)
    ax1.plot(dr.history["x"], dr.history["y"], "r-", alpha=0.2)
    ax1.plot(dr.pos[0], dr.pos[1], "ro", label=f"R: {dr.phase}")
    ax1.plot(sim.hand_r[0], sim.hand_r[1], "rx", markersize=10)
    ax1.legend(loc="lower right")
    ax1.set_title("Top View (XY Plane)")

    # --- ZR View (Cylindrical Profile) ---
    ax2.clear()
    ax2.set_xlim(0, 4)
    ax2.set_ylim(0, 4)
    # PS Wall Profile
    z_range = np.linspace(0, 4, 100)
    r_boundary = [sim.get_radius(z) for z in z_range]
    ax2.plot(r_boundary, z_range, "k--", alpha=0.3, label="PS Boundary")

    # Trajectories in ZR
    ax2.plot(dl.history["r"], dl.history["z"], "b-", alpha=0.3)
    ax2.plot(np.linalg.norm(dl.pos[:2]), dl.pos[2], "bo")
    ax2.plot(dr.history["r"], dr.history["z"], "r-", alpha=0.3)
    ax2.plot(np.linalg.norm(dr.pos[:2]), dr.pos[2], "ro")

    # Hand Targets in ZR
    ax2.plot(np.linalg.norm(sim.hand_l[:2]), sim.hand_l[2], "bx", markersize=8)
    ax2.plot(np.linalg.norm(sim.hand_r[:2]), sim.hand_r[2], "rx", markersize=8)

    ax2.set_xlabel("R (Distance from center)")
    ax2.set_ylabel("Z (Height)")
    ax2.set_title("ZR Path Profile (Personal Space Wall)")
    ax2.legend(loc="upper right")


ani = FuncAnimation(fig, update, frames=500, interval=25, repeat=False)
plt.show()
