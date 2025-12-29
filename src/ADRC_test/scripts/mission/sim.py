import numpy as np
import matplotlib.pyplot as plt
from ladrc import LADRC
from mission import Mission
from model import UAV

h = 0.01
sim_time = 20.0
steps = int(sim_time / h)

ctrl_x = LADRC(b0=1.0, wc=6.0, wo=24.0, h=h)
ctrl_y = LADRC(b0=1.0, wc=6.0, wo=24.0, h=h)
ctrl_z = LADRC(b0=1.0, wc=6.0, wo=24.0, h=h)

# 起始已经是zero vector
waypoints = np.array([
    [0.0, 0.0, 1.0],   # 起飞
    [1.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [0.0, 1.0, 1.0],
    [0.0, 0.0, 0.0]    # 降落
])

mission = Mission(waypoints)

uav = UAV()

traj = []

for k in range(steps):
    pos = uav.pos.copy()
    ref = mission.update(pos)

    ax = ctrl_x.step(ref[0], pos[0])
    ay = ctrl_y.step(ref[1], pos[1])
    az = ctrl_z.step(ref[2], pos[2])

    acc = np.array([ax, ay, az])

    uav.step(acc, h)

    traj.append(uav.pos.copy())

traj = np.array(traj)

# ======================

from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label='UAV Trajectory')
ax.scatter(
    waypoints[:, 0],
    waypoints[:, 1],
    waypoints[:, 2],
    c='r', marker='o', label='Waypoints'
)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('ADRC UAV Mission Simulation')
ax.legend()
ax.grid()

plt.show()

