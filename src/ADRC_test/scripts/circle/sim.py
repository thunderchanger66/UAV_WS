import matplotlib.pyplot as plt
import ladrc
from uav_model import UAV
from trajectory import circle_traj

h = 0.001
uav = UAV(h)

ctrl_x = ladrc.LADRC(b0=1.0, wc=6.0, wo=24.0, h=h)
ctrl_y = ladrc.LADRC(b0=1.0, wc=6.0, wo=24.0, h=h)
ctrl_z = ladrc.LADRC(b0=1.0, wc=6.0, wo=24.0, h=h)

xs, ys, zs = [], [], []

for k in range(10000):
    t = k * h
    xr, yr, zr = circle_traj(t)

    ux = ctrl_x.step(xr, uav.x)
    uy = ctrl_y.step(yr, uav.y)
    uz = ctrl_z.step(zr, uav.z)

    x, y, z = uav.step(ux, uy, uz)

    xs.append(x)
    ys.append(y)
    zs.append(z)

plt.plot(xs, ys)
plt.axis("equal")
plt.title("Circle Tracking")
plt.show()
