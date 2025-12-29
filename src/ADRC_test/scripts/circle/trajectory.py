import numpy as np

def circle_traj(t, R=2.0, w=1.0, z=1.0):
    x = R * np.cos(w * t)
    y = R * np.sin(w * t)
    return x, y, z
