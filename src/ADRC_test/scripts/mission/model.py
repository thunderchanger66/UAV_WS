import numpy as np

class UAV:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

    def step(self, acc, h):
        self.vel += h * acc
        self.pos += h * self.vel
