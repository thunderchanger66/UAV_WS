import numpy as np

class Mission:
    def __init__(self, waypoints, tol=0.01):
        self.waypoints = waypoints
        self.tol = tol
        self.idx = 0
        self.finished = False

    def update(self, pos):
        if self.finished:
            return self.waypoints[-1]

        target = self.waypoints[self.idx]
        if np.linalg.norm(pos - target) < self.tol:
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self.finished = True
                self.idx = len(self.waypoints) - 1

        return self.waypoints[self.idx]
