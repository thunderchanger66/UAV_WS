class UAV:
    def __init__(self, h):
        self.h = h
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0

    def step(self, ux, uy, uz):
        self.x += self.h * self.vx
        self.vx += self.h * (ux + 1.0)#加入常值干扰

        self.y += self.h * self.vy
        self.vy += self.h * (uy + 1.0)

        self.z += self.h * self.vz
        self.vz += self.h * (uz + 1.0)

        return self.x, self.y, self.z
