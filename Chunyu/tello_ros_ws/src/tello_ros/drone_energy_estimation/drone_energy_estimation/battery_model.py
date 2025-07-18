class BatteryModel:
    def __init__(self, V0, alpha, Q, R, dt, it=0, V=0):
        self.V0 = V0
        self.alpha = alpha
        self.Q = Q
        self.R = R
        self.dt = dt
        self.it = it
        self.V = V

    def compute_capacity(self, i):
        self.it += i * self.dt
        self.V = self.V0 + self.alpha * (1 - self.it / self.Q) - self.R * i