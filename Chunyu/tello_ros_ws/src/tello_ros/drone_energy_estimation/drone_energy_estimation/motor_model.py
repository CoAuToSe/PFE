class MotorModel:
    def __init__(self, L, R, K_e, K_T, K_d, J, f, dt, e=0, i=0, i_dot=0, V=0, Tm=0):
        self.L = L
        self.R = R
        self.K_e = K_e
        self.K_T = K_T
        self.K_d = K_d
        self.J = J
        self.f = f
        self.dt = dt
        self.e = e
        self.i = i
        self.i_dot = i_dot
        self.V = V
        self.Tm = Tm

    def update(self, omega, omega_dot):
        self.Tm = self.J * omega_dot + self.f * omega + self.K_d * omega * omega
        i_current = self.Tm / self.K_T
        self.e = self.K_e * omega

        self.i_dot = (i_current - self.i) / self.dt
        self.V = self.L * self.i_dot + self.R * self.i + self.e
        self.i = i_current