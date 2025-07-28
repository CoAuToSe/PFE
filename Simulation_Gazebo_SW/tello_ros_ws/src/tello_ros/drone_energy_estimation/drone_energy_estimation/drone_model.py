import numpy as np


class DroneKinematic:
    def __init__(self, dt):
        self.dt = dt
        self.x_ddot = self.y_ddot = self.z_ddot = 0.0
        self.x_dot = self.y_dot = self.z_dot = 0.0
        self.phi_ddot = self.theta_ddot = self.psi_ddot = 0.0
        self.p = self.q = self.r = 0.0
        self.phi = self.theta = self.psi = 0.0

    def update(self, orientation, angular_vel, linear_acc):
        self.phi_ddot, self.theta_ddot, self.psi_ddot = (angular_vel - np.array([self.p, self.q, self.r])) / self.dt
        self.p, self.q, self.r = angular_vel[0], angular_vel[1], angular_vel[2]
        self.phi, self.theta, self.psi = quaternion_to_euler(orientation[3], orientation[0], orientation[1], orientation[2])
        self.x_dot, self.y_dot, self.z_dot = np.array([self.x_dot, self.y_dot, self.z_dot]) + np.array(linear_acc) * self.dt
        self.x_ddot, self.y_ddot, self.z_ddot = linear_acc[0], linear_acc[1], linear_acc[2]
        state = [self.x_ddot, self.y_ddot, self.z_ddot, self.x_dot, self.y_dot, self.z_dot, self.phi_ddot, self.theta_ddot, self.psi_ddot, self.p, self.q, self.r, self.phi, self.theta, self.psi]
        return state

class DroneDynamics:
    def __init__(self, m, Ix, Iy, Iz, kx, ky, kz, d, rho, L, gamma, g=9.8):
        self.m = m
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz
        self.kx = kx
        self.ky = ky
        self.kz = kz
        self.g = g
        self.d = d
        self.rho = rho
        self.L = L
        self.gamma = gamma

    def compute_rotating_speed(self, state):
        x_ddot, y_ddot, z_ddot, x_dot, y_dot, z_dot, phi_ddot, theta_ddot, psi_ddot, p, q, r, phi, theta, psi = state
        d1, d2, d3, d4, d5, d6 = self.d

        U1 = (x_ddot * self.m + self.kx * x_dot - d1 * self.m +
              y_ddot * self.m + self.ky * y_dot - d2 * self.m +
              z_ddot * self.m + self.kz * z_dot + self.g * self.m - d3 * self.m) / (
                     np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi) +
                     np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi) +
                     np.cos(phi) * np.cos(theta))

        U2 = phi_ddot * self.Ix - (self.Iy - self.Iz) * q * r - d4 * self.Ix
        U3 = theta_ddot * self.Iy - (self.Iz - self.Ix) * p * r - d5 * self.Iy
        U4 = psi_ddot * self.Iz - (self.Ix - self.Iy) * p * q - d6 * self.Iz

        F = np.array([U1, U2, U3, U4])

        A = np.array([[self.rho, self.rho, self.rho, self.rho],
                      [0, -self.L * self.rho, 0, self.L * self.rho],
                      [-self.L * self.rho, 0, self.L * self.rho, 0],
                      [-self.gamma, self.gamma, -self.gamma, self.gamma]])

        omegas = np.sqrt(np.linalg.solve(A, F))

        return omegas
    
def quaternion_to_euler(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return np.array([yaw, pitch, roll])