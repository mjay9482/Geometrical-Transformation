import numpy as np
from utils import *
from plot_transformation import Animation

class SE3Pose:
    def __init__(self, pos, rot_mat, linear_vel=None, angular_vel=None):
        self.pos = np.array(pos)
        self.rot_mat = np.array(rot_mat)
        self.linear_velocity = np.zeros(3) if linear_vel is None else linear_vel
        self.angular_velocity = np.zeros(3) if angular_vel is None else angular_vel
        self.speed = np.linalg.norm(self.linear_velocity)

    def __repr__(self):
        lv = np.round(self.linear_velocity, 3)
        av = np.round(self.angular_velocity, 3)
        return f"Pose={self.pos} LinVel={lv} AngVel={av}"


def main():
    waypoints = np.array([
        [-1,  1, 1],
        [ 1,  4, 1],
        [ 1, -2, 1],
        [ 4, -2, 1]
    ])

    rotations = np.array([
        [[ 1,  -0.4, 0.1],
         [ 0,   0.6, 1. ],
         [ 0.2,  1. , 0. ]],

        [[ 1,   0.4, 0.1],
         [ 0,  -0.6, 1. ],
         [ 0.34, 1. , 0. ]],

        [[-1,  -0.4, 0.1],
         [ 0,   0.6,-1. ],
         [ 0.7,  1. , 0. ]],

        [[ 1,   0. , -1. ],
         [-0.5, 1. , 1. ],
         [ 0,  -1. , 0.5]]
    ])

    N = waypoints.shape[0]
    steps = 100
    dt = 1.0 / steps
    alphas = velocity_profile(steps)

    alpha_dot = np.empty_like(alphas)
    alpha_dot[1:-1] = (alphas[2:] - alphas[:-2]) / (2*dt)
    alpha_dot[0]     = (alphas[1] - alphas[0])   / dt
    alpha_dot[-1]    = (alphas[-1] - alphas[-2]) / dt

    trajectory = []
    for i in range(N - 1):
        P1 = waypoints[i]
        P2 = waypoints[i + 1]
        P0 = waypoints[i - 1] if i > 0     else P1
        P3 = waypoints[i + 2] if i + 2 < N  else P2

        R1 = rotations[i]
        R2 = rotations[i + 1]

        for j, alpha in enumerate(alphas):
            pos = catmull_rom_point(P0, P1, P2, P3, alpha)
            Jp = catmull_rom_derivative(P0, P1, P2, P3, alpha)
            lin_vel = Jp * alpha_dot[j]

            q1 = rotation_matrix_to_quaternion(R1)
            q2 = rotation_matrix_to_quaternion(R2)
            q = slerp(q1, q2, alpha)
            delta = 1e-3
            qp = slerp(q1, q2, min(alpha + delta, 1.0))
            qm = slerp(q1, q2, max(alpha - delta, 0.0))
            dq_dalpha = (qp - qm) / (2 * delta)
            omega_quat = 2 * quat_mul(dq_dalpha, quat_conjugate(q))
            ang_vel = omega_quat[:3] * alpha_dot[j]

            rot = quaternion_to_rotation_matrix(q)
            trajectory.append(SE3Pose(pos, rot, linear_vel=lin_vel, angular_vel=ang_vel))

    anim = Animation(trajectory)
    anim.animate()

if __name__ == "__main__":
    main()