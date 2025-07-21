import numpy as np
from utils import *
from plot_transformation import Animation

class SE3Pose: 
    def __init__(self, pos, rot_mat=None, velocity=None):
        self.pos = np.array(pos)
        self.rot_mat = rot_mat if rot_mat is not None else np.eye(3)
        self.velocity = np.array(velocity) if velocity is not None else np.zeros(3)
        self.speed = np.linalg.norm(self.velocity)

    def __repr__(self):
        return f"Pose={self.pos} Velocity={self.velocity}"

def velocity_profile(num_steps):
    t = np.linspace(-0.5, 0.5, num_steps)
    sigmoid = 1 / (1 + np.exp(-t))
    return (sigmoid - sigmoid.min()) / (sigmoid.max() - sigmoid.min())

def catmull_rom_point(P0, P1, P2, P3, τ):
    return 0.5 * (
        (2*P1) +
        (P2 - P0)*τ +
        (2*P0 - 5*P1 + 4*P2 - P3)*(τ**2) +
        (-P0 + 3*P1 - 3*P2 + P3)*(τ**3)
    )

def catmull_rom_derivative(P0, P1, P2, P3, τ):
    return 0.5 * (
        (P2 - P0) +
        2*(2*P0 - 5*P1 + 4*P2 - P3)*τ +
        3*(-P0 + 3*P1 - 3*P2 + P3)*(τ**2)
    )

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
    steps_per_segment = 100
    dt = 1.0 / steps_per_segment
    alphas = velocity_profile(steps_per_segment)

    alpha_dots = np.empty_like(alphas)
    alpha_dots[1:-1] = (alphas[2:] - alphas[:-2]) / (2 * dt)
    alpha_dots[0]     = (alphas[1] - alphas[0])   / dt
    alpha_dots[-1]    = (alphas[-1] - alphas[-2]) / dt

    trajectory = []
    for i in range(N - 1):
        P1 = waypoints[i]
        P2 = waypoints[i+1]
        P0 = waypoints[i-1] if i-1 >= 0 else P1
        P3 = waypoints[i+2] if i+2 < N else P2

        R1 = rotations[i]
        R2 = rotations[i+1]

        for j, α in enumerate(alphas):

            pos = catmull_rom_point(P0, P1, P2, P3, α)
            dp_dα = catmull_rom_derivative(P0, P1, P2, P3, α)
            velocity = dp_dα * alpha_dots[j]

            q1 = rotation_matrix_to_quaternion(R1)
            q2 = rotation_matrix_to_quaternion(R2)
            q_interp = slerp(q1, q2, α)
            rot = quaternion_to_rotation_matrix(q_interp)

            pose = SE3Pose(pos, rot, velocity)
            trajectory.append(pose)

    anim = Animation(trajectory)
    anim.animate()

if __name__ == "__main__":
    main()
