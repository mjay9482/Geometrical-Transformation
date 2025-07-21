import numpy as np
from utils import *
from plot_transformation import Animation

class SE3Pose: 
    def __init__(self, pos, rot_mat=None, velocity=None):
        self.pos = np.array(pos)
        self.rot_mat = rot_mat if rot_mat is not None else np.eye(3)
        self.velocity = np.array(velocity) if velocity is not None else np.zeros(3)

    def __repr__(self):
        return f"Pose={self.pos} Orientation={self.rot_mat} Velocity={self.velocity}"
    
    def interpolate(self, next, alpha):
        interpolated_pos = (1 - alpha) * self.pos + alpha * next.pos 
        q0, q1 = rotation_matrix_to_quaternion(self.rot_mat), rotation_matrix_to_quaternion(next.rot_mat) 
        interpolated_q = slerp(q0, q1, alpha) 
        interpolated_r = quaternion_to_rotation_matrix(interpolated_q)
        return SE3Pose(interpolated_pos, interpolated_r)

def velocity_profile(n):
    t = np.linspace(-6, 6, n)  
    sigmoid = 1 / (1 + np.exp(-t))
    return (sigmoid - sigmoid.min()) / (sigmoid.max() - sigmoid.min())  

def main():
    waypoints = np.array([[-1, 1, 1],[1, 4, 1],[1, -2, 1],[4, -2, 1]])
    rotations = np.array([[
                          [1, -0.4, 0.1],
                          [0, 0.6, 1],
                          [0.2, 1, 0]],
                          
                          [[1, 0.4, 0.1],
                          [0, -0.6, 1],
                          [0.34, 1, 0]],
                          
                          [[-1, -0.4, 0.1],
                          [0, 0.6, -1],
                          [0.7, 1, 0]],
                          
                          [[1, 0, -1],
                          [-0.5, 1, 1],
                          [0, -1, 0.5]]])
    
    n = len(waypoints)
    trajectory = [] 
    for i in range(n-1):
        pose_i = SE3Pose(waypoints[i],rotations[i])
        pose_next_i = SE3Pose(waypoints[(i+1)%n],rotations[(i+1)%n])
        
        n = 100 
        alphas = velocity_profile(n)
        in_between_pose = [pose_i.interpolate(pose_next_i, alpha) for alpha in alphas]
        dt = 1.0/n 
        
        for i in range(1, n-1):
            vel = (in_between_pose[i+1].pos - in_between_pose[i-1].pos) / (2*dt)
            in_between_pose[i].velocity = vel

        in_between_pose[0].velocity = (in_between_pose[1].pos - in_between_pose[0].pos) / dt 
        in_between_pose[-1].velocity = (in_between_pose[-1].pos - in_between_pose[-2].pos) / dt 
        
        for pose in in_between_pose:
            pose.speed = np.linalg.norm(pose.velocity) 
        
        trajectory.extend(in_between_pose)
        
    anim = Animation(trajectory)
    anim.animate()

if __name__ == "__main__":
    main()
