import numpy
from utils import *
from plot_transformation import Animation

class SE3Pose: 
    def __init__(self, pos, rot_mat = None):
        self.pos = np.array(pos)
        self.rot_mat = rot_mat if rot_mat is not None else np.eye(3)

    def __repr__(self):
        return f"Pose={self.pos} Orientation={self.rot_mat}"
    
    def interpolate(self, next, alpha):
        interploated_pos = (1-alpha) * self.pos + alpha * next.pos 
        q0, q1 = rotation_matrix_to_quaternion(self.rot_mat), rotation_matrix_to_quaternion(next.rot_mat) 
        interpolated_q = slerp(q0,q1,alpha) 
        interpolated_r = quaternion_to_rotation_matrix(interpolated_q)
        return SE3Pose(interploated_pos, interpolated_r)

def main():
    start = np.array([-1,-1,1])
    rot_i = np.array([[1,-0.4,0.1],
                    [0,0.6,1],
                    [0.2,1,0]])
    goal = np.array([3,3,3])
    rot_g = np.array([[1,0,-1],
                    [-0.5,1,1],
                    [0,-1,0.5]])
    poseA = SE3Pose(start, rot_i)
    poseB = SE3Pose(goal, rot_g)

    in_between_pos = [poseA.interpolate(poseB,alpha) for alpha in np.linspace(0,1,200)]

    anim = Animation(in_between_pos)
    anim.animate()

if __name__=="__main__":
    main()