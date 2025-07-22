
import numpy as np 

def rotation_matrix_to_quaternion(R):
    q = np.empty(4)
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2,1] - R[1,2]) * s
        q[1] = (R[0,2] - R[2,0]) * s
        q[2] = (R[1,0] - R[0,1]) * s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            q[3] = (R[2,1] - R[1,2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0,1] + R[1,0]) / s
            q[2] = (R[0,2] + R[2,0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            q[3] = (R[0,2] - R[2,0]) / s
            q[0] = (R[0,1] + R[1,0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            q[3] = (R[1,0] - R[0,1]) / s
            q[0] = (R[0,2] + R[2,0]) / s
            q[1] = (R[1,2] + R[2,1]) / s
            q[2] = 0.25 * s
    return q / np.linalg.norm(q)

def quaternion_to_rotation_matrix(q):
    x, y, z, w = q
    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y]
    ])
    return R

def slerp(q0,q1,t):
    dot_pdt = np.dot(q0, q1)
    if dot_pdt < 0:
        q1 = -q1 
        dot_pdt = -dot_pdt
    if dot_pdt > 0.99:
        res = (1-t)*q0 + t*q1
        return res / np.linalg.norm(res)
    omega = np.arccos(dot_pdt)
    sin_omega = np.sin(omega)
    w0 = np.sin((1-t)*omega)/sin_omega 
    w1 = np.sin(t*omega)/sin_omega
    return w0*q0 + w1*q1

def quat_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])

def quat_mul(q, r):
    x1, y1, z1, w1 = q
    x2, y2, z2, w2 = r
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])
    
    
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