# SE(3) Trajectory Generation

This implementation generates smooth trajectories in SE(3) (3D position + orientation) space between waypoints, with proper velocity computations.

## 🚀 Core Components

### 1. Waypoint Interpolation (Position)
- **Method**: Catmull-Rom Splines  
- **Purpose**: Smoothly interpolates between 3D position waypoints  
- **Properties**:
  - Guarantees C¹ continuity (continuous position and velocity)
  - Passes exactly through control points
  - Local control (changing one point affects limited neighborhood)

### 2. Linear Velocity Computation
- **Calculation Process**:
  1. Compute position Jacobian: $$J_p = \frac{\partial \text{position}}{\partial \alpha}$$ 
  2. Multiply by temporal derivative: $$v = J_p \cdot \frac{\partial \alpha}{\partial t}$$
- **Result**: Linear velocity vector at each trajectory point

### 3. Orientation Interpolation
- **Method**: Spherical Linear Interpolation (SLERP)  
- **Input**: Rotation matrices converted to quaternions  
- **Formula**:
  $$q(t) = \frac{\sin((1-t)\theta)}{\sin\theta} q_0 + \frac{\sin(t\theta)}{\sin\theta} q_1$$
  Where $\theta = \cos^{-1}(q_0 \cdot q_1)$

### 4. Angular Velocity Computation
- **Calculation Process**:
    1.  Estimate quaternion derivative
    $$\frac{dq}{dt} \approx \frac{q_{t+\Delta t} - q_t}{\Delta t}$$

    2.  Compute angular velocity via quaternion algebra:
    $$\omega = 2 \cdot \left( \frac{dq}{dt} \otimes q^* \right)_{\text{vector part}}$$

### 5. Pose Construction
## SE(3) Pose Representation

| Component          | Symbol | Type        |  Units       | Description                     |
|--------------------|--------|-------------|-------------|---------------------------------|
| **Position**       | _p_    | `[x, y, z]` |  meters (m)  | 3D Cartesian coordinates        |
| **Orientation**    | _q_    | `[qw, qx, qy, qz]` | Unitless | Unit quaternion (rotation)      |
| **Linear Velocity** | _v_    | `[vx, vy, vz]` | m/s         | Velocity vector in world frame |
| **Angular Velocity** | _ω_    | `[ωx, ωy, ωz]` | rad/s      | Angular velocity vector        |

### Matrix Representation

![SE3 Trajectory Flowchart][def]

![Transformation Animation](assets/transformation.gif)


[def]: assets/se3_flowchart.svg