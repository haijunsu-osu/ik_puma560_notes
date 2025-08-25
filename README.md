# PUMA 560 Kinematics

This repository contains a Python implementation of the forward and inverse kinematics algorithms for the PUMA 560 robot arm. The code is based on the procedures and equations described in the document [Inverse Kinematics of PUMA 560 Robot.pdf](./Inverse%20Kinematics%20of%20PUMA%20560%20Robot.pdf).

## Overview

- **Forward Kinematics**: Computes the end-effector pose (position and orientation) given the joint angles of the robot.
- **Inverse Kinematics**: Computes all possible sets of joint angles that achieve a desired end-effector pose.

The algorithms follow the Denavit-Hartenberg convention and include base and tool transformations for accurate modeling.

## Forward Kinematics Steps
1. **Define Base and Tool Transforms**: The base (`G`) and tool (`H`) transforms are set as translations along the z-axis.
2. **Set DH Parameters**: The Denavit-Hartenberg parameters for the PUMA 560 are defined as follows:

| Joint | a (m)   | alpha (deg) | d (m)    | theta (variable) |
|-------|---------|-------------|----------|------------------|
| 1     | 0.0     | -90         | 0.0      | θ₁               |
| 2     | 0.4318  | 0           | 0.0      | θ₂               |
| 3     | 0.0203  | -90         | 0.14909  | θ₃               |
| 4     | 0.0     | 90          | 0.43307  | θ₄               |
| 5     | 0.0     | -90         | 0.0      | θ₅               |
| 6     | 0.0     | 0           | 0.0      | θ₆               |
3. **Compute DH Transform for Each Joint**: For each joint, a DH transformation matrix is constructed using the following formula:

   T = [
	   [cos(θ),            -sin(θ)*cos(α),  sin(θ)*sin(α),   a*cos(θ)],
	   [sin(θ),             cos(θ)*cos(α), -cos(θ)*sin(α),   a*sin(θ)],
	   [0,                  sin(α),         cos(α),          d],
	   [0,                  0,              0,               1]
   ]

where:
- θ: joint angle (variable)
- α: link twist (in radians)
- a: link length
- d: link offset
4. **Multiply Transformations**: The overall arm transformation is computed by multiplying the DH matrices for all joints:

	T_arm = T1 × T2 × T3 × T4 × T5 × T6

where T1, T2, ..., T6 are the DH transformation matrices for each joint, constructed as described above.
5. **Apply Base and Tool Transforms**: The final pose is obtained by applying the base and tool transforms to the arm transformation:

	T = G × T_arm × H

where:
- G: base transform matrix
- T_arm: product of DH matrices for all joints
- H: tool transform matrix

## Inverse Kinematics Steps
1. **Remove Base/Tool Offsets**: Compute the effective transformation by removing the base and tool transforms from the target pose:
   
	T' = G⁻¹ · T · H⁻¹

2. **Extract Wrist Center**: The wrist center position is extracted from the transformation matrix:
   
	p_wc = T'[0:3, 3]

3. **Solve for First Joint Angle (θ₁)**: Use a trigonometric equation to solve for possible values of θ₁:
   
	-sin(θ₁)·pₓ + cos(θ₁)·p_y = d₃
	(rewritten as: py·cos(θ₁) + (-px)·sin(θ₁) = d₃)
   
	Use the helper: solve_trig_equation(py, -px, d3)

4. **Solve for Third Joint Angle (θ₃)**: Use a distance equation and trigonometric solver to find possible θ₃ values:
   
	2·a₂·(a₃·cos(θ₃) - d₄·sin(θ₃)) = (pₓ² + p_y² + p_z²) - (a₂² + a₃² + d₃² + d₄²)
	(rewritten as: A·cos(θ₃) + B·sin(θ₃) = C, with A = 2·a₂·a₃, B = -2·a₂·d₄, C = (pₓ² + p_y² + p_z²) - (a₂² + a₃² + d₃² + d₄²))
   
	Use the helper: solve_trig_equation(2*a2*a3, -2*a2*d4, C3)

5. **Solve for Second Joint Angle (θ₂)**: For each (θ₁, θ₃) pair, solve for θ₂ using the robot geometry:
   
	c₂·(c₁·pₓ + s₁·p_y) - s₂·p_z = a₂ + a₃·cos(θ₃) - d₄·sin(θ₃)
	-s₂·(c₁·pₓ + s₁·p_y) - c₂·p_z = a₃·sin(θ₃) + d₄·cos(θ₃)
   
	θ₂ = atan2(s₂, c₂)

6. **Compute Wrist Rotation**: Calculate the wrist rotation matrix and solve for the last three joint angles (θ₄, θ₅, θ₆) using spherical wrist formulas:
   
	R₀₃ = rotation from first three joints
	R' = rotation part of T'
	R₃₆ = R₀₃ᵀ · R'
   
	For the spherical wrist:
	- θ₅: acos(R₃₆[2,2]) and -acos(R₃₆[2,2])
	- θ₄: atan2(R₃₆[1,2]/(-sin(θ₅)), R₃₆[0,2]/(-sin(θ₅)))
	- θ₆: atan2(R₃₆[2,1]/(-sin(θ₅)), R₃₆[2,0]/sin(θ₅))

7. **Normalize and Verify Solutions**: All solutions are normalized to the range [-π, π] and verified against the original pose.
   
	Use: normalize_all(q)

## Usage
Run the Python script to test the kinematics algorithms with example joint angles. The script prints the results of forward and inverse kinematics, and verifies the solutions.

---

For more details, refer to the [Inverse Kinematics of PUMA 560 Robot.pdf](./Inverse%20Kinematics%20of%20PUMA%20560%20Robot.pdf).
