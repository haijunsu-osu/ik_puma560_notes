
---
<script>
window.MathJax = {
    tex: {
        inlineMath: [['$', '$'], ['\\(', '\\)']]
    }
};
</script>
<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

# PUMA 560 Kinematics

This repository contains a Python implementation of the forward and inverse kinematics algorithms for the PUMA 560 robot arm. The code is based on the procedures and equations described in the document [Inverse Kinematics of PUMA 560 Robot.pdf](./Inverse%20Kinematics%20of%20PUMA%20560%20Robot.pdf).

## Overview

- **Forward Kinematics**: Computes the end-effector pose (position and orientation) given the joint angles of the robot.
- **Inverse Kinematics**: Computes all possible sets of joint angles that achieve a desired end-effector pose.

The algorithms follow the Denavit-Hartenberg convention and include base and tool transformations for accurate modeling.
Step 1. **Define Base and Tool Transforms**: The base (`G`) and tool (`H`) transforms are set as translations along the z-axis.

Step 2. **Set DH Parameters**: The Denavit-Hartenberg parameters for the PUMA 560 are defined as follows:

| Joint | $a$ (m)   | $\alpha$ (deg) | $d$ (m)    | $\theta$ (variable) |
|-------|---------|-------------|----------|------------------|
| 1     | $0.0$     | $-90$         | $0.0$      | $\theta_1$               |
| 2     | $0.4318$  | $0$           | $0.0$      | $\theta_2$               |
| 3     | $0.0203$  | $-90$         | $0.14909$  | $\theta_3$               |
| 4     | $0.0$     | $90$          | $0.43307$  | $\theta_4$               |
| 5     | $0.0$     | $-90$         | $0.0$      | $\theta_5$               |

   $$
   T = \begin{bmatrix}
	   \cos\theta & -\sin\theta\cos\alpha & \sin\theta\sin\alpha & a\cos\theta \\
	   \sin\theta & \cos\theta\cos\alpha & -\cos\theta\sin\alpha & a\sin\theta \\
	   0 & \sin\alpha & \cos\alpha & d \\
	   0 & 0 & 0 & 1
   \end{bmatrix}
   $$

where:
- $\theta$: joint angle (variable)
- $\alpha$: link twist (in radians)
- $a$: link length
- $d$: link offset

Step 3. **Multiply Transformations**: The overall arm transformation is computed by multiplying the DH matrices for all joints:

	$$
	T_{arm} = T_1 \times T_2 \times T_3 \times T_4 \times T_5 \times T_6
	$$

where $T_1, T_2, ..., T_6$ are the DH transformation matrices for each joint, constructed as described above.

Step 4. **Multiply The Base Frame and The Tool Frame**: :

	$$
	T = G \times T_{arm} \times H
	$$

where:
- $G$: base transform matrix
- $T_{arm}$: product of DH matrices for all joints
- $H$: tool transform matrix

## Inverse Kinematics Steps
Step 1. **Remove Base/Tool Offsets**: Compute the effective transformation by removing the base and tool transforms from the target pose:

	$$
	T' = G^{-1} \cdot T \cdot H^{-1}
	$$

Step 2. **Extract Wrist Center**: The wrist center position is extracted from the transformation matrix:

	$$
	\mathbf{p}_{wc} = T'[0:3, 3]
	$$

Step 3. **Solve for First Joint Angle ($\theta_1$)**: Use a trigonometric equation to solve for possible values of $\theta_1$:

	$$
	-\sin\theta_1 \cdot p_x + \cos\theta_1 \cdot p_y = d_3
	$$
	(rewritten as: $p_y \cdot \cos\theta_1 + (-p_x) \cdot \sin\theta_1 = d_3$)

	Use the helper: $\text{solve\_trig\_equation}(p_y, -p_x, d_3)$

Step 4. **Solve for Third Joint Angle ($\theta_3$)**: Use a distance equation and trigonometric solver to find possible $\theta_3$ values:

	$$
	2a_2(a_3\cos\theta_3 - d_4\sin\theta_3) = (p_x^2 + p_y^2 + p_z^2) - (a_2^2 + a_3^2 + d_3^2 + d_4^2)
	$$
	(rewritten as: $A\cos\theta_3 + B\sin\theta_3 = C$, with $A = 2a_2a_3$, $B = -2a_2d_4$, $C = (p_x^2 + p_y^2 + p_z^2) - (a_2^2 + a_3^2 + d_3^2 + d_4^2)$)

	Use the helper: $\text{solve\_trig\_equation}(2a_2a_3, -2a_2d_4, C_3)$

Step 5. **Solve for Second Joint Angle ($\theta_2$)**: For each ($\theta_1$, $\theta_3$) pair, solve the following linear equations for $c_2$ and $s_2$:

$$
\begin{align*}
    c_2(c_1p_x + s_1p_y) - s_2p_z &= a_2 + a_3\cos\theta_3 - d_4\sin\theta_3 \\
    -s_2(c_1p_x + s_1p_y) - c_2p_z &= a_3\sin\theta_3 + d_4\cos\theta_3
\end{align*}
$$
And then solve for $\theta_2$ by 
$$\theta_2 = \text{atan2}(s_2, c_2)$$

Step 6. **Compute Wrist Rotation**: Calculate the wrist rotation matrix and solve for the last three joint angles ($\theta_4$, $\theta_5$, $\theta_6$) using spherical wrist formulas:

	$$
	R_{03} = \text{rotation from first three joints}
	$$
	$$
	R' = \text{rotation part of } T'
	$$
	$$
	R_{36} = R_{03}^T \cdot R'
	$$

	For the spherical wrist:
	- $\theta_5$: $\arccos(R_{36}[2,2])$ and $-\arccos(R_{36}[2,2])$
	- $\theta_4$: $\text{atan2}(R_{36}[1,2]/(-\sin\theta_5), R_{36}[0,2]/(-\sin\theta_5))$
	- $\theta_6$: $\text{atan2}(R_{36}[2,1]/(-\sin\theta_5), R_{36}[2,0]/\sin\theta_5)$

Step 7. **Normalize and Verify Solutions**: All solutions are normalized to the range $[-\pi, \pi]$ and verified against the original pose.

	Use: $\text{normalize\_all}(q)$

## Solve Trig Equation
The helper function `solve_trig_equation(A, B, C)` solves the equation:

$$
A \cos(x) + B \sin(x) = C
$$

**Details:**
- Returns two solutions $[x_+, x_-]$ if real solutions exist, or an empty list if $\|C\| > \sqrt{A^2+B^2}$.
- If $\sqrt{A^2+B^2}$ is very small (degenerate case), returns an empty list.
- If $\|C\| > \sqrt{A^2+B^2}$, there is no real solution.
- Otherwise, computes:
  - $r = \sqrt{A^2 + B^2}$
  - Solutions: $x_+ = \arctan2(B, A) + \arccos(C / r)$, $x_- = \arctan2(B, A) - \arccos(C / r)$

**Python Implementation:**
```python
# Solve the equation: A*cos(x) + B*sin(x) = C.
# Returns two solutions [x_plus, x_minus] if real solutions exist,
# or an empty list if |C| > sqrt(A^2+B^2).
def solve_trig_equation(A, B, C):
    r = sqrt(A*A + B*B)
    if r < 1e-15:
        return []  # degenerate case
    if abs(C) > r:
        return []  # no real solution
    base_angle = atan2(B, A)
    delta = acos(C / r)
    return [base_angle + delta, base_angle - delta]
```

## Usage
Run the Python script to test the kinematics algorithms with example joint angles. The script prints the results of forward and inverse kinematics, and verifies the solutions.

---