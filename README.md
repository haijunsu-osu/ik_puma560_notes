# PUMA 560 Kinematics

This repository contains a Python implementation of the forward and inverse kinematics algorithms for the PUMA 560 robot arm. The code is based on the procedures and equations described in the document [Inverse Kinematics of PUMA 560 Robot.pdf](./Inverse%20Kinematics%20of%20PUMA%20560%20Robot.pdf).

## Overview

- **Forward Kinematics**: Computes the end-effector pose (position and orientation) given the joint angles of the robot.
- **Inverse Kinematics**: Computes all possible sets of joint angles that achieve a desired end-effector pose.

The algorithms follow the Denavit-Hartenberg convention and include base and tool transformations for accurate modeling.

## Files
- `PUMA560Kinematics_FollowNotes_Answer.py`: Main Python script implementing the kinematics algorithms.
- `Inverse Kinematics of PUMA 560 Robot.pdf`: Reference document for the algorithm (see link above).

## Forward Kinematics Steps
1. **Define Base and Tool Transforms**: The base (`G`) and tool (`H`) transforms are set as translations along the z-axis.
2. **Set DH Parameters**: The Denavit-Hartenberg parameters for the PUMA 560 are defined.
3. **Compute DH Transform for Each Joint**: For each joint, a DH transformation matrix is constructed.
4. **Multiply Transformations**: The overall arm transformation is computed by multiplying the DH matrices for all joints.
5. **Apply Base and Tool Transforms**: The final pose is obtained by applying the base and tool transforms to the arm transformation.

## Inverse Kinematics Steps
1. **Remove Base/Tool Offsets**: Compute the effective transformation by removing the base and tool transforms from the target pose.
2. **Extract Wrist Center**: The wrist center position is extracted from the transformation matrix.
3. **Solve for First Joint Angle (θ₁)**: Use a trigonometric equation to solve for possible values of θ₁.
4. **Solve for Third Joint Angle (θ₃)**: Use a distance equation and trigonometric solver to find possible θ₃ values.
5. **Solve for Second Joint Angle (θ₂)**: For each (θ₁, θ₃) pair, solve for θ₂ using the robot geometry.
6. **Compute Wrist Rotation**: Calculate the wrist rotation matrix and solve for the last three joint angles (θ₄, θ₅, θ₆) using spherical wrist formulas.
7. **Normalize and Verify Solutions**: All solutions are normalized to the range [-π, π] and verified against the original pose.

## Usage
Run the Python script to test the kinematics algorithms with example joint angles. The script prints the results of forward and inverse kinematics, and verifies the solutions.

---

For more details, refer to the [Inverse Kinematics of PUMA 560 Robot.pdf](./Inverse%20Kinematics%20of%20PUMA%20560%20Robot.pdf).
