import numpy as np
from math import sin, cos, atan2, acos, sqrt, pi, radians, degrees

# ------------------------------------------------------------------------------  
# Define Base and Tool Transforms.
# ------------------------------------------------------------------------------  
# In our case, G translates by b along the z-axis and H translates by l along z.
b = 0.5   # base offset along z
l = 0.06  # tool offset along z

G = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, b],
    [0, 0, 0, 1]
])

H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, l],
    [0, 0, 0, 1]
])

# ------------------------------------------------------------------------------  
# Denavit-Hartenberg Parameters for the PUMA 560 (pure model)
# ------------------------------------------------------------------------------  
# (d6 is combined into H so we set d6 = 0)
a2 = 0.4318
a3 = 0.0203
d3 = 0.14909
d4 = 0.43307
d6 = 0.0

# ------------------------------------------------------------------------------  
# Helper: Solve A*cos(x) + B*sin(x) = C
# ------------------------------------------------------------------------------  
def solve_trig_equation(A, B, C):
    # Solve the equation: A*cos(x) + B*sin(x) = C.
    # Returns two solutions [x_plus, x_minus] if real solutions exist,
    # or an empty list if |C| > sqrt(A^2+B^2).
    r = sqrt(A*A + B*B)
    if r < 1e-15:
        return []
    if abs(C) > r:
        return []
    base_angle = atan2(B, A)
    delta = acos(C / r)
    return [base_angle + delta, base_angle - delta]

# ------------------------------------------------------------------------------  
# Denavit-Hartenberg Transform Function
# ------------------------------------------------------------------------------  
def dh_transform(a, alpha_deg, d, theta):
    # Construct the 4x4 Denavit-Hartenberg transform matrix.
    alpha = radians(alpha_deg)
    T = np.array([
        [cos(theta),            -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),   a*cos(theta)],
        [sin(theta),             cos(theta)*cos(alpha), -cos(theta)*sin(alpha),   a*sin(theta)],
        [0,                      sin(alpha),             cos(alpha),              d],
        [0,                      0,                      0,                       1]
    ])
    return T

# ------------------------------------------------------------------------------  
# Forward Kinematics (including G and H)
# ------------------------------------------------------------------------------  
def forward_kinematics(q, G, H, a2, a3, d3, d4, d6):
    # Compute the end-effector pose T for the PUMA 560, including base and tool transforms.
    # q = [θ1, θ2, θ3, θ4, θ5, θ6] in radians.
    th1, th2, th3, th4, th5, th6 = q

    T1 = dh_transform(0.0, -90.0, 0.0, th1)
    T2 = dh_transform(a2, 0.0, 0.0, th2)
    T3 = dh_transform(a3, -90.0, d3, th3)
    T4 = dh_transform(0.0, 90.0, d4, th4)
    T5 = dh_transform(0.0, -90.0, 0.0, th5)
    T6 = dh_transform(0.0, 0.0, d6, th6)

    T_arm = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    T = G @ T_arm @ H
    return T

# ------------------------------------------------------------------------------  
# Forward Kinematics for First Three Joints
# ------------------------------------------------------------------------------  
def forward_kinematics_first_three(th1, th2, th3, a2, a3, d3):
    # Compute the transform from the base to the wrist (frame 3) using the first three joints.
    T1 = dh_transform(0.0, -90.0, 0.0, th1)
    T2 = dh_transform(a2, 0.0, 0.0, th2)
    T3 = dh_transform(a3, -90.0, d3, th3)
    T = T1 @ T2 @ T3
    return T

# ------------------------------------------------------------------------------  
# Normalize Angle
# ------------------------------------------------------------------------------  
def normalize_angle(a):
    # Wrap angle a into [-pi, pi].
    return (a + pi) % (2*pi) - pi

# ------------------------------------------------------------------------------  
# Normalize All Joint Angles
# ------------------------------------------------------------------------------  
def normalize_all(q):
    # Normalize each angle in list q to [-pi, pi].
    return np.array([normalize_angle(a) for a in q])

# ------------------------------------------------------------------------------  
# Check if Two Sets of Joint Angles are Equivalent
# ------------------------------------------------------------------------------  
def is_equivalent(q1, q2, tol):
    # Normalize both sets
    q1_norm = normalize_all(q1)
    q2_norm = normalize_all(q2)
    return np.all(np.abs(q1_norm - q2_norm) < tol)

# ------------------------------------------------------------------------------  
# Inverse Kinematics for PUMA 560
# ------------------------------------------------------------------------------  
def puma_inverse_kinematics(T, G, H, a2, a3, d3, d4, d6):
    # Compute all IK solutions for the PUMA 560.
    #
    # Procedure:
    #   1. Compute T' = G⁻¹ · T · H⁻¹.
    #   2. Extract the wrist center (4th column of T').
    #   3. Solve for θ₁ from: -sin(θ₁)*pₓ + cos(θ₁)*p_y = d₃ (using solve_trig_equation).
    #   4. Solve for θ₃ from the distance equation:
    #          pₓ²+p_y²+p_z² = a₂²+a₃²+d₃²+d₄² + 2*a₂*(a₃ cos(θ₃)- d₄ sin(θ₃))
    #      rewritten as: A*cos(θ₃) + B*sin(θ₃) = C, with
    #          A = 2*a₂*a₃,  B = -2*a₂*d₄,  C = (pₓ²+p_y²+p_z²) - (a₂²+a₃²+d₃²+d₄²).
    #      (Use solve_trig_equation).
    #   5. For each (θ₁,θ₃) combination, solve for θ₂ from:
    #          c₂*(c₁*pₓ+s₁*p_y) - s₂*p_z = a₂+a₃ cos(θ₃)- d₄ sin(θ₃)
    #         -s₂*(c₁*pₓ+s₁*p_y) - c₂*p_z = a₃ sin(θ₃)+ d₄ cos(θ₃)
    #      to obtain θ₂ = atan2(s₂, c₂).
    #   6. Compute R₀₃ from the arm (first three joints) and then the wrist rotation:
    #          R₃₆ = R₀₃ᵀ · R'
    #      where R' is the rotation part of T'.
    #      Solve for (θ₄,θ₅,θ₆) using the spherical-wrist formulas.

    solutions = []

    # Step 1: Remove base/tool offsets.
    T_prime = np.linalg.inv(G) @ T @ np.linalg.inv(H)

    # Step 2: Extract wrist center.
    p_wc = T_prime[:3, 3]
    px, py, pz = p_wc

    # Step 3: Solve for θ₁.
    # Equation: -sin(θ₁)*px + cos(θ₁)*py = d₃
    # Rewritten as:  py*cos(θ₁) + (-px)*sin(θ₁) = d₃.
    theta1_candidates = solve_trig_equation(py, -px, d3)
    if not theta1_candidates:
        return solutions

    # Step 4: Solve for θ₃.
    # Equation: 2*a2*(a3*cos(θ₃) - d4*sin(θ₃)) = (px²+py²+pz²) - (a2²+a3²+d3²+d4²)
    C3 = (px**2 + py**2 + pz**2) - (a2**2 + a3**2 + d3**2 + d4**2)
    theta3_candidates = solve_trig_equation(2*a2*a3, -2*a2*d4, C3)
    if not theta3_candidates:
        return solutions

    # Step 5: For each (θ₁, θ₃), solve for θ₂.
    for th1 in theta1_candidates:
        c1 = cos(th1)
        s1 = sin(th1)
        X_val = c1*px + s1*py  # effective horizontal distance
        for th3 in theta3_candidates:
            c3 = cos(th3)
            s3 = sin(th3)
            A1 = a2 + a3*c3 - d4*s3
            B1 = a3*s3 + d4*c3
            denom = X_val**2 + pz**2
            if abs(denom) < 1e-12:
                continue
            c2 = (X_val*A1 - pz*B1) / denom
            s2 = (-pz*A1 - X_val*B1) / denom
            th2 = atan2(s2, c2)

            # Now we have (th1, th2, th3) for the arm.
            # Step 6: Solve for the wrist angles.
            T0_3 = forward_kinematics_first_three(th1, th2, th3, a2, a3, d3)
            R0_3 = T0_3[:3, :3]
            R_prime = T_prime[:3, :3]
            R3_6 = R0_3.T @ R_prime

            # For the spherical wrist:
            #   Let a33 = R3_6[2,2] and solve for θ₅.
            if abs(R3_6[2,2]) > 1.0:
                continue
            th5_candidates = [acos(R3_6[2,2]), -acos(R3_6[2,2])]
            for th5 in th5_candidates:
                if abs(sin(th5)) < 1e-6:
                    th4 = 0.0
                    th6 = atan2(-R3_6[0,1], R3_6[0,0])
                    sol = [th1, th2, th3, th4, th5, th6]
                    solutions.append(normalize_all(sol))
                else:
                    th4 = atan2(R3_6[1,2]/(-sin(th5)), R3_6[0,2]/(-sin(th5)))
                    th6 = atan2(R3_6[2,1]/(-sin(th5)), R3_6[2,0]/sin(th5))
                    sol = [th1, th2, th3, th4, th5, th6]
                    solutions.append(normalize_all(sol))
    return solutions

# ------------------------------------------------------------------------------  
# Demo / Test with Verification
# ------------------------------------------------------------------------------  
if __name__ == "__main__":
    # Define a test joint set (in radians)
    # Given test joint angles in degrees: 30, -40, -45, 45, 25, 33
    test_deg = [30, -40, -45, 45, 25, 33]
    # Convert to radians.
    test_q = np.radians(test_deg)

    # test_q = [0.4, -0.3, 0.8, 0.2, -0.6, 0.0]  # Note: d6 is built into H so we use 0 here.

    # Compute the forward kinematics for the test joint set.
    T_test = forward_kinematics(test_q, G, H, a2, a3, d3, d4, d6)
    print('Test joint angles (degrees):')
    print(np.round(np.degrees(test_q), 4))
    print('\nForward kinematics T_test:')
    print(np.round(T_test, 4))

    # Compute IK solutions from T_test.
    ik_sols = puma_inverse_kinematics(T_test, G, H, a2, a3, d3, d4, d6)
    print(f'\nFound {len(ik_sols)} IK solutions:')
    for i, sol in enumerate(ik_sols):
        print(f'Solution {i+1} (degrees): ', np.round(np.degrees(sol), 4))

    # Verification: Check if one solution is equivalent to test_q.
    found_equiv = False
    tol = 1e-3
    for sol in ik_sols:
        if is_equivalent(sol, test_q, tol):
            found_equiv = True
            break
    if found_equiv:
        print('\nSuccess: One of the IK solutions is equivalent to the test joint set.')
    else:
        print('\nFailed: None of the IK solutions match the test joint set.')

    # Additionally, verify by checking forward kinematics differences.
    print('\nForward kinematics verification (norm of difference):')
    for i, sol in enumerate(ik_sols):
        T_sol = forward_kinematics(sol, G, H, a2, a3, d3, d4, d6)
        diff = np.linalg.norm(T_test - T_sol, ord='fro')
        print(f'  Sol {i+1}: diff = {diff:.2e}')
