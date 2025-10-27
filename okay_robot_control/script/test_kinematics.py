from dataclasses import dataclass
from typing import Union

import numpy as np
import sympy as sp

np.set_printoptions(precision=3, suppress=True)


@dataclass
class DH:
    a: float
    d: float
    alpha: float
    theta: Union[float, sp.Symbol]


def dh_to_transform(dh: DH, theta: float = 0.0) -> np.ndarray:
    new_theta = dh.theta + theta

    sin_theta = np.sin(new_theta)
    cos_theta = np.cos(new_theta)
    sin_alpha = np.sin(dh.alpha)
    cos_alpha = np.cos(dh.alpha)

    htm = np.array(
        [
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, cos_theta * dh.a],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, sin_theta * dh.a],
            [0.0, sin_alpha, cos_alpha, dh.d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    return htm


def forward_kinematics(dh_chain: list[DH], joint_angles: list[float]) -> np.ndarray:
    assert len(dh_chain) == len(joint_angles) + 2

    dh_joint_list = zip(dh_chain[1:-1], joint_angles)
    running_htm = dh_to_transform(dh_chain[0])
    for i, (dh, theta) in enumerate(dh_joint_list):
        dh_htm = dh_to_transform(dh, theta)
        running_htm = np.matmul(running_htm, dh_htm)

    forward_htm = np.matmul(running_htm, dh_to_transform(dh_chain[-1]))
    return forward_htm


def sp_dh_to_transform(dh: DH, i: int) -> sp.Matrix:
    theta = sp.symbols(f"theta{i}")

    sin_theta = sp.sin(theta)
    cos_theta = sp.cos(theta)
    sin_alpha = np.sin(dh.alpha)
    cos_alpha = np.cos(dh.alpha)

    htm = np.array(
        [
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, cos_theta * dh.a],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, sin_theta * dh.a],
            [0.0, sin_alpha, cos_alpha, dh.d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    return htm


# R01
# [
#   [cos(theta1), 0.0, sin(theta1)],
#   [sin(theta1), 0.0, -cos(theta1)],
#   [0, 1.0, 0.0]
# ]

# R02
# [
#   [cos(theta1)*cos(theta2), -sin(theta2)*cos(theta1), sin(theta1)],
#   [sin(theta1)*cos(theta2), -sin(theta1)*sin(theta2), -cos(theta1)],
#   [sin(theta2), cos(theta2), 0.0]
# ]

# R03
# [
#   [cos(theta1)*cos(theta2 + theta3), sin(theta1), sin(theta2 + theta3)*cos(theta1)],
#   [sin(theta1)*cos(theta2 + theta3), -cos(theta1), sin(theta1)*sin(theta2 + theta3)],
#   [sin(theta2 + theta3), 0.0, -cos(theta2 + theta3)]
# ]


def sp_forward_kinematics(dh_chain: list[DH], starting_index: int = 1) -> sp.Matrix:
    running_htm = sp.eye(4)
    for i, dh in enumerate(dh_chain):
        dh_htm = sp_dh_to_transform(dh, i + starting_index)
        running_htm = sp.simplify(np.matmul(running_htm, dh_htm))
        print(running_htm[:3, :3])

    return running_htm


def main() -> None:
    # float d0 = 0.041;
    d0 = 0.041
    dh0 = DH(0.0, d0, 0.0, 0.0)

    # float a1 = 0.040;
    # float d1 = 0.042;
    # float alpha1 = rad_90;
    # float theta1 = 0.0;
    d1 = 0.042
    a1 = 0.040
    alpha1 = np.deg2rad(90.0)
    theta1 = 0.0
    dh1 = DH(a1, d1, alpha1, theta1)

    # float a2_a = 0.150;
    # float a2_b = 0.028;
    # float phi2 = std::tan(a2_b / a2_a);
    # float a2 = std::sqrt(a2_a * a2_a + a2_b * a2_b);
    # float theta2 = rad_90 + phi2;
    a2_a = 0.150
    a2_b = 0.028
    phi2 = np.atan2(a2_b, a2_a)
    a2 = np.sqrt(a2_a * a2_a + a2_b * a2_b)
    theta2 = np.deg2rad(90.0) + phi2
    dh2 = DH(a2, 0.0, 0.0, theta2)

    # float a3 = 0.0600;
    # float alpha3 = rad_90;
    # float theta3 = -phi2;
    a3 = 0.060
    alpha3 = np.deg2rad(90.0)
    theta3 = -phi2
    dh3 = DH(a3, 0.0, alpha3, theta3)

    # float d4 = 0.155;
    # float alpha4 = -rad_90;
    # float theta4 = 0.0;
    d4 = 0.155
    alpha4 = -np.deg2rad(90.0)
    theta4 = 0.0
    dh4 = DH(0.0, d4, alpha4, theta4)

    # float alpha5 = rad_90;
    # float theta5 = 0.0;
    alpha5 = np.deg2rad(90.0)
    theta5 = 0.0
    dh5 = DH(0.0, 0.0, alpha5, theta5)

    # float d6 = 0.065;
    # float theta6 = 0.0;
    d6 = 0.065
    theta6 = 0.0
    dh6 = DH(0.0, d6, 0.0, theta6)

    # float d7 = 0.079;
    d7 = 0.079
    dh7 = DH(0.0, d7, 0.0, 0.0)

    dh_chain = [dh0, dh1, dh2, dh3, dh4, dh5, dh6, dh7]

    # UNCOMMENT TO GENERATE
    # sp_forward_kinematics(dh_chain[4:7], 4)

    # R36
    # [
    #   [-sin(theta4)*sin(theta6) + cos(theta4)*cos(theta5)*cos(theta6), -sin(theta4)*cos(theta6) - sin(theta6)*cos(theta4)*cos(theta5), sin(theta5)*cos(theta4)],
    #   [sin(theta4)*cos(theta5)*cos(theta6) + sin(theta6)*cos(theta4), -sin(theta4)*sin(theta6)*cos(theta5) + cos(theta4)*cos(theta6), sin(theta4)*sin(theta5)],
    #   [-sin(theta5)*cos(theta6), sin(theta5)*sin(theta6), cos(theta5)]
    # ]

    # joint_angles = [
    #     np.deg2rad(90.0),
    #     np.deg2rad(90.0),
    #     np.deg2rad(270.0),
    #     np.deg2rad(180.0),
    #     np.deg2rad(90.0),
    #     np.deg2rad(180.0),
    # ]
    joint_angles = [
        np.deg2rad(20.0),
        np.deg2rad(0.0),
        np.deg2rad(30.0),
        np.deg2rad(20.0),
        np.deg2rad(10.0),
        np.deg2rad(0.0),
    ]
    fk_htm = forward_kinematics(dh_chain, joint_angles)
    print(f"theta1: {np.rad2deg(joint_angles[0])}")
    print(f"theta2: {np.rad2deg(joint_angles[1])}")
    print(f"theta3: {np.rad2deg(joint_angles[2])}")
    print(f"theta4: {np.rad2deg(joint_angles[3])}")
    print(f"theta5: {np.rad2deg(joint_angles[4])}")
    print(f"theta6: {np.rad2deg(joint_angles[5])}")

    # IK -------------------------------------------------------------------------------------------
    eef_offset = (d6 + d7) * fk_htm[:3, 2]
    position = fk_htm[:3, 3] - eef_offset
    rotation = fk_htm[:3, :3]

    x = position[0]
    y = position[1]
    z = position[2]
    print(f"position: {fk_htm[:3, 3]}")

    mult = 1.0

    theta1 = np.atan2(y, x)

    s = z - (d0 + d1)
    r = np.sqrt(x**2 + y**2) - a1
    alpha = np.atan2(s, r)
    D_beta = (a2**2 + (r**2 + s**2) - (a3**2 + d4**2)) / (2.0 * a2 * np.sqrt(r**2 + s**2))
    beta = np.atan2(mult * np.sqrt(1 - D_beta**2), D_beta)
    theta2 = -np.deg2rad(90.0) - phi2 + (alpha + mult * beta)

    D_theta3 = (a2**2 + (a3**2 + d4**2) - (s**2 + r**2)) / (2.0 * a2 * np.sqrt(a3**2 + d4**2))
    theta3 = -np.deg2rad(90.0) - phi2 + np.atan2(mult * np.sqrt(1.0 - D_theta3**2), D_theta3)

    # R03
    # [
    #   [cos(theta1)*cos(theta2 + theta3), sin(theta1), sin(theta2 + theta3)*cos(theta1)],
    #   [sin(theta1)*cos(theta2 + theta3), -cos(theta1), sin(theta1)*sin(theta2 + theta3)],
    #   [sin(theta2 + theta3), 0.0, -cos(theta2 + theta3)]
    # ]

    # for a spherical wrist:
    # R36 = [
    #   [c4c5c6 - s4s6, -c4c5s6 - s4c6, c4s5]
    #   [s4c5c6 + c4s6, -s4c5s6 + c4c6, s4s5]
    #   [        -s5c6,           s5s6,   c5]
    # ]

    # theta4 = atan2(r13, r23)
    # theta5 = atan2(r33, sqrt(1 - r33**2))
    # theta6 = atan2(-r31, r32)

    # where:
    # R36 = (R03).T @ R
    # R = [
    #   [r11, r12, r13]
    #   [r21, r22, r23]
    #   [r31, r32, r33]
    # ]

    mult = 1.0
    r13 = (
        rotation[0, 2] * (np.cos(theta1) * np.cos(theta2 + theta3))
        + rotation[1, 2] * (np.sin(theta1) * np.cos(theta2 + theta3))
        + rotation[2, 2] * (np.sin(theta2 + theta3))
    )
    r23 = rotation[0, 2] * np.sin(theta1) - rotation[1, 2] * np.cos(theta1)
    theta4 = np.deg2rad(90.0) - np.atan2(r13, r23)

    r33 = (
        rotation[0, 2] * (np.sin(theta2 + theta3) * np.cos(theta1))
        + rotation[1, 2] * (np.sin(theta1) * np.sin(theta2 + theta3))
        - rotation[2, 2] * (np.cos(theta2 + theta3))
    )
    theta5 = -np.atan2(r33, mult * np.sqrt(1.0 - r33**2))

    r31 = (
        rotation[0, 0] * (np.cos(theta1) * np.cos(theta2 + theta3))
        + rotation[1, 0] * (np.sin(theta1) * np.cos(theta2 + theta3))
        + rotation[2, 0] * (np.sin(theta2 + theta3))
    )
    r32 = (
        rotation[0, 1] * (np.cos(theta1) * np.cos(theta2 + theta3))
        + rotation[1, 1] * (np.sin(theta1) * np.cos(theta2 + theta3))
        + rotation[2, 1] * (np.sin(theta2 + theta3))
    )
    theta6 = np.deg2rad(90.0) - np.atan2(-r31, r32)

    print(f"theta1: {np.rad2deg(theta1)}")
    print(f"theta2: {np.rad2deg(theta2)}")
    print(f"theta3: {np.rad2deg(theta3)}")
    print(f"theta4: {np.rad2deg(theta4)}")
    print(f"theta5: {np.rad2deg(theta5)}")
    print(f"theta6: {np.rad2deg(theta6)}")

    new_joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
    new_fk_htm = forward_kinematics(dh_chain, new_joint_angles)
    print(f"position: {new_fk_htm[:3, 3]}")


if __name__ == "__main__":
    main()
