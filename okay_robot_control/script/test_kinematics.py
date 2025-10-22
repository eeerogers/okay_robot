from dataclasses import dataclass

import numpy as np

np.set_printoptions(precision=3, suppress=True)


@dataclass
class DH:
    a: float
    d: float
    alpha: float
    theta: float


def dh_to_transform(dh: DH, theta: float = 0.0) -> np.ndarray:
    # const float new_theta = dh.theta + theta;

    # const float sin_theta = std::sin(new_theta);
    # const float cos_theta = std::cos(new_theta);
    # const float sin_alpha = std::sin(dh.alpha);
    # const float cos_alpha = std::cos(dh.alpha);

    # Eigen::Matrix4f m;
    # m << cos_theta, -sin_theta, 0.0, dh.a, sin_theta * cos_alpha, cos_theta * cos_alpha, -sin_alpha,
    #     -sin_alpha * dh.d, sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha,
    #     cos_alpha * dh.d, 0.0, 0.0, 0.0, 1.0;

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
    offset1 = np.deg2rad(90.0)
    dh1 = DH(a1, d1, alpha1, theta1 - offset1)

    # float a2_a = 0.150;
    # float a2_b = 0.028;
    # float phi2 = std::tan(a2_b / a2_a);
    # float a2 = std::sqrt(a2_a * a2_a + a2_b * a2_b);
    # float theta2 = rad_90 + phi2;
    a2_a = 0.150
    a2_b = 0.028
    phi2 = np.tan(a2_b / a2_a)
    a2 = np.sqrt(a2_a * a2_a + a2_b * a2_b)
    theta2 = np.deg2rad(90.0) + phi2
    offset2 = np.deg2rad(90.0)
    dh2 = DH(a2, 0.0, 0.0, theta2 - offset2)

    # float a3 = 0.0600;
    # float alpha3 = rad_90;
    # float theta3 = -phi2;
    a3 = 0.060
    alpha3 = np.deg2rad(90.0)
    theta3 = -phi2
    offset3 = np.deg2rad(270.0)
    dh3 = DH(a3, 0.0, alpha3, theta3 - offset3)

    # float d4 = 0.155;
    # float alpha4 = -rad_90;
    # float theta4 = 0.0;
    d4 = 0.155
    alpha4 = -np.deg2rad(90.0)
    theta4 = 0.0
    offset4 = np.deg2rad(180.0)
    dh4 = DH(0.0, d4, alpha4, theta4 - offset4)

    # float alpha5 = rad_90;
    # float theta5 = 0.0;
    alpha5 = np.deg2rad(90.0)
    theta5 = 0.0
    offset5 = np.deg2rad(90.0)
    dh5 = DH(0.0, 0.0, alpha5, theta5 - offset5)

    # float d6 = 0.065;
    # float theta6 = 0.0;
    d6 = 0.065
    theta6 = 0.0
    offset6 = np.deg2rad(180.0)
    dh6 = DH(0.0, d6, 0.0, theta6 - offset6)

    # float d7 = 0.079;
    d7 = 0.079
    dh7 = DH(0.0, d7, 0.0, 0.0)

    dh_chain = [
        (dh0, np.deg2rad(0.0)),
        (dh1, np.deg2rad(90.0)),
        (dh2, np.deg2rad(90.0)),
        (dh3, np.deg2rad(270.0)),
        (dh4, np.deg2rad(180.0)),
        (dh5, np.deg2rad(90.0)),
        (dh6, np.deg2rad(180.0)),
        (dh7, np.deg2rad(0.0)),
    ]
    running_htm = np.eye(4)
    for i, (dh, theta) in enumerate(dh_chain):
        dh_htm = dh_to_transform(dh, theta)
        running_htm = np.matmul(running_htm, dh_htm)

        print(f"joint {i + 1}")
        print(running_htm)


if __name__ == "__main__":
    main()
