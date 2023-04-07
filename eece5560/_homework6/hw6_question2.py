from math import cos, radians, sin


def _rotation_matrix(angle: float) -> list:

    angle = radians(angle)
    return [[cos(angle), -sin(angle)], [sin(angle), cos(angle)]]


def _movement_to_ref_frame(x_axis: float, y_axis: float) -> list:

    return [[x_axis], [y_axis], [1]]


def _euclidean_group(rot_mat: list, coords_matrix: list) -> list:

    return [
        [rot_mat[0][0], rot_mat[0][1], coords_matrix[0][0]],
        [rot_mat[1][0], rot_mat[1][1], coords_matrix[1][0]],
        [0, 0, 1]
    ]


def _matmul(mat1: list, mat2: list) -> list:

    result = [[0], [0], [0]]
    for i in range(len(mat1)):
        for j in range(len(mat2[0])):
            for k in range(len(mat2)):
                result[i][j] += mat1[i][k] * mat2[k][j]
    return result


def _ref_frame_to_coords(ref_frame: list) -> tuple:

    return (ref_frame[0][0], ref_frame[1][0])



def main():

    sensor_to_robot_angle = _rotation_matrix(180)
    sensor_to_robot_pos = _movement_to_ref_frame(-1, 0)
    sensor_to_robot_eu = _euclidean_group(
        sensor_to_robot_angle, sensor_to_robot_pos
    )

    robot_to_world_angle = _rotation_matrix(135)
    robot_to_world_pos = _movement_to_ref_frame(3, 2)
    robot_to_world_eu = _euclidean_group(
        robot_to_world_angle, robot_to_world_pos
    )

    pt1 = [[4], [3], [1]]
    pt2 = [[8], [2], [1]]

    one_robot = _matmul(sensor_to_robot_eu, pt1)
    two_robot = _matmul(sensor_to_robot_eu, pt2)

    one_world = _matmul(robot_to_world_eu, one_robot)
    two_world = _matmul(robot_to_world_eu, two_robot)

    print('Point 1 world coords: {}'.format(
        _ref_frame_to_coords(one_world)
    ))
    print('Point 2 world coords: {}'.format(
        _ref_frame_to_coords(two_world)
    ))

    arm_to_robot_angle = _rotation_matrix(25)
    arm_to_robot_pos = _movement_to_ref_frame(2, 0)
    arm_to_robot_eu = _euclidean_group(
        arm_to_robot_angle, arm_to_robot_pos
    )

    arm_pos = [[0], [0], [1]]
    arm_to_robot = _matmul(arm_to_robot_eu, arm_pos)
    arm_world = _matmul(robot_to_world_eu, arm_to_robot)

    print('Arm world coords: {}'.format(
        _ref_frame_to_coords(arm_world)
    ))


if __name__ == '__main__':

    main()
