from math import cos, radians, sin


def turn_and_move(curr_coords: tuple, angle: float, x_move: float,
                  y_move: float) -> tuple:

    rot_mat = _rotation_matrix(angle)
    start_loc_matrix = _coords_to_matrix(curr_coords[0], curr_coords[1])
    eu_group = _euclidean_group(rot_mat, start_loc_matrix)
    movement_ref_frame = _movement_to_ref_frame(x_move, y_move)
    end_ref_frame = _matmul(eu_group, movement_ref_frame)
    return _ref_frame_to_coords(end_ref_frame)


def _rotation_matrix(angle: float) -> list:

    angle = radians(angle)
    return [[cos(angle), -sin(angle)], [sin(angle), cos(angle)]]


def _coords_to_matrix(x_coord: float, y_coord: float) -> list:

    return [[x_coord], [y_coord]]


def _euclidean_group(rot_mat: list, coords_matrix: list) -> list:

    return [
        [rot_mat[0][0], rot_mat[0][1], coords_matrix[0][0]],
        [rot_mat[1][0], rot_mat[1][1], coords_matrix[1][0]],
        [0, 0, 1]
    ]


def _movement_to_ref_frame(x_axis: float, y_axis: float) -> list:

    return [[x_axis], [y_axis], [1]]


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

    curr_angle = 0
    starting_coords = (0, 0)
    step_1 = turn_and_move(starting_coords, curr_angle -120, 10, 0)
    curr_angle = curr_angle - 120
    print('Start at (0, 0), turn 120 deg. right, move 10m forward: {}'.format(
        step_1
    ))
    step_2 = turn_and_move(step_1, curr_angle + 45, 5, 0)
    curr_angle = curr_angle + 45
    print('Start at {}, turn 45 deg. left, move 5m forward: {}'.format(
        step_1, step_2
    ))


if __name__ == '__main__':

    main()
