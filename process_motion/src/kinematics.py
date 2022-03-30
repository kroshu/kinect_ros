#!/usr/bin/python3

"""
Kinematics calculations of a robot

    Denavit-Hartenberg parameters:
"""

import math
import os
from pathlib import Path
import csv
import numpy as np
import sympy as sp
import yaml

LOWER_LIMITS = [-170, -120, -170, -120, -170, -120, -175]
UPPER_LIMITS = [170, 120, 170, 120, 170, 120, 175]

LOWER_LIMITS_R = [math.radians(deg) for deg in LOWER_LIMITS]
UPPER_LIMITS_R = [math.radians(deg) for deg in UPPER_LIMITS]

def denavit_to_matrix(s_a, s_alpha, s_d, s_theta, tool_length=0):
    """
    Calculates homogenous tranformation matrix from Denavit-Hartenberg parameters
        (Khalil-modification)
    """
    a = sp.parsing.parse_expr(s_a)
    alpha = sp.parsing.parse_expr(s_alpha)
    d = sp.parsing.parse_expr(s_d) + tool_length
    theta = sp.parsing.parse_expr(s_theta)

    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, a],
                 [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha),
                  -d * sp.sin(alpha)],
                 [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha),
                  d * sp.cos(alpha)],
                 [0, 0, 0, 1]])
    return T

def calc_jacobian(dh_params, joint_pos=None, orientation=True):
    """
    Calculates the Jacobian matrix for a robot chain from base to end effector based on
        Denavit-Hartenberg parameters
    """

    joint_count = 0
    while f'joint_{joint_count + 1}' in dh_params.keys():
        joint_count += 1

    # Calculate base to end effector transform
    trans_matrix = calc_transform(dh_params)

    # Calculate Jacobian from Transform matrix and rotations
    q_symbols = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = trans_matrix[:3, 3]
    Jv = abs_pos.jacobian(q_symbols)

    if orientation:
        rot_matrix = trans_matrix[:3, :3]
        c_p = sp.sqrt(rot_matrix[0,0]**2 + rot_matrix[1,0]**2) # this equals cos(pitch)

        yaw = sp.atan2(rot_matrix[1,0], rot_matrix[0,0])
        pitch = sp.atan2(-rot_matrix[2,0], c_p)
        roll = sp.atan2(rot_matrix[2,1], rot_matrix[2,2])
        if abs(abs(pitch.subs(zip(q_symbols, joint_pos))).evalf() - sp.pi/2) < 0.05:
            Jw = sp.Matrix([roll - yaw, pitch]).jacobian(q_symbols)
        else:
            Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q_symbols)

        J = sp.Matrix([Jv, Jw])
    else:
        J = Jv

    if joint_pos is None:
        return J

    if  orientation and c_p.subs(zip(q_symbols, joint_pos)).evalf() < 1e-6:
        print("Pitch is almost +-90°")
        # in this case, the solution is not straighforward
        # atan2(0, 0) would be nan -> orientation is calculated with a different joint setup
        # only the first (from TCP) joint is modified, that changes pitch angle slightly
        j_s = joint_pos.copy()
        for i in reversed(range(joint_count)):
            j_s[i] += 0.001
            if c_p.subs(zip(q_symbols, j_s)).evalf() < 1e-6:
                j_s[i] -= 0.001
            else:
                joint_pos = j_s.copy()
                break
    return J.subs(zip(q_symbols, joint_pos)).evalf()


def pseudo_inverse(J):
    """
    Calculates the pseudo inverse for a given Jacobian matrix
    """
    det = (J * J.transpose()).det()
    if det < 1e-8:
        print(f'Singularity reached, determinant: {det}')
        return -1

    J_inv = J.transpose() * (J * J.transpose()).inv()
    return J_inv

def pseudo_inverse_svd(J):
    """
    Calculates the pseudo inverse for a given Jacobian matrix based on SVD
    """
    U, S, V = np.linalg.svd(np.array(J).astype(np.float64))
    if np.linalg.det(np.diag(S)) < 1e-8:
        print("Singularity reached")
        return -1
    S_inv = np.linalg.inv(np.diag(S))
    columns_to_add = V.shape[0] - S.shape[0]
    while columns_to_add > 0:
        columns_to_add -= 1
        S_inv = np.vstack([S_inv, [0 for i in range(U.shape[0])]])

    J_inv = np.matmul(np.matmul(V.transpose(), S_inv), U.transpose())

    return sp.Matrix(J_inv)

def damped_least_squares(J, mu):
    """
    Calculates matrix for servoing with damped least squares method
    """
    J_inv = J.transpose() * (J * J.transpose() + mu * np.eye(sp.shape(J)[0])).inv()
    return J_inv

def servo_calcs(dh_params, goal_pos, joint_states, orientation=True, max_iter=500, pos_tol=1e-5,
                rot_tol=1e-3, set_last = 0, joint_limits = False):
    """
    Calculates the joint states for a given cartesian position with servoing in cartesian space
        - param goal_pos: target cartesian position with roll-pitch-yaw orientation
        - param joint_states: actual joint values
        - orientation: False, if only position is to be set, True otherwise
        - max_iter: maximum number of iterations
        - pos_tol: tolerance for cartesian position
        - rot_tol: tolerance for orientation
        - set_last: whether the value for the last joint should be set to be closer to goal
            0 and 1 mean no, other integers equal the number of values to try
        - joint_limits: whether to try to get away from the joint limits

    """
    goal_pos = sp.Matrix(goal_pos).transpose()
    start_joints = joint_states
    joint_count = len(joint_states)

    with open('log.csv', 'w', encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow([f'joint{i + 1}' for i in range(joint_count)])

    trans_matrix = calc_transform(dh_params)
    if orientation:
        goal_pos_tmp = adjust_goal_pos(trans_matrix, joint_states, goal_pos, set_last)
    else:
        goal_pos_tmp = goal_pos.copy()
    actual_pos = calc_forw_kin(trans_matrix, joint_states)

    sp.pprint(goal_pos_tmp.evalf(3))
    sp.pprint(actual_pos.transpose().evalf(3))

    if not orientation:
        rot_tol = float('inf')

    diff = sp.Matrix([goal_pos_tmp]).transpose() - sp.Matrix([actual_pos])
    i = 0
    print(f'Cartesian distance: {diff[:3,:].norm()}')
    min_dist = float('inf')
    min_js = []
    min_diff = []
    if diff[:3,:].norm() > 0.2:  # TODO
        print("Initial distance too big")
        return -1, -1
    while ((diff[:3,:].norm() > pos_tol or diff[3:,:].norm() > rot_tol) and i < max_iter):
        if diff.norm() < min_dist:
            min_js = joint_states
            min_diff = diff
        print(diff[:3,:].norm(), diff[3:,:].norm().evalf(), i)
        i += 1
        J = calc_jacobian(dh_params, joint_states, orientation)
        J_inv = pseudo_inverse_svd(J)
        if J_inv == -1:
            J_inv = damped_least_squares(J, 0.01)
        if orientation:
            delta_theta = J_inv * diff
        else:
            delta_theta = J_inv * diff[:3, :]

        max_change = 0.1
        # maximize the joint change per iteration to $max_change rad
        if max(abs(delta_theta)) > max_change:
            print(f'Reduced big jump in joint angle: {max(abs(delta_theta.evalf()))}')
            delta_theta /= max(abs(delta_theta)) / max_change

        # this matrix projects any vector on the nullspace of J
        # therefore a gradient descent can be added to the update formula
        #   without changing the goal position:
        # delta_theta -= null_space_proj * grad(function to minimize)
        goal_vector = sp.Matrix([0, 0, 0, 0, 0, 0, 0])
        null_space_proj = np.eye(joint_count) - J_inv * J
        if joint_limits:
            for j in range(joint_count):
                if abs(joint_states[j]) > sp.pi:
                    print('Joint value exceeds limit, wrapping around')
                    joint_states[j] -= np.sign(joint_states[j]) * 2 * sp.pi.evalf()
            # TODO: factor
            goal_vector = (sp.Matrix(joint_states) - (sp.Matrix(LOWER_LIMITS_R)
                           + sp.Matrix(UPPER_LIMITS_R)))
            limit_length = (sp.Matrix(UPPER_LIMITS_R) - sp.Matrix(LOWER_LIMITS_R))
            for j in range(len(LOWER_LIMITS)):
                goal_vector[j] /= limit_length[j]
        else:
            goal_vector = (sp.Matrix(joint_states) - sp.Matrix(start_joints)) # TODO: factor

        if max(abs(goal_vector)) > max(abs(delta_theta)):
            goal_vector /= max(abs(goal_vector)) / max(abs(delta_theta))
        delta_theta -= null_space_proj * goal_vector

        new_joints = sp.Matrix(joint_states) + delta_theta
        sp.pprint(new_joints.transpose().evalf(3))

        with open('log.csv', 'a', encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(new_joints.evalf(5))

        joint_states = [item for sublist in new_joints.tolist() for item in sublist]

        if orientation:
            goal_pos_tmp = adjust_goal_pos(trans_matrix, joint_states, goal_pos)
        else:
            goal_pos_tmp = goal_pos.copy()

        actual_pos = calc_forw_kin(trans_matrix, joint_states)
        sp.pprint(actual_pos.transpose().evalf(3))

        if orientation:
            diff = sp.Matrix([goal_pos_tmp]).transpose() - sp.Matrix([actual_pos])
        else:
            diff = sp.Matrix([goal_pos_tmp[:3]]).transpose() - actual_pos[:3,:]
        sp.pprint(diff.transpose().evalf(3))
    if i == max_iter:
        print("Could not reach target position in given iterations")
        if __name__ == "__main__":
            print("Returning closest solution")
            return sp.Matrix([min_js]).evalf(3), min_diff.evalf(3)
        return -1, -1
    return sp.Matrix([joint_states]).evalf(4), diff.evalf(4)

def calc_transform(dh_params):
    """
    Calculates base to end effector transform
    """

    joint_count = 0
    while f'joint_{joint_count + 1}' in dh_params.keys():
        joint_count += 1

    trans_matrix = np.identity(4)
    tool_length = 0
    if 'tool_length' in dh_params.keys():
        tool_length = dh_params['tool_length']
    for i in range(joint_count):
        joint_dh = dh_params[f'joint_{i + 1}'][0].split(' ')

        if i == joint_count -1:
            trans_matrix = trans_matrix * denavit_to_matrix(*joint_dh, tool_length)
        else:
            trans_matrix = trans_matrix * denavit_to_matrix(*joint_dh)
    return trans_matrix


def calc_forw_kin(T, joint_pos, all_dof=False):
    """
    Calculates cartesian position and orientation from the transormation matrix
        based on given joint positions
    """

    joint_count = len(joint_pos)

    q_symbols = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = T[:3, 3]
    R = T[:3, :3]

    c_p = sp.sqrt(R[0,0]**2 + R[1,0]**2) # this equals cos(pitch)
    roll = sp.atan2(R[2,1], R[2,2])
    pitch = sp.atan2(-R[2,0], c_p)
    yaw = sp.atan2(R[1,0], R[0,0])

    if c_p.subs(zip(q_symbols, joint_pos)).evalf() < 1e-6:
        print("Pitch is almost +-90°")
        # in this case, the RPY angles are not straighforward
        #   roll and yaw angles axis are the same in opposite directions
        #   atan2(0, 0) would be nan
        # orientation is calculated with a slightly different joint setup
        # only the first (from TCP) joint is modified, which changes pitch angle
        j_s = joint_pos.copy()
        for i in reversed(range(joint_count)):
            j_s[i] += 0.001
            if c_p.subs(zip(q_symbols, j_s)).evalf() < 1e-6:
                j_s[i] -= 0.001
            else:
                joint_pos = j_s.copy()
                break
    if not all_dof and abs(abs(pitch.subs(zip(q_symbols, joint_pos))).evalf() - sp.pi/2) < 0.05:
        return sp.Matrix([abs_pos, roll-yaw, pitch]).subs(zip(q_symbols, joint_pos)).evalf()

    return sp.Matrix([abs_pos, roll, pitch, yaw]).subs(zip(q_symbols, joint_pos)).evalf()

def adjust_goal_pos(trans_matrix, joint_states, goal_pos, tries=1):
    """
    Adjusts the goal position based on the current joint states:
        1. By transition -180° <-> 180° add +- 360° to orientation permanently
        2. If pitch is near 90° -> reduce DOF-s temporarily
            -> two goal_pos variables must be returned (permanent and temporary)
    Besides, can set the 7th joint of the robot so, that the resulting position is the 'closest'
        to the goal position
        - param tries: odd integer, meaning the number of evenly distributed values for joint7
            to try out, tries = 1 means joint states are not modified
        - returns the the two adjusted goal positions and the adjusted joint states
    """
    min_diff = float('inf')
    js_tmp = joint_states.copy()
    goal_pos_i = goal_pos.copy()
    tries = max(tries, 1)
    if tries %2 == 0:
        tries += 1
    for i in range(tries):
        # create tries+1 intervals and iterate through split points with last joint
        if tries > 1:
            js_tmp[6] = np.round((i - (tries - 1) / 2) * 6.28 / (tries + 1), 4)
        actual_pos = calc_forw_kin(trans_matrix, js_tmp)

        # Wrap around orientation values (-180° -> 180° transition)
        for j in range(3, min(len(actual_pos), len(goal_pos))):
            if abs(actual_pos[j] - goal_pos_i[j]) > 3.2:  # create hysteresis
                goal_pos_i[j] += np.sign(actual_pos[j]) * 2 * sp.pi.evalf()
                if tries == 1:
                    print('Wrapped around orientation, new goal position:')
                    sp.pprint(goal_pos_i.evalf(3))

        # Reduce DOF-s if pitch is near 90°
        if abs(abs(actual_pos[4])-sp.pi/2) < 0.05:
            goal_pos_tmp_i = goal_pos_i[:, :5]
            goal_pos_tmp_i[3] = goal_pos_i[3] - goal_pos_i[5]
            if tries == 1:
                print('Reduced DOF-s')
        else:
            goal_pos_tmp_i = goal_pos_i.copy()

        if tries > 1:
            diff = sp.Matrix([goal_pos_tmp_i]).transpose() - sp.Matrix([actual_pos])
            if diff.norm() < min_diff:
                min_diff = diff.norm()
                joint_states[6] = js_tmp[6]  # Modify with reference
                goal_pos[:, 3:] = goal_pos_i[:, 3:]  # Modify with reference
                goal_pos_tmp = goal_pos_tmp_i.copy()
        else:
            goal_pos[:, 3:] = goal_pos_i[:, 3:]  # Modify with reference
            goal_pos_tmp = goal_pos_tmp_i
    if tries > 1:
        print(f'closest js: {joint_states}')
    return goal_pos_tmp

if __name__ == "__main__":
    CONFIG_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                            'config', 'LBR_iiwa_DH.yaml')
    with open(CONFIG_PATH, 'r', encoding="utf-8") as config_file:
        config_dict = yaml.safe_load(config_file)
    DH_PARAMS = config_dict["DH_params"]

    JOINT_STATES = [1.1, 0.65, 1.23, -0.23, 0.14, -1.1, 0.8]
    GOAL_POS = [0.952, 0, 0.34, 0, 1.53, 0]

    result, difference = servo_calcs(DH_PARAMS, GOAL_POS, JOINT_STATES, max_iter=100)
    sp.pprint(result)
    sp.pprint(difference)
