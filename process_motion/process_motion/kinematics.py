#!/usr/bin/python3

"""Kinematics calculations of a robot."""

import csv
import math

import numpy as np
import sympy as sp

from enum import Enum

LOWER_LIMITS = [-170, -120, -170, -120, -170, -120, -175]
UPPER_LIMITS = [170, 120, 170, 120, 170, 120, 175]

LOWER_LIMITS_R = [0.9 * math.radians(deg) for deg in LOWER_LIMITS]
UPPER_LIMITS_R = [0.9 * math.radians(deg) for deg in UPPER_LIMITS]


class JointLimits(Enum):
    UNCHECKED = 0
    REDUCE_DOF = 1
    GOAL_VECTOR = 2
    COMBINED = 3


def denavit_to_matrix(s_a, s_alpha, s_d, s_theta, tool_length=0):
    """
    Calculate homogenous tranformation matrix from DH parameters.

    Using the Denavit-Hartenberg convention by Khalil
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


def calc_jacobian(dh_params, joint_pos=None, exceeded=None):
    """
    Calculate the Jacobian matrix for a robot chain from base to end effector.

        - param dh_params: Denavit-Hartenberg parameters of the robot
        - param exceeded: if a joint limit is exceeded by exactly one joint, the specified joint is
            set as constant in the jacobian calculations (joint_pos must not be None!)
    """
    joint_count = 0
    while f'joint_{joint_count + 1}' in dh_params.keys():
        joint_count += 1

    # Calculate base to end effector transform
    trans_matrix = calc_transform(dh_params)

    # Calculate Jacobian from Transform matrix and rotations
    q_symbols = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = trans_matrix[:3, 3]

    rot_matrix = trans_matrix[:3, :3]
    c_p = sp.sqrt(rot_matrix[0, 0]**2 + rot_matrix[1, 0]**2)  # this equals cos(pitch)
    yaw = sp.atan2(rot_matrix[1, 0], rot_matrix[0, 0])
    pitch = sp.atan2(-rot_matrix[2, 0], c_p)
    roll = sp.atan2(rot_matrix[2, 1], rot_matrix[2, 2])

    if joint_pos is None:
        Jv = abs_pos.jacobian(q_symbols)
        Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q_symbols)
        J = sp.Matrix([Jv, Jw])
        return J

    if exceeded is not None and len(exceeded) == 1:
        removed_joint = (q_symbols[exceeded[0] - 1], joint_pos[exceeded[0] - 1])
        abs_pos = abs_pos.subs(*removed_joint)
        yaw = yaw.subs(*removed_joint)
        pitch = pitch.subs(*removed_joint)
        roll = roll.subs(*removed_joint)
    elif exceeded is not None:
        q_remove = [q_symbols[i - 1] for i in exceeded]
        values = [joint_pos[i - 1] for i in exceeded]
        substitutes = dict(zip(q_remove, values))
        abs_pos = abs_pos.subs(substitutes)
        yaw = yaw.subs(substitutes)
        pitch = pitch.subs(substitutes)
        roll = roll.subs(substitutes)

    Jv = abs_pos.jacobian(q_symbols)
    if abs(abs(pitch.subs(zip(q_symbols, joint_pos))).evalf() - sp.pi/2) < 0.05:
        Jw = sp.Matrix([roll - yaw, pitch]).jacobian(q_symbols)
    else:
        Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q_symbols)

    Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q_symbols)
    J = sp.Matrix([Jv, Jw])

    if c_p.subs(zip(q_symbols, joint_pos)).evalf() < 1e-6:
        print('Pitch is almost +-90°')
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
    """Calculate the pseudo inverse for a given Jacobian matrix."""
    det = (J * J.transpose()).det()
    if det < 1e-9:
        print(f'Singularity reached, determinant: {det}')
        return -1

    J_inv = J.transpose() * (J * J.transpose()).inv()
    return J_inv


def pseudo_inverse_svd(J):
    """Calculate the pseudo inverse for a given Jacobian matrix based on SVD."""
    U, S, V = np.linalg.svd(np.array(J).astype(np.float64))
    if np.linalg.det(np.diag(S)) < 1e-9:
        print('Singularity reached')
        return -1
    S_inv = np.linalg.inv(np.diag(S))
    columns_to_add = V.shape[0] - S.shape[0]
    while columns_to_add > 0:
        columns_to_add -= 1
        S_inv = np.vstack([S_inv, [0 for i in range(U.shape[0])]])

    J_inv = np.matmul(np.matmul(V.transpose(), S_inv), U.transpose())

    return sp.Matrix(J_inv)


def damped_least_squares(J, mu):
    """Calculate matrix for servoing with damped least squares method."""
    J_inv = J.transpose() * (J * J.transpose() + mu * np.eye(sp.shape(J)[0])).inv()
    return J_inv


def check_joint_limits(joint_states):
    """Return, whether the given joint states are within the limits."""
    exceeded = []
    if len(joint_states) != len(LOWER_LIMITS):
        print('Limits are invalid for this robot')
        return False, exceeded
    for i in range(len(joint_states)):
        if (joint_states[i] < LOWER_LIMITS_R[i] or joint_states[i] > UPPER_LIMITS_R[i]):
            print(f'Limits exceeded by joint {i + 1}')
            exceeded.append(i + 1)
        valid = len(exceeded) == 0
    return valid, exceeded


def servo_calcs(dh_params, goal_pos, joint_states, max_iter=500, pos_tol=1e-5,
                rot_tol=1e-3, set_last=0, joint_limits=0):
    """
    Calculate the joint states for a given cartesian position with servoing in cartesian space.

        - param goal_pos: target cartesian position with roll-pitch-yaw orientation
        - param joint_states: actual joint values
        - max_iter: maximum number of iterations
        - pos_tol: tolerance for cartesian position
        - rot_tol: tolerance for orientation
        - set_last: whether the value for the last joint should be set to be closer to goal
            0 and 1 mean no, other integers equal the number of values to try
        - joint_limits (int): whether to try to get away from the joint limits
            - 1 means to enforce limits during the iterations (exclude joint from Jacobian)
            - 2 means to calculate with additional goal vector
            - 3 means to enforce limits during the iterations with additionan goal vector
    """
    goal_pos = sp.Matrix(goal_pos).transpose()
    start_joints = joint_states
    joint_count = len(joint_states)

    with open('log.csv', 'w', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow([f'joint{i + 1}' for i in range(joint_count)])

    trans_matrix = calc_transform(dh_params)
    goal_pos_tmp = adjust_goal_pos(trans_matrix, joint_states, goal_pos, set_last)

    actual_pos = calc_forw_kin(trans_matrix, joint_states)
    sp.pprint(goal_pos_tmp.evalf(3))
    sp.pprint(actual_pos.transpose().evalf(3))

    diff = sp.Matrix([goal_pos_tmp]).transpose() - sp.Matrix([actual_pos])
    i = 0
    print(f'Cartesian distance: {diff[:3,:].norm()}')
    # if diff[:3,:].norm() > 0.5:  # TODO
    #     print("[ERROR] Initial distance too big")
    #     return None, None
    blocked = False
    while ((diff[:3, :].norm() > pos_tol or diff[3:, :].norm() > rot_tol or blocked)
           and i < max_iter):
        print(diff[:3, :].norm(), diff[3:, :].norm().evalf(), i)
        i += 1
        J = calc_jacobian(dh_params, joint_states)
        J_inv = pseudo_inverse_svd(J)
        if J_inv == -1:
            J_inv = damped_least_squares(J, 0.001)
        delta_theta = J_inv * diff

        max_change = 0.1
        # maximize the joint change per iteration to $max_change rad
        if max(abs(delta_theta)) > max_change:
            delta_theta /= max(abs(delta_theta)) / max_change

        if delta_theta.norm() < 1e-5:
            delta_theta = sp.Matrix([0, 0, 0, 0, 0, 0, 0])

        # Exclude joints at limits from the Jacobian
        if joint_limits in [JointLimits.REDUCE_DOF, JointLimits.COMBINED]:
            exceeded = check_joint_limits((sp.Matrix(joint_states) + delta_theta).evalf(4))[1]
            if len(exceeded) > 0:
                J = calc_jacobian(dh_params, joint_states, exceeded)
                J_inv = pseudo_inverse_svd(J)
                if J_inv == -1:
                    J_inv = damped_least_squares(J, 0.001)
                delta_theta = J_inv * diff
                if max(abs(delta_theta)) > max_change:
                    delta_theta /= max(abs(delta_theta)) / max_change
            # Make sure blocked joints are not moved even in singularities
            for j in exceeded:
                delta_theta[j - 1] = 0

        # this matrix projects any vector on the nullspace of J
        # therefore a gradient descent can be added to the update formula
        #   without changing the goal position:
        # delta_theta -= null_space_proj * grad(function to minimize)
        goal_vector = sp.Matrix([0, 0, 0, 0, 0, 0, 0])
        null_space_proj = np.eye(joint_count) - J_inv * J
        if joint_limits == JointLimits.GOAL_VECTOR:
            # goal vector will be a linear, x = 90 to 100%, y = 0 to 1
            limit_length = (sp.Matrix(UPPER_LIMITS_R) - sp.Matrix(LOWER_LIMITS_R))
            goal_vector = (sp.Matrix(joint_states) - (sp.Matrix(LOWER_LIMITS_R)
                           + sp.Matrix(UPPER_LIMITS_R)))
            for j in range(joint_count):
                if abs(joint_states[j]) > sp.pi:
                    print('Joint value exceeds limit, wrapping around')
                    joint_states[j] -= np.sign(joint_states[j]) * 2 * sp.pi.evalf()
                    goal_vector[j] = joint_states[j] - (LOWER_LIMITS_R[j] + UPPER_LIMITS_R[j])
                goal_vector[j] /= limit_length[j]
                if abs(goal_vector[j]) < 0.45:
                    goal_vector[j] = 0
                else:
                    goal_vector[j] = 10 * (goal_vector[j] - 0.45 * np.sign(goal_vector[j]))
        elif joint_limits in [JointLimits.UNCHECKED, JointLimits.COMBINED]:
            goal_vector = (sp.Matrix(joint_states) - sp.Matrix(start_joints))
            if set_last:
                goal_vector[-1] = 0
        goal_adjust = null_space_proj * goal_vector
        delta_theta -= goal_adjust

        new_joints = sp.Matrix(joint_states) + delta_theta
        sp.pprint(new_joints.transpose().evalf(3))

        with open('log.csv', 'a', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(new_joints.evalf(5))

        joint_states = [item for sublist in new_joints.tolist() for item in sublist]

        # Limits are enforced, but new delta_theta in reduced DOF scenario can exceed a limit
        #   for another joint
        if joint_limits in [JointLimits.REDUCE_DOF, JointLimits.COMBINED]:
            for j in range(joint_count):
                if joint_states[j] > UPPER_LIMITS_R[j]:
                    joint_states[j] = UPPER_LIMITS_R[j] - 0.0005
                elif joint_states[j] < LOWER_LIMITS_R[j]:
                    joint_states[j] = LOWER_LIMITS_R[j] + 0.0005

        goal_pos_tmp = adjust_goal_pos(trans_matrix, joint_states, goal_pos)
        actual_pos = calc_forw_kin(trans_matrix, joint_states)
        diff = sp.Matrix([goal_pos_tmp]).transpose() - sp.Matrix([actual_pos])

        if joint_limits == JointLimits.GOAL_VECTOR and not check_joint_limits(joint_states)[0]:
            blocked = True
            print('Joint limits exceeded, termination blocked')
        else:
            blocked = False
    if i == max_iter:
        print('[ERROR] Could not reach target position in given iterations')
        return None, None
    return sp.Matrix([joint_states]).evalf(4), diff.evalf(4)


def servo_all_methods(dh_params, goal_pos, joint_states, set_last=0):
    """Run servo_calcs with 3 different methods (if necessary) to avoid joint limits."""
    j_s = joint_states.copy()
    servo_joints = servo_calcs(dh_params, goal_pos, j_s, set_last=set_last)[0]
    process_result(servo_joints)

    # Check joint limits and re-run test, if they were exceeded
    if servo_joints is not None and not check_joint_limits(servo_joints)[0]:
        print('[WARNING] Exceeded limits, runnning with new configuration')
        servo_joints = servo_calcs(dh_params, goal_pos, j_s, set_last=set_last,
                                   max_iter=250, joint_limits=JointLimits.REDUCE_DOF)[0]
        if servo_joints is None:
            print('[WARNING] Enforcing joint limits also failed, retrying with goal vector')
            servo_joints = servo_calcs(dh_params, goal_pos, j_s, set_last=set_last,
                                       max_iter=250, joint_limits=JointLimits.GOAL_VECTOR)[0]
            if servo_joints is None:
                print('[WARNING] Goal vector failed, last attempt with enforcing + goal vector')
                servo_joints = servo_calcs(dh_params, goal_pos, j_s, set_last=set_last,
                                           max_iter=250, joint_limits=JointLimits.COMBINED)[0]
        process_result(servo_joints)
        if servo_joints is not None and not check_joint_limits(servo_joints)[0]:
            print('[ERROR] New configuration was also not successful')
            servo_joints = None
    if servo_joints is not None:
        print(f'[INFO] Found valid solution: {servo_joints}')
    return servo_joints


def process_result(joint_result):
    """Check for joint values that can be wrapped around."""
    if joint_result is not None:
        for i in range(len(joint_result.tolist())):
            if abs(joint_result[i]) > sp.pi:
                print('Joint value exceeds limit, but can be made valid')
                cycles = int(abs(joint_result[i] / (2 * sp.pi)))
                joint_result[i] -= np.sign(joint_result[i]) * 2 * sp.pi.evalf() * (cycles + 1)
        processed_js = [round(item, 4) for sublist in joint_result.tolist() for item in sublist]
        return processed_js
    return None


def calc_transform(dh_params):
    """Calculate base to end effector transform."""
    joint_count = 0
    while f'joint_{joint_count + 1}' in dh_params.keys():
        joint_count += 1

    trans_matrix = np.identity(4)
    tool_length = 0
    if 'tool_length' in dh_params.keys():
        tool_length = dh_params['tool_length']
    for i in range(joint_count):
        joint_dh = dh_params[f'joint_{i + 1}'][0].split(' ')

        if i == joint_count - 1:
            trans_matrix = trans_matrix * denavit_to_matrix(*joint_dh, tool_length)
        else:
            trans_matrix = trans_matrix * denavit_to_matrix(*joint_dh)
    return trans_matrix


def calc_forw_kin(T, joint_pos):
    """
    Calculate cartesian position and orientation.

        - param T: transformation matrix (parametric)
    """
    joint_count = len(joint_pos)

    q_symbols = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = T[:3, 3]
    R = T[:3, :3]

    c_p = sp.sqrt(R[0, 0]**2 + R[1, 0]**2)  # this equals cos(pitch)
    roll = sp.atan2(R[2, 1], R[2, 2])
    pitch = sp.atan2(-R[2, 0], c_p)
    yaw = sp.atan2(R[1, 0], R[0, 0])

    if c_p.subs(zip(q_symbols, joint_pos)).evalf() < 1e-6:
        print('Pitch is almost +-90°')
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
    if abs(abs(pitch.subs(zip(q_symbols, joint_pos))).evalf() - sp.pi/2) < 0.05:
        return sp.Matrix([abs_pos, roll-yaw, pitch]).subs(zip(q_symbols, joint_pos)).evalf()

    return sp.Matrix([abs_pos, roll, pitch, yaw]).subs(zip(q_symbols, joint_pos)).evalf()


def adjust_goal_pos(trans_matrix, joint_states, goal_pos, tries=1):
    """
    Adjust the goal position based on the current joint states.

        1. By transition -180° <-> 180° add +- 360° to orientation permanently
        2. If pitch is near 90° -> reduce DOF-s temporarily
            -> two goal_pos variables must be returned (permanent and temporary)
        Besides, can set the 7th joint of the robot so, that the resulting position
            is the 'closest' to the goal position
        - param tries: odd integer, meaning the number of evenly distributed values for joint7
            to try out, tries = 1 means joint states are not modified
        - returns the the two adjusted goal positions and the adjusted joint states
    """
    min_diff = float('inf')
    js_tmp = joint_states.copy()
    goal_pos_i = goal_pos.copy()
    tries = max(tries, 1)
    if tries % 2 == 0:
        tries += 1
    for i in range(tries):
        # create tries+1 intervals and iterate through split points with last joint
        if tries > 1:
            js_tmp[6] = np.round((i - (tries - 1) / 2) * 6.28 / (tries + 1), 4)
        actual_pos = calc_forw_kin(trans_matrix, js_tmp)
        # Wrap around orientation values (-180° -> 180° transition)
        for j in range(3, min(len(actual_pos), len(goal_pos))):
            if abs(actual_pos[j] - goal_pos_i[j]) > 3.2:  # create hysteresis
                goal_pos_i[j] -= np.sign(goal_pos_i[j]) * 2 * sp.pi.evalf()
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
