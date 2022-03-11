#!/usr/bin/python3

"""
Kinematics calculations of a robot

    Denavit-Hartenberg parameters:
"""

import os
from pathlib import Path
import numpy as np
import sympy as sp
import yaml

CONFIG_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                           'config', 'LBR_iiwa_DH.yaml')

with open(CONFIG_PATH, 'r', encoding="utf-8") as file:
    config_dict = yaml.safe_load(file)

DH_PARAMS = config_dict["DH_params"]

joint_count = 0

while f'joint_{joint_count + 1}' in DH_PARAMS.keys():
    joint_count += 1
print(f'Number of joints: {joint_count}')


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

        Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q_symbols)

        # This was different orientation
        # Jw = sp.Matrix()
        # for i in range(len(rot_matrices)):
        #     Jw = Jw.col_insert(i, rot_matrices[i]*sp.Matrix([0, 0, 1]))

        J = sp.Matrix([Jv, Jw])
    else:
        J = Jv

    if joint_pos is None:
        return J

    if  orientation and c_p.subs(zip(q_symbols, joint_pos)).evalf() < 1e-6:
        j_s = joint_pos
        print("Pitch is almost +-90°")
        # in this case, the solution is not straighforward
        # atan2(0, 0) would be nan -> orientation is calculated with a different joint setup
        # only the first (from TCP) joint is modified, that changes pitch angle slightly
        for i in reversed(range(joint_count)):
            j_s[i] = joint_pos[i] + 0.001
            if c_p.subs(zip(q_symbols, j_s)).evalf() < 1e-6:
                j_s[i] = joint_pos[i]
            else:
                break
        print(j_s)
    return J.subs(zip(q_symbols, joint_pos)).evalf()


def pseudo_inverse(J):
    """
    Calculates the pseudo inverse for a given Jacobian matrix
    """
    if (J * J.transpose()).det() == 0:
        print('Singularity reached!!')
        return -1

    J_inv = J.transpose() * (J * J.transpose()).inv()
    return J_inv

def damped_least_squares(J, mu):
    """
    Calculates matrix for servoing with damped least squares method
    """

    J_inv = J.transpose() * (J * J.transpose() + mu * np.eye(6)).inv()
    return J_inv


def servo_calcs(dh_params, goal_pos, joint_states, orientation=True, max_iter=500, pos_tol=1e-3,
                rot_tol= 0.01):
    """
    Calculates the joint states for a given cartesian position with servoing in cartesian space
        - param goal_pos: target cartesian position with roll-pitch-yaw orientation
        - param joint_states: actual joint values
        - orientation: False, if only position is to be set, True otherwise
        - max_iter: maximum number of iterations
        - pos_tol: tolerance for cartesian position
        - rot_tol: tolerance for orientation
    """
    j_s = joint_states
    actual_pos = calc_forw_kin(calc_transform(dh_params), j_s)
    sp.pprint(actual_pos.transpose())
    if orientation:
        diff = sp.Matrix([goal_pos]).transpose() - sp.Matrix([actual_pos])
    else:
        diff = sp.Matrix([goal_pos[:3]]).transpose() - actual_pos[:3,:]
    i = 0
    print(f'Cartesian distance: {diff[:3,:].norm()}')
    if diff[:3,:].norm() > 0.25:
        print("Initial distance too big")
        return actual_pos, diff
    while (diff[:3,:].norm() > pos_tol and diff[3:,:].norm() > rot_tol and i < max_iter):
        print(diff[:3,:].norm(), diff[3:,:].norm(), i)
        i += 1
        J_inv = pseudo_inverse(calc_jacobian(dh_params, joint_states, orientation))
        if orientation:
            delta_theta = J_inv * diff
        else:
            print(diff[:3, :])
            delta_theta = J_inv * diff[:3, :]
        #delta_theta = delta_theta / max(delta_theta) * 0.02
        new_joints = sp.Matrix(j_s) + delta_theta * 0.3 # TODO: fine tune multiplier
        # TODO: maybe split the cartesian difference into small distances
        j_s = [item for sublist in new_joints.tolist() for item in sublist]
        actual_pos = calc_forw_kin(calc_transform(dh_params), j_s)
        sp.pprint(actual_pos.transpose())
        if orientation:
            diff = sp.Matrix([goal_pos]).transpose() - sp.Matrix([actual_pos])
        else:
            diff = sp.Matrix([goal_pos[:3]]).transpose() - actual_pos[:3,:]
        sp.pprint(diff.transpose())
    if i == max_iter:
        print("Could not reach target position in given iterations")
    return j_s, diff

def calc_transform(dh_params):
    """
    Calculates base to end effector transform
    """
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


def calc_forw_kin(T, joint_pos):
    """
    Calculates cartesian position and orientation from the transormation matrix
        based on given joint positions
    """
    j_s = joint_pos
    q_symbols = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = T[:3, 3]
    R = T[:3, :3]

    c_p = sp.sqrt(R[0,0]**2 + R[1,0]**2) # this equals cos(pitch)
    roll = sp.atan2(R[2,1], R[2,2])
    pitch = sp.atan2(-R[2,0], c_p)
    yaw = sp.atan2(R[1,0], R[0,0])

    if c_p.subs(zip(q_symbols, j_s)).evalf() < 1e-6:
        print("Pitch is almost +-90°")
        # in this case, the RPY angles are not straighforward
        # atan2(0, 0) would be nan
        # orientation is calculated with a slightly different joint setup
        # only the first (from TCP) joint is modified, which changes pitch angle
        for i in reversed(range(joint_count)):
            j_s[i] = joint_pos[i] + 0.001
            if c_p.subs(zip(q_symbols, j_s)).evalf() < 1e-6:
                j_s[i] = joint_pos[i]
            else:
                break

    return sp.Matrix([abs_pos, roll, pitch, yaw]).subs(zip(q_symbols, j_s)).evalf()




JOINT_STATES = [1, 1, 1, 1, 0.3, 0, 0]
GOAL_POS = [0.60361, 0.2166, 0.88038, 0.8564, -0.122, 1.4759]

result, difference = servo_calcs(DH_PARAMS, GOAL_POS, JOINT_STATES, orientation=True)

sp.pprint(result)
sp.pprint(difference)
