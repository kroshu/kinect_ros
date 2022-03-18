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

import csv

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

def calc_jacobian(dh_params, joint_pos=None, orientation=True, pitch_lock=False):
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
        if pitch_lock:
            Jw = sp.Matrix([roll - yaw, pitch]).jacobian(q_symbols)
        else:
            Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q_symbols)

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
                rot_tol=1e-3):
    """
    Calculates the joint states for a given cartesian position with servoing in cartesian space
        - param goal_pos: target cartesian position with roll-pitch-yaw orientation
        - param joint_states: actual joint values
        - orientation: False, if only position is to be set, True otherwise
        - max_iter: maximum number of iterations
        - pos_tol: tolerance for cartesian position
        - rot_tol: tolerance for orientation
    """

    with open('log.csv', 'a') as file:
        writer = csv.writer(file)
        writer.writerow([f'joint{i + 1}' for i in range(joint_count)])

    # if pitch is around 90°, roll and yaw axis are the same, but point to opposite directions
    # therefore we can set pitch to pitch-yaw and neglect yaw in jacobian
    # TODO: maybe we should do that only when actual cartesian position is near that point
    if abs(abs(goal_pos[4])-sp.pi/2) < 0.1:
        goal_pos[3] = goal_pos[3] - goal_pos[5]
        goal_pos = goal_pos[:5]
        pitch_lock = True


    j_s = joint_states
    actual_pos = calc_forw_kin(calc_transform(dh_params), j_s, pitch_lock)
    sp.pprint(actual_pos.transpose().evalf(3))

    if not orientation:
        rot_tol = float('inf')
    diff = sp.Matrix([goal_pos]).transpose() - sp.Matrix([actual_pos])
    i = 0
    print(f'Cartesian distance: {diff[:3,:].norm()}')
    min_dist = 100
    min_js = []
    min_diff = []
    if diff[:3,:].norm() > 0.55:  # TODO
        print("Initial distance too big")
        return actual_pos, diff
    while ((diff[:3,:].norm() > pos_tol or diff[3:,:].norm() > rot_tol) and i < max_iter):
        if diff.norm() < min_dist:
            min_js = j_s
            min_diff = diff
        print(diff[:3,:].norm(), diff[3:,:].norm().evalf(), i)
        # if max(abs(diff)) > 0.1:
        #     diff /= max(abs(diff)) * 10  # split Cartesian position to a max diff of 0.2
        i += 1
        J = calc_jacobian(dh_params, j_s, orientation, pitch_lock)
        J_inv = pseudo_inverse(J)
        if J_inv == -1:
            J_inv = damped_least_squares(J, 0.01)
        if orientation:
            delta_theta = J_inv * diff
        else:
            delta_theta = J_inv * diff[:3, :]

        max_change = 0.1
        # if max(abs(J)) > 10:
        #     max_change = 1 / max(abs(J))
        #     print(f'Reduced max angle change to {max_change}')
        # maximize the joint change per iteration to 0.05 rad
        if max(abs(delta_theta)) > max_change:
            print(f'Reduced big jump in joint angle: {max(abs(delta_theta.evalf()))}')
            sp.pprint(delta_theta.evalf(4).transpose())
            delta_theta /= max(abs(delta_theta)) / max_change
        # for j in get_leaps_from_jacobian(J):
        #     delta_theta[j] = np.sign(delta_theta[j]) * min(abs(delta_theta[j]), 0.0001)
        #     # delta_theta[i] = 0
        sp.pprint(delta_theta.evalf(4).transpose())
        new_joints = sp.Matrix(j_s) + delta_theta
        sp.pprint(new_joints.transpose().evalf(3))

        with open('log.csv', 'a') as file:
            writer = csv.writer(file)
            writer.writerow(new_joints.evalf(5))

        j_s = [item for sublist in new_joints.tolist() for item in sublist]
        actual_pos = calc_forw_kin(calc_transform(dh_params), j_s, pitch_lock)
        sp.pprint(actual_pos.transpose().evalf(3))
        if orientation:
            diff = sp.Matrix([goal_pos]).transpose() - sp.Matrix([actual_pos])
        else:
            diff = sp.Matrix([goal_pos[:3]]).transpose() - actual_pos[:3,:]
        sp.pprint(diff.transpose().evalf(3))
    if i == max_iter:
        print("Could not reach target position in given iterations")
        print("Returning closest solution")
        return sp.Matrix([min_js]).evalf(3), min_diff.evalf(3)
    return sp.Matrix([j_s]).evalf(3), diff.evalf(3)

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


def calc_forw_kin(T, joint_pos, pitch_lock=False):
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
    if pitch_lock:
        return sp.Matrix([abs_pos, roll-yaw, pitch]).subs(zip(q_symbols, j_s)).evalf()

    return sp.Matrix([abs_pos, roll, pitch, yaw]).subs(zip(q_symbols, j_s)).evalf()


def get_leaps_from_jacobian(J):
    leap_joints = []
    for i in range(J.shape[1]):
        if max(J.col(i)) > 20 :
            leap_joints.append(i)
    return leap_joints



JOINT_STATES = [-0.208, 1.36, 0.036, 0.649, 0.809, 0.6756, -0.81]

GOAL_POS = [0.9137, -0.1546, 0.5375, 1, 1.57, 0]




result, difference = servo_calcs(DH_PARAMS, GOAL_POS, JOINT_STATES, orientation=True)

sp.pprint(result)
sp.pprint(difference)
