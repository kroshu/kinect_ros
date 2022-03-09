#!/usr/bin/python3

"""
Kinematics calculations of a robot

    Denavit-Hartenberg parameters:
"""

import os
import numpy as np
import sympy as sp
from pathlib import Path
import yaml

CONFIG_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                           'config', 'LBR_iiwa_DH.yaml')

with open(CONFIG_PATH, 'r', encoding="utf-8") as file:
    config_dict = yaml.safe_load(file)

dh_params = config_dict["DH_params"]

joint_count = 0

while f'joint_{joint_count + 1}' in dh_params.keys():
    joint_count += 1
print(f'Number of joints: {joint_count}')


def DH2T(s_a, s_alpha, s_d, s_theta, tool_length=0):
    """
    Calculates homogenous tranformation matrix from Denavit-Hartenberg parameters (Khalil-modification)
    """
    a = sp.parsing.parse_expr(s_a)
    alpha = sp.parsing.parse_expr(s_alpha)
    d = sp.parsing.parse_expr(s_d) + tool_length
    theta = sp.parsing.parse_expr(s_theta)

    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, a],
                 [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha), -d * sp.sin(alpha)],
                 [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha), d * sp.cos(alpha)],
                 [0, 0, 0, 1]])                 
    return T

def calc_jacobian(dh_params, joint_pos=None, orientation=True):
    """
    Calculates the Jacobian matrix for a robot chain from base to end effector based on Denavit-Hartenberg parameters
    """

    # Calculate base to end effector transform
    T_full = calc_transform(dh_params)

    # Calculate Jacobian from Transform matrix and rotations
    q = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = T_full[:3, 3]    
    Jv = abs_pos.jacobian(q)

    if orientation:
        R = T_full[:3, :3]
        cp = sp.sqrt(R[0,0]**2 + R[1,0]**2) # this equals cos(pitch)
               
        yaw = sp.atan2(R[1,0], R[0,0])
        pitch = sp.atan2(-R[2,0], cp)
        roll = sp.atan2(R[2,1], R[2,2])

        Jw = sp.Matrix([roll, pitch, yaw]).jacobian(q)

        # This was different orientation
        # Jw = sp.Matrix()
        # for i in range(len(rot_matrices)):
        #     Jw = Jw.col_insert(i, rot_matrices[i]*sp.Matrix([0, 0, 1]))

        J=sp.Matrix([Jv, Jw])
    else:
        J = Jv

    if joint_pos == None:
        return J
    else:
        if  orientation and cp.subs(zip(q, joint_pos)).evalf() < 1e-6:
            js = joint_pos
            print("Pitch is almost +-90°")
            # in this case, the solution is not straighforward
            # atan2(0, 0) would be nan -> orientation is calculated with a different joint setup
            # only the first (from TCP) joint is modified, that changes pitch angle slightly
            for i in reversed(range(joint_count)):
                js[i] = joint_pos[i] + 0.001
                if cp.subs(zip(q, js)).evalf() < 1e-6:
                    js[i] = joint_pos[i]
                else:
                    break
            print(js) 
        return J.subs(zip(q, joint_pos)).evalf()


def pseudo_inverse(J):
    """
    Calculates the pseudo inverse for a given Jacobian matrix
    """
    if (J * J.transpose()).det() == 0:
        print('Singularity reached!!')
        return -1
    else:
        J_inv = J.transpose() * (J * J.transpose()).inv()

    return J_inv

def damped_least_squares(J, mu):
    """
    Calculates matrix for servoing with damped least squares method
    """

    J_inv = J.transpose() * (J * J.transpose() + mu * np.eye(6)).inv()

    return J_inv


def servo_calcs(goal_pos, joint_states, orientation=True, max_iter=500):
    js = joint_states
    actual_pos = calc_forw_kin(calc_transform(dh_params), js)
    sp.pprint(actual_pos)
    if orientation:
        diff = sp.Matrix([goal_pos]).transpose() - sp.Matrix([actual_pos])
    else:
        diff = sp.Matrix([goal_pos[:3]]).transpose() - actual_pos[:3,:]
    i = 0
    if diff.norm() > 1:
        print("Initial distance too big")
        return actual_pos, diff
    while (diff.norm() > 1e-3 and i < max_iter):
        print(diff.norm(), i)
        i += 1
        J_inv = pseudo_inverse(calc_jacobian(dh_params, joint_states, orientation))
        if orientation:
            delta_theta = J_inv * diff
        else:
            print(diff[:3, :])
            delta_theta = J_inv * diff[:3, :]
        #delta_theta = delta_theta / max(delta_theta) * 0.02
        new_joints = sp.Matrix(js) + delta_theta * 0.3 # TODO: fine tune multiplier
        # TODO: maybe split the cartesian difference into small distances
        js = [item for sublist in new_joints.tolist() for item in sublist]
        actual_pos = calc_forw_kin(calc_transform(dh_params), js)
        sp.pprint(actual_pos.transpose())
        if orientation:
            diff = sp.Matrix([goal_pos]).transpose() - sp.Matrix([actual_pos])
        else:
            diff = sp.Matrix([goal_pos[:3]]).transpose() - actual_pos[:3,:]
        sp.pprint(diff.transpose())

    return js, diff

def calc_transform(dh_params):
    # Calculate base to end effector transform
    T_full = np.identity(4)
    tool_length = 0
    if 'tool_length' in dh_params.keys():
        tool_length = dh_params['tool_length']
    for i in range(joint_count):
        joint_dh = dh_params[f'joint_{i + 1}'][0].split(' ')

        if i == joint_count -1:
            T_full = T_full * DH2T(*joint_dh, tool_length)
        else:
            T_full = T_full * DH2T(*joint_dh)
    return T_full


def calc_forw_kin(T, joint_pos):
    js=joint_pos
    q = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = T[:3, 3]
    R = T[:3, :3]

    cp = sp.sqrt(R[0,0]**2 + R[1,0]**2) # this equals cos(pitch)
    roll = sp.atan2(R[2,1], R[2,2])
    pitch = sp.atan2(-R[2,0], cp)
    yaw = sp.atan2(R[1,0], R[0,0])    
    
    if cp.subs(zip(q, js)).evalf() < 1e-6: 
        print("Pitch is almost +-90°")
        # in this case, the RPY angles are not straighforward
        # atan2(0, 0) would be nan -> orientation is calculated with a slightly different joint setup
        # only the first (from TCP) joint is modified, which changes pitch angle
        for i in reversed(range(joint_count)):
            js[i] = joint_pos[i] + 0.001
            if cp.subs(zip(q, js)).evalf() < 1e-6:
                js[i] = joint_pos[i]
            else:
                break

    return sp.Matrix([abs_pos, roll, pitch, yaw]).subs(zip(q, js)).evalf()




joint_states = [1, 1, 1, 1, 0, 0, 0]


# J = calc_jacobian(dh_params, joint_states)
# sp.pprint(J.evalf(3)[3:,:])

# cartesian = calc_forw_kin(calc_transform(dh_params), joint_states)
# sp.pprint(cartesian)



js, diff = servo_calcs([0.60361, 0.2166, 0.88038, 0.8564, -0.122, 1.4759], joint_states, True)

sp.pprint(js)
sp.pprint(diff)

