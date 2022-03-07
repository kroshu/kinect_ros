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

CONFIG_PATH = os.path.join(str(Path(os.getcwd()).parent.absolute()),
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

def calc_jacobian(dh_params, joint_pos=None):
    """
    Calculates the Jacobian matrix for a robot chain from base to end effector based on Denavit-Hartenberg parameters
    """

    # Calculate base to end effector transform
    T_full = np.identity(4)
    rot_matrices = []
    tool_length = 0
    if 'tool_length' in dh_params.keys():
        tool_length = dh_params['tool_length']
    for i in range(joint_count):
        joint_dh = dh_params[f'joint_{i + 1}'][0].split(' ')

        if i == joint_count -1:
            T_full = T_full * DH2T(*joint_dh, tool_length)
        else:
            T_full = T_full * DH2T(*joint_dh)
        rot_matrices.append(T_full[:3, :3])

    # Calculate Jacobian from Transform matrix and rotations
    q = [sp.symbols(f'q{i + 1}') for i in range(joint_count)]
    abs_pos = T_full[:3, 3]    
    Jv = abs_pos.jacobian(q)

    Jw=sp.Matrix()
    for i in range(len(rot_matrices)):
        Jw = Jw.col_insert(i, rot_matrices[i]*sp.Matrix([0, 0, 1]))

    J=sp.Matrix([Jv, Jw])

    if joint_pos == None:
        return J
    else:        
        subs = zip(q, joint_pos)
        return J.subs(subs)


def pseudo_inverse(J):
    """
    Calculates the pseuso inverse for a given Jacobian matrix
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

J = calc_jacobian(dh_params, [0, 0, 0, 0, 0, 0, 0]).evalf()

sp.pprint(J.evalf(3))

pseudo_inv = pseudo_inverse(J)

damped_l = damped_least_squares(J, 0.001)

sp.pprint(damped_l.evalf(3))

