#!/usr/bin/python3

"""
Kinematics calculations of a robot

    Denavit-Hartenberg parameters:
"""

import os
from pathlib import Path
import csv
import math
import random
import numpy as np
import sympy as sp
import yaml

import kinematics as kn

SET_LAST = 0  # should equal the $tries argument of kinematic.adjust_goal function (odd integer or 0)

CONFIG_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                           'config', 'LBR_iiwa_DH.yaml')

with open(CONFIG_PATH, 'r', encoding="utf-8") as config_file:
    config_dict = yaml.safe_load(config_file)

DH_PARAMS = config_dict["DH_params"]

joint_count = 0

while f'joint_{joint_count + 1}' in DH_PARAMS.keys():
    joint_count += 1
print(f'Number of joints: {joint_count}')

# Print headers
with open('test.csv', 'w', encoding="utf-8") as file:
    writer = csv.writer(file)
    original = [f'joint{i + 1}' for i in range(joint_count)]
    modified = [f'j{i + 1}_mod' for i in range(joint_count)]
    servoed = [f'j{i + 1}_servo' for i in range(joint_count)]
    results = ['success', 'mod_norm', 'result_norm', 'min_dist']
    writer.writerow(original + modified + servoed + results)


for i in range(500):
    print(f'{i + 1}. iteration')
    js_orig = []  # this should be reached
    diff = []
    joint_states = []  # this is measured by the camera
    success = []
    distances = []
    for j in range(joint_count):
        # Specific joint limits for iiwa:
        if j in [1, 3, 5]:
            js_orig.append(round(random.uniform(math.radians(-120), math.radians(120)), 3))
        else:
            js_orig.append(round(random.uniform(math.radians(-170), math.radians(170)), 3))

        # Slight changes in joints based on experience
        if j in range(4):
            diff.append(round(random.uniform(-0.2, 0.2), 3))
        else:
            diff.append(round(random.uniform(-0.5, 0.5), 3))
    for j in range(joint_count):
        joint_states.append(js_orig[j] + diff[j])

    # js_orig = [2.846, 1.865, -1.154, 0.659, 2.714, -0.116, -2.326]
    # joint_states = [2.879, 1.786, -1.046, 0.65, 3.085, -0.29, -2.005]
    # zip_object = zip(joint_states, js_orig)
    # diff = []
    # for list1_i, list2_i in zip_object:
    #     diff.append(list1_i-list2_i)


    if SET_LAST:
        joint_states[-1] = 0
    goal_pos = kn.calc_forw_kin(kn.calc_transform(DH_PARAMS), js_orig)
    servo_joints = kn.servo_calcs(DH_PARAMS, goal_pos, joint_states, orientation=True, max_iter=500,
                                  set_last=SET_LAST)[0]
    if servo_joints == -1:
        success.append(0)
        servo_list = [np.nan] * 7
    else:
        for j in range(joint_count):
            if abs(servo_joints[j]) > sp.pi:
                cycles = int(abs(servo_joints[j] / (2 * sp.pi)))
                print(cycles, servo_joints[j])
                servo_joints[j] -= np.sign(servo_joints[j]) * 2 * sp.pi.evalf() * (cycles + 1)
        success.append(1)
        servo_list = [item for sublist in servo_joints.tolist() for item in sublist]
        diff_mod = sp.Matrix(joint_states).transpose() - servo_joints
        # Ignore last joint in evaluation of solution, as that is 'unknown' due to camera precision issue
        if (SET_LAST):
            diff = diff[:6]
            diff_mod = diff_mod[6]
        distances.append(round(sp.Matrix(diff).norm(), 4))
        distances.append(round(diff_mod.norm(), 4))
        if sp.Matrix(diff).norm() > diff_mod.norm():
            distances.append(1)
        else:
            distances.append(0)
    with open('test.csv', 'a', encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(js_orig + joint_states + servo_list + success + distances)
