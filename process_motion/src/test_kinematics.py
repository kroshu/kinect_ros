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
    results = ['success', 'avg_orig', 'max_orig', 'avg_mod', 'max_mod']
    writer.writerow(original + modified + servoed + results)


# TODO: try with multiple values for joint7
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
    # joint_states[-1] = 0
    goal_pos = kn.calc_forw_kin(kn.calc_transform(DH_PARAMS), joint_states).transpose()
    servo_joints = kn.servo_calcs(DH_PARAMS, goal_pos, js_orig, orientation=True, max_iter=100)[0]
    if servo_joints == -1:
        success.append(0)
        servo_list = [np.nan] * 7
    else:
        success.append(1)
        servo_list = [item for sublist in servo_joints.tolist() for item in sublist]
        diff_orig = np.array(sp.Matrix(js_orig).transpose() - servo_joints).astype(np.float64)
        diff_mod = np.array(sp.Matrix(joint_states).transpose() - servo_joints).astype(np.float64)
        distances.append(round(np.mean(abs(diff_orig)), 3))
        distances.append(round(np.amax(abs(diff_orig)), 3))
        distances.append(round(np.mean(abs(diff_mod)), 3))
        distances.append(round(np.amax(abs(diff_mod)), 3))
    with open('test.csv', 'a', encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(js_orig + joint_states + servo_list + success + distances)
