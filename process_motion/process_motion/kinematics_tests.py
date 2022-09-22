#!/usr/bin/python3

"""Run tests to determine effectiveness of kinematics calculations."""

import csv
import os
from pathlib import Path
import random

import kinematics as kn
import pandas as pd
import sympy as sp
import yaml


# should equal the $tries argument of kinematic.adjust_goal function (odd integer or 0)
SET_LAST = 11

CONFIG_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                           'config', 'LBR_iiwa_DH.yaml')

CSV_PATH = os.path.join(str(Path(__file__).parent.parent.parent.parent.parent.absolute()),
                        'data', 'set_test_constants', '500_tests.csv')

with open(CONFIG_PATH, 'r', encoding='utf-8') as config_file:
    config_dict = yaml.safe_load(config_file)

DH_PARAMS = config_dict['DH_params']

joint_count = 0

while f'joint_{joint_count + 1}' in DH_PARAMS.keys():
    joint_count += 1
print(f'Number of joints: {joint_count}')

# Print headers
with open('test.csv', 'w', encoding='utf-8') as file:
    writer = csv.writer(file)
    original = [f'joint{i + 1}' for i in range(joint_count)]
    modified = [f'j{i + 1}_mod' for i in range(joint_count)]
    servoed = [f'j{i + 1}_servo' for i in range(joint_count)]
    results = ['success', 'mod_norm', 'result_norm', 'min_dist']
    writer.writerow(original + modified + servoed + results)


i = 0
data_csv = pd.read_csv(CSV_PATH, sep=',', decimal='.')
while i < 500:
    success = []
    distances = []
    diff = []
    print(f'{i + 1}. iteration')
    # Run pre-defined test cases
    if i < data_csv.shape[0]:
        js_orig = data_csv.iloc[i][:7].values.tolist()
        joint_states = data_csv.iloc[i][7:].values.tolist()
        zip_object = zip(joint_states, js_orig)
        for list1_i, list2_i in zip_object:
            diff.append(list1_i-list2_i)
    else:
        js_orig = []  # this should be reached
        joint_states = []  # this is measured by the camera

        for j in range(joint_count):
            js_orig.append(round(random.uniform(kn.LOWER_LIMITS_R[j], kn.UPPER_LIMITS_R[j]), 3))

            # Slight changes in joints based on experience
            if j in range(4):
                diff.append(round(random.uniform(-0.2, 0.2), 3))
            else:
                diff.append(round(random.uniform(-0.5, 0.5), 3))
        for j in range(joint_count):
            joint_states.append(js_orig[j] + diff[j])
            if joint_states[j] > kn.UPPER_LIMITS_R[j]:
                joint_states[j] = round(kn.UPPER_LIMITS_R[j] - 0.0005, 3)
            elif joint_states[j] < kn.LOWER_LIMITS_R[j]:
                joint_states[j] = round(kn.LOWER_LIMITS_R[j] + 0.0005, 3)
    i += 1
    if SET_LAST:
        joint_states[-1] = 0
    goal_pos = kn.calc_forw_kin(kn.calc_transform(DH_PARAMS), js_orig)
    servo_joints = kn.servo_calcs(DH_PARAMS, goal_pos, joint_states, set_last=SET_LAST)[0]
    servo_list = kn.process_result(servo_joints)

    # Check joint limits and re-run test, if they were exceeded
    if servo_joints is not None and not kn.check_joint_limits(servo_joints)[0]:
        success = [-1]
        print('Exceeded limits, runnning with new configuration')
        with open('test.csv', 'a', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(js_orig + joint_states + servo_list
                            + success)
        success = []
        servo_joints = kn.servo_calcs(DH_PARAMS, goal_pos, joint_states, set_last=SET_LAST,
                                      max_iter=250, joint_limits=1)[0]
        if servo_joints is None:
            print('Enforcing joint limits also failed, retrying with goal vector')
            servo_joints = kn.servo_calcs(DH_PARAMS, goal_pos, joint_states, set_last=SET_LAST,
                                          max_iter=250, joint_limits=2)[0]
            if servo_joints is None:
                print('Goal vector failed, last attempt with enforcing + goal vector')
                servo_joints = kn.servo_calcs(DH_PARAMS, goal_pos, joint_states, set_last=SET_LAST,
                                              max_iter=250, joint_limits=3)[0]
        servo_list = kn.process_result(servo_joints)
        if servo_joints is not None and not kn.check_joint_limits(servo_joints)[0]:
            print('New configuration was also not successful')
            success = [0]
            servo_joints = None

    if servo_joints is not None:
        success = [1]
        diff_mod = sp.Matrix(joint_states).transpose() - servo_joints

        # Ignore last joint in evaluation of solution,
        #   as that is 'unknown' due to camera precision issue
        if SET_LAST:
            diff = sp.Matrix(diff[:6])
            diff_mod = sp.Matrix(diff_mod[:6])
        distances.append(round(sp.Matrix(diff).norm(), 4))
        distances.append(round(diff_mod.norm(), 4))
        if round(sp.Matrix(diff).norm(), 4) < round(diff_mod.norm(), 4):
            distances.append(0)
        else:
            distances.append(1)
    else:
        success = [0]
    with open('test.csv', 'a', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(js_orig + joint_states + servo_list + success + distances)
