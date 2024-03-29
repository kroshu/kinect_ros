#!/usr/bin/python3.10

"""
Sets one or both endpoints of a motion to an exact position.

    Configuration file: config/set_endpoint_i.yaml
"""

import os
from pathlib import Path
import sys

import kinematics as kn
import numpy as np
import pandas as pd
import yaml


ALLOWED_KEYS = ['reference_count', 'position', 'orientation']
SET_LAST = 11


class Dict2Class:
    """Converts the yaml dictionary to an object with the required member variables."""

    def __init__(self, my_dict):
        self.reference_count = 5
        self.position = [0.0, 0.0, 1.292]
        self.orientation = [0.0, 0.0, 0.0]
        for key in my_dict:
            if key in ALLOWED_KEYS:
                setattr(self, key, my_dict[key])
            else:
                print(f'[ERROR] Invalid key found in yaml: {key}')


def set_endpoint(data_csv, config, first):
    """Set exact endpoints to a motion based on the config file."""
    if first:
        joint_states = data_csv.iloc[0].values.tolist()
    else:
        joint_states = data_csv.iloc[-1].values.tolist()
    goal_pos = [config.position['x'], config.position['y'], config.position['z'],
                config.orientation['r'], config.orientation['p'], config.orientation['y']]

    result = kn.servo_all_methods(DH_PARAMS, goal_pos, joint_states, set_last=SET_LAST)

    if result is None:
        print('[WARNING] Setting endpoint not successful with first attempt')
        print('[WARNING] Trying with different values for joint 7')
        for i in range(11):
            joint_states_mod = joint_states.copy()
            joint_states_mod[6] = np.round((i - (SET_LAST - 1) / 2) * 6.28 / (SET_LAST + 1), 4)
            result = kn.servo_all_methods(DH_PARAMS, goal_pos, joint_states_mod)
            if result is not None:
                break
            if i == SET_LAST - 1:
                print('[ERROR] Setting endpoint not successful')
                return data_csv
    result_arr = (np.array(result.transpose()).astype(np.float64))
    result_df = pd.DataFrame(result_arr)[0]
    diff = result_df - joint_states
    if max(abs(diff[:6])) > 0.25:
        print('[WARNING] There was a big change in one of the joint values')
        print(diff)

    last_joint_mod = abs(int(diff.iloc[-1] / 0.075)) + 1
    if last_joint_mod > data_csv.shape[0] / 2:
        last_joint_mod = int(data_csv.shape[0] / 2)
    if last_joint_mod > config.reference_count:
        for i in range(last_joint_mod):
            if first:
                data_csv.iloc[i, -1] = (data_csv.iloc[i, -1] + diff.iloc[-1] *
                                        (last_joint_mod - i) / last_joint_mod)
            else:
                data_csv.iloc[-i - 1, -1] = (data_csv.iloc[-i - 1, -1] + diff.iloc[-1] *
                                             (last_joint_mod - i) / last_joint_mod)
        diff.iloc[-1] = 0

    for i in range(config.reference_count):
        if first:
            data_csv.iloc[i] = (data_csv.iloc[i] + diff * (config.reference_count - i) /
                                config.reference_count)
        else:
            data_csv.iloc[-i - 1] = (data_csv.iloc[-i - 1] + diff * (config.reference_count - i) /
                                     config.reference_count)

    return data_csv


ARG_COUNT = len(sys.argv)
print('Files to set endpoints to:', ARG_COUNT-1)

WS_DIR = str(Path(__file__).parent.parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
CONFIG_FILES = [os.path.join(str(Path(__file__).parent.parent.absolute()), 'config',
                             f'set_endpoint_{sys.argv[i]}.yaml') for i in range(1, ARG_COUNT)]
MODELL_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                           'config', 'LBR_iiwa_DH.yaml')

with open(MODELL_PATH, 'r', encoding='utf-8') as config_file:
    config_dict = yaml.safe_load(config_file)
DH_PARAMS = config_dict['DH_params']

for j in range(1, ARG_COUNT):
    if not Path(CONFIG_FILES[j - 1]).is_file():
        print(f'[ERROR] Config file not found for motion_{sys.argv[j]}, terminating')
        sys.exit()
    with open(CONFIG_FILES[j - 1], 'r', encoding='utf-8') as file:
        config_dict = yaml.safe_load(file)
    data = pd.read_csv(CSV_DIR + f'motion{sys.argv[j]}.csv',
                       sep=',', decimal='.', header=None)
    if 'first' in config_dict['set_endpoint'].keys():
        configs = Dict2Class(config_dict['set_endpoint']['first'])
        data = set_endpoint(data, configs, True)
    if 'last' in config_dict['set_endpoint'].keys():
        configs = Dict2Class(config_dict['set_endpoint']['last'])
        data = set_endpoint(data, configs, False)

    # this overrides original data series!!
    data.to_csv(CSV_DIR + f'motion{sys.argv[j]}.csv',
                sep=',', decimal='.', header=None,  index=False)
