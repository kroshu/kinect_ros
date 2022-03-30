#!/usr/bin/python3.10

"""
Sets one or both endpoints of a motion to an exact position

    Configuration file: config/set_endpoint_i.yaml
"""

import os
import sys
from pathlib import Path
import pandas as pd
import numpy as np
import yaml

import kinematics as kn

ALLOWED_KEYS = ['reference_count', 'position', 'orientation']

class Dict2Class:
    """
    Converts the yaml dictionary to an object with the required member variables
    """
    def __init__(self, my_dict):
        self.reference_count = 5
        self.position = [0.0, 0.0, 1.292]
        self.orientation = [0.0, 0.0, 0.0]
        for key in my_dict:
            if key in ALLOWED_KEYS:
                setattr(self, key, my_dict[key])
            else:
                print(f'Invalid key found in yaml: {key}')

def set_endpoint(data_csv, config, first):
    """
    Set exact endpoints to a motion based on the config file.
    """
    if first:
        joint_states = data_csv.iloc[0]
    else:
        joint_states = data_csv.iloc[-1]
    goal_pos = [config.position['x'], config.position['y'], config.position['z'],
                config.orientation['r'], config.orientation['p'], config.orientation['y']]
    try:
        result = (np.array(kn.servo_calcs(DH_PARAMS, goal_pos, joint_states)[0].transpose())
                  .astype(np.float64))
    except AttributeError:
        print('Setting endpoint not successful')
        return data_csv
    result_df = pd.DataFrame(result)[0]
    diff = result_df - joint_states
    print(diff)
    for i in range (config.reference_count):
        if first:
            data_csv.iloc[i] = (data_csv.iloc[i] + diff * (config.reference_count + 1 - i) /
                                (config.reference_count + 1))
        else:
            data_csv.iloc[-i] = (data_csv.iloc[-i] + diff * (config.reference_count + 1 - i) /
                                 (config.reference_count + 1))
    return data_csv


ARG_COUNT = len(sys.argv)
print('Files to set endpoints to:', ARG_COUNT-1)

WS_DIR = str(Path(__file__).parent.parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
CONFIG_FILES = [os.path.join(str(Path(__file__).parent.parent.absolute()), 'config',
                           f'set_endpoint_{sys.argv[i]}.yaml') for i in range(1, ARG_COUNT)]
MODELL_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
                           'config', 'LBR_iiwa_DH.yaml')

with open(MODELL_PATH, 'r', encoding="utf-8") as config_file:
    config_dict = yaml.safe_load(config_file)
DH_PARAMS = config_dict["DH_params"]

for j in range(1, ARG_COUNT):
    if not Path(CONFIG_FILES[j - 1]).is_file():
        print(f'Config file not found for motion_{sys.argv[j]}, terminating')
        sys.exit()
    with open(CONFIG_FILES[j - 1], 'r', encoding="utf-8") as file:
        config_dict = yaml.safe_load(file)
    data = pd.read_csv(CSV_DIR + f'motion{sys.argv[j]}.csv',
                           sep=',', decimal='.', header=None)
    if 'first' in config_dict['set_endpoint'].keys():
        configs = Dict2Class(config_dict['set_endpoint']['first'])
        data = set_endpoint(data, configs, True)
    if 'last' in config_dict['set_endpoint'].keys():
        configs = Dict2Class(config_dict['set_endpoint']['last'])
        data = set_endpoint(data, configs, False)

    data.to_csv(CSV_DIR + f'motion{sys.argv[j]}_tmp.csv',
                    sep=',', decimal='.', header=None,  index=False)
