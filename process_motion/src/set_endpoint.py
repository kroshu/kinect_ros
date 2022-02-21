#!/usr/bin/python3.10

"""
Sets one or both endpoints of a motion to an exact position

    Configuration file: config/set_endpoint_i.yaml
"""

import os
import sys
from cmath import nan
from pathlib import Path
import pandas as pd
import yaml

ALLOWED_KEYS = ['mode', 'reference_count', 'position', 'orientation']


class Dict2Class:
    """
    Converts the yaml dictionary to an object with the required member variables
    """
    def __init__(self, my_dict):
        self.mode = 'position_last'
        self.reference_count = 5
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [nan, nan, nan, nan]
        for key in my_dict:
            if key in ALLOWED_KEYS:
                setattr(self, key, my_dict[key])
            else:
                print(f'Invalid key found in yaml: {key}')
        # TODO: check param validity

def set_endpoint(data_csv, config, first):
    """
    Set exact endpoints to a motion based on the config file.
    """
    # TODO: check distance of actual and desired positions
    return data_csv


ARG_COUNT = len(sys.argv)
print('Files to set endpoints to:', ARG_COUNT-1)

WS_DIR = str(Path(os.getcwd()).parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
CONFIG_PATH = os.path.join(str(Path(os.getcwd()).parent.absolute()),
                           'config', 'set_endpoint_')

CONFIG_FILES = [os.path.join(str(Path(os.getcwd()).parent.absolute()),
                           'config', f'set_endpoint_{sys.argv[i]}.yaml') for i in range(1, ARG_COUNT)]




#TODO: check if config exists for all required files
for j in range(1, ARG_COUNT):
    if not Path(CONFIG_FILES[j - 1]).is_file():
        print(f'Config file not found for motion_{sys.argv[j]}, terminating')
        sys.exit()
    with open(CONFIG_FILES[j - 1], 'r', encoding="utf-8") as file:
        config_dict = yaml.safe_load(file)
    data_csv = pd.read_csv(CSV_DIR + f'motion{sys.argv[j]}.csv',
                           sep=',', decimal='.', header=None)
    if 'first' in config_dict['set_endpoint'].keys():
        config = Dict2Class(config_dict['set_endpoint']['first'])
        data_csv = set_endpoint(data_csv, config, True)
    if 'last' in config_dict['set_endpoint'].keys():
        config = Dict2Class(config_dict['set_endpoint']['last'])
        data_csv = set_endpoint(data_csv, config, False)


    data_csv.to_csv(CSV_DIR + f'motion{sys.argv[j]}_tmp.csv',
                    sep=',', decimal='.', header=None,  index=False)
