#!/usr/bin/python3.10

import os
import sys
import pandas as pd
import numpy as np
import scipy.signal as sg
from pathlib import Path
import yaml


print(sys.version)
allowed_keys=["type", "window_size", "keep_first", "keep_last"]

# Turns a dictionary into a class
class Dict2Class(object):      
    def __init__(self, my_dict):
        self.type="simple"
        self.window_size=5
        self.keep_beginning=False
        self.keep_end=False
        for key in my_dict:
            if key in allowed_keys:
                setattr(self, key, my_dict[key])
            else:
                print(f'Invalid key found in yaml: {key}')



def MA(window, periods):
    moving_avg=data.rolling(window, min_periods=periods).mean()
    return moving_avg

def WMA(window, periods):
    weights = np.full((1, 2), 1/window)
    moving_avg=data.rolling(window, min_periods=periods).apply(lambda x: np.sum(weights*x))
    return moving_avg

def symmetric(window, periods):
    moving_avg=data.rolling(window, center= True, min_periods=periods).mean()
    return moving_avg

def filter(window):
    cutoff=1/window
    # by default sampling freq = Nyquist freq
    # cutoff specifies the ratio of cutoff and Nyquist (=sampling) frequencies
    b, a = sg.butter(4, cutoff)
    moving_avg=pd.DataFrame(sg.filtfilt(b, a, data, axis=0))  # TODO: try different padding and gust method
    return moving_avg


def smooth_graph(config):
    match config.type:
        case "simple":
            if config.keep_first:
                return MA(config.window_size, 1)
            else:
                return MA(config.window_size, config.window_size)
        case "filter":
            return filter(config.window_size)


n = len(sys.argv)
print("Files to process:", n-1)
 

ws_dir=str(Path(os.getcwd()).parent.parent.parent.absolute())
csv_dir=os.path.join(ws_dir, "replay", "")
config_path=os.path.join(str(Path(os.getcwd()).parent.absolute()), "config", "moving_average.yaml")
print(config_path)
print(ws_dir)


with open(config_path, 'r') as file:
    config_dict=yaml.safe_load(file)

default_config=Dict2Class(config_dict["moving_average"]["default"])
if "first" in config_dict["moving_average"].keys():
    first_config=Dict2Class(config_dict["moving_average"]["first"])
else:
    first_config=default_config
if "last" in config_dict["moving_average"].keys():
    last_config=Dict2Class(config_dict["moving_average"]["last"])
else:
    last_config=default_config


for i in range(1, n):
    data = pd.read_csv(csv_dir+"motion"+str(sys.argv[i])+".csv", sep=',', decimal='.', header=None)
    if i == 1:
        moving_avg=smooth_graph(first_config)
    elif i is n-1:
        moving_avg=smooth_graph(last_config)
    else:
        moving_avg=smooth_graph(default_config)
    moving_avg.dropna(inplace=True)
    print(moving_avg)
    moving_avg.to_csv(csv_dir+"motion"+str(sys.argv[i])+"_tmp.csv", sep=',', decimal='.', header=None,  index=False)
