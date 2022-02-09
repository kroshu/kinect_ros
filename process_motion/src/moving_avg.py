#!/usr/bin/python3.10

from fcntl import DN_DELETE
import os
import sys
import pandas as pd
import numpy as np
import scipy.signal as sg
from pathlib import Path
import yaml

joint_count = 7
allowed_keys = ['type', 'window_size', 'keep_first', 'keep_last']

# Turns a dictionary into a class


class Dict2Class(object):

    def __init__(self, my_dict):
        self.type = 'simple'
        self.window_size = [2, 2, 2, 2, 5, 5, 1]
        self.keep_beginning = False
        self.keep_end = False
        for key in my_dict:
            if key in allowed_keys:
                setattr(self, key, my_dict[key])
            else:
                print(f'Invalid key found in yaml: {key}')
        if len(self.window_size) != joint_count:
            print('Invalid length for window_size, terminating')
            sys.exit()


moving_avg = pd.DataFrame([])


def MA(window, periods):
    for i in range(joint_count):
        moving_avg[i] = data[i].rolling(window[i], min_periods=periods[i]).mean()
    return moving_avg


def calc_weights(x):
    if len(x) < 3:
        # weights are assigned evenly, like SMA, because window is too small
        weights = np.full(1, 1/len(x))
    elif (x.iloc[-1] - x.iloc[-2]) * (x.iloc[-2] - x.iloc[-3]) > 0:
        weights = [1 / (len(x) + 2)] * (len(x) - 2)
        weights.extend([1.5 / (len(x) + 2), 2.5 / (len(x) + 2)])
    else:
        weights = np.full(1, 1/len(x))
    return np.sum(weights*x)


def WMA(window, periods):
    for i in range(joint_count):
        moving_avg[i] = data[i].rolling(window[i],
                                        min_periods=periods[i]).apply(lambda x: calc_weights(x))
    return moving_avg


def symmetric(window, periods):
    for i in range(joint_count):
        moving_avg[i] = data[i].rolling(window[i], center=True, min_periods=periods[i]).mean()
    return moving_avg


def filter(window):
    for i in range(joint_count):
        cutoff = 1 / window[i]
        # by default sampling freq = Nyquist freq
        # cutoff specifies the ratio of cutoff and Nyquist (=sampling) frequencies
        b, a = sg.butter(4, cutoff)
        moving_avg[i] = pd.DataFrame(sg.filtfilt(b, a, data[i], axis=0))
        # TODO: try different padding and gust method
    return moving_avg


def smooth_graph(config):
    match config.type:
        case 'simple':
            if config.keep_first:
                result = MA(config.window_size, [1] * joint_count)
            else:
                result = MA(config.window_size, config.window_size)
            if config.keep_last:
                result.iloc[-1] = data.iloc[-1]
            return result

        case 'filter':
            return filter(config.window_size)
            # TODO: keep_first, keep_last
        case 'symmetric':
            result = symmetric(config.window_size, config.window_size)
            if config.keep_last:
                result.iloc[-1] = data.iloc[-1]
            if config.keep_first:
                result.iloc[0] = data.iloc[0]
            return result
        case 'weighted':
            if config.keep_first:
                result = WMA(config.window_size, [1] * joint_count)
            else:
                result = WMA(config.window_size, config.window_size)
            if config.keep_last:
                result.iloc[-1] = data.iloc[-1]
            return result
        case _:
            print('Unknown type, terminating')
            sys.exit()


n = len(sys.argv)
print('Files to process:', n-1)

ws_dir = str(Path(os.getcwd()).parent.parent.parent.parent.absolute())
csv_dir = os.path.join(ws_dir, 'replay', 'data', '')
config_path = os.path.join(str(Path(os.getcwd()).parent.absolute()),
                           'config', 'moving_average.yaml')

with open(config_path, 'r') as file:
    config_dict = yaml.safe_load(file)

default_config = Dict2Class(config_dict['moving_average']['default'])
if 'first' in config_dict['moving_average'].keys():
    first_config = Dict2Class(config_dict['moving_average']['first'])
else:
    first_config = default_config
if 'last' in config_dict['moving_average'].keys():
    last_config = Dict2Class(config_dict['moving_average']['last'])
else:
    last_config = default_config


for i in range(1, n):
    data = pd.read_csv(csv_dir+'motion'+str(sys.argv[i]) + '.csv',
                       sep=',', decimal='.', header=None)
    if i == 1:
        smoothed = smooth_graph(first_config)
    elif i is n - 1:
        smoothed = smooth_graph(last_config)
    else:
        smoothed = smooth_graph(default_config)

    # this removes all lines with NaN values
    # the different lengthes due to different window sizes are resolved
    smoothed.dropna(inplace=True)
    print(smoothed.iloc[[0, -1]])
    print(data.iloc[[0, -1]])

    smoothed.to_csv(csv_dir+'motion'+str(sys.argv[i])+'_tmp.csv',
                    sep=',', decimal='.', header=None,  index=False)
