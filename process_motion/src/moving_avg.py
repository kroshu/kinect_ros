#!/usr/bin/python3.10

"""
Applies a smoothing method on the recorded movement of the robot based on the configuration.

    Configuration file: config/moving_average.yaml
"""

import os
import sys
from cmath import sqrt
from statistics import mean
from pathlib import Path
import pandas as pd
import numpy as np
import scipy.signal as sg
import yaml

JOINT_COUNT = 7
ALLOWED_KEYS = ['type', 'window_size', 'keep_first', 'keep_last', 'pad']


class Dict2Class:
    """
    Converts the yaml dictionary to an object with the required member variables
    """
    def __init__(self, my_dict):
        self.type = 'simple'
        self.window_size = [2, 2, 2, 2, 5, 5, 1]
        self.keep_beginning = False
        self.keep_end = False
        self.pad = False
        for key in my_dict:
            if key in ALLOWED_KEYS:
                setattr(self, key, my_dict[key])
            else:
                print(f'Invalid key found in yaml: {key}')
        if len(self.window_size) != JOINT_COUNT:
            print('Invalid length for window_size, terminating')
            sys.exit()


def mov_avg(data, window, periods):
    """
    Implements a simple moving average
    """
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        moving_avg_tmp = data[i].rolling(window[i], min_periods=periods[i]).mean()
        moving_avg = pd.concat([moving_avg, moving_avg_tmp.dropna().reset_index(drop=True)],
                               axis=1)
    return moving_avg


def calc_weights(w_data):
    """
    Calculate weights for WMA
    Latest value has a bigger weight, if the end of the window is monotonous
    """
    if len(w_data) < 4:
        # weights are assigned evenly, like SMA, because window is too small
        weights = np.full(1, 1/len(w_data))
    elif ((w_data.iloc[-1] - w_data.iloc[-2]) * (w_data.iloc[-2] - w_data.iloc[-3]) > 0 and
          (w_data.iloc[-3] - w_data.iloc[-4]) * (w_data.iloc[-2] - w_data.iloc[-3]) > 0):
        weights = [1 / (len(w_data) + 2)] * (len(w_data) - 3)
        weights.extend([1.5 / (len(w_data) + 2), 1.6 / (len(w_data) + 2), 1.9 / (len(w_data) + 2)])
    else:
        weights = np.full(1, 1/len(w_data))
    return np.sum(weights * w_data)


def calc_weights2(w_data):
    """
    Calculate weights for WMA
    Latest value has a bigger weight, if the end of the window is significantly different
    from the average
    """
    if len(w_data) < 4:
        # weights are assigned evenly, like SMA, because window is too small
        weights = np.full(1, 1/len(w_data))
        # TODO: adjust limits
    elif (abs(w_data.iloc[-1] - w_data.mean()) > 0.05 * sqrt(len(w_data))):
        weights = [1 / (len(w_data) + 2)] * (len(w_data) - 3)
        weights.extend([1.5 / (len(w_data) + 2), 1.6 / (len(w_data) + 2), 1.9 / (len(w_data) + 2)])
    else:
        weights = np.full(1, 1/len(w_data))
    return np.sum(weights * w_data)


def w_moving_avg(data, window, periods):
    """
    Implements weighted moving average with not constant weights
    """
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        moving_avg_tmp = data[i].rolling(window[i],
                                         min_periods=periods[i]).apply(calc_weights2)
        moving_avg = pd.concat([moving_avg, moving_avg_tmp.dropna().reset_index(drop=True)],
                               axis=1)
    return moving_avg


def cw_moving_avg(data, window, periods):
    """
    Implements weighted moving average with constant weights
    """
    moving_avg = pd.DataFrame([])
    weights = [1]
    for i in range(len(data.columns)):
        if window[i] > 3:
            weights = [1 / (window[i] + 2)] * (window[i] - 3)
            weights.extend([1.5 / (window[i] + 2), 1.6 / (window[i] + 2), 1.9 / (window[i] + 2)])
        elif window[i] > 1:
            weights = [1 / (window[i] + 1)] * (window[i] - 1)
            weights.extend([2 / (window[i] + 1)])
        moving_avg_tmp = data[i].rolling(window[i],
                                         min_periods=periods[i]).apply(np.mean)
        moving_avg = pd.concat([moving_avg, moving_avg_tmp.dropna().reset_index(drop=True)],
                               axis=1)
    return moving_avg


def check_monotony(w_data, mon_count):
    """
    Checks whether the end of a window is monoton
    """
    if mon_count < 3:
        print("Monotony window too small")
        return False
    if mon_count > len(w_data):
        print("Monotony window bigger than observed")
        return False
    i = 0
    while i < mon_count - 2:
        if ((w_data.iloc[-(i + 1)] - w_data.iloc[-(i + 2)])
           * (w_data.iloc[-(i + 2)] - w_data.iloc[-(i + 3)]) < 0):
            return False
        i += 1
    return True


def w_moving_avg_2(data, window, periods, mon_count=4):
    """
    Implements weighted moving average with not constant weights
    The effect of the past is not completely neglected
    """
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        moving_avg_tmp = []
        window_data = []
        k = 0
        if window[i] < mon_count:
            simple_ma = data[i].rolling(window[i], min_periods=periods[i]).mean()
            simple_ma.dropna(inplace=True)
            moving_avg = pd.concat([moving_avg, simple_ma.reset_index(drop=True)], axis=1)
            continue
        weighted_sum = 0
        while k < len(data):
            prev_weighted_sum = weighted_sum
            old_data = 0
            if len(window_data) != 0:
                old_data = window_data.iloc[0]

            if k + 1 < window[i]:
                if periods[i] == window[i]:
                    k += 1
                    continue
                window_data = data[i][0:k + 1]
                # in this case no data is outdated, but a multiplication with (w - 1) / w is needed
                #  -> sum / w is substracted
                old_data = weighted_sum / len(window_data)
            else:
                window_data = data[i][k - window[i] + 1:k + 1]
            w_len = len(window_data)
            if (k + 1) == window[i] and window[i] == periods[i]:
                weighted_sum = mean(window_data)
            elif check_monotony(window_data, mon_count):
                weighted_sum = ((weighted_sum - old_data / w_len) * (w_len - 2) / (w_len - 1)
                                + 2 * window_data.iloc[-1] / w_len)
                # weighted_sum = weighted_sum / w * (w - 3) + 3 * window_data.iloc[-1] / w
            else:
                weighted_sum = weighted_sum - old_data / w_len + window_data.iloc[-1] / w_len
                # weighted_sum = weighted_sum / w * (w - 1) + window_data.iloc[-1] / w
            if weighted_sum > max(window_data):
                weighted_sum = np.mean([max(window_data), prev_weighted_sum])
            elif weighted_sum < min(window_data):
                weighted_sum = np.mean([min(window_data), prev_weighted_sum])

            moving_avg_tmp = np.append(moving_avg_tmp, [weighted_sum])
            k += 1
        moving_avg = pd.concat([moving_avg,
                               pd.DataFrame(moving_avg_tmp).dropna().reset_index(drop=True)],
                               axis=1, ignore_index=True)
    return moving_avg


def filter_butter(data, window):
    """
    Implements low-pass Butterworth filter
    """
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        cutoff = 1 / window[i]
        if cutoff == 1:
            cutoff = 0.999
        # by default sampling freq = Nyquist freq
        # cutoff specifies the ratio of cutoff and Nyquist (=sampling) frequencies
        num, den = sg.butter(4, cutoff)
        moving_avg_tmp = pd.DataFrame(sg.filtfilt(num, den, data[i], axis=0))
        moving_avg = pd.concat([moving_avg, moving_avg_tmp.dropna().reset_index(drop=True)],
                               axis=1, ignore_index=True)
    return moving_avg


def smooth_graph(data, config):
    """
    Chooses the mode of processing based on the config file
    """
    if config.keep_first:
        min_periods = [1] * JOINT_COUNT
    else:
        min_periods = config.window_size
    match config.type:
        case 'simple':
            result = mov_avg(data, config.window_size, min_periods)

        case 'const_weighted':
            result = w_moving_avg(data, config.window_size, min_periods)

        case 'weighted':
            result = w_moving_avg_2(data, config.window_size, min_periods)

        case 'filter':
            result = filter_butter(data, config.window_size)
            if config.keep_first:
                result.iloc[0] = data.iloc[0]

        case _:
            print('Unknown type, terminating')
            sys.exit()
    if config.keep_last:
        if max(abs(result.iloc[-1]-data.iloc[-1])) > 0.1:
            index = np.argmax(abs(result.iloc[-1]-data.iloc[-1]))
            print('Difference between end of original and smoothed is big: ')
            print(result.iloc[-1][index], data.iloc[-1][index])
        result.iloc[-1] = data.iloc[-1]
    if config.pad:
        result.dropna(how='all', inplace=True)
        for i in range(len(result.columns)):
            nan_count = result[i].isna().sum()
            if nan_count > 0:
                result[i].fillna(result[i].iloc[- (nan_count + 1)], inplace=True)
    else:
        result.dropna(inplace=True)
    return result


ARG_COUNT = len(sys.argv)
print('Files to smooth:', ARG_COUNT-1)

WS_DIR = str(Path(os.getcwd()).parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
CONFIG_PATH = os.path.join(str(Path(os.getcwd()).parent.absolute()),
                           'config', 'moving_average.yaml')

with open(CONFIG_PATH, 'r', encoding="utf-8") as file:
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


for j in range(1, ARG_COUNT):
    data_csv = pd.read_csv(CSV_DIR + f'motion{sys.argv[j]}.csv',
                           sep=',', decimal='.', header=None)
    if j == 1:
        smoothed = smooth_graph(data_csv, first_config)
    elif j is ARG_COUNT - 1:
        smoothed = smooth_graph(data_csv, last_config)
    else:
        smoothed = smooth_graph(data_csv, default_config)

    print(smoothed.iloc[[0, -1]])
    print(data_csv.iloc[[0, -1]])

    smoothed.to_csv(CSV_DIR + f'motion{sys.argv[j]}_tmp.csv',
                    sep=',', decimal='.', header=None,  index=False)
