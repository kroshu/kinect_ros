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
                print(f'[ERROR] Invalid key found in yaml: {key}')
        if len(self.window_size) != JOINT_COUNT:
            print('[ERROR] Invalid length for window_size, terminating')
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
    elif (abs(w_data.iloc[-1] - w_data.mean()) > 0.02 * sqrt(len(w_data))):
        weights = [1 / (len(w_data) + 3)] * (len(w_data) - 3)
        weights.extend([1.5 / (len(w_data) + 3), 1.6 / (len(w_data) + 3), 2.9 / (len(w_data) + 3)])
    else:
        weights = np.full(1, 1/len(w_data))
    return np.sum(weights * w_data)


def calc_weights_const(w_data):
    """
    Calculate weights for CWMA
    Applies constant weights
    """
    weights = [1]
    if len(w_data) > 3:
        weights = [1 / (len(w_data) + 2)] * (len(w_data) - 3)
        weights.extend([1.2 / (len(w_data) + 2), 1.3 / (len(w_data) + 2), 2.5 / (len(w_data) + 2)])
    elif len(w_data) > 1:
        weights = [1 / (len(w_data) + 1)] * (len(w_data) - 1)
        weights.extend([2 / (len(w_data) + 1)])
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
            weights.extend([1.2 / (window[i] + 2), 1.3 / (window[i] + 2), 2.5 / (window[i] + 2)])
        elif window[i] > 1:
            weights = [1 / (window[i] + 1)] * (window[i] - 1)
            weights.extend([2 / (window[i] + 1)])
        moving_avg_tmp = data[i].rolling(window[i],
                                         min_periods=periods[i]).apply(calc_weights_const)

        moving_avg = pd.concat([moving_avg, moving_avg_tmp.dropna().reset_index(drop=True)],
                               axis=1)
    return moving_avg


def check_monotony(w_data, mon_count):
    """
    Checks whether the end of a window is monoton
    """
    if mon_count < 3:
        print('[WARNING] Monotony window too small')
        return False
    if mon_count > len(w_data):
        print('[WARNING] Monotony window bigger than observed')
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


def merge_data(forw_result, reversed_result):
    """
    Merges two data series that have been filtered from opposite directions
    """
    result = pd.DataFrame()
    half = (int)(forw_result.shape[0] / 2)
    for i in range(len(forw_result.columns)):
        if forw_result.shape[0] % 2 == 0:
            result[i] = pd.concat([forw_result[i].iloc[:half], reversed_result[i].iloc[-half:]],
                                  ignore_index=True)
        else:
            result[i] = pd.concat([forw_result[i].iloc[:half + 1],
                                  reversed_result[i].iloc[-half:]], ignore_index=True)
        if forw_result.shape[0] > 20:
            for j in range(10):
                index = half - 5 + j
                result[i].iloc[index] = (forw_result[i].iloc[index] * (1 - (j + 1) / 10) +
                                         reversed_result[i].iloc[index] * ((j + 1) / 10))
    return result


def smooth_graph(data, config):
    """
    Chooses the mode of processing based on the config file
    """
    keep_both = False
    if config.keep_first:
        min_periods = [1] * JOINT_COUNT
        if config.keep_last:
            data_reversed = data[::-1].reset_index(drop=True)
            keep_both = True
    elif config.keep_last:
        min_periods = [1] * JOINT_COUNT
        data = data[::-1].reset_index(drop=True)
    else:
        min_periods = config.window_size

    match config.type:
        case 'simple':
            result = mov_avg(data, config.window_size, min_periods)
            if keep_both:
                reversed_result = mov_avg(data_reversed, config.window_size, min_periods)

        case 'const_weighted':
            result = cw_moving_avg(data, config.window_size, min_periods)
            if keep_both:
                reversed_result = cw_moving_avg(data_reversed, config.window_size, min_periods)

        case 'weighted':
            result = w_moving_avg_2(data, config.window_size, min_periods)
            if keep_both:
                reversed_result = w_moving_avg_2(data_reversed, config.window_size, min_periods)

        case 'filter':
            result = filter_butter(data, config.window_size)
            if config.keep_first:
                result.iloc[0] = data.iloc[0]
                if config.keep_last:
                    result.iloc[-1] = data.iloc[-1]
            elif config.keep_last:
                result.iloc[0] = data.iloc[0]  # data and result is in reversed order

        case _:
            print('[ERROR] Unknown type, terminating')
            sys.exit()

    if config.keep_last and not config.keep_first:
        result = result[::-1].reset_index(drop=True)
    elif config.keep_last and config.type != 'filter':
        reversed_result = reversed_result[::-1].reset_index(drop=True)
        result = merge_data(result, reversed_result)

    if config.pad:
        result.dropna(how='all', inplace=True)
        for i in range(len(result.columns)):
            nan_count = result[i].isna().sum()
            padding = pd.Series([result[i].iloc[0]] * nan_count, dtype='float64')
            result[i] = pd.concat([padding, result[i]], ignore_index=True)
    else:
        result.dropna(inplace=True)

    return result


ARG_COUNT = len(sys.argv)
print('Files to smooth:', ARG_COUNT-1)

WS_DIR = str(Path(__file__).parent.parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
CONFIG_PATH = os.path.join(str(Path(__file__).parent.parent.absolute()),
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

    # this overrides original data series!!
    smoothed.to_csv(CSV_DIR + f'motion{sys.argv[j]}.csv',
                    sep=',', decimal='.', header=None,  index=False)
