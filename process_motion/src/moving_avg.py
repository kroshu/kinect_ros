#!/usr/bin/python3.10

import os
from statistics import mean
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



window_data = []


def MA(data, window, periods):
    moving_avg = pd.DataFrame([])    
    for i in range(len(data.columns)):
        moving_avg[i] = data[i].rolling(window[i], min_periods=periods[i]).mean()
    return moving_avg


def calc_weights(x):
    if len(x) < 4:
        # weights are assigned evenly, like SMA, because window is too small
        weights = np.full(1, 1/len(x))
    elif (x.iloc[-1] - x.iloc[-2]) * (x.iloc[-2] - x.iloc[-3]) > 0 and (x.iloc[-3] - x.iloc[-4]) * (x.iloc[-2] - x.iloc[-3]) > 0:
        weights = [1 / (len(x) + 2)] * (len(x) - 3)
        weights.extend([1.5 / (len(x) + 2), 1.6 / (len(x) + 2), 1.9 / (len(x) + 2)])
    else:
        weights = np.full(1, 1/len(x))
    return np.sum(weights*x)


def WMA(data, window, periods):
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        moving_avg[i] = data[i].rolling(window[i],
                                        min_periods=periods[i]).apply(lambda x: calc_weights(x))
    return moving_avg

def check_monotony(x, mon_count):
    if (mon_count < 3):
        print("Monotony window too small")
        return False
    if (mon_count > len(x)):
        print("Monotony window bigger than observed")
        return False
    i = 0
    while i < mon_count -2:
        if ((x.iloc[-(i + 1)] - x.iloc[-(i + 2)]) * (x.iloc[-(i + 2)] - x.iloc[-(i + 3)]) < 0):
            return False
        i += 1
    return True


def WMA2(data, window, periods, mon_count=4):
    moving_avg = pd.DataFrame([])
    global window_data    
    for j in range(len(data.columns)):
        moving_avg_tmp = []
        window_data = []
        i = 0
        if window[j] < mon_count:
            simple_MA = data[j].rolling(window[j], min_periods=periods[j]).mean()
            simple_MA.dropna(inplace=True)
            moving_avg[j] = simple_MA 
            continue
        weighted_sum = 0                
        while i < len(data):
            prev_weighted_sum = weighted_sum
            if len(window_data) != 0:
                old_data=window_data.iloc[0]
            else:
                old_data = 0
            if i + 1 < window[j]:
                if periods[j] == window[j]:
                    i +=1
                    continue                
                window_data = data[j][0:i + 1]
                # in this case no data is outdated, but a multiplication with (w - 1) / w is needed
                #  -> sum / w is substracted
                old_data = weighted_sum / len(window_data)
            else:
                window_data = data[j][i - window[j] + 1:i + 1]
            w = len(window_data)
            if i == 0:
                weighted_sum = mean(window_data)
            elif (i + 1) == window[j] and window[j] == periods[j]:
                weighted_sum = mean(window_data)
            elif check_monotony(window_data, mon_count):
                weighted_sum = (weighted_sum - old_data / w) * (w -2) / (w - 1)  + 2 * window_data.iloc[-1] / w
                #weighted_sum = weighted_sum / w * (w - 3) + 3 * window_data.iloc[-1] / w
            else:
                weighted_sum = weighted_sum - old_data / w  + window_data.iloc[-1] / w
                #weighted_sum = weighted_sum / w * (w - 1) + window_data.iloc[-1] / w
            if weighted_sum > max(window_data):
                weighted_sum = np.mean([max(window_data), prev_weighted_sum])
            elif weighted_sum < min(window_data):
                weighted_sum = np.mean([min(window_data), prev_weighted_sum])
            
            moving_avg_tmp = np.append(moving_avg_tmp,[weighted_sum])
            i += 1
        print(moving_avg_tmp)
        moving_avg[j]=pd.DataFrame(moving_avg_tmp, columns=['aa']) # TODO: first value is lost!!
        print(moving_avg[j])
    return moving_avg



def symmetric(data, window, periods):
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        moving_avg[i] = data[i].rolling(window[i], center=True, min_periods=periods[i]).mean()
    return moving_avg


def filter(data, window):
    moving_avg = pd.DataFrame([])
    for i in range(len(data.columns)):
        cutoff = 1 / window[i]
        if cutoff == 1:
            cutoff = 0.999
        # by default sampling freq = Nyquist freq
        # cutoff specifies the ratio of cutoff and Nyquist (=sampling) frequencies
        b, a = sg.butter(4, cutoff)
        moving_avg[i] = pd.DataFrame(sg.filtfilt(b, a, data[i], axis=0))
        # TODO: try different padding and gust method
    return moving_avg


def smooth_graph(data, config):
    match config.type:
        case 'simple':
            if config.keep_first:
                result = MA(data, config.window_size, [1] * joint_count)
            else:
                result = MA(data, config.window_size, config.window_size)
            if config.keep_last:
                result.iloc[-1] = data.iloc[-1]
            return result

        case 'filter':
            return filter(data, config.window_size)
            # TODO: keep_first, keep_last
        case 'symmetric':
            result = symmetric(data, config.window_size, config.window_size)
            if config.keep_last:
                result.iloc[-1] = data.iloc[-1]
            if config.keep_first:
                result.iloc[0] = data.iloc[0]
            return result
        case 'weighted':
            if config.keep_first:
                result = WMA2(data, config.window_size, [1] * joint_count)
            else:
                result = WMA2(data, config.window_size, config.window_size)
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
    data_csv = pd.read_csv(csv_dir+'motion'+str(sys.argv[i]) + '.csv',
                       sep=',', decimal='.', header=None)
    if i == 1:
        smoothed = smooth_graph(data_csv, first_config)
    elif i is n - 1:
        smoothed = smooth_graph(data_csv, last_config)
    else:
        smoothed = smooth_graph(data_csv, default_config)

    # this removes all lines with NaN values
    # the different lengthes due to different window sizes are resolved
    smoothed.dropna(inplace=True)
    print(smoothed.iloc[[0, -1]])
    print(data_csv.iloc[[0, -1]])

    smoothed.to_csv(csv_dir+'motion'+str(sys.argv[i])+'_tmp.csv',
                    sep=',', decimal='.', header=None,  index=False)
