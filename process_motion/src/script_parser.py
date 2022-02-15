#!/usr/bin/python3

"""
Manages post-processing of movements according to the file numbers specified.

    [param_name]_files: int array of files on which to run specific script
"""

import os
from pathlib import Path
import sys


moving_avg_files = [1]

WS_DIR = str(Path(os.getcwd()).parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')


file_count = 0
print(CSV_DIR + 'motion' + str(file_count+1)+'.csv')
while os.path.isfile(CSV_DIR+'motion' + str(file_count+1) + '.csv'):
    file_count += 1

print(file_count)


for i in moving_avg_files:
    if not isinstance(i, int):
        print('Index not integer, terminating')
        sys.exit()
moving_avg_files.sort()
if moving_avg_files[0] < 1:
    print('Invalid file number, terminating')
    sys.exit()
if moving_avg_files[-1] > file_count:
    print(f'There is only {file_count} files to process, terminating')
    sys.exit()


SRC_DIR = os.path.dirname(os.path.realpath(__file__))
moving_avg_params = ''
for f in moving_avg_files:
    moving_avg_params += ' ' + str(f)


os.system(SRC_DIR + '/moving_avg.py' + moving_avg_params)
