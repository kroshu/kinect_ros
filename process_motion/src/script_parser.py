#!/usr/bin/python3

"""
Manages post-processing of movements according to the file numbers specified.

    [param_name]_files: int array of files on which to run specific script
"""

import os
from pathlib import Path
import sys


moving_avg_files = [1]
set_endpoint_files = [1]

WS_DIR = str(Path(__file__).parent.parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
SRC_DIR = os.path.dirname(os.path.realpath(__file__))

file_count = 0
while os.path.isfile(CSV_DIR + f'motion{file_count + 1}.csv'):
    file_count += 1

print(f'Found {file_count} files to process')


# Check validity of moving_avg_files
if len(moving_avg_files) > 0:
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

    moving_avg_params = ''
    for f in moving_avg_files:
        moving_avg_params += ' ' + str(f)

    # run script
    os.system(SRC_DIR + '/moving_avg.py' + moving_avg_params)


# Check validity of set_endpoint_files
if len(set_endpoint_files) > 0:
    for i in set_endpoint_files:
        if not isinstance(i, int):
            print('Index not integer, terminating')
            sys.exit()
    set_endpoint_files.sort()
    if set_endpoint_files[0] < 1:
        print('Invalid file number, terminating')
        sys.exit()
    if set_endpoint_files[-1] > file_count:
        print(f'There is only {file_count} files to process, terminating')
        sys.exit()

    set_endpoint_params = ''
    for f in set_endpoint_files:
        set_endpoint_params += ' ' + str(f)

    # run script
    os.system(SRC_DIR + '/set_endpoint.py' + set_endpoint_params)
