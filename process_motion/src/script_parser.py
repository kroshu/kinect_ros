#!/usr/bin/python3

import os
from pathlib import Path
import sys


# Parameters:
#   - [param_name]_files: int array of files on which to run specific script

moving_avg_files = [1]

ws_dir = str(Path(os.getcwd()).parent.parent.parent.parent.absolute())
csv_dir = os.path.join(ws_dir, 'replay', 'data', '')


file_count = 0
print(csv_dir + 'motion' + str(file_count+1)+'.csv')
while os.path.isfile(csv_dir+'motion' + str(file_count+1) + '.csv'):
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


src_dir = os.path.dirname(os.path.realpath(__file__))
moving_avg_params = ''
for i in range(0, len(moving_avg_files)):
    moving_avg_params += ' ' + str(moving_avg_files[i])


os.system(src_dir + '/moving_avg.py' + moving_avg_params)
