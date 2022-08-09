# Copyright 2022 Áron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.from launch import LaunchDescription

"""
Manages post-processing of movements according to the file numbers specified.

    [param_name]_files: int array of files on which to run specific script
"""

import os
from pathlib import Path
import sys

import rclpy
import rclpy.node

WS_DIR = str(Path(__file__).parent.parent.parent.parent.absolute())
CSV_DIR = os.path.join(WS_DIR, 'replay', 'data', '')
SRC_DIR = os.path.dirname(os.path.realpath(__file__))


class ScriptParser(rclpy.node.Node):
    """Class managing script parsing."""

    def __init__(self):
        super().__init__('script_parser_node')

        self.declare_parameter('smoothing_files', [0])
        self.declare_parameter('set_endpoint_files', [0])
        self.smoothing_files = []
        self.set_endpoint_files = []
        self.file_count = 0

    def init_node(self):
        """Initialize ROS2 node parameters."""
        while os.path.isfile(CSV_DIR + f'motion{self.file_count + 1}.csv'):
            self.file_count += 1
        print(f'Found {self.file_count} files to process')
        self.smoothing_files = (self.get_parameter('smoothing_files').get_parameter_value().
                                integer_array_value.tolist())
        self.set_endpoint_files = (self.get_parameter('set_endpoint_files').get_parameter_value().
                                   integer_array_value.tolist())

    def smooth_motion(self):
        """Call smooting script with arguments based on ROS2 parameter."""
        # Check validity of smoothing_files
        if len(self.smoothing_files) > 0:
            for i in self.smoothing_files:
                if not isinstance(i, int):
                    print('Index not integer, terminating')
                    sys.exit()
            self.smoothing_files.sort()
            if self.smoothing_files[0] < 0:
                print('Invalid file number, terminating')
                sys.exit()
            if self.smoothing_files[-1] > self.file_count:
                print(f'There is only {self.file_count} files to process, terminating')
                sys.exit()

            smoothing_params = ''
            for f in self.smoothing_files:
                if f > 0:
                    smoothing_params += ' ' + str(f)

            # run script
            os.system(SRC_DIR + '/smoothing.py' + smoothing_params)

    def set_endpoint(self):
        """Call endpoint setting script with arguments based on ROS2 parameter."""
        # Check validity of set_endpoint_files
        if len(self.set_endpoint_files) > 0:
            for i in self.set_endpoint_files:
                if not isinstance(i, int):
                    print('Index not integer, terminating')
                    sys.exit()
            self.set_endpoint_files.sort()
            if self.set_endpoint_files[0] < 0:
                print('Invalid file number, terminating')
                sys.exit()
            if self.set_endpoint_files[-1] > self.file_count:
                print(f'There is only {self.file_count} files to process, terminating')
                sys.exit()

            set_endpoint_params = ''
            for f in self.set_endpoint_files:
                if f > 0:
                    set_endpoint_params += ' ' + str(f)

            # run script
            os.system(SRC_DIR + '/set_endpoint.py' + set_endpoint_params)


def main():
    print('Starting ScriptParser Node...')
    rclpy.init()

    node = ScriptParser()
    node.init_node()
    node.smooth_motion()
    node.set_endpoint()

    # Context is needed only for parameter initialization
    # Giving 1 sec timeout to spin enables stopping process after it finishes
    rclpy.spin_once(node, timeout_sec=1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
