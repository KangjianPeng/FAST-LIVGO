#!/usr/bin/env python3

import os
import subprocess

class RosNode:
    def __init__(self, package, executable):
        self._package = package
        self._executable = executable
        self._param_string = ''
    
    def add_parameters(self, namespace, parameter_dictionary):
        for key in parameter_dictionary.keys():
            if isinstance(parameter_dictionary[key], dict):
                # 递归处理嵌套字典
                self.add_parameters(namespace + key + '.', parameter_dictionary[key])
            else:
                # 添加参数到参数字符串
                self._param_string += f' --ros-args -p {namespace}{key}:={parameter_dictionary[key]}'
        
    def run(self, parameter_dictionary, namespace=''):
        # 添加参数
        self.add_parameters(namespace, parameter_dictionary)
        print('Starting ROS 2 node with parameters: ' + self._param_string)
        
        # 构建命令
        command = ['ros2', 'run', self._package, self._executable] + self._param_string.split()
        
        try:
            # 使用 subprocess 运行命令
            subprocess.run(command, check=True)
            print('ROS 2 node finished processing.')
        except subprocess.CalledProcessError as e:
            print(f'Error: ROS 2 node failed with error code {e.returncode}')
        except FileNotFoundError:
            print('Error: ros2 command not found. Make sure ROS 2 is sourced.')