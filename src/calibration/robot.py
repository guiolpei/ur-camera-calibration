import socket
import select
import struct
import time
import os
import numpy as np
import utils
from camera import Camera

class Robot(object):
    def __init__(self, tcp_host_ip, tcp_port, rtc_host_ip, rtc_port, camera_calibration_file = ''):
         
        # Connect to robot client
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

        # Connect as real-time client to parse state data
        self.rtc_host_ip = rtc_host_ip
        self.rtc_port = rtc_port

        # Default home joint configuration
        self.home_joint_config = [
            utils.degrees_to_radians(90),   #BASE
            utils.degrees_to_radians(-135), #HOMBRO
            utils.degrees_to_radians(145),  #CODO
            utils.degrees_to_radians(-100), #MUÑ1
            utils.degrees_to_radians(270),  #MUÑ2
            utils.degrees_to_radians(0),    #MUÑ3
        ]

        # Default photo joint configuration
        self.photo_joint_config = [
            utils.degrees_to_radians(115.60), #BASE
            utils.degrees_to_radians(-57.30), #HOMBRO
            utils.degrees_to_radians(8.65),   #CODO
            utils.degrees_to_radians(-40.91), #MUÑ1
            utils.degrees_to_radians(273.94), #MUÑ2
            utils.degrees_to_radians(-65.93), #MUÑ3
        ] 

        # Default joint speed configuration
        self.JOINT_ACC_DEFAULT = 8
        self.JOINT_VEL_DEFAULT = 3
        self.JOINT_ACC_SAFE = 1.4
        self.JOINT_VEL_SAFE = 1.05
        self.joint_acc = self.JOINT_ACC_DEFAULT  # default: 8, safe: 1.4 m/s^2
        self.joint_vel = self.JOINT_VEL_DEFAULT  # default: 3, safe: 1.05 m/s

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.1

        # Default tool speed configuration
        self.TOOL_ACC_DEFAULT = 1.2
        self.TOOL_VEL_DEFAULT = 0.25
        self.TOOL_ACC_SAFE = 0.5
        self.TOOL_VEL_SAFE = 0.2
        self.tool_acc = self.TOOL_ACC_DEFAULT # default: 1.2, safe: 0.5
        self.tool_vel = self.TOOL_VEL_DEFAULT # default: 0.25, safe: 0.2


        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = [0.002,0.002,0.002,0.01,0.01,0.01]

        # Fetch RGB-D data from RealSense camera
        self.camera = Camera()
        
        # Load camera pose (from running calibrate.py), intrinsics and depth scale
        self.cam_pose = np.loadtxt('camera_pose.txt', delimiter=' ')
        self.cam_depth_scale = np.loadtxt('camera_depth_scale.txt', delimiter=' ')


    def get_state(self):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        state_data = self.tcp_socket.recv(2048)
        state_data = self.__assure_is_robot_state(state_data) #First package contains version info (MSG_TYPE 20)
        self.tcp_socket.close()
        return state_data

    def __assure_is_robot_state(self, state_data):
         # Read package header
        data_bytes = bytearray()
        data_bytes.extend(state_data)
        data_length = struct.unpack("!i", data_bytes[0:4])[0];
        robot_message_type = data_bytes[4]
        if robot_message_type == 16: #Version Message = 20 (only sent once), Robot State Message = 16
            return state_data
        else:
            state_data = self.tcp_socket.recv(2048)
            return self.__assure_is_robot_state(state_data)

    def parse_rtc_state_data(self, state_data):

        # Read package header
        data_bytes = bytearray()
        data_bytes.extend(state_data)
        data_length = struct.unpack("!i", data_bytes[0:4])[0];
        assert(data_length == 812)
        byte_idx = 4 + 8 + 8*48 + 24 + 120
        TCP_forces = [0,0,0,0,0,0]
        for joint_idx in range(6):
            TCP_forces[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
            byte_idx += 8

        return TCP_forces


    def parse_tcp_state_data(self, state_data, subpackage):

        # Read package header
        data_bytes = bytearray()
        data_bytes.extend(state_data)
        data_length = struct.unpack("!i", data_bytes[0:4])[0];
        robot_message_type = data_bytes[4]
        assert(robot_message_type == 16) #Version Message = 20 (only sent once), Robot State Message = 16
        byte_idx = 5

        # Parse sub-packages
        subpackage_types = {'joint_data' : 1, 'cartesian_info' : 4, 'force_mode_data' : 7, 'tool_data' : 2}
        while byte_idx < data_length:
            # package_length = int.from_bytes(data_bytes[byte_idx:(byte_idx+4)], byteorder='big', signed=False)
            package_length = struct.unpack("!i", data_bytes[byte_idx:(byte_idx+4)])[0]
            byte_idx += 4
            package_idx = data_bytes[byte_idx]
            if package_idx == subpackage_types[subpackage]:
                byte_idx += 1
                break
            byte_idx += package_length - 4

        def parse_joint_data(data_bytes, byte_idx):
            actual_joint_positions = [0,0,0,0,0,0]
            target_joint_positions = [0,0,0,0,0,0]
            for joint_idx in range(6):
                actual_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
                target_joint_positions[joint_idx] = struct.unpack('!d', data_bytes[(byte_idx+8):(byte_idx+16)])[0]
                byte_idx += 41
            return actual_joint_positions

        def parse_cartesian_info(data_bytes, byte_idx):
            actual_tool_pose = [0,0,0,0,0,0]
            for pose_value_idx in range(6):
                actual_tool_pose[pose_value_idx] = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
                byte_idx += 8
            return actual_tool_pose

        def parse_tool_data(data_bytes, byte_idx):
            byte_idx += 2
            tool_analog_input2 = struct.unpack('!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
            return tool_analog_input2

        parse_functions = {'joint_data' : parse_joint_data, 'cartesian_info' : parse_cartesian_info, 'tool_data' : parse_tool_data}
        return parse_functions[subpackage](data_bytes, byte_idx)


    def go_home(self):

        self.move_joints(self.home_joint_config)


    def go_photo(self):

        self.move_joints(self.photo_joint_config)

    def set_digital_out_signal(self, number, value):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = f"set_digital_out({number}, {value})\n"
        self.tcp_socket.send(str.encode(tcp_command))

    def move_joints(self, joint_configuration):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "movej([%f" % joint_configuration[0]
        for joint_idx in range(1,6):
            tcp_command = tcp_command + (",%f" % joint_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f)\n" % (self.joint_acc, self.joint_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(2048)
        state_data = self.__assure_is_robot_state(state_data)
        actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
        while not all([np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            state_data = self.tcp_socket.recv(2048)
            state_data = self.__assure_is_robot_state(state_data)
            actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
            time.sleep(0.01)

        self.tcp_socket.close()


    def move_to(self, tool_position, tool_orientation):

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (tool_position[0],tool_position[1],tool_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2],self.tool_acc,self.tool_vel)
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches target tool position
        state_data = self.tcp_socket.recv(2048)
        state_data = self.__assure_is_robot_state(state_data)
        actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_pose[j] - tool_position[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = self.tcp_socket.recv(2048)
            state_data = self.__assure_is_robot_state(state_data)
            prev_actual_tool_pose = np.asarray(actual_tool_pose).copy()
            actual_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        self.tcp_socket.close()

    def get_current_pose(self):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        
        # Ping the server with anything
        self.tcp_socket.send(b'asdf')

        state_data = self.tcp_socket.recv(2048)
        state_data = self.__assure_is_robot_state(state_data)
        current_tool_pose = self.parse_tcp_state_data(state_data, 'cartesian_info')
        self.tcp_socket.close()
        return current_tool_pose