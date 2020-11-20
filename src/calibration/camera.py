#!/usr/bin/env python

import socket
import numpy as np
import cv2
import os
import time
import struct


class Camera(object):

    def __init__(self):

        # Data options (change me)
        self.im_height = 720
        self.im_width = 1280
        self.tcp_host_ip = '192.168.1.63'
        self.tcp_port = 50000
        self.buffer_size = 4098 # 4 KiB

        # Connect to server
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))

        self.intrinsics = None
        self.get_data()


    def get_data(self):

        # Ping the server with anything
        self.tcp_socket.send(b'asdf')

        # Fetch TCP data:
        #     color camera intrinsics, 9 floats, number of bytes: 9 x 4
        #     depth scale for converting depth from uint16 to float, 1 float, number of bytes: 4
        #     depth image, self.im_width x self.im_height uint16, number of bytes: self.im_width x self.im_height x 2
        #     color image, self.im_width x self.im_height x 3 uint8, number of bytes: self.im_width x self.im_height x 3
        data = b''
        while len(data) < (10*4 + self.im_height*self.im_width*5):
            data += self.tcp_socket.recv(self.buffer_size)

        # Reorganize TCP data into color and depth frame
        self.intrinsics = np.fromstring(data[0:(9*4)], np.float32).reshape(3, 3)
        depth_scale = np.fromstring(data[(9*4):(10*4)], np.float32)[0]
        depth_img = np.fromstring(data[(10*4):((10*4)+self.im_width*self.im_height*2)], np.uint16).reshape(self.im_height, self.im_width)
        color_img = np.fromstring(data[((10*4)+self.im_width*self.im_height*2):], np.uint8).reshape(self.im_height, self.im_width, 3)
        depth_img = depth_img.astype(float) * depth_scale
        return color_img, depth_img

#import numpy as np 
#import pyrealsense2 as rs
#import json
#
#class Camera(object):
#    def __init__(self, calibration_file):
#
#        self.calibration_file = calibration_file
#        self.intrinsics = None
#
#        if calibration_file:
#            dev = self.__find_device_that_supports_advanced_mode()
#            advnc_mode = rs.rs400_advanced_mode(dev)
#            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")
#            # Loop until we successfully enable advanced mode
#            while not advnc_mode.is_enabled():
#                print("Trying to enable advanced mode...")
#                advnc_mode.toggle_advanced_mode(True)
#                # At this point the device will disconnect and re-connect.
#                print("Sleeping for 5 seconds...")
#                time.sleep(5)
#                # The 'dev' object will become invalid and we need to initialize it again
#                dev = self.__find_device_that_supports_advanced_mode()
#                advnc_mode = rs.rs400_advanced_mode(dev)
#                print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")
#
#            f = open(self.calibration_file)
#            as_json_object = json.load(f)
#            json_string = str(as_json_object).replace("'", '\"')
#            advnc_mode.load_json(json_string)
#
#        # Start streaming
#        self.pipeline = rs.pipeline()
#        config = rs.config()
#        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#        profile = self.pipeline.start(config)
#
#        for _ in range(30):
#            self.pipeline.wait_for_frames()
#
#        frames = self.pipeline.wait_for_frames()
#        depth_frame = frames.get_depth_frame()
#        color_frame = frames.get_color_frame()
#        self.intrinsics = None
#        if color_frame:
#            self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
#
#    def get_data(self):
#        frames = self.pipeline.wait_for_frames()
#        depth_frame = frames.get_depth_frame()
#        color_frame = frames.get_color_frame()
#
#        camera_depth_img = None
#        camera_color_img = None
#
#        if depth_frame and color_frame:
#            # Convert images to numpy arrays
#            camera_depth_img = np.asanyarray(depth_frame.get_data())
#            camera_color_img = np.asanyarray(color_frame.get_data())
#
#        return camera_color_img, camera_depth_img
#
#    def __find_device_that_supports_advanced_mode(self):
#        DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07","0B3A"]
#        ctx = rs.context()
#        ds5_dev = rs.device()
#        devices = ctx.query_devices();
#        for dev in devices:
#            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
#                if dev.supports(rs.camera_info.name):
#                    print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
#                return dev
#        raise Exception("No device that supports advanced mode was found")
