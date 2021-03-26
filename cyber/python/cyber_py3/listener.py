#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for example of listener."""
import sys,os,time
sys.path.append('/home/inem-laptop/apollo/')
from cyber.python.cyber_py3 import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark
from modules.common.util.testdata.simple_pb2 import SimpleMessage
##
#from __future__ import print_function
from modules.drivers.proto.sensor_image_pb2 import CompressedImage

import numpy as np
import cv2
## CyberRT messages
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
from modules.routing.proto.routing_pb2 import RoutingRequest
from modules.routing.proto.routing_pb2 import LaneWaypoint
from modules.routing.proto.routing_pb2 import RoutingResponse

###############
from modules.drivers.proto.conti_radar_pb2 import ContiRadar
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose
from modules.localization.proto.gps_pb2 import Gps
from modules.drivers.gnss.proto.imu_pb2 import Imu

#####################
os.makedirs('/apollo/test', exist_ok=True)
######## Synchronization of initial timestamps 
img_start_time = time.time()
control_time = time.time()
cyber_start_time = time.time()
print('cyber_start_time is :',cyber_start_time)

def perception_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:perception_callback msg->:")
    #print(data.header)
    obstacle_info = data.perception_obstacle
    msg_new = PerceptionObstacles()
    desc_str = msg_new.SerializeToString(obstacle_info)
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    print(desc_str)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/test_record_file.txt','a+') as record_file:
                record_file.write(str(obstacle_info))
    print("=" * 80)

def control_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    global cyber_start_time
    global control_time
    global img_start_time
    
    control_time = time.time()
    
    print('current timestamp is:',control_time)
    print("=" * 80)
    
    if (control_time-img_start_time)>10:
        img_start_time = control_time
        print('         ##################################')
        print('         #              control           #')
        print('         # cyber is waiting for shut down #')
        print('         #          press crtl + c        #')
        print('         ##################################')
        
        cyber.waitforshutdown() 
        #while not cyber.is_shutdown():
         
            
        if cyber.is_shutdown():
            
            #cyber.shutdown() 
            print('cyber is shut down for control')

        

def waypoint_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    new_msg = LaneWaypoint()
    print(data.waypoint)
    # desc_str = new_msg.SerializeToString(data)
    # print(desc_str)
    # with open('/apollo/test_record_file.txt','a+') as record_file:
    #             record_file.write(str(data))
    print("=" * 80)

def image_callback(image):
    """
    Reader img callback.
    """
    print("=" * 80)

    global cyber_start_time
    global control_time
    global img_start_time

    img_start_time = time.time()
    timestamp = str(round(img_start_time - cyber_start_time,1))
    arr = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    ###################### Module imconpatible
    #cv2.waitKey(500)
    ######################
    save = cv2.imwrite('/apollo/test/'+timestamp+'.jpg', img)

    print("Received image:")
    print("  data =", len(image.data), "bytes")
    print('current timestamp is:',img_start_time)
    print("=" * 80)

    if (control_time-img_start_time)>10:
        # img_start_time = 0
        # control_time  = 0
        # cyber_start_time = 0
        print('         ##################################')
        print('         #            image               #')
        print('         # cyber is waiting for shut down #')
        print('         #          press crtl + c        #')
        print('         ##################################')
        cyber.waitforshutdown() 
        #cyber.shutdown()       
        if cyber.is_shutdown():
            print('cyber is shut down for camera')


# def on_image(image):
#   print("Received image:")
#   print("  format =", image.format)
#   print("  data =", len(image.data), "bytes")

#   arr = np.fromstring(image.data, np.uint8)
#   img = cv2.imdecode(arr, cv2.CV_LOAD_IMAGE_COLOR)
#   print("  size = {}x{}".format(img.shape[1], img.shape[0]))

def radar_callback(data):
    """
    Reader message callback.
    """

    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/radar/'+timestamp+'.txt','a+') as record_file:
        record_file.write(str(data))


def gnss_callback(data):
    """
    Reader message callback.
    """

    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/gps/'+timestamp+'.txt','a+') as record_file:
        record_file.write(str(data))


def odometry_callback(data):
    """
    Reader message callback.
    """

    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/odometry/'+timestamp+'.txt','a+') as record_file:
        record_file.write(str(data))

def imu_callback(data):
    """
    Reader message callback.
    """

    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/imu/'+timestamp+'.txt','a+') as record_file:
        record_file.write(str(data))

def test_listener_class():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("listener0")
    while not cyber.is_shutdown():
        #test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, image_callback)
        #test_node.create_reader("/apollo/control", ControlCommand, control_callback) ## it works

        #test_node.create_reader("/apollo/canbus/chassis", Chassis, callback)
        test_node.create_reader("/apollo/sensor/conti_radar", ContiRadar, radar_callback)
        test_node.create_reader("/apollo/sensor/gnss/best_pose", GnssBestPose, gnss_callback)
        test_node.create_reader("/apollo/sensor/gnss/odometry", Gps, odometry_callback)
        test_node.create_reader("/apollo/sensor/gnss/imu", Imu, imu_callback)
        #test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, callback)  ## it works
        #test_node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, perception_callback)  ## it works
        #test_node.create_reader("/apollo/routing_request", RoutingRequest, waypoint_callback)
        #test_node.create_reader("/apollo/routing_response", RoutingResponse, callback)
        #test_node.create_reader("/tf", TransformStampeds, callback) ## it works
        #test_node.create_reader("/apollo/prediction", PredictionObstacles, callback)
        #print(test_node)
        # node = cyber.Node("MyNodeName")
        # test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, on_image)
        #test_node.create_reader("channel/chatter", ChatterBenchmark, callback)

        #while not cyber.is_shutdown():
        test_node.spin()
        #test_node.__del__()



if __name__ == '__main__':
    cyber.init()
    test_listener_class()
    print('         \t')
    print('       #########################################')
    print('       #             press crtl + c            #')
    print('       #  Multi-simulation has been finished!  #')
    print('       #           Pls check the data          #')
    print('       #########################################')
    
    #sys.modules[__name__].__dict__.clear()
