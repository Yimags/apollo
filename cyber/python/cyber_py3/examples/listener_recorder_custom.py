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
import sys
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
from modules.routing.proto.routing_pb2 import RoutingResponse

Chassis_msg = "apollo.canbus.Chassis"
Localization_msg = "apollo.localization.LocalizationEstimate"
Obstacle_msg = "apollo.perception.PerceptionObstacles"
Transform_msg = "apollo.transform.TransformStampeds"
Control_msg = "apollo.control.ControlCommand"



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
    print(obstacle_info)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/record_to_text/test_record.txt','a+') as record_file:
                record_file.write(str(obstacle_info))
    print("=" * 80)

def pose_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:perception_callback msg->:")
    #print(data.header)
    pose_info = data.perception_obstacle
    msg_new = PerceptionObstacles()
    desc_str = msg_new.SerializeToString(obstacle_info)
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    print(obstacle_info)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/record_to_text/test_record.txt','a+') as record_file:
                record_file.write(str(obstacle_info))
    print("=" * 80)

def callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    print(data)
    print("=" * 80)


    def test_record_creater(writer_path):
        """
        Record writer.
        """
        fwriter = record.RecordWriter()
        fwriter.set_size_fileseg(0)
        fwriter.set_intervaltime_fileseg(0)

        if not fwriter.open(writer_path):
            print('Failed to open record writer!')
            return
        print('+++ Begin to writer +++')

        msg_new = Chassis()
        desc_str = msg_new.SerializeToString()
        fwriter.write_channel("/apollo/canbus/chassis",Chassis_msg , desc_str)

        test_record_file = "/apollo/self_test.record"

        print('Begin to write record file: {}'.format(test_record_file))
        test_record_creater(test_record_file)


# def on_image(image):
#   print("Received image:")
#   print("  format =", image.format)
#   print("  data =", len(image.data), "bytes")

#   arr = np.fromstring(image.data, np.uint8)
#   img = cv2.imdecode(arr, cv2.CV_LOAD_IMAGE_COLOR)
#   print("  size = {}x{}".format(img.shape[1], img.shape[0]))

def test_listener_class():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("listener")


    test_node.create_reader("/apollo/canbus/chassis", Chassis, callback)
    #test_node.create_reader("/apollo/control", ControlCommand, callback) ## it works
    #test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, callback)  ## it works
    #test_node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, perception_callback)  ## it works
    #test_node.create_reader("/apollo/routing_request", RoutingRequest, callback)
    #test_node.create_reader("/apollo/routing_response", RoutingRequest, callback)
    #test_node.create_reader("/tf", TransformStampeds, callback) ## it works
    #test_node.create_reader("/apollo/prediction", PredictionObstacles, callback)
    #print(test_node)
    #test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, on_image)

    # node = cyber.Node("MyNodeName")
    # test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, on_image)
    #test_node.create_reader("channel/chatter", ChatterBenchmark, callback)
    
    test_node.spin()


if __name__ == '__main__':
    cyber.init()
    test_listener_class()
    cyber.shutdown()
