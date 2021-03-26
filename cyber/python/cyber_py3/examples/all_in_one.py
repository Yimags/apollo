#!/usr/bin/env python3

"""
Insert routing request
Collect data from bridge and save in .txt

Usage:
    all_in_one.py
"""
import argparse
import os
import sys
import time
import numpy as np
import random
### import for auto routing
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.routing.proto import routing_pb2

### impoet for listener
from cyber.python.cyber_py3 import cyber
from modules.drivers.proto.sensor_image_pb2 import CompressedImage

import cv2
# CyberRT messages
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
from modules.routing.proto.routing_pb2 import RoutingRequest
from modules.routing.proto.routing_pb2 import LaneWaypoint
from modules.routing.proto.routing_pb2 import RoutingResponse

SIM_EPOCH = sys.argv[1]
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/image/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/lidar/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/text/', exist_ok=True)

tmp_time = 0

def set_destination():
    """
    Main rosnode
    """
    cyber.init()
    node = cyber.Node("mock_routing_requester")
    sequence_num = 0

    routing_request = routing_pb2.RoutingRequest()

    routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    routing_request.header.module_name = 'routing_request'
    routing_request.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1

    waypoint = routing_request.waypoint.add()
    waypoint.pose.x = 587061.3193778992
    waypoint.pose.y = 4141628.6499671936
    waypoint.id = 'lane_16'
    waypoint.s = 1.7566458424736489

    # waypoint = routing_request.waypoint.add()
    # waypoint.pose.x = 587035.4540141824
    # waypoint.pose.y = 4141522.369452626
    # waypoint.id = 'lane_30'
    # waypoint.s = 31.396361050985014

    destination_point = [0,1,2,3]
    case = random.choice(destination_point)
    if case == 1:
        ## Ending 1
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 587041.8436057615

        waypoint.pose.y = 4141219.619665432

        waypoint.id = 'lane_24'
        waypoint.s = 61.48374661751174

        
    if case == 2:
        # ## Ending 2
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 586952.9497209189
        waypoint.pose.y = 4141211.7531940676
        waypoint.id = 'lane_21'
        waypoint.s = 12.476310898516962

    ## Ending 3
    ## not working
    if case == 3:
    
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 587041.825080872
        waypoint.pose.y = 4141557.835714340
        waypoint.id = 'lane_32'
        waypoint.s = 34.940624074

    if case == 0:
    
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 587054.102204323
        waypoint.pose.y = 4141602.562294006
        waypoint.id = 'lane_40'
        waypoint.s = 3.555938797





    writer = node.create_writer('/apollo/routing_request',
                                routing_pb2.RoutingRequest)
    time.sleep(2.0)
    print("routing_request", routing_request)
    writer.write(routing_request)



def perception_callback(data):
    """
    Reader obstacle callback.
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
    Reader pose callback.
    """
    print("=" * 80)
    print("py:perception_callback msg->:")
    global tmp_time 
    tmp_time = int(tmp_time)
    if tmp_time== int(data.header.timestamp_sec):
        cyber.shutdown()
        #pass
    #print(data.header)
    pose_info = data
    # msg_new = PerceptionObstacles()
    # desc_str = msg_new.SerializeToString(pose_info)
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    print(pose_info)  ## It ends with the smallest message type defined in protobuf.
    # with open('/apollo/record_to_text/test_record.txt','a+') as record_file:
    #             record_file.write(str(pose_info))
    
    print("=" * 80)
    #time.sleep(1.5)
    

def callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    print(data)
    print("=" * 80)
    time.sleep(1.5)


def image_callback(image):
    """
    Reader img callback.
    """
    print("=" * 80)
    print("Received image:")
    print("  format =", image.format)
    print("  data =", len(image.data), "bytes")
    print('time = ',image.header.timestamp_sec)
    
    global tmp_time 
    arr = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    ###################### Module imconpatible
    #cv2.waitKey(500)
    ######################
    save = cv2.imwrite('/apollo/record_to_text/'+SIM_EPOCH+'/image/img_'+str(round(image.header.timestamp_sec,2))+'.jpg', img)
    if save:
        print("img saved successfully,  size = {}x{}".format(img.shape[1], img.shape[0]))
    # if not save:
    #     None
    #     #cyber.shutdown()
    print("=" * 80)
    
    #time.sleep(1.5)
    

def lidar_callback(data):
    """
    Reader lidar callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    print(data)
    print("=" * 80)


def sim_listener_class():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("listener")
    test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, image_callback)
    #test_node.create_reader("/apollo/sensor/lidar128/compensator/PointCloud2", PointCloud, lidar_callback)
    


    #test_node.create_reader("/apollo/canbus/chassis", Chassis, callback)
    #test_node.create_reader("/apollo/control", ControlCommand, callback) ## it works
    #test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, pose_callback)  ## it works
    #test_node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, callback)  ## it works
    #test_node.create_reader("/apollo/routing_request", RoutingRequest, callback)
    #test_node.create_reader("/apollo/routing_response", RoutingRequest, callback)
    #test_node.create_reader("/tf", TransformStampeds, callback) ## it works
    #test_node.create_reader("/apollo/prediction", PredictionObstacles, callback)
    #print(test_node)

    
    
    #test_node.create_reader("/apollo/sensor/camera/front_6mm/image", Image, image_callback)
    #test_node.create_reader("/apollo/sensor/lidar128/compensator/PointCloud2", CompressedImage, lidar_callback)

    
    # test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, on_image)
    #test_node.create_reader("channel/chatter", ChatterBenchmark, callback)
    
    test_node.spin()






if __name__ == '__main__':
    
    cyber.init()
    set_destination()
    sim_listener_class()
    cyber.shutdown()

