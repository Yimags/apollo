#!/usr/bin/env python3

"""
Insert routing request
Collect data from bridge and save in .txt

Usage:
    realtime_data_storage.py
"""
import argparse
import os
import sys
import time
import math
import numpy as np
import random
### import for auto routing
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.routing.proto import routing_pb2

### import for listener
from cyber.python.cyber_py3 import cyber
from modules.drivers.proto.sensor_image_pb2 import CompressedImage
from modules.drivers.proto.pointcloud_pb2 import PointCloud
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

SIM_EPOCH = sys.argv[1] # relative Path to save
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/image/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/image_seg/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/lidar/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/text/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/text/prediction/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/text/obstacles/', exist_ok=True)

img_start_time = 0
#print('start_time is :',start_time)
cyber_start_time = time.time()
print('cyber_start_time is :',cyber_start_time)

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

    ########### Randomly choose a destination
    destination_point = [1,2,3,5]
    case = random.choice(destination_point)

    ## Ending 1
    ## straight run, uneven road, letf_turn  
    if case == 1:
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 587027.25919068151
        waypoint.pose.y = 4141223.0597835081
        waypoint.id = 'lane_24'
        waypoint.s = 46.504607512178261

    ## Ending 2
    ## straight run, uneven road       
    if case == 2:
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 586951.11713548924
        waypoint.pose.y = 4141204.2440729048
        waypoint.id = 'lane_21'
        waypoint.s = 20.203213911260271

    ## Ending 3
    ## left_turn, intersection
    if case == 3:
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 587094.68502816546
        waypoint.pose.y = 4141565.2227073964
        waypoint.id = 'lane_6'
        waypoint.s = 33.06953341711916

    ## Ending 4
    ## 
    # if case == 4:
    #     waypoint = routing_request.waypoint.add()
    #     waypoint.pose.x = 587054.102204323
    #     waypoint.pose.y = 4141602.562294006
    #     waypoint.id = 'lane_40'
    #     waypoint.s = 3.555938797

    ## Ending 3
    ## left_turn, intersection, single lane-changing
    if case == 5:
        waypoint = routing_request.waypoint.add()
        waypoint.pose.x = 587092.84538193734
        waypoint.pose.y = 4141562.598682947
        waypoint.id = 'lane_5'
        waypoint.s = 32.026536175747864

    writer = node.create_writer('/apollo/routing_request',
                                routing_pb2.RoutingRequest)
    time.sleep(2.0)
    print("routing_request", routing_request)
    writer.write(routing_request)



def obstacle_callback(data):
    """
    Reader obstacle callback.
    """
    #print("=" * 80)
    #print("py:perception_callback msg->:")
    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1)) #str(round(data.header.timestamp_sec,2))
    obstacle_info = data.perception_obstacle
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    #print(obstacle_info)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/obstacles/'+timestamp+'.txt','a+') as record_file:
                record_file.write(str(obstacle_info)+'\n')
                
    #print("=" * 80)


def prediction_callback(data):
    """
    Reader obstacle callback.
    """
    #print("=" * 80)
    #print("py:perception_callback msg->:")
    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1)) #str(round(data.header.timestamp_sec,2))
    prediction_info = data.prediction_obstacle
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    #print(prediction_info)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/prediction/'+timestamp+'.txt','a+') as record_file:
                record_file.write(str(prediction_info)+'\n')
                
    #print("=" * 80)



def pose_callback(data):
    """
    Reader pose callback.
    """
    #print("=" * 80)
    #print("py:perception_callback msg->:")
    #print(data)
    global cyber_start_time
    current_time = time.time()
    pose_info_time = str(round(current_time - cyber_start_time,2)) #str(round(data.header.timestamp_sec,2))
    position_x = str(round(data.pose.position.x,2))
    position_y = str(round(data.pose.position.y,2))
    velo_x = str(round(data.pose.linear_velocity.x,2))
    velo_y = str(round(data.pose.linear_velocity.y,2))
    velocity = str(round(math.sqrt(pow(data.pose.linear_velocity.x,2)+\
        pow(data.pose.linear_velocity.y,2)),2))
    acce_x = str(round(data.pose.linear_acceleration.x,2))
    acce_y = str(round(data.pose.linear_acceleration.y,2))
    acceleration = str(round(math.sqrt(pow(data.pose.linear_acceleration.x,2)+\
        pow(data.pose.linear_acceleration.y,2)),2))
  
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/pose.txt','a+') as record_file: 
        record_file.write(pose_info_time+'\t'+position_x+'\t'+position_y+'\t'+velocity+'\t'+acceleration+'\n')  
    #print("=" * 80)
    

def chassis_callback(data):
    """
    Reader message callback.
    """
    #print("=" * 80)
    #print("py:callback msg->:")
    global cyber_start_time
    current_time = time.time()
    chassis_info_time = str(round(current_time - cyber_start_time,2)) #str(round(data.header.timestamp_sec,2))
    chassis_info_rpm = str(round(data.engine_rpm,2))
    chassis_info_throttle = str(round(data.throttle_percentage,2))
    chassis_info_brake = str(round(data.brake_percentage,2))
    chassis_info_steering = str(round(data.steering_percentage,2))
    #print("=" * 80) 
    #print(desc_str)
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/chassis.txt','a+') as record_file:
        record_file.write(chassis_info_time+'\t'+ chassis_info_rpm+'\t'+chassis_info_throttle+'\t'+\
            chassis_info_brake+'\t'+chassis_info_steering+'\n')
        #record_file.write('This is the Simple message')

def control_callback(data):
    """
    Reader message callback.
    """
    # print("=" * 80)
    # print("py:callback msg->:")
    # print(data)
    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,2)) #str(round(data.header.timestamp_sec,2))
    control_throttle = str(round(data.throttle,2))
    control_brake = str(round(data.brake,2))
    control_steering_rate = str(round(data.steering_rate,2))
    control_steering_target = str(round(data.steering_target,2))
    control_acce = str(round(data.acceleration,2))
    control_gear = str(data.gear_location)
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/control.txt','a+') as record_file:
        record_file.write(timestamp+'\t'+ control_throttle+'\t'+control_brake+'\t'+\
            control_steering_rate+'\t'+control_steering_target+'\t'+ control_acce +'\t'+control_gear+'\n')
    # print("=" * 80)



def lidar_callback(data):
    """
    Reader message callback.
    """
    # print("=" * 80)
    # print("py:callback msg->:")
    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    cloudpoint = data.point
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/lidar/'+timestamp+'.txt','a+') as record_file:
        record_file.write(str(cloudpoint))
    # print("=" * 80)
    


def image_callback(image):
    """
    Reader img callback.
    """
    # print("=" * 80)
    # print("Received image:")
    # print("  format =", image.format)
    # print("  data =", len(image.data), "bytes")
    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    arr = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    ###################### Module imconpatible
    #cv2.waitKey(500)
    ######################
    save = cv2.imwrite('/apollo/record_to_text/'+SIM_EPOCH+'/image/'+timestamp+'.jpg', img)
    # if save:
    #     print("img saved successfully,  size = {}x{}".format(img.shape[1], img.shape[0]))
    # if not save:
    #     None
    #     #cyber.shutdown()
    # print("=" * 80)
    
def image_seg_callback(image):
    """
    Reader img callback.
    """
    # print("=" * 80)
    # print("Received image:")
    # print("  format =", image.format)
    # print("  data =", len(image.data), "bytes")
    global cyber_start_time
    current_time = time.time()
    timestamp = str(round(current_time - cyber_start_time,1))
    arr = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    ###################### Module imconpatible
    #cv2.waitKey(500)
    ######################
    save = cv2.imwrite('/apollo/record_to_text/'+SIM_EPOCH+'/image_seg/'+timestamp+'.jpg', img)
    # if save:
    #     print("img saved successfully,  size = {}x{}".format(img.shape[1], img.shape[0]))
    # if not save:
    #     None
    #     #cyber.shutdown()
    # print("=" * 80)    
    

def callback(data):
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
    test_node.create_reader("/simulator/segmentation_camera", CompressedImage, image_seg_callback)
    test_node.create_reader("/apollo/sensor/lidar128/compensator/PointCloud2", PointCloud, lidar_callback)
    
    test_node.create_reader("/apollo/prediction", PredictionObstacles, prediction_callback)
    test_node.create_reader("/apollo/canbus/chassis", Chassis, chassis_callback)
    test_node.create_reader("/apollo/control", ControlCommand, control_callback)
    test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, pose_callback)  
    test_node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, obstacle_callback) 

    #test_node.create_reader("/apollo/routing_request", RoutingRequest, callback)
    #test_node.create_reader("/apollo/routing_response", RoutingRequest, callback)
    #test_node.create_reader("/tf", TransformStampeds, callback) ## it works

    #print(test_node)
    test_node.spin()


if __name__ == '__main__':

    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/control.txt','a+') as record_file:
                record_file.write('Timestamps'+'\t'+'Control_throttle'+'\t'+'Control_brake'+'\t'+\
        'Control_steering_rate'+'\t'+'Control_steering_target'+'\t'+'Control_acce'+'\t'+'Control_gear'+'\n')

    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/chassis.txt','a+') as record_file:
                record_file.write('Timestamps'+'\t'+'Rpm'+'\t'+'Throttle '+'\t'+ 'Brake'+\
                    ' \t' +'Steering' +'\n')
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/pose.txt','a+') as record_file:
        record_file.write('Timestamps'+'\t'+'Posi_x'+'\t'+'Posi_y'+'\t'+\
        'Velocity '+'\t'+ 'Acce'+'\n')
    
    cyber.init()
    set_destination()
    sim_listener_class()
    cyber.shutdown()

