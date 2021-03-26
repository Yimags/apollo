#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script for collection of data from cyberRT.

Run with:
    bazel run //cyber/python/cyber_py3/examples:listener_custom (File_name you want to save under)
"""
import sys,os,time,math
#sys.path.append('/home/inem-laptop/apollo/')
from cyber.python.cyber_py3 import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark
from modules.common.util.testdata.simple_pb2 import SimpleMessage
##

from modules.drivers.proto.sensor_image_pb2 import CompressedImage
from modules.drivers.proto.sensor_image_pb2 import Image
from modules.drivers.proto.pointcloud_pb2 import PointCloud


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

SIM_EPOCH = sys.argv[1]
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/image/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/lidar/', exist_ok=True)
os.makedirs('/apollo/record_to_text/'+SIM_EPOCH+'/text/', exist_ok=True)

tmp_time = 0

def obstacle_callback(data):
    """
    Reader obstacle callback.
    """
    print("=" * 80)
    print("py:perception_callback msg->:")
    timestamp = str(round(data.header.timestamp_sec,2))
    obstacle_info = data.perception_obstacle
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    print(obstacle_info)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/obstacles.txt','a+') as record_file:
                record_file.write('timestamp:'+timestamp+'\n'+str(obstacle_info)+'\n'+'\n'+'\n')
                
    print("=" * 80)


def prediction_callback(data):
    """
    Reader obstacle callback.
    """
    print("=" * 80)
    print("py:perception_callback msg->:")
    timestamp = str(round(data.header.timestamp_sec,2))
    prediction_info = data.prediction_obstacle
    # obstacle_info = data.perception_obstacle.position
    #desc_str = obstacle_info.ParseFromString(msg)
    print(prediction_info)  ## It ends with the smallest message type defined in protobuf.
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/prediction.txt','a+') as record_file:
                record_file.write('timestamp:'+timestamp+'\n'+str(prediction_info)+'\n'+'\n'+'\n')
                
    print("=" * 80)



def pose_callback(data):
    """
    Reader pose callback.
    """
    print("=" * 80)
    print("py:perception_callback msg->:")
    print(data)
    pose_info_time = str(round(data.header.timestamp_sec,2))
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
    print("=" * 80)
    

def chassis_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    chassis_info_time = str(round(data.header.timestamp_sec,2))
    chassis_info_rpm = str(round(data.engine_rpm,2))
    chassis_info_throttle = str(round(data.throttle_percentage,2))
    chassis_info_brake = str(round(data.brake_percentage,2))
    chassis_info_steering = str(round(data.steering_percentage,2))
    print('timestamp = '+chassis_info_time+' engine_rpm = '+chassis_info_rpm+\
        ' engine_throttle = '+chassis_info_throttle\
        +' engine_brake = '+chassis_info_brake+' steering = '+\
            chassis_info_steering )
    print("=" * 80) 
    #print(msg_new.)
    #print(msg_new.header.module_name)
    #print(desc_str)
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/chassis.txt','a+') as record_file:
        record_file.write(chassis_info_time+'\t'+ chassis_info_rpm+'\t'+chassis_info_throttle+'\t'+\
            chassis_info_brake+'\t'+chassis_info_steering+'\n')
        #record_file.write('This is the Simple message')

def control_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    print(data)
    timestamp = str(round(data.header.timestamp_sec,2))
    control_throttle = str(round(data.throttle,2))
    control_brake = str(round(data.brake,2))
    control_steering_rate = str(round(data.steering_rate,2))
    control_steering_target = str(round(data.steering_target,2))
    control_acce = str(round(data.acceleration,2))
    control_gear = str(data.gear_location)
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/control.txt','a+') as record_file:
        record_file.write(timestamp+'\t'+ control_throttle+'\t'+control_brake+'\t'+\
            control_steering_rate+'\t'+control_steering_target+'\t'+ control_acce +'\t'+control_gear+'\n')
    print("=" * 80)



def lidar_callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:callback msg->:")
    timestamp = str(round(data.header.timestamp_sec,2))
    cloudpoint = data.point
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/lidar/pointcloud_'+timestamp+'.txt','a+') as record_file:
        record_file.write(str(cloudpoint))
    print("=" * 80)
    


def image_callback(image):
    """
    Reader img callback.
    """
    print("=" * 80)
    print("Received image:")
    print("  format =", image.format)
    print("  data =", len(image.data), "bytes")
    print('time = ',image.header.timestamp_sec)
    
    #global tmp_time 
    arr = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    ###################### Module imconpatible
    #cv2.waitKey(500)
    ######################
    save = cv2.imwrite('/apollo/record_to_text/'+SIM_EPOCH+'/image/img_'+str(round(image.header.timestamp_sec,2))+'.jpg', img)
    # if save:
    #     print("img saved successfully,  size = {}x{}".format(img.shape[1], img.shape[0]))
    # if not save:
    #     None
    #     #cyber.shutdown()
    print("=" * 80)
    
    
    

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
    test_node = cyber.Node("listener8")
    test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, image_callback)
    test_node.create_reader("/apollo/sensor/lidar128/compensator/PointCloud2", PointCloud, lidar_callback)
    
    test_node.create_reader("/apollo/prediction", PredictionObstacles, prediction_callback)
    test_node.create_reader("/apollo/canbus/chassis", Chassis, chassis_callback)
    test_node.create_reader("/apollo/control", ControlCommand, control_callback) ## it works
    test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, pose_callback)  ## it works
    test_node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, obstacle_callback)  ## it works

    #test_node.create_reader("/apollo/routing_request", RoutingRequest, callback)
    #test_node.create_reader("/apollo/routing_response", RoutingRequest, callback)
    #test_node.create_reader("/tf", TransformStampeds, callback) ## it works
    
    #print(test_node)

    
    
    #test_node.create_reader("/apollo/sensor/camera/front_6mm/image", Image, image_callback)
    #test_node.create_reader("/apollo/sensor/lidar128/compensator/PointCloud2", CompressedImage, lidar_callback)

    
    # test_node.create_reader("/apollo/sensor/camera/front_6mm/image/compressed", CompressedImage, on_image)
    #test_node.create_reader("channel/chatter", ChatterBenchmark, callback)
    
    test_node.spin()
    
    
    


if __name__ == '__main__':

    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/control.txt','a+') as record_file:
        record_file.write('timestamp'+'\t'+'control_throttle'+'\t'+'control_brake'+'\t'+\
            'control_steering_rate'+'\t'+'control_steering_target'+'\t'+'control_acce'+'\t'+'control_gear'+'\n')

    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/chassis.txt','a+') as record_file:
                record_file.write('Timstap'+'\t'+'rpm'+'\t'+'throt '+'\t'+ 'brake'+\
                    ' \t' +'steering' +'\n')
    with open('/apollo/record_to_text/'+SIM_EPOCH+'/text/pose.txt','a+') as record_file:
        record_file.write('Timestamp'+'\t'+'Posi_x'+'\t'+'Posi_y'+'\t'+\
        'Velocity '+'\t'+ 'Acce'+'\n')

    cyber.init()
    sim_listener_class()
    cyber.shutdown()
