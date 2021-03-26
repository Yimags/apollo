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
"""
Module for example of record.

Run with:
    bazel run //cyber/python/cyber_py3/examples:record
"""

import time
import math
import os,sys
from google.protobuf.descriptor_pb2 import FileDescriptorProto
sys.path.append('/home/inem-laptop/apollo/')
from cyber.proto.unit_test_pb2 import Chatter
from cyber.python.cyber_py3 import record
from modules.common.util.testdata.simple_pb2 import SimpleMessage

## CyberRT messages
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.control.proto.control_cmd_pb2 import ControlCommand
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles

#MSG_TYPE = 'apollo.localization.LocalizationEstimate'
MSG_TYPE = "apollo.common.util.test.SimpleMessage"
MSG_TYPE_CHATTER = "apollo.cyber.proto.Chatter"

Chassis_msg = "apollo.canbus.Chassis"
Localization_msg = "apollo.localization.LocalizationEstimate"
Obstacle_msg = "apollo.perception.PerceptionObstacles"
Transform_msg = "apollo.transform.TransformStampeds"
Control_msg = "apollo.control.ControlCommand"

class chassis_class:
    def _int_(self,time,rpm,throttle,brake,steering):
        self.time = 0
        self.rpm = 0
        self.throttle = 0
        self.brake = 0
        self.steering = 0


class pose_class:
    def _int_(self,time,position_x,position_y,velocity_x,velocity_y,acce_x,acce_y):
        self.time = 0
        self.position_x = 0
        self.position_y = 0
        self.velocity_x = 0
        self.velocity_y = 0
        self.acce_x = 0
        self.acce_y = 0


def test_record_trans(reader_path):
    """
    Record trans.
    """
    fwriter = record.RecordWriter()
    if not fwriter.open(TEST_RECORD_FILE):
        print('Failed to open record writer!')
        return
    print('+++ Begin to trans +++')

    fread = record.RecordReader(reader_path)
    count = 0
    for channelname, msg, datatype, timestamp in fread.read_messages():
        # print channelname, timestamp, fread.get_messagenumber(channelname)
        desc = fread.get_protodesc(channelname)
        fwriter.write_channel(channelname, datatype, desc)
        fwriter.write_message(channelname, msg, timestamp)
        count += 1
    print('-' * 80)
    print('Message count: %d' % count)
    print('Channel info: ')
    channel_list = fread.get_channellist()
    print('Channel count: %d' % len(channel_list))
    print(channel_list)



def test_record_reader(reader_path):
    """
    Record reader.
    """
    freader = record.RecordReader(reader_path)
    time.sleep(1)
    print('+' * 80)
    print('+++ Begin to read +++')
    count = 0
    for channel_name, msg, datatype, timestamp in freader.read_messages():
        count += 1
        print('=' * 80)
        print('read [%d] messages' % count)
        print('channel_name -> %s' % channel_name)
        print('msgtime -> %d' % timestamp)
        print('msgnum -> %d' % freader.get_messagenumber(channel_name))
        print('msgtype -> %s' % datatype)
        #print('message is -> %s' % msg)
        print('***After parse(if needed),the message is ->')


        if datatype == Obstacle_msg:
            msg_new = PerceptionObstacles()
            msg_new.ParseFromString(msg)
            desc_str = msg_new.SerializeToString()
            print(msg_new)
            #print(msg_new.header.timestamp_sec,msg_new.perception_obstacle.id)
            #print(desc_str)
            with open('/apollo/test_record_file.txt','a+') as record_file:
                record_file.write(str(desc_str))
                #record_file.write('This is the Simple message')


        elif datatype == Control_msg:
            msg_new = ControlCommand()
            msg_new.ParseFromString(msg)
            desc_str = msg_new.SerializeToString()
            print(msg_new)
            print(desc_str)
            with open('/apollo/test_record_file.txt','a+') as record_file:
                record_file.write(str(desc_str))
                #record_file.write('This is the Simple message')


        elif datatype == Localization_msg:
            pose_info = pose_class()
            msg_new = LocalizationEstimate()
            msg_new.ParseFromString(msg)
            desc_str = msg_new.SerializeToString()
            pose_info.time = str(round(msg_new.header.timestamp_sec,2))
            position_x = str(round(msg_new.pose.position.x,2))
            position_y = str(round(msg_new.pose.position.y,2))
            velo_x = str(round(msg_new.pose.linear_velocity.x,2))
            velo_y = str(round(msg_new.pose.linear_velocity.y,2))
            velocity = str(round(math.sqrt(pow(msg_new.pose.linear_velocity.x,2)+\
                pow(msg_new.pose.linear_velocity.y,2)),2))
            acce_x = str(round(msg_new.pose.linear_acceleration.x,2))
            acce_y = str(round(msg_new.pose.linear_acceleration.y,2))
            acceleration = str(round(math.sqrt(pow(msg_new.pose.linear_acceleration.x,2)+\
                pow(msg_new.pose.linear_acceleration.y,2)),2))
            #desc_str = msg_new.SerializeToString()
            print(velocity,acceleration)
            #print(desc_str)
            with open('/apollo/record_to_text/pose.txt','a+') as record_file:
                record_file.write(pose_info.time+'\t'+position_x+'\t'+position_y+'\t'+velocity+'\t'+acceleration+'\n')
            #     #record_file.write('This is the Simple message')


        elif datatype == Chassis_msg:
            chassis_info = chassis_class()
            msg_new = Chassis()
            msg_new.ParseFromString(msg)
            chassis_info.time = str(round(msg_new.header.timestamp_sec,2))
            chassis_info.rpm = str(round(msg_new.engine_rpm,2))
            chassis_info.throttle = str(round(msg_new.throttle_percentage,2))
            chassis_info.brake = str(round(msg_new.brake_percentage,2))
            chassis_info.steering = str(round(msg_new.steering_percentage,2))
            print('timestamp = '+chassis_info.time+' engine_rpm = '+chassis_info.rpm+\
                ' engine_throttle = '+chassis_info.throttle\
                +' engine_brake = '+chassis_info.brake+' engine_steering = '+\
                    chassis_info.steering )
            desc_str = msg_new.SerializeToString()
            #print(msg_new.)
            #print(msg_new.header.module_name)
            #print(desc_str)
            with open('/apollo/record_to_text/chassis.txt','a+') as record_file:
                record_file.write(chassis_info.time+'\t'+ chassis_info.rpm+'\t'+chassis_info.throttle+'\t'+\
                    chassis_info.brake+'\t'+chassis_info.steering+'\n')
                #record_file.write('This is the Simple message')


        elif datatype == Transform_msg:
            msg_new = TransformStampeds()
            msg_new.ParseFromString(msg)
            desc_str = msg_new.SerializeToString()
            print(msg_new)
            print(pose_info.time)
            #print(desc_str)
            # with open('/apollo/test_record_file.txt','ab+') as record_file:
            #     record_file.write(msg)
            #     #record_file.write('This is the Simple message')




if __name__ == '__main__':
    #test_record_file = '/apollo/test_pose.record.00000'
    #test_record_file = '/apollo/test_chassis.record.00000'
    #test_record_file = '/apollo/test_obstacles.record.00000'
    #test_record_file = '/apollo/test_imu.record.00000'
    test_record_file = '/apollo/test_chassis.record.00000'

    # test_record_file = "/tmp/test_writer.record"



    print('Begin to read record file: {}'.format(test_record_file))
    # with open('/apollo/record_to_text/chassis.txt','a+') as record_file:
    #             record_file.write('Timstap'+'\t'+'rpm'+'\t'+'throt '+'\t'+ 'brake'+\
    #                 ' \t' +'steering' +'\n')
    with open('/apollo/record_to_text/pose.txt','a+') as record_file:
                record_file.write('Timstap'+'\t'+'Posi_x'+'\t'+'Posi_y'+'\t'+\
                    'Velo '+'\t'+ 'Acce'+'\n')
    test_record_reader(test_record_file)



    # if len(sys.argv) < 2:
    #     print('Usage: %s record_file' % sys.argv[0])
    #     sys.exit(0)

    # cyber.init()
    # test_record_trans('/apollo/test_obstacles.record.00000')  ##sys.argv[1]
    # cyber.shutdown()
