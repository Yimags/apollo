#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Insert routing request
Usage:
    mock_routing_request.py
"""
import argparse
import os
import sys
import time
import random

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.routing.proto import routing_pb2


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


if __name__ == '__main__':
    set_destination()
