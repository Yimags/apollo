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
"""Module for example of talker."""

import time

from cyber.python.cyber_py3 import cyber
from modules.routing.proto.routing_pb2 import RoutingRequest

def test_talker_class():
    """
    Test talker.
    """
    msg = RoutingRequest()
    msg.waypoint.id = "lane_30"
    msg.waypoint.pose.x = 587005
    msg.waypoint.pose.y = 4141406
    print(msg)
    test_node = cyber.Node("request")
    g_count = 1

    writer = test_node.create_writer("/apollo/routing_request", RoutingRequest, 15)
    # while not cyber.is_shutdown():
    #     time.sleep(1)
    #     g_count = g_count + 1
    #     msg.seq = g_count
    #     msg.content = "I am python talker."
    #     print("=" * 80)
    #     print("write msg -> %s" % msg)
    #     writer.write(msg)
    writer.write(msg)


if __name__ == '__main__':
    cyber.init("talker_sample")
    test_talker_class()
    cyber.shutdown()
