#pip3 install --user -e.
# test
import os,sys
sys.path.append('/home/inem-laptop/PythonAPI/lgsvl')
import lgsvl
from lgsvl.utils import transform_to_matrix
import time
from cyber.python.cyber_py3.example.listener import test_listener_class
import numpy as np

# import math
# import time
# import random

# from PIL import Image
# # import sys

# load
sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
if sim.current_scene == "BorregasAve":
    sim.reset()
else:
    sim.load("BorregasAve")

## DIR checking
def check_prep(path): 
    if not os.path.exists(path):
        os.makedirs(path) 

cam_dirs = "/home/inem-laptop/lgsvlsimulator-linux64/data/camera/"
lidar_dirs = "/home/inem-laptop/lgsvlsimulator-linux64/data/lidar/"

check_prep(cam_dirs)
check_prep(lidar_dirs)

# weather
#w = sim.weather
# w.rain = 0.5     # set rain to 50%
#sim.weather = w


# random spawn
spawns = sim.get_spawn()
print('spawn is', spawns)


sx = spawns[0].position.x
sy = spawns[0].position.y
sz = spawns[0].position.z

mindist = 20.0
maxdist = 100.0
# ego vehicle

state = lgsvl.AgentState()
state.transform = spawns[0]
ego = sim.add_agent("Lincoln2017MKZ (Apollo 5.0)", lgsvl.AgentType.EGO, state)

# An EGO will not connect to a bridge unless commanded to
print("Bridge connected:", ego.bridge_connected)

# The EGO is now looking for a bridge at the specified IP and port
ego.connect_bridge("127.0.0.1", 9090)

print("Waiting for connection...")

while not ego.bridge_connected:
    time.sleep(1)

print("Bridge connected:", ego.bridge_connected)


sensors = ego.get_sensors()

# Sensors have an enabled/disabled state that can be set
# By default all sensors are enabled
# Disabling sensor will prevent it to send or receive messages to ROS or Cyber bridges
#for s in sensors:
#    print(type(s), s.enabled)


# ##other agents


forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])

# for i, name in enumerate(["Sedan", "SUV", "Jeep", "Hatchback",'SchoolBus','BoxTruck']):


# # Creates a random point around the EGO
#   angle = random.uniform(0.0, 2*math.pi)
#   dist = random.uniform(mindist, maxdist)

#   point = lgsvl.Vector(sx + dist * math.cos(angle), sy, sz + dist * math.sin(angle))

# # Creates an NPC on a lane that is closest to the random point
#   state = lgsvl.AgentState()
#   state.transform = sim.map_point_on_lane(point)
#   if i==1:
#     sedan = sim.add_agent(name, lgsvl.AgentType.NPC, state)
#   elif i==2:
#     suv = sim.add_agent(name, lgsvl.AgentType.NPC, state)
#   elif i==3:
#     jeep = sim.add_agent(name, lgsvl.AgentType.NPC, state)
#   elif i==4:
#     hatchback = sim.add_agent(name, lgsvl.AgentType.NPC, state)
#   elif i==5:
#     schoolbus = sim.add_agent(name, lgsvl.AgentType.NPC, state)
#   else:
#     boxtruck = sim.add_agent(name, lgsvl.AgentType.NPC, state)


# sedan.follow_closest_lane(True, 11.1)
# suv.follow_closest_lane(True, 11.1)
# jeep.follow_closest_lane(True, 11.1)
# hatchback.follow_closest_lane(True, 11.1)
# schoolbus.follow_closest_lane(True, 11.1)
# boxtruck.follow_closest_lane(True, 11.1)


objects = {
    ego: "EGO",
    # suv: "SUV",
    # sedan: "Sedan",
    # jeep:'Jeep',
    # hatchback:'Hatchback',
    # boxtruck:'BoxTruck'
    # person:'Bob'
}

# Collsion callback
# This function gets called whenever ego vehicle above collides with anything


def on_collision(agent1, agent2, contact):
    name1 = objects[agent1]
    name2 = objects[agent2] if agent2 is not None else "OBSTACLE"

    toc = time.time()
    timestamp = round(sim.current_time,3)
    print("{} collided with {} at {} after {} seconds ".format(
        name1, name2, contact, timestamp))
    sim.stop()

    for sensor in ego.get_sensors():
        if sensor.name == "Main Camera":
            sensor.save(path="./data/camera_" +
                        str(timestamp)+".png", compression=5)

        if sensor.name == "Lidar":
            sensor.save(path="./data/lidar_"+str(timestamp)+".pcd")

def onCustom(agent, kind, context):
  if kind == "comfort":
    print("Comfort sensor callback!", context)
  else:
    # ignore other custom callbacks
    pass

# set callback & run simulation


def datagenerator(datagenerator):
    while True:
        time.sleep(1)
        toc = time.time()
        timestamp = toc-tic
        for sensor in ego.get_sensors():
            if sensor.name == "Main Camera":
                sensor.save(path="./data/camera_" +
                            str(timestamp)+".png", compression=0)


# # The above on_collision function needs to be added to the callback list of each vehicle


# # If the passed bool is False, then the NPC will not moved
# # The float passed is the maximum speed the NPC will drive
# # 11.1 m/s is ~40 km/h
#sedan.follow_closest_lane(True, 11.1)

# # 5.6 m/s is ~20 km/h
#suv.follow_closest_lane(True, 5.6)
sim.add_random_agents(lgsvl.AgentType.NPC)
sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)
tic = time.time()
ego.on_collision(on_collision)
#ego.on_custom(onCustom)
#datagenerator(datagenerator)
test_listener_class()
sim.run()



# space for delate the exceeding data