# test
import lgsvl
from lgsvl.utils import transform_to_matrix
import os
import math
import time
import random
#import numpy as np
from PIL import Image
import sys


def auto_data_gen():

    # load
    sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
    if sim.current_scene == "BorregasAve":
        sim.reset()
    else:
        sim.load("BorregasAve")


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

    spawns = sim.get_spawn()
    print('spawn is', spawns)
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



    objects = {
        ego: "EGO",
        
    }

    # Collsion callback
    # This function gets called whenever ego vehicle above collides with anything


    def on_collision(agent1, agent2, contact):
        name1 = objects[agent1]
        name2 = objects[agent2] if agent2 is not None else "OBSTACLE"

        toc = time.time()
        timestamp = toc-tic
        print("{} collided with {} at {} after {} seconds ".format(
            name1, name2, contact, timestamp))
        

        for sensor in ego.get_sensors():
            if sensor.name == "Main Camera":
                sensor.save(path="./data/camera_" +
                            str(timestamp)+".png", compression=5)

            if sensor.name == "Lidar":
                sensor.save(path="./data/lidar_"+str(timestamp)+".pcd")

          
        sim.stop()
        sim.close()

    # # The above on_collision function needs to be added to the callback list of each vehicle
    sim.add_random_agents(lgsvl.AgentType.NPC)
    sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)
    tic = time.time()
    ego.on_collision(on_collision)

    sim.run()
  

if __name__ == '__main__':
    epochs = 5#input('ply define number of epochs you want to run the simulation:')

    for i in range(epochs):
        auto_data_gen()
        print('{} of all {} epochs has been finished'.format(i+1,epochs))


    print('Simulation is finished, data has been saved')