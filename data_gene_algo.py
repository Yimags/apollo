# This script spawns the EGO vehicle in a random position in the target map
# Then a number of NPC vehicles are randomly spawned around3 EGO

# Install numpy and PIL before running this script
# SIMULATOR_HOST environment variable also needs to be set before running the script
'''
# 2 command line arguements are required when running this script. The arguements are:
# number of epoches to run (int)
# relative path to save sensor data (str)
'''
import lgsvl
from lgsvl.utils import transform_to_matrix
import os
import math
import time
import random
import numpy as np
from PIL import Image
import sys

epoch_num = int(sys.argv[1])
BASE_PATH = 'record_to_text'
TASK_PATH = sys.argv[2]
ENV_INDEX = os.path.join(BASE_PATH, TASK_PATH+'/text')
### Optional map assignment feature for simulation
#MAP_TO_RUN = sys.argv[3]
safety_flag = None
if os.path.exists(ENV_INDEX):
    print('\n Error: This path has been used! \n')
    sys.exit()  
os.makedirs(ENV_INDEX, exist_ok=True)
with open(ENV_INDEX + '/env_index.txt','a+') as record_file:
    record_file.write('Epoch'+'\t' +'Rain'+'\t'+'Fog'+'\t'+'Wetness'+'\t'+'Safety'+'\n')

class DataGenerator:
    def __init__(self, scene_name, agent_name,current_num):
        self.scene_name = scene_name
        self.agent_name = agent_name
        self.sim = None
        self.ego = None
        self.ego_state = None
        self.sensor_camera = None
        self.sensor_lidar = None
        self.sensor_imu = None
        self.npcs = []
        self.npcs_state = []
        self.current_num = current_num
        self.image_path = None
        self.lidar_pcd_path = None
        self.lidar_bin_path = None
        self.env_path = None
        self.idx = 0

        

        # Sensor Calibrations: intrinsic & extrinsic
        self.camera_intrinsics = None
        self.projection_matrix = None
        self.rectification_matrix = None
        self.tr_velo_to_cam = None
        self.tr_imu_to_velo = None

    # Collsion callback
    # This function gets called whenever ego vehicle collides with anything    
    def on_collision(self,agent1, agent2, contact):
        global safety_flag
        timestamp = round(self.sim.current_time,3)
        print("Ego vehicle collided at {} with orientation of {} after {} seconds ".format(
            contact, self.ego.transform.rotation,timestamp))
        safety_flag = '0'
        with open(ENV_INDEX + '/env_index.txt','a+') as record_file:
            record_file.write(str(safety_flag) +'\n')
        self.sim.stop()
        self.sim.close()

    # Arrival callback
    # This function gets called whenever ego vehicle arrive at destination   
    def on_destination(self,agent, kind, context):
        global safety_flag
        if kind == "IMU Plugin":
            timestamp = round(self.sim.current_time,3)
            print("Destination has been reached after {} seconds !".format(timestamp))
            #num_succeed = num_succeed + 1
            safety_flag = '1'
            with open(ENV_INDEX + '/env_index.txt','a+') as record_file:
                record_file.write(str(safety_flag) +'\n')
            self.sim.stop()
            self.sim.close()
        else:
            # ignore other custom callbacks
            pass


    # Sets up the required folder hierarchy and starts the simulator
    def sim_load(self):
        # os.makedirs(self.image_path, exist_ok=True)
        # os.makedirs(self.lidar_bin_path, exist_ok=True)
        # os.makedirs(self.lidar_pcd_path, exist_ok=True)
        self.sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
        if self.sim.current_scene == "BorregasAve":
            self.sim.reset()
            self.load_scene()
        else:
            self.load_scene()

        ## Random generation ########### 1st Method
        ############################################################
        # origin = lgsvl.Transform()
        # sx = origin.position.x
        # sy = origin.position.y
        # sz = origin.position.z
        # mindist = 0.0
        # maxdist = 100
        # angle = random.uniform(0.0, 2 * math.pi)
        # dist = random.uniform(mindist, maxdist)
        # point = lgsvl.Vector(sx + dist * math.cos(angle), sy, sz + dist * math.sin(angle))

        # transform = self.sim.map_point_on_lane(point)
        ############################################################

        ## Fixed point defined by UNity ########## 2nd Method
        ############################################################
        state = lgsvl.AgentState()
        spawns = self.sim.get_spawn()
        transform = spawns[0]
        ############################################################
        state.transform = transform
        self.ego = self.sim.add_agent(self.agent_name, lgsvl.AgentType.EGO,state)
        # cache the state for later queries
        self.ego_state = self.ego.state

        # print("Bridge connected:", self.ego.bridge_connected)
        # # The EGO is now looking for a bridge at the specified IP and port
        # self.ego.connect_bridge("127.0.0.1", 9090)
        # print("Waiting for connection...")
        # while not self.ego.bridge_connected:
        #     time.sleep(1)
        # print("Bridge connected:", self.ego.bridge_connected)
        # #self.load_sensors()
        # print("multi_sim starts success!")

    
# Loads the scene specified when DataGenerator is created. To save time, the scene is loaded only if it has not already be loaded
    def load_scene(self):
        print("Loading {} scene...".format(self.scene_name))
        if self.sim.current_scene != self.scene_name:
            self.sim.load(self.scene_name)
        print("\n{} scene has been loaded!".format(self.scene_name))

        # Weather
        set_time = round(random.uniform(7,20),1)
        self.sim.set_time_of_day(set_time, fixed=True)
        print("Current time of day:", self.sim.time_of_day)

        # Time
        rain_index = round(random.random(),1)
        fog_index = round(random.random(),1)
        wetness_index = round(random.random(),1)
        self.sim.set_weather = lgsvl.simulator.WeatherState(rain_index,fog_index,wetness_index)
        print('rain level =',self.sim.weather.rain)
        print('fog level =',self.sim.weather.fog)
        print('wetness level =',self.sim.weather.wetness)
        with open(ENV_INDEX + '/env_index.txt','a+') as record_file:
                 record_file.write(str(current_num)+'\t'+ str(rain_index)+'\t'+str(fog_index)+'\t'+str(wetness_index) +'\t')

# Saves the sensor objects for later use
    def load_sensors(self):
        print("\nAvailable sensors:")
        for sensor in self.ego.get_sensors():
            print("{}: {}".format(sensor.name, sensor.transform))
            if sensor.name == "Main Camera":
                self.sensor_camera = sensor
            if sensor.name == "Lidar":
                self.sensor_lidar = sensor
            if sensor.name == "IMU":
                self.sensor_imu = sensor
            # if sensor.name == "3D_ground_truth":
            #     self.sensor_truth = sensor

# Self generation of NPC vehicles, comment it out if you use random generation API
# Finds a random point near the EGO on the map. 
# Once that is done, randomly find another nearby point so that the NPCs are spawned on different lanes.
########################################################################################################
    def get_npc_random_transform(self):
        ego_transform = self.ego_state.transform
        sx = ego_transform.position.x
        sy = ego_transform.position.y
        sz = ego_transform.position.z
        ry = ego_transform.rotation.y

        mindist = 30.0
        maxdist = 300.0
        dist = random.uniform(mindist, maxdist)
        angle = random.uniform(0.0, 2 * math.pi)
        point = lgsvl.Vector(sx + dist * math.sin(angle), sy, sz + dist * math.cos(angle))

        transform = self.sim.map_point_on_lane(point)

        return transform
########################################################################################################

# Removes all spawned NPCs
    def reset_npcs(self):
        for npc in self.npcs:
            self.sim.remove_agent(npc)
        self.npcs = []
        self.npcs_state = []

# Creates a random number of NPCs
# Each NPC is randomly placed as long as the random position passes some checks
# This will timeout after 9 seconds
    def setup_npcs(self):
        self.reset_npcs()
        ############### Method 1: Self-spawn NPCs, comment out Method 2
        if self.scene_name == 'SanFrancisco':
            num_npcs = random.randint(30,35)
        else:
            num_npcs = random.randint(20,30)
        print("Placing {} NPCs...".format(num_npcs))
        t0 = time.time()
        while len(self.npcs) < num_npcs:
            if time.time() - t0 > 5:
                print("Timeout! Stop placing NPCs")
                break
            npc_transform = self.get_npc_random_transform()

            if self.is_npc_too_close_to_ego(npc_transform):
                continue

            if self.is_npc_too_close(npc_transform):
                continue

            self.position_npc(npc_transform)
        print("Done placing {} NPCs ({:.3f} s)".format(len(self.npcs), time.time() - t0))

        ############### Method 2: Auto-spawn NPCs,  comment out Method 1
        # self.sim.add_random_agents(lgsvl.AgentType.NPC)
        # self.sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)


# Creates a random NPC type at the given location
    def position_npc(self, transform):
        npc_state = lgsvl.AgentState()
        npc_state.transform = transform
        available_npcs = ['Sedan', 'SUV', 'Jeep', 'Hatchback','SchoolBus', 'BoxTruck']  # 
        npc_type = available_npcs[random.randint(0, len(available_npcs) - 1)]
        npc = self.sim.add_agent(npc_type, lgsvl.AgentType.NPC, npc_state)
        npc.follow_closest_lane(True, 25)
        self.npcs.append(npc)
        self.npcs_state.append(npc_state)

# Checks if the given position is too close to the EGO
    def is_npc_too_close_to_ego(self, npc_transform):
        for agent, agent_state in zip([self.ego] + self.npcs, [self.ego_state] + self.npcs_state):
            if abs(npc_transform.position.x - agent_state.transform.position.x) < 5 and abs(npc_transform.position.z - agent_state.transform.position.z) < 5:
                return True

        return False


# Checks if the given position is too close to the otherNPC
    def is_npc_too_close(self, npc_transform):
        if self.npcs_state != None:
            for agent_state in self.npcs_state:
                if abs(npc_transform.position.x - agent_state.transform.position.x) < 5 and abs(npc_transform.position.z - agent_state.transform.position.z) < 5:
                    return True

        return False


# Run the simulation
    def multi_run(self):
        self.objects = {
        self.ego: "EGO",
        # suv: "SUV",
        # sedan: "Sedan",
        # jeep:'Jeep',
        # hatchback:'Hatchback'
        # boxtruck:'BoxTruck'
        # person:'Bob'
         }

        self.ego.on_collision(self.on_collision)
        # set callback & run simulation
        self.ego.on_custom(self.on_destination)
        self.sim.run()

# Saves camera, lidar, ground truth, and calibration data
    def capture_data(self):
        if len(self.npcs) == 0:
            print("No NPCs! Skip frame.")
            return

        self.save_camera_image()
        self.save_lidar_point()
        # self.save_calibration()
        # self.save_ground_truth()

        self.idx += 1

# Saves a camera image from the EGO main camera as a jpg or png
    def save_camera_image(self):
        if self.sensor_camera:
            t0 = time.time()
            timestamp = round(self.sim.current_time,3)
            self.sensor_camera.save(self.image_path+
                        str(timestamp)+".jpg", compression=5)
            

            ## Umcomment if you want PNG format of image
            ##############################################
            # im = Image.open(self.image_path+ str(timestamp)+".jpg")
            # png_file = os.path.join(IMAGE_PNG_PATH, self.get_filename("png"))
            # im.save(png_file)
            ##############################################

            print("{} ({:.3f} s)".format(self.image_path+
                        str(timestamp)+".jpg", time.time() - t0))
        else:
            print("Warn: Camera sensor is not available")

# Saves a LIDAR scan from the EGO as a bin
    def save_lidar_point(self):
        if self.sensor_lidar:
            t0 = time.time()
            pcd_file = os.path.join(self.lidar_pcd_path, self.get_filename("pcd"))
            self.sensor_lidar.save(pcd_file)
            with open(pcd_file, "rb") as f:
                pc = self.parse_pcd_file(f)
            bin_file = os.path.join(self.lidar_bin_path, self.get_filename("bin"))
            pc.tofile(bin_file)
            print("{} ({:.3f} s)".format(bin_file, time.time() - t0))
        else:
            print("Warn: Lidar sensor is not available")

# Converts the lidar PCD to binary which is required for KITTI
    def parse_pcd_file(self, pcd_file):
        header = {}
        while True:
            ln = pcd_file.readline().strip()
            field = ln.decode('ascii').split(' ', 1)
            header[field[0]] = field[1]
            if ln.startswith(b"DATA"):
                break

        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.uint8),
        ])
        size = int(header['POINTS']) * dtype.itemsize
        buf = pcd_file.read(size)
        lst = np.frombuffer(buf, dtype).tolist()
        out = []
        for row in lst:
            out.append((row[0], row[1], row[2], row[3] / 255))
        pc = np.array(out).astype(np.float32)

        return pc


# Returns the current filename given an extension
    def get_filename(self, ext):
        return "{:06d}.{}".format(self.idx, ext)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("incorrect number of arguments")
        sys.exit()

    t_ini = time.time()
    for current_num in range (epoch_num):
        # This can be editted to load whichever map and vehicle
        
        data_gene = DataGenerator('BorregasAve', "Lincoln2017MKZ (Apollo 5.0)",current_num)
        data_gene.sim_load()
        data_gene.setup_npcs()
        data_gene.multi_run()
        # data_gene.capture_data()

        print("Elapsed time per frame: {:.3f} s".format(time.time() - t_ini))
    print("\nTotal elapsed time for {} Epochs: {:.3f} s".format(epoch_num, time.time() - t_ini))
