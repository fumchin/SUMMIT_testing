import glob
import os
import sys
import cv2

try:
    sys.path.append(glob.glob(os.path.abspath('%s/../../carla/dist/carla-*%d.%d-%s.egg' % (
        os.path.realpath(__file__),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass

os.environ["PYRO_LOGFILE"] = "pyro.log"
os.environ["PYRO_LOGLEVEL"] = "DEBUG"

from collections import defaultdict
from multiprocessing import Process
from threading import RLock
import Pyro4
import argparse
import carla
import math
import numpy as np
import random
import time
if sys.version_info.major == 2:
    from pathlib2 import Path
else:
    from pathlib import Path


IM_WIDTH = 640
IM_HEIGHT = 480
DATA_PATH = Path(os.path.realpath(__file__)).parent.parent.parent/'Data'

def process_img(image):
	i = np.array(image.raw_data)
	#print(dir(i))
	i2 = i.reshape((IM_HEIGHT,IM_WIDTH,4))
	i3 = i2[:, :, :3]
	#print(i3.shape)
	cv2.imshow("",i3)
	cv2.waitKey(10000)
	return i3/255.0


def main():
    actor_list = []

    try:
        #connect to server
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        
        #load sumo network
        sumo_network = carla.SumoNetwork.load(str(DATA_PATH/'{}.net.xml'.format("meskel_square")))
        world = client.get_world()
        
        # Get arbitrary position.
        position = carla.Vector2D(350, 350)

        # Get nearest route point on SUMO network.
        route_point = sumo_network.get_nearest_route_point(position)
        # Get route point position.
        route_point_position = sumo_network.get_route_point_position(route_point)
        
        print("route_point_position: ",route_point_position)
        # Get route point transform
        route_point_transform = carla.Transform(carla.Location(x=route_point_position.x,y=route_point_position.y,z=0.5))
        print("route_point_transform: ",route_point_transform)


        blueprint_library = world.get_blueprint_library()

        #choose the vehicle you want
        bp = blueprint_library.filter("model3")[0]							
        print(bp)
        print("spawn_point: ",route_point_transform)

        vehicle = world.spawn_actor(bp,route_point_transform)				
        #let vechicle move			
        #vehicle.apply_control(carla.VehicleControl(throttle=0.5,steer=0.0))
        actor_list.append(vehicle)

        
        #camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')	
        camera_bp.set_attribute("image_size_x",str(IM_WIDTH))
        camera_bp.set_attribute("image_size_y",str(IM_HEIGHT))
        camera_bp.set_attribute("fov","120")
        
        #relative to center of vehicle (left hand coordination)
        spawn_point = carla.Transform(carla.Location(x=-5,y=3,z=5),carla.Rotation(pitch=-40,yaw=-50))				
        
        #attach sensor to vehicle
        camera = world.spawn_actor(camera_bp, spawn_point, attach_to=vehicle)	
        
        actor_list.append(camera)
        camera.listen(lambda data : process_img(data))

        time.sleep(20)
        

    finally:

        print('destroying actors')
        for actor in actor_list:
            #print(actor)
            actor.destroy()
        print('done.')


if __name__ == '__main__':

    main()
