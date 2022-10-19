import glob
import os
import sys
import numpy as np
import cv2
import json
import argparse
try:
    sys.path.append(glob.glob('/home/z644250/CARLA_9.13/CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg')[0])
except IndexError:
    pass

import carla
import math 
import random 
import time 
import queue
from tqdm import tqdm

from ClientSideBoundingBoxes import GetBoundingBoxes
from utils import NpEncoder

def get_args():

    parser = argparse.ArgumentParser()

    parser.add_argument('--dataset_1_name', type=str,
                        help='name of the collected dataset for camera position #1. eg. front_center_car')
    parser.add_argument('--dataset_2_name', type=str,
                        help='name of the collected dataset for camera position #2. eg. front_center_truck')
    parser.add_argument('--save_path', type=str,
                        help='Path to parent folder to save the images and annotations')
    parser.add_argument('--duration', type=int, default=2000,
                        help='number of images to be recorded per simulation')
    parser.add_argument('--dataset_split', type=str, default='train', help='dataset split - one of train, val or test')
    parser.add_argument('--rgb_cam_1_params', type=float, nargs='+',
                        help='provide a list of 6 elements of the form [x, y, z, roll_angle, pitch_angle, yaw_angle]. Dist is in meters and angles are in degrees')
    parser.add_argument('--rgb_cam_2_params', type=float, nargs='+',
                        help='provide a list of 6 elements of the form [x, y, z, roll_angle, pitch_angle, yaw_angle]. Dist is in meters and angles are in degrees')
    parser.add_argument('--vehicles', type=int, default=40,
                        help='number of vehiceles in the simulation')
    parser.add_argument('--pedestrians', type=int, default=75,
                        help='no of pedestrians in the simulation')
    parser.add_argument('--depth_maps', action='store_true',
                        help='save depth images from a depth camera')
    args = parser.parse_args()

    return args

def get_class(npc):
    npc_type = str(npc.type_id)
    if 'harley-davidson' in npc_type or 'vespa' in npc_type or 'kawasaki' in npc_type or 'yamaha' in npc_type:
        category = 5 # motorbike
    elif 'micro' in npc_type or 'diamondback' in npc_type or 'gazelle' in npc_type or 'bh' in npc_type:
        category = 4 # bicycle
    elif 'carlamotors' in npc_type or 'ambulance' in npc_type:
        category = 3 # big_vehicles
    elif 'pedestrian' in npc_type:
        category = 2 # pedestrians
    else:
        category = 1 # cars

    return category

def main():

    args = get_args()

    path_1 = args.save_path + args.dataset_1_name + '/'
    img_path_1 = path_1 + 'images/'
    annot_path_1 = path_1 + 'annotations/'

    path_2 = args.save_path + args.dataset_2_name + '/'
    img_path_2 = path_2 + 'images/'
    annot_path_2 = path_2 + 'annotations/'
    
    simulation_datset_1 = []
    simulation_datset_2 = []
    images = []

    # Connect to the client and get the world object
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)

    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.05 # must be less than 0.1, or else physics will be noisy
    
    # must use fixed delta seconds and synchronous mode for python api controlled sim, or else camera and sensor data may not match simulation properly and will be noisy 
    settings.synchronous_mode = True 
    world.apply_settings(settings)

    # Get the blueprint library and the spawn points for the map
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # Get the blueprint for the vehicle you want
    vehicle_bp = bp_lib.find('vehicle.audi.tt') 

    # Try spawning the vehicle at a randomly chosen spawn point
    vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

    # Add traffic to the simulation
    for i in range(args.vehicles): 
        vehicle_bp = random.choice(bp_lib.filter('vehicle')) 
        npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    
    # Get pedestrians and controller blueprints
    walker_bp = world.get_blueprint_library().filter('walker.pedestrian.*')
    controller_bp = world.get_blueprint_library().find('controller.ai.walker')

    controllers = []
    walkers = []

    # Generate pedestrians
    for i in range(args.pedestrians):
        
        #location
        trans = carla.Transform()
        trans.location = world.get_random_location_from_navigation()
        trans.location.z += 1
        
        # walker actor
        walker = random.choice(walker_bp)
        actor = world.try_spawn_actor(walker, trans)
        walkers.append(actor)
        
        # AI controller
        controller = world.spawn_actor(controller_bp, carla.Transform(), actor)
        controllers.append(controller)

    # Spawn RGB cammeras with an offset from the vehicle center
    camera_bp = bp_lib.find('sensor.camera.rgb')

    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')

    cam1_att = args.rgb_cam_1_params

    camera_init_trans_1 = carla.Transform(carla.Location(x=cam1_att[0], y=cam1_att[1], z=cam1_att[2]),
                                        carla.Rotation(pitch=cam1_att[4], yaw=cam1_att[5], roll=cam1_att[3])) # position of the camera relative to the center of the vehicle
    camera_1 = world.spawn_actor(camera_bp, camera_init_trans_1, attach_to=vehicle)
    image_queue_1 = queue.Queue()
    camera_1.listen(image_queue_1.put)

    cam2_att = args.rgb_cam_2_params

    camera_init_trans_2 = carla.Transform(carla.Location(x=cam2_att[0], y=cam2_att[1], z=cam2_att[2]),
                                        carla.Rotation(pitch=cam2_att[4], yaw=cam2_att[5], roll=cam2_att[3])) # position of the camera relative to the center of the vehicle
    camera_2 = world.spawn_actor(camera_bp, camera_init_trans_2, attach_to=vehicle)
    image_queue_2 = queue.Queue()
    camera_2.listen(image_queue_2.put)

    if args.depth_maps:
        depth_bp = bp_lib.find('sensor.camera.depth')
        depth_cam = world.spawn_actor(depth_bp, camera_init_trans_1, attach_to=vehicle)
        depth_queue = queue.Queue()
        depth_cam.listen(depth_queue.put)

    # Get the attributes from the camera
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    # Start simulation
    for j in tqdm(range(args.duration)):
    
        # step
        world.tick()
        
        # get rgb images
        image_1 = image_queue_1.get()
        image_2 = image_queue_2.get()
        
        # Set the all vehicles in motion using the Traffic Manager
        for v in world.get_actors().filter('*vehicle*'): 
            v.set_autopilot(True)
        
        controller = world.spawn_actor(controller_bp, carla.Transform(), actor)
        for i in range(args.pedestrians):
            # AI controller
            if walkers[i] is not None:
                controllers[i].start()
        
        classes = ['vehicle', 'pedestrian']
        
        for npc in world.get_actors():
    
            npc_type = str(npc.type_id)
            if any(x in npc_type for x in classes):

                dist = npc.get_transform().location.distance(vehicle.get_transform().location)

                if dist < 70:
                
                    if npc.id != vehicle.id:
                        forward_vec = vehicle.get_transform().get_forward_vector()
                        ray = npc.get_transform().location - vehicle.get_transform().location

                        if forward_vec.dot(ray) > 1:

                            x_min_1, y_min_1, height_1, width_1 = GetBoundingBoxes.get_bb(npc, camera_1, image_h, image_w)
                            x_min_2, y_min_2, height_2, width_2 = GetBoundingBoxes.get_bb(npc, camera_2, image_h, image_w)  

                            category = get_class(npc)
                            annot_1 = GetBoundingBoxes.get_dict(j, image_1, category, x_min_1, y_min_1, height_1, width_1)
                            annot_2 = GetBoundingBoxes.get_dict(j, image_2, category, x_min_2, y_min_2, height_2, width_2)
                            simulation_datset_1.append(annot_1)
                            simulation_datset_2.append(annot_2)
                        
        image = {
                "id": image_1.frame,
                "file_name": f'{image_1.frame}.png',
                "height": image_h,
                "width": image_w,
                "date_captured": None
                }
        image_1.save_to_disk(img_path_1 + args.dataset_split + f'/{image_1.frame}.png')
        image_2.save_to_disk(img_path_2 + args.dataset_split + f'/{image_2.frame}.png')
        images.append(image)

        if args.depth_maps:
            depth = depth_queue.get()
            depth.save_to_disk(path_1 + 'depth/' + f'{depth.frame}.png')

    refined_datset_1 = []
    refined_datset_2 = []
    for annot_1, annot_2 in zip(simulation_datset_1, simulation_datset_2):
        if annot_1['bbox'][0] in range(0, image_w) and annot_1['bbox'][1] in range (0,image_h):
            refined_datset_1.append(annot_1)
        if annot_2['bbox'][0] in range(0, image_w) and annot_2['bbox'][1] in range (0,image_h):
            refined_datset_2.append(annot_2)

    categories = [
                {"supercategory": "vehicle","id": 1,"name": "car"}, 
                {"supercategory": "person","id": 2,"name": "person"}, 
                {"supercategory": "vehicle","id": 3,"name": "big_vehicles"}, 
                {"supercategory": "vehicle","id": 4,"name": "bicycle"}, 
                {"supercategory": "vehicle","id": 5,"name": "motorbike"}
                ]
            
    annot_1 = {
        'categories': categories,
        'images': images,
        'annotations': refined_datset_1
        }

    annot_2 = {
        'categories': categories,
        'images': images,
        'annotations': refined_datset_2
        }

    os.makedirs(annot_path_1, exist_ok=True)
    os.makedirs(annot_path_2, exist_ok=True)

    with open(annot_path_1 + 'instances_' + args.dataset_split + '.json', 'w') as f:
        json.dump(annot_1, f, cls=NpEncoder)
    with open(annot_path_2 + 'instances_' + args.dataset_split + '.json', 'w') as f:
        json.dump(annot_2, f, cls=NpEncoder)

if __name__ == '__main__':
    main()
    print('Done!')