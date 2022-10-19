# CARLA_data_generation

This repo provides a set of scripts built on the CARLA Python API to directly start collecting data for different autonomous driving scenarios from CARLA eg. different set of camera positions, different set of sensors like rgb and depth cameras, alter the density of vehicles and pedestrians...

## Installation and Getting started

Install CARLA version 0.9.13 or later to be able to get bounding boxes for your synthetic data generation scenarios. Refer to https://carla.readthedocs.io/en/latest/start_quickstart/ for quick start package installation on your Linux machine - NO need to build it from source for the data generations tasks !! 

Create a venv and install the required packages from requirements.txt

## Usage
Just edit the the absolute path for the .egg file for importing carla package and you are good to go. This package can be found in the following directory inisde the installation folder - 'CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg'

To disable and enable rendering via the command line, run the following commands:\
cd PythonAPI/util && python3 config.py --no-rendering\
cd PythonAPI/util && python3 config.py --rendering

Steps to start data generation:
1. Start the CARLA simulation server in Epic qulaity mode using the following command:\
./CarlaUE4.sh -quality-level=Epic
2. Run the generate.sh shell script with the below arguments:\
./generate.sh

list of arguments:

--dataset_1_name: name of the collected dataset for camera position #1. eg. front_center_car\
--dataset_2_name: name of the collected dataset for camera position #2. eg. front_center_truck\
--save_path: Path to parent folder to save the images and annotations\
--duration: number of images to be recorded per simulation\
--dataset_split: dataset split - one of train, val or test\
--rgb_cam_1_params: provide 6 elements of the form [x, y, z, roll_angle, pitch_angle, yaw_angle]. Dist is in meters and angles are in degrees. eg. 2 0 2 0 0 0 \
--rgb_cam_2_params: provide 6 elements of the form [x, y, z, roll_angle, pitch_angle, yaw_angle]. Dist is in meters and angles are in degrees. eg. 2 0 2 0 -15 0\
--vehicles: number of vehiceles in the simulation\
--pedestrians: no of pedestrians in the simulation\
--depth_maps: save depth images from a depth camera
