
################CAMERA FPS TEST#########

import cv2 # imported cv2 module
import time # imported time module

def measure_fps(camera_index=0, num_frames=100): # defining the function measure for the fps with camera-index and number of frames
    
    cap = cv2.VideoCapture(camera_index) # Initialize the camera

    # Verify if the camera opened successfully
    if not cap.isOpened(): # if the camera is not opened then 
        print("Error: Could not open camera.")
        return None

    # Start time
    start_time = time.time() # start the time 

    # Capture frames
    for i in range(num_frames): # check the range with having number of frames
        ret, frame = cap.read()
        if not ret: # if not return 
            print("Error: Could not read frame.") # print that it could not read the frame
            break

    end_time = time.time()  # End time

    # Release the camera
    cap.release() 

    # Calculate FPS
    elapsed_time = end_time - start_time # defined the equation to find the elapsed time 
    fps = num_frames / elapsed_time # defined the equation to find the frame per second.
    print(f"Approximate FPS: {fps:.2f}") # print the approximate the FPS having the 2 decimal places.
    return fps # return the frame per seconds


measure_fps() # Call the function to measure FPS

##############TEST WITH HARDWARE TRIGGER####

import sys # imported module sys
import time # imported time module
import numpy as np # imported numpy module
import open3d as o3d # imported open3d module
import cv2 # imported cv2 module
import os # imported os module
from pathlib import Path # imported Path module
from sys import platform # imported platform module
from typing import List # imported List module from the class typing
from genicam.genapi import NodeMap
from harvesters.core import Component2DImage, Harvester
from photoneo_genicam.default_gentl_producer import producer_path
from photoneo_genicam.features import enable_hardware_trigger
from photoneo_genicam.user_set import load_default_user_set

def display_texture_if_available(texture_component): # defining the function to check the texture is available or not 
    if texture_component.width == 0 or texture_component.height == 0: # if the texture width and height is empty
        print("Texture is empty!")
        return
    
    texture = texture_component.data.reshape(texture_component.height, texture_component.width, 1).copy() # creating the 2D image
    texture_screen = cv2.normalize(texture, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
    cv2.imshow("Texture", texture_screen) #  showing the 2D texture
    return

def display_color_image_if_available(color_component, name): # defining the function for color image is available or not
    if color_component.width == 0 or color_component.height == 0:# if the color components width and height is zero then
        print(name + " is empty!") # the texture is empty!
        return
    
    color_image = color_component.data.reshape(color_component.height, color_component.width, 3).copy() # creating the 3D array image   
    color_image = cv2.normalize(color_image, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX) # normalize the image 
    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR) 
    cv2.imshow(name, color_image) # show the image of 3D created
    return

def display_pointcloud_if_available(pointcloud_comp, normal_comp, texture_comp, texture_rgb_comp):# defining the function for displaying the pointcloud is available or not
    if pointcloud_comp.width == 0 or pointcloud_comp.height == 0:# if the pointcloud width and height is zero then
        print("PointCloud is empty!") # print the pointcloud is empty
        return
    
    pointcloud = pointcloud_comp.data.reshape(pointcloud_comp.height * pointcloud_comp.width, 3).copy()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)

    if normal_comp.width > 0 and normal_comp.height > 0: # if the normal_comp width and height is greater than zero then.
        norm_map = normal_comp.data.reshape(normal_comp.height * normal_comp.width, 3).copy() # assigning the norm_map and reshaping it to 2D array into 3 columns
        pcd.normals = o3d.utility.Vector3dVector(norm_map)

    texture_rgb = np.zeros((pointcloud_comp.height * pointcloud_comp.width, 3))
    if texture_comp.width > 0 and texture_comp.height > 0: # if the texture_comp width and height is greater than zero then
        texture = texture_comp.data.reshape(texture_comp.height, texture_comp.width, 1).copy() # reshaping the texture into 3D array
        texture_rgb[:, 0] = np.reshape(1 / 65536 * texture, -1)
        texture_rgb[:, 1] = np.reshape(1 / 65536 * texture, -1)
        texture_rgb[:, 2] = np.reshape(1 / 65536 * texture, -1)        
    elif texture_rgb_comp.width > 0 and texture_rgb_comp.height > 0:
        texture = texture_rgb_comp.data.reshape(texture_rgb_comp.height, texture_rgb_comp.width, 3).copy() # reshaped into 3D array having the 3 colors Red, Green and Blue
        texture_rgb[:, 0] = np.reshape(1 / 65536 * texture[:, :, 0], -1)
        texture_rgb[:, 1] = np.reshape(1 / 65536 * texture[:, :, 1], -1)
        texture_rgb[:, 2] = np.reshape(1 / 65536 * texture[:, :, 2], -1)
    else:
        print("Texture and TextureRGB are empty!") # print the texture and TextureRGB as empty 
        return

    texture_rgb = cv2.normalize(texture_rgb, dst=None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)# texture_rgb contains the input array and consider the scale up the range as 0,1
    pcd.colors = o3d.utility.Vector3dVector(texture_rgb)
    o3d.visualization.draw_geometries([pcd], width=800, height=600) # visualize the image with width and height with specific values
    return

def measure_fps_and_capture(device_sn: str, num_frames: int = 100): # function defined for measuring the fps and capture.
    try:
        with Harvester() as h: # defining the harvesters as h
            h.add_file(str(producer_path), check_existence=True, check_validity=True) # checking the producer_path , its existence and validity
            h.update() # updating the harvester

            print(f"Connecting to device with serial number: {device_sn}") # connecting to the device with serial number
            with h.create({"serial_number": device_sn}) as ia:
                features: NodeMap = ia.remote_device.node_map

                load_default_user_set(features) # loading the default user set features
                enable_hardware_trigger(features) # enabling the hardware trigger

                ia.start() # acquisition started
                timeout = 180 # defining the timeout
                print(f"Waiting {timeout}s for hardware-trigger signal...")

                start_time = time.time() # starting the time 

                for i in range(num_frames): # checking the range in number of frames
                    features.TriggerFrame.execute()  # Trigger frame
                    with ia.fetch(timeout=timeout) as buffer:
                        payload = buffer.payload

                        # Process components
                        texture_component = payload.components[0]
                        display_texture_if_available(texture_component)
                        
                        texture_rgb_component = payload.components[1]
                        display_color_image_if_available(texture_rgb_component, "TextureRGB") # checking if the display color image if available or not as TextureRGB

                        point_cloud_component = payload.components[2]
                        norm_component = payload.components[3]
                        display_pointcloud_if_available(point_cloud_component, norm_component, texture_component, texture_rgb_component)

                end_time = time.time() # defining the end time

                elapsed_time = end_time - start_time # equation for finding the elapsed time
                fps = num_frames / elapsed_time # equation for finding the fps
                print(f"Approximate FPS: {fps:.2f}") # printing the FPS having the decimal with 2 numbers.
                return fps

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        device_id = sys.argv[1]
        num_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 100  # Default to 100 frames
        print(f"Device ID: {device_id}, Number of frames: {num_frames}") # printing the device id and number of frames.
    except IndexError:
        print("Error: No device given, please run with the device serial number as argument:") # exception errors that can occur
        print(f"    {Path(__file__).name} <device serial> [num_frames]")
        sys.exit(1)
    except ValueError:
        print("Error: Number of frames must be an integer.")
        sys.exit(1)

    if not device_id:
        print("Error: Device ID cannot be None.")
        sys.exit(1)

    measure_fps_and_capture(device_id, num_frames) # calling the function to measure the fps and capture the image


