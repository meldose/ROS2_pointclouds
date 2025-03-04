import numpy as np # imported numpy module
import open3d as o3d # imported open 3d module
import cv2 # imported cv2 module
import os # imported os 
import sys # imported sys 
from sys import platform # imported platform module from sys class
from harvesters.core import Harvester

def display_texture_if_available(texture_component): # defined the function for texture component.
    if texture_component.width == 0 or texture_component.height == 0: # if the texture is zero then
        print("Texture is empty!")
        return
    
    # Reshape 1D array to 2D array with image size
    texture = texture_component.data.reshape(texture_component.height, texture_component.width, 1).copy() # reshaped the texture component from 1D to 2D array with image size 
    texture_screen = cv2.normalize(texture, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX) # converted image of 2d array

    cv2.imshow("Texture", texture_screen) # Show image
    return 

def display_color_image_if_available(color_component, name): # defined function display_color_image_if_available
    if color_component.width == 0 or color_component.height == 0: # if the texture is zero then
        print(name + " is empty!")
        return
    
    # Reshape 1D array to 2D RGB image
    color_image = color_component.data.reshape(color_component.height, color_component.width, 3).copy()
    # Normalize array to range 0 - 65535
    color_image = cv2.normalize(color_image, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    # Show image
    cv2.imshow(name, color_image)
    return

def display_pointcloud_if_available(pointcloud_comp, normal_comp, texture_comp, texture_rgb_comp): # deined the function display_pointcloud_if_available
    if pointcloud_comp.width == 0 or pointcloud_comp.height == 0: # if the texture is zero then
        print("PointCloud is empty!") # print the pointcloud is empty
        return
    
    # Reshape for Open3D visualization to N x 3 arrays
    pointcloud = pointcloud_comp.data.reshape(pointcloud_comp.height * pointcloud_comp.width, 3).copy()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)

    if normal_comp.width > 0 and normal_comp.height > 0: # if the normal component is not empty
        norm_map = normal_comp.data.reshape(normal_comp.height * normal_comp.width, 3).copy()
        pcd.normals = o3d.utility.Vector3dVector(norm_map)

    # Reshape 1D array to 2D (3 channel) array with image size
    texture_rgb = np.zeros((pointcloud_comp.height * pointcloud_comp.width, 3))
    if texture_comp.width > 0 and texture_comp.height > 0: # if the texture component is not empty
        texture = texture_comp.data.reshape(texture_comp.height, texture_comp.width, 1).copy()
        texture_rgb[:, 0] = np.reshape(1/65536 * texture, -1) # reshaped the texture component from 1D to 2D array with image size
        texture_rgb[:, 1] = np.reshape(1/65536 * texture, -1) # reshaped the texture component from 1D to 2D array with image size
        texture_rgb[:, 2] = np.reshape(1/65536 * texture, -1) # reshaped the texture component from 1D to 2D array with image size    
    elif texture_rgb_comp.width > 0 and texture_rgb_comp.height > 0:
        texture = texture_rgb_comp.data.reshape(texture_rgb_comp.height, texture_rgb_comp.width, 3).copy()
        texture_rgb[:, 0] = np.reshape(1/65536 * texture[:, :, 0], -1) # reshaped the texture component from 1D to 2D array with image size
        texture_rgb[:, 1] = np.reshape(1/65536 * texture[:, :, 1], -1) # reshaped the texture component from 1D to 2D array with image size
        texture_rgb[:, 2] = np.reshape(1/65536 * texture[:, :, 2], -1) # reshaped the texture component from 1D to 2D array with image size
    else:
        print("Texture and TextureRGB are empty!") # print the texture is empty
        return
    texture_rgb = cv2.normalize(texture_rgb, dst=None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX) # set 3D array
    pcd.colors = o3d.utility.Vector3dVector(texture_rgb) # 
    o3d.visualization.draw_geometries([pcd], width=800,height=600) # set the geometry for image
    return



def software_trigger(iterations=100): # defining the number of time it take the pictures
    # PhotoneoTL_DEV_<ID>
     # if the lenght of the argument is 2
    device_id = "PhotoneoTL_DEV_TER-008"
    print("--> device_id: ", device_id)

    if platform == "linux": # if the platform is linux
        cti_file_path_suffix = "/API/lib/photoneo.cti" # provide the cti file path
    else:
        cti_file_path_suffix = "/API/lib/photoneo.cti"
    cti_file_path = os.getenv('PHOXI_CONTROL_PATH') + cti_file_path_suffix
    print("--> cti_file_path: ", cti_file_path)

    with Harvester() as h: # conidering the harvester
        h.add_file(cti_file_path, True, True)
        h.update()

        # Print out available devices
        print()
        print("Name : ID")
        print("---------")
        for item in h.device_info_list: # check the available devices
            print(item.property_dict['serial_number'], ' : ', item.property_dict['id_'])
        print()

        with h.create({'id_': device_id}) as ia:
            features = ia.remote_device.node_map

            #print(dir(features))
            print("TriggerMode BEFORE: ", features.PhotoneoTriggerMode.value)
            features.PhotoneoTriggerMode.value = "Software"
            print("TriggerMode AFTER: ", features.PhotoneoTriggerMode.value)


            # Send every output structure
            features.SendTexture.value = True
            features.SendPointCloud.value = True
            features.SendNormalMap.value = True
            features.SendDepthMap.value = True
            features.SendConfidenceMap.value = True
            #features.SendEventMap.value = True         # MotionCam-3D exclusive
            #features.SendColorCameraImage.value = True # MotionCam-3D Color exclusive

            ia.start()

            # Trigger frame by calling property's setter.
            # Must call TriggerFrame before every fetch.
            features.TriggerFrame.execute() # trigger first frame
            with ia.fetch(timeout=10.0) as buffer:
                # grab first frame
                # do something with first frame
                print(buffer)

                # The buffer object will automatically call its dto once it goes
                # out of scope and releases internal buffer object.
            
            for i in range(iterations=100): # defining the number of time it take the pictures
                print(f"\n-- Capturing frame {i+1}/{iterations} --")

            features.TriggerFrame.execute() # trigger second frame
            with ia.fetch(timeout=10.0) as buffer: # defining the timeout as 10
                # grab second frame
                # do something with second frame
                payload = buffer.payload

                texture_component = payload.components[0]
                display_texture_if_available(texture_component)
                
                texture_rgb_component = payload.components[1]
                display_color_image_if_available(texture_rgb_component, "TextureRGB")
                color_image_component = payload.components[7]
                display_color_image_if_available(color_image_component, "ColorCameraImage")

                point_cloud_component = payload.components[2]
                norm_component = payload.components[3]
                display_pointcloud_if_available(point_cloud_component, norm_component, texture_component, texture_rgb_component)

                # The buffer object will automatically call its dto once it goes
                # out of scope and releases internal buffer object.

            """
            # also possible use with error checking:
            features.TriggerFrame.execute() # trigger third frame
            buffer = ia.try_fetch(timeout=10.0) # grab newest frame
            if buffer is None:
                # check if device is still connected
                is_connected = features.IsConnected.value
                if not is_connected:
                    sys.exit('Device disconnected!')
            # ...
            # work with buffer
            # ...
            # release used buffer (only limited buffers available)
            buffer.queue()
            """

            # The ia object will automatically call its dtor
            # once it goes out of scope.

        # The h object will automatically call its dtor
        # once it goes out of scope.

software_trigger(iterations=100) # calling the function with 100 iterations
