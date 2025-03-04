
import numpy as np # import numpy as module
import open3d as o3d # import o3d module
import cv2 # imported module cv2
import os # imported os module
import sys # imported sys module
from sys import platform # imported platform from sys class
from harvesters.core import Harvester
from ultralytics import YOLO
import math

# Load the YOLO model
model = YOLO("yolo-Weights/yolov8n.pt")

# Object classes
classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]


def display_texture_if_available(texture_component): # defining an function for display texture if available
    if texture_component.width == 0 or texture_component.height == 0: # checking if the texture is empty or not
        print("Texture is empty!")
        return
    

def display_pointcloud_if_available(pointcloud_comp, normal_comp, texture_comp, texture_rgb_comp): # defining the function for display pointcloud is available or not
    if pointcloud_comp.width == 0 or pointcloud_comp.height == 0: # check if the texture is empty or not
        print("PointCloud is empty!")
        return
    
    
def display_color_image_with_detection(color_component, name): # defined a function with color_with image detection
    if color_component.width == 0 or color_component.height == 0: # check with the texture is empty or not
        print(name + " is empty!")
        return

    # Convert to 3-channel color image
    color_image = color_component.data.reshape(color_component.height, color_component.width, 3).copy() # convert it to 1D to 2D array 
    color_image = cv2.normalize(color_image, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX) # convert 2D to 3D array 
    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

    # Apply YOLO object detection
    results = model(color_image, stream=True)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            confidence = math.ceil((box.conf[0] * 100)) / 100
            cls = int(box.cls[0])

            # Draw bounding box and label
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.putText(color_image, f"{classNames[cls]} {confidence}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the color image with detections
    cv2.imshow(name, color_image)

def software_trigger(): # defined an function for software trigger 
    
    device_id = "PhotoneoTL_DEV_TER-008"
    print("--> device_id: ", device_id)

    if platform == "linux":
        cti_file_path_suffix = "/API/lib/photoneo.cti"
    else:
        cti_file_path_suffix = "/API/lib/photoneo.cti"
    cti_file_path = os.getenv('PHOXI_CONTROL_PATH') + cti_file_path_suffix
    print("--> cti_file_path: ", cti_file_path)

    with Harvester() as h:
        h.add_file(cti_file_path, True, True)
        h.update()

        print("\nName : ID")
        print("---------")
        for item in h.device_info_list:
            print(item.property_dict['serial_number'], ' : ', item.property_dict['id_'])
        print()

        with h.create({'id_': device_id}) as ia:
            features = ia.remote_device.node_map

            # Configure trigger and data streaming
            features.PhotoneoTriggerMode.value = "Software"
            features.SendTexture.value = True
            features.SendPointCloud.value = True
            features.SendNormalMap.value = True
            features.SendDepthMap.value = True
            features.SendConfidenceMap.value = True

            ia.start() # start acquisition 

            while True:
                print("\n-- Capturing frame --")
                features.TriggerFrame.execute()
                with ia.fetch(timeout=10.0) as buffer:
                    payload = buffer.payload

                    # Display texture
                    texture_component = payload.components[0]
                    display_texture_if_available(texture_component)

                    # Display color image with YOLO detection
                    texture_rgb_component = payload.components[1]
                    display_color_image_with_detection(texture_rgb_component, "TextureRGB")

                    # Display 3D point cloud if required
                    point_cloud_component = payload.components[2]
                    norm_component = payload.components[3]
                    display_pointcloud_if_available(point_cloud_component, norm_component, texture_component, texture_rgb_component)

                # Press 'q' to quit the loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Exiting capture loop.")
                    break

            ia.stop() # stop the acquisition 

software_trigger() # calling the function to trigger the camera

