import numpy as np # imported an module numpy as np
import open3d as o3d 
import cv2 # imported cv2 module
import os # imported os module
import sys # imported sys module
from sys import platform
from harvesters.core import Harvester
import logging # imported logging module
import json # imported json module
import yaml # imported yaml module
import matplotlib.pyplot as plt # imported matplotlib module as plt

# ==================== Configuration ====================

# Define a configuration dictionary
config = {
    "camera": {
        "device_id": "PhotoneoTL_DEV_TER-008",  # Default device ID
        "cti_file_suffix": "/API/lib/photoneo.cti",
        "trigger_mode": "Software",
        "send_data": {
            "texture": True, # putiing texture as True
            "point_cloud": True,
            "normal_map": True,
            "depth_map": True,
            "confidence_map": True
        }
    },
    "image_processing": {
        "canny_threshold1": 50,
        "canny_threshold2": 150,
        "contour_area_min": 100,
        "shape_approx_epsilon": 0.02
    },
    "pointcloud_processing": {
        "plane_distance_threshold": 0.01,
        "ransac_n": 3,
        "ransac_iterations": 1000,
        "dbscan_eps": 0.07,
        "dbscan_min_points": 10,
        "voxel_size": 0.005  # For downsampling
    },
    "logging": {
        "level": "INFO",
        "format": "%(asctime)s [%(levelname)s] %(message)s"
    },
    "output": {
        "object_info_file": "object_info.json"
    }
}

# ==================== Logging Setup ====================

# Configure logging based on the configuration
logging.basicConfig(
    level=getattr(logging, config["logging"]["level"].upper(), logging.INFO),
    format=config["logging"]["format"]
)

# ==================== Object Information Storage ====================

# Initialize a list to store object information
object_info_list = []

# ==================== Function Definitions ====================

def display_pointcloud_with_matplotlib(pointcloud): # defining the function that display pointcloud with matplotlib
    """
    Visualize the point cloud using Matplotlib.
    Note: For real-time applications, Open3D visualization is recommended.
    """
    plt.ion()  # Enable interactive mode
    fig = plt.figure(figsize=(8, 6)) # plotting the figure_size
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2],
                         s=1, c=pointcloud[:, 2], cmap='viridis')
    ax.set_xlabel("X axis") # setting the X axis
    ax.set_ylabel("Y axis") # setting the Y axis
    ax.set_zlabel("Z axis") # setting the Z axis
    ax.view_init(elev=30, azim=120)
    plt.draw() # plotting the figure
    plt.pause(0.001)
    plt.clf()


def display_texture_if_available(texture_component): # defining the function texture_if_available
    """Display texture if available and dimensions match expectations."""
    if texture_component.width == 0 or texture_component.height == 0: # if the texture component width and height is zero then
        logging.warning("Texture is empty!") # print texture is empty
        return

    expected_size = texture_component.height * texture_component.width # setting the expected_size
    if texture_component.data.size != expected_size:
        logging.error("Mismatch in texture dimensions!")
        return

    texture = texture_component.data.reshape(texture_component.height, texture_component.width, 1).copy() # setting the 1D size array figure
    
    # Improved contrast for better visualization
    texture_screen = cv2.normalize(texture, dst=None, alpha=50, beta=200, norm_type=cv2.NORM_MINMAX).astype(np.uint8)
    
    # Noise reduction
    texture_screen = cv2.fastNlMeansDenoising(texture_screen, None, 10, 7, 21)

    cv2.imshow("Texture", texture_screen) # show the Texture file
    cv2.waitKey(1)
    return


def display_color_image_if_available(color_component, name): # defining the function if the color_image_is_available or not 
    """Display color image if available and dimensions match expectations."""
    if color_component.width == 0 or color_component.height == 0: # if the texture component width and height is zero then
        logging.warning(f"{name} is empty!") # print texture is empty
        return

    expected_size = color_component.height * color_component.width * 3 # setting the size to be 3 times
    if color_component.data.size != expected_size: # if the size of the color_component is not as expected then
        logging.error(f"Mismatch in {name} dimensions!")# mismatch happening
        return

    color_image = color_component.data.reshape(color_component.height, color_component.width, 3).copy() # creating an 3D array
    
    # Convert color_image to 8-bit if it's not already
    if color_image.dtype != np.uint8:
        color_image = cv2.normalize(color_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

    # Apply denoising for clearer image
    color_image = cv2.fastNlMeansDenoisingColored(color_image, None, 10, 10, 7, 21)
    
    # Shape detection
    edges = cv2.Canny(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY), 
                      threshold1=config["image_processing"]["canny_threshold1"], 
                      threshold2=config["image_processing"]["canny_threshold2"])
    detect_shapes_from_edges(edges, color_image)
    
    cv2.imshow(name, color_image)
    return


def detect_shapes_from_edges(edges, color_image): # defining an function that detect the shapes_from_edges
    """Detect and annotate geometric shapes based on edge detection."""
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < config["image_processing"]["contour_area_min"]:
            continue  # Filter out small contours

        epsilon = config["image_processing"]["shape_approx_epsilon"] * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        x, y, w, h = cv2.boundingRect(approx)
###### checking the shape of the objects #####       
        shape_name = "Unknown"
        if len(approx) == 3:
            shape_name = "Triangle"
        elif len(approx) == 4:
            aspect_ratio = float(w) / h
            shape_name = "Square" if 0.95 < aspect_ratio < 1.05 else "Rectangle"
        elif len(approx) > 4:
            # Use HoughCircles for better circle detection
            roi = edges[y:y+h, x:x+w]
            circles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                                       param1=50, param2=30, minRadius=10, maxRadius=0)
            if circles is not None:
                shape_name = "Circle"
            # else:
            #     shape_name = "Not defined"
        
        logging.info(f"Detected {shape_name} with dimensions (w: {w}, h: {h})")
        cv2.drawContours(color_image, [approx], -1, (0, 255, 0), 2)
        cv2.putText(color_image, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 2)

############
def display_pointcloud_if_available(pointcloud_comp, normal_comp, texture_comp, texture_rgb_comp): # defining the function that display_point_lcoud_is_available or not
    """Process and visualize point cloud data, including segmentation, clustering, and bounding boxes."""
    if pointcloud_comp.width == 0 or pointcloud_comp.height == 0: # checking if the texture is empty or not 
        logging.warning("PointCloud is empty!") # print the pointcloud is empty
        return

    pointcloud = pointcloud_comp.data.reshape(pointcloud_comp.height * pointcloud_comp.width, 3).copy()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)

    if normal_comp.width > 0 and normal_comp.height > 0: # checking if the component width and height is greater than zero
        norm_map = normal_comp.data.reshape(normal_comp.height * normal_comp.width, 3).copy()
        pcd.normals = o3d.utility.Vector3dVector(norm_map)

    # Downsample the point cloud for performance
    voxel_size = config["pointcloud_processing"]["voxel_size"]
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    logging.debug(f"PointCloud downsampled with voxel size: {voxel_size}")

    # Shape and size detection in point cloud
    object_cloud = segment_pointcloud(pcd)
    clusters = cluster_objects(object_cloud)
    if not clusters:
        logging.info("No clusters detected.")
    for idx, cluster in enumerate(clusters):
        size = calculate_bounding_box(cluster, idx)
        logging.info(f"Object {idx+1} size (X: {size['AABB_Size']['X']:.2f}, "
                     f"Y: {size['AABB_Size']['Y']:.2f}, Z: {size['AABB_Size']['Z']:.2f})")

  #### Process texture RGB ######
    texture_rgb = np.zeros((pointcloud_comp.height * pointcloud_comp.width, 3))
    if texture_comp.width > 0 and texture_comp.height > 0:
        texture = texture_comp.data.reshape(texture_comp.height, texture_comp.width, 1).copy()
        texture_rgb[:, 0] = np.reshape(1 / 65535 * texture, -1)
        texture_rgb[:, 1] = np.reshape(1 / 65535 * texture, -1)
        texture_rgb[:, 2] = np.reshape(1 / 65535 * texture, -1)
    elif texture_rgb_comp.width > 0 and texture_rgb_comp.height > 0:
        texture = texture_rgb_comp.data.reshape(texture_rgb_comp.height, texture_rgb_comp.width, 3).copy()
        texture_rgb[:, 0] = np.reshape(1 / 65535 * texture[:, :, 0], -1)
        texture_rgb[:, 1] = np.reshape(1 / 65535 * texture[:, :, 1], -1)
        texture_rgb[:, 2] = np.reshape(1 / 65535 * texture[:, :, 2], -1)
    else:
        logging.error("Texture and TextureRGB are empty!")
        return
    
    texture_rgb = cv2.normalize(texture_rgb, dst=None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    pcd.colors = o3d.utility.Vector3dVector(texture_rgb)
    
    # Visualize point cloud with Open3D
    o3d.visualization.draw_geometries([pcd], width=1024, height=768)
    return



def segment_pointcloud(pcd): # defining the function for segment_pointcloud 
    """Segment the plane (e.g., table) from the point cloud to isolate objects."""
    plane_model, inliers = pcd.segment_plane(distance_threshold=config["pointcloud_processing"]["plane_distance_threshold"],
                                             ransac_n=config["pointcloud_processing"]["ransac_n"],
                                             num_iterations=config["pointcloud_processing"]["ransac_iterations"])
    logging.info(f"Plane model: {plane_model}, Number of inliers: {len(inliers)}")
    object_cloud = pcd.select_by_index(inliers, invert=True)
    return object_cloud


def cluster_objects(pcd): # defining an function for cluster_objects
    """Cluster the segmented point cloud to identify distinct objects."""
    labels = np.array(pcd.cluster_dbscan(eps=config["pointcloud_processing"]["dbscan_eps"],
                                        min_points=config["pointcloud_processing"]["dbscan_min_points"],
                                        print_progress=True))
    if labels.size == 0:
        return []
    max_label = labels.max()
    if max_label == -1:
        return []
    clusters = [pcd.select_by_index(np.where(labels == i)[0]) for i in range(max_label + 1)]
    logging.info(f"Detected {len(clusters)} clusters.")
    return clusters


def calculate_bounding_box(cluster, cluster_id): # defining an function for calculating the bounding box
    """Calculate and visualize the bounding box for a given cluster."""
    if not cluster.has_points():
        logging.warning(f"Cluster {cluster_id+1} has no points.")
        return {"AABB_Size": {"X": 0, "Y": 0, "Z": 0}}

    # Axis-Aligned Bounding Box
    aabb = cluster.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)  # Red color
    aabb_size = aabb.get_extent()

    # Oriented Bounding Box
    obb = cluster.get_oriented_bounding_box()
    obb.color = (0, 1, 0)  # Green color
    obb_size = obb.get_extent()
    obb_center = obb.get_center()

    # Centroid
    centroid = aabb.get_center()

    # Visualize bounding boxes
    o3d.visualization.draw_geometries(
        [cluster, aabb, obb],
        window_name=f"Cluster {cluster_id+1}",
        width=800,
        height=600
    )
##### PREPARE OBJECT INFORMATION ##################

    object_info = {
        "Object_ID": cluster_id + 1,
        "AABB_Size": {
            "X": aabb_size[0],
            "Y": aabb_size[1],
            "Z": aabb_size[2]
        },
        "Centroid": {
            "X": centroid[0],
            "Y": centroid[1],
            "Z": centroid[2]
        },
        "OBB_Size": {
            "X": obb_size[0],
            "Y": obb_size[1],
            "Z": obb_size[2]
        },
        "OBB_Orientation": obb.R.tolist()
    }

    object_info_list.append(object_info)

    return object_info


def save_object_info(): # defining the function for save_object_info
    """Save the collected object information to a JSON file."""
    output_file = config["output"]["object_info_file"]
    try:
        with open(output_file, "w") as f:
            json.dump(object_info_list, f, indent=4)
        logging.info(f"Saved object information to {output_file}")
    except Exception as e:
        logging.error(f"Failed to save object information: {e}")


def software_trigger(): # defining the function called software trigger.
    """Main function to configure the camera, capture data, and process frames."""
    device_id = config["camera"]["device_id"]
    device_id = "PhotoneoTL_DEV_TER-008"
    logging.info(f"--> device_id: {device_id}")

    phoxi_control_path = os.getenv('PHOXI_CONTROL_PATH') # setting the phoxi_control_path 
    if not phoxi_control_path:
        logging.error("Environment variable 'PHOXI_CONTROL_PATH' is not set.")
        sys.exit(1)

    cti_file_path_suffix = config["camera"]["cti_file_suffix"].lstrip('/\\')  # Remove leading slashes
    cti_file_path = os.path.join(phoxi_control_path, cti_file_path_suffix)
    logging.info(f"--> cti_file_path: {cti_file_path}")

    if not os.path.isfile(cti_file_path): # if the path file is not there then:
        logging.error(f"CTI file not found at: {cti_file_path}")
        sys.exit(1)

    with Harvester() as h: # creating the harvester a h 
        h.add_file(cti_file_path, True, True) # adding the file in the file path
        h.update() # update the harvester

        logging.info("Name : ID")
        logging.info("---------")
        for item in h.device_info_list:
            serial_number = item.property_dict.get('serial_number', 'Unknown')
            device_id_found = item.property_dict.get('id_', 'Unknown')
            logging.info(f"{serial_number} : {device_id_found}")
        logging.info("")

        try:
            with h.create({'id_': device_id}) as ia:
                features = ia.remote_device.node_map

                logging.info(f"TriggerMode BEFORE: {features.PhotoneoTriggerMode.value}")
                features.PhotoneoTriggerMode.value = config["camera"]["trigger_mode"]
                logging.info(f"TriggerMode AFTER: {features.PhotoneoTriggerMode.value}")

                features.SendTexture.value = config["camera"]["send_data"]["texture"]
                features.SendPointCloud.value = config["camera"]["send_data"]["point_cloud"]
                features.SendNormalMap.value = config["camera"]["send_data"]["normal_map"]
                features.SendDepthMap.value = config["camera"]["send_data"]["depth_map"]
                features.SendConfidenceMap.value = config["camera"]["send_data"]["confidence_map"]

                ia.start() # start the harvester

                logging.info("Camera started. Press 'q' in any OpenCV window to exit.")

                while True:
                    logging.info("\n-- Capturing frame --")
                    features.TriggerFrame.execute()
                    with ia.fetch(timeout=1.0) as buffer:
                        payload = buffer.payload

                        # Verify payload has enough components
                        expected_components = 8  # Adjust based on actual components used
                        if len(payload.components) < expected_components: # if the length of the payload_component is less than the expected components
                            logging.error(f"Expected at least {expected_components} components, got {len(payload.components)}") # expected to be at least this much and got this much.
                            continue

                        # Display point cloud with Matplotlib (optional)
                        point_cloud_component = payload.components[2]
                        pointcloud_data = point_cloud_component.data.reshape(point_cloud_component.height * point_cloud_component.width, 3).copy()
                        display_pointcloud_with_matplotlib(pointcloud_data)

                        # Display texture
                        texture_component = payload.components[0]
                        display_texture_if_available(texture_component)

                        # Display texture RGB
                        texture_rgb_component = payload.components[1]
                        display_color_image_if_available(texture_rgb_component, "TextureRGB")

                        # Display color camera image
                        color_image_component = payload.components[7]
                        display_color_image_if_available(color_image_component, "ColorCameraImage")

                        # Display and process point cloud
                        norm_component = payload.components[3]
                        display_pointcloud_if_available(point_cloud_component, norm_component, texture_component, texture_rgb_component)

                    # Check for 'q' key press to exit
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        logging.info("Exiting capture loop.")
                        break

        except KeyboardInterrupt:
            logging.info("Interrupted by user. Exiting.")
        except Exception as e:
            logging.exception(f"An unexpected error occurred: {e}")
        finally:
            # Ensure that resources are released properly
            try:
                ia.stop() # stopping the harvester
            except:
                pass
            save_object_info()
            cv2.destroyAllWindows() # destroy all the windows


# ==================== Main Execution ====================

if __name__ == "__main__": 
    software_trigger() # calling the function software trigger


