
import numpy as np
import open3d as o3d
import cv2
import os
import sys
from sys import platform
from harvesters.core import Harvester

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def display_pointcloud_with_matplotlib(pointcloud):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.set_zlabel("Z axis")
    ax.view_init(elev=30, azim=120)

def display_texture_if_available(texture_component):
    """Display texture if available and dimensions match expectations."""
    if texture_component.width == 0 or texture_component.height == 0:
        print("Texture is empty!")
        return

    expected_size = texture_component.height * texture_component.width
    if texture_component.data.size != expected_size:
        print("Mismatch in texture dimensions!")
        return

    texture = texture_component.data.reshape(texture_component.height, texture_component.width, 1).copy()
    texture_screen = cv2.normalize(texture, dst=None, alpha=50, beta=200, norm_type=cv2.NORM_MINMAX).astype(np.uint8)
    texture_screen = cv2.fastNlMeansDenoising(texture_screen, None, 10, 7, 21)

    cv2.imshow("Texture", texture_screen)
    cv2.waitKey(1)
    return

def display_color_image_if_available(color_component, name):
    """Display color image if available and dimensions match expectations."""
    if color_component.width == 0 or color_component.height == 0:
        print(f"{name} is empty!")
        return

    expected_size = color_component.height * color_component.width * 3
    if color_component.data.size != expected_size:
        print(f"Mismatch in {name} dimensions!")
        return

    color_image = color_component.data.reshape(color_component.height, color_component.width, 3).copy()
    if color_image.dtype != np.uint8:
        color_image = cv2.normalize(color_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
    color_image = cv2.fastNlMeansDenoisingColored(color_image, None, 10, 10, 7, 21)
    
    # Shape detection
    edges = cv2.Canny(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY), threshold1=50, threshold2=150)
    detect_shapes_from_edges(edges, color_image)
    
    cv2.imshow(name, color_image)
    return

def detect_shapes_from_edges(edges, color_image):
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        x, y, w, h = cv2.boundingRect(approx)
        shape_name = "Unknown"
        if len(approx) == 3:
            shape_name = "Triangle"
        elif len(approx) == 4:
            aspect_ratio = float(w) / h
            shape_name = "Square" if 0.95 < aspect_ratio < 1.05 else "Rectangle"
        elif len(approx) > 4:
            shape_name = "Circle"
        
        print(f"Detected {shape_name} with dimensions (w: {w}, h: {h})")
        cv2.drawContours(color_image, [approx], -1, (0, 255, 0), 2)

def display_pointcloud_if_available(pointcloud_comp, normal_comp, texture_comp, texture_rgb_comp):
    if pointcloud_comp.width == 0 or pointcloud_comp.height == 0:
        print("PointCloud is empty!")
        return

    pointcloud = pointcloud_comp.data.reshape(pointcloud_comp.height * pointcloud_comp.width, 3).copy()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)

    if normal_comp.width > 0 and normal_comp.height > 0:
        norm_map = normal_comp.data.reshape(normal_comp.height * normal_comp.width, 3).copy()
        pcd.normals = o3d.utility.Vector3dVector(norm_map)

    # Shape and size detection in point cloud
    object_cloud = segment_pointcloud(pcd)
    clusters = cluster_objects(object_cloud)
    for cluster in clusters:
        size = calculate_bounding_box(cluster)
        print(f"Object size (X: {size[0]:.2f}, Y: {size[1]:.2f}, Z: {size[2]:.2f})")

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
        print("Texture and TextureRGB are empty!")
        return
    
    texture_rgb = cv2.normalize(texture_rgb, dst=None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    pcd.colors = o3d.utility.Vector3dVector(texture_rgb)
    
    o3d.visualization.draw_geometries([pcd], width=1024, height=768)
    return

def segment_pointcloud(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    object_cloud = pcd.select_by_index(inliers, invert=True)
    return object_cloud

def cluster_objects(pcd):
    labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
    max_label = labels.max()
    clusters = [pcd.select_by_index(np.where(labels == i)[0]) for i in range(max_label + 1)]
    return clusters

def calculate_bounding_box(cluster):
    bbox = cluster.get_axis_aligned_bounding_box()
    size = bbox.get_extent()
    o3d.visualization.draw_geometries([cluster, bbox])
    return size

def software_trigger():
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

        print("Name : ID")
        print("---------")
        for item in h.device_info_list:
            print(item.property_dict['serial_number'], ' : ', item.property_dict['id_'])
        print()

        with h.create({'id_': device_id}) as ia:
            features = ia.remote_device.node_map

            print("TriggerMode BEFORE: ", features.PhotoneoTriggerMode.value)
            features.PhotoneoTriggerMode.value = "Software"
            print("TriggerMode AFTER: ", features.PhotoneoTriggerMode.value)

            features.SendTexture.value = True
            features.SendPointCloud.value = True
            features.SendNormalMap.value = True
            features.SendDepthMap.value = True
            features.SendConfidenceMap.value = True

            ia.start()

            while True:
                print("\n-- Capturing frame --")
                features.TriggerFrame.execute()
                with ia.fetch(timeout=10.0) as buffer:
                    payload = buffer.payload

                    point_cloud_component = payload.components[2]
                    pointcloud = point_cloud_component.data.reshape(point_cloud_component.height * point_cloud_component.width, 3).copy()
                    display_pointcloud_with_matplotlib(pointcloud)

                    texture_component = payload.components[0]
                    display_texture_if_available(texture_component)

                    texture_rgb_component = payload.components[1]
                    display_color_image_if_available(texture_rgb_component, "TextureRGB")
                    color_image_component = payload.components[7]
                    display_color_image_if_available(color_image_component, "ColorCameraImage")

                    point_cloud_component = payload.components[2]
                    norm_component = payload.components[3]
                    display_pointcloud_if_available(point_cloud_component, norm_component, texture_component, texture_rgb_component)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Exiting capture loop.")
                    cv2.destroyAllWindows()
                    break

            ia.stop()
            
software_trigger()
