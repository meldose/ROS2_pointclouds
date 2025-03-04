import numpy as np # imported numpy as np
import open3d as o3d # imported module o3d
from genicam.genapi import NodeMap # imported module NodeMap
from harvesters.core import Component2DImage, ImageAcquirer

from .components import enable_components # imported function enable_components
from .features import enable_software_trigger # imported function enable_software_trigger
from .user_set import load_default_user_set # imported function load_default_user_set
from .utils import measure_time # imported function measure_time


def calculate_point_cloud_from_projc(depth_map: np.ndarray, coordinate_map: np.ndarray) -> np.array:
    return depth_map[:, np.newaxis] * coordinate_map

# defining an function for constructing the coordinate map
def construct_coordinate_map(
    coordinate_a: np.ndarray,
    coordinate_b: np.ndarray,
    focal_length: float,
    aspect_ratio: float,
    principal_point_u: float,
    principal_point_v: float,
) -> np.array:
    x = (coordinate_a - principal_point_u) / focal_length # define the x coordinate
    y = (coordinate_b - principal_point_v) / (focal_length * aspect_ratio) # define the y coordinate
    z = np.ones_like(x) # define the z coordinate
    return np.stack([x, y, z], axis=-1)


# The astype(np.float64) operation is required to make this operation fast
def create_3d_vector(input_array_as_np: np.ndarray): # define the function for 3d vector 
    return o3d.utility.Vector3dVector(input_array_as_np.reshape(-1, 3).astype(np.float64))

def map_texture(texture: Component2DImage) -> o3d.utility.Vector3dVector:
    # Open3D point colors property expects (num_points, 3) format with values in the range [0, 1]
    if texture.data_format == "BGR8":  # Handle BGR8 format
        return o3d.utility.Vector3dVector(texture.data.reshape(-1, 3).astype(np.float64) / 255.0)
    if texture.data_format == "Mono12":  # Handle Mono12 format
        normalized = texture.data.reshape(-1, 1).astype(np.float64) / 4096.0
        return o3d.utility.Vector3dVector(np.repeat(normalized, 3, axis=-1))
    if texture.data_format == "Mono16":  # Handle Mono16 format
        normalized = texture.data.reshape(-1, 1).astype(np.float64) / 65536.0
        return o3d.utility.Vector3dVector(np.repeat(normalized, 3, axis=-1))
    if texture.data_format == "Mono20":  # Handle Mono20 format
        normalized = texture.data.reshape(-1, 1).astype(np.float64) / 1048576.0
        return o3d.utility.Vector3dVector(np.repeat(normalized, 3, axis=-1))



@measure_time
def pre_fetch_coordinate_maps(ia: ImageAcquirer) -> np.ndarray: # defined the function having ImageAcquirer
    assert not ia.is_acquiring(), "Acquisition is not stopped"

    features: NodeMap = ia.remote_device.node_map # cosnider the features as NodeMap

    trigger_mode_before_pre_fetch = features.TriggerMode.value # consider the trigger_mode_before_pre_fetch as the trigger_mode
    trigger_source_before_pre_fetch = features.TriggerSource.value # consider the trigger_source_before_pre_fetch as the trigger_source

    enable_software_trigger(features) # enabling the software trigger
    enable_components(features, ["CoordinateMapA", "CoordinateMapB"])

    focal_length: float = features.Scan3dFocalLength.value # consider the focal length as float
    aspect_ratio: float = features.Scan3dAspectRatio.value # consider the aspect ratio as float
    ppu: float = features.Scan3dPrincipalPointU.value # consider the ppu as float
    ppv: float = features.Scan3dPrincipalPointV.value # consider the ppv as float

    ia.start() # start acquisition
    features.TriggerSoftware.execute() # execute and trigger the software 

    coord_map: np.ndarray = None
    with ia.fetch(timeout=3) as buffer: 
        coordinate_map_a: Component2DImage = buffer.payload.components[0]
        coordinate_map_b: Component2DImage = buffer.payload.components[1]
        coord_map = construct_coordinate_map(
            coordinate_map_a.data, coordinate_map_b.data, focal_length, aspect_ratio, ppu, ppv
        )
    ia.stop() # stop the acquisition 

    features.TriggerMode.value = trigger_mode_before_pre_fetch # setting the TriggerMode value 
    features.TriggerSource.value = trigger_source_before_pre_fetch # setting the TriggerSource value
    return coord_map
