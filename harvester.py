import os
import sys
import numpy as np
from platform import system as platform
from harvesters.core import Harvester

def initialize_harvester(device_id="TER-008"):
    # Check and load cti file path
    cti_file_path_suffix = "/API/lib/photoneo.cti"
    cti_file_path = os.getenv('PHOXI_CONTROL_PATH')
    
    if cti_file_path is None:
        raise EnvironmentError("Environment variable 'PHOXI_CONTROL_PATH' not set.")
    
    cti_file_path += cti_file_path_suffix
    print("--> cti_file_path:", cti_file_path)
    
    h = Harvester()
    h.add_file(cti_file_path, True, True)
    h.update()

    # Print out available devices
    print("Available Devices:")
    for item in h.device_info_list:
        print(f"{item.property_dict['serial_number']} : {item.property_dict['id_']}")
    
    return h

def software_trigger(h, device_id, iterations=100):
    try:
        with h.create({'id_': device_id}) as ia:
            features = ia.remote_device.node_map
            print("TriggerMode BEFORE:", features.PhotoneoTriggerMode.value)
            features.PhotoneoTriggerMode.value = "Software"
            print("TriggerMode AFTER:", features.PhotoneoTriggerMode.value)
            
            # Configure outputs
            features.SendTexture.value = True
            features.SendPointCloud.value = True
            features.SendNormalMap.value = True
            features.SendDepthMap.value = True
            features.SendConfidenceMap.value = True
            
            ia.start()
            for _ in range(iterations):
                features.TriggerFrame.execute()
                with ia.fetch(timeout=10.0) as buffer:
                    print(buffer)  # Perform additional processing as needed
            ia.stop()
    except Exception as e:
        print("Error during acquisition:", e)

def configure_and_acquire(h):
    try:
        ia = h.create(0)
        ia.remote_device.node_map.Width.value = 8
        ia.remote_device.node_map.Height.value = 8
        ia.remote_device.node_map.PixelFormat.value = 'Mono8'
        ia.start()
        
        with ia.fetch() as buffer:
            component = buffer.payload.components[0]
            _1d = component.data
            print("1D Array:", _1d)
            _2d = _1d.reshape(component.height, component.width)
            print("2D Array:\n", _2d)
            print(f"Average: {np.average(_2d)}, Min: {_2d.min()}, Max: {_2d.max()}")
            
        ia.stop()
        ia.destroy()
    except Exception as e:
        print("Error during acquisition:", e)

# Main execution
if __name__ == "__main__":
    device_id = "PhotoneoTL_DEV_" + sys.argv[1] if len(sys.argv) == 2 else "TER-008"
    h = initialize_harvester(device_id)
    software_trigger(h, device_id)
    configure_and_acquire(h)
    h.reset()
