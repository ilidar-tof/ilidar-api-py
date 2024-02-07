'''
OpenCV and configuring iTFS example with python

To check configurable parameter, refer ilidar_modules.py > _INFO

Please connect only 1 iTFS sensor at a time with this script. 
'''

from ilidar import iTFS
import cv2
import time
import numpy as np
import copy

# Script starts from here #
# Create the sensor
ilidar = iTFS()

# Wait for the sensor ready
while True:
    print("Finding iTFS sensor...")
    if ilidar.received_ip != "":
        ilidar.Set_listening_IP(ilidar.received_ip)
        break
    else:
        time.sleep(0.1)

# Create opencv window
window_name = "iTFS"
cv2.namedWindow(window_name, flags=cv2.WINDOW_NORMAL)

# To fast keyboard input cause freeze of the script 
# So, apply timer to keep fast keyboard input
already_input_timer = 0 

# Main loop starts from here #
while True:
    
    ## Display section
    # deep copy data
    data = copy.deepcopy(ilidar.Get_data())
    # depth data conversion for binary and viewer
    depth_data_binary = copy.deepcopy(data[0 : ilidar.max_row, :])
    depth_data_viewer = copy.deepcopy(
        (data[0 : ilidar.max_row, :]).astype(np.float32) 
        / 8000.0 
        * 255.0
    )
    depth_data_viewer = depth_data_viewer.astype(np.uint8)
    # intensity data conversion for binary and viewer
    inten_data_binary = copy.deepcopy(data[ilidar.max_row : 2 * ilidar.max_row, :])
    inten_data_viewer = copy.deepcopy(
        (
            data[ilidar.max_row : 2 * ilidar.max_row, :].astype(np.float32)
            / 5000.0
            * 255.0
        ).astype(np.uint8)
    )
    viewer = np.concatenate((depth_data_viewer, inten_data_viewer))


    ## Keyboard input and configuration section
    cv2.imshow(window_name, viewer)
    key_input = cv2.waitKey(10)
    
    # Apply o prevent too quick keyboard input
    if already_input_timer <= 0 :
        # c: EXIT Program
        if (key_input == ord('c')) or (key_input == ord('C')):
            already_input_timer = 100
            break
        
        # z: Store present configuration
        elif (key_input == ord('z')) or (key_input == ord('Z')):
            already_input_timer = 100
            ilidar.send_store()
            print('Store present configuration')
            time.sleep(0.5)

        # q: Setting mode 1 : No binning
        elif (key_input == ord('q')) or (key_input == ord('Q')):
            already_input_timer = 100
            new_info = copy.deepcopy(ilidar.info)
            new_info.capture_mode = 1
            ilidar.send_info(new_info)
            print('Set mode 1')
            time.sleep(0.5)

        # w: Setting mode 2 : V binning
        elif (key_input == ord('w')) or (key_input == ord('W')):
            already_input_timer = 100
            new_info = copy.deepcopy(ilidar.info)
            new_info.capture_mode = 2
            ilidar.send_info(new_info)
            print('Set mode 2')
            time.sleep(0.5)

        # e: Setting mode 3 : HV binning
        elif (key_input == ord('e')) or (key_input == ord('E')):
            already_input_timer = 100
            new_info = copy.deepcopy(ilidar.info)
            new_info.capture_mode = 3
            ilidar.send_info(new_info)
            print('Set mode 3')
            time.sleep(0.5)

        # a: Setting ip as 192.168.0.X and STORE
        # need manual reboot of sensor and this script
        elif (key_input == ord('a')) or (key_input == ord('A')):
            already_input_timer = 100
            # SET IP
            new_info = copy.deepcopy(ilidar.info)
            new_info.data_sensor_ip = [192,168,0,10]
            new_info.data_dest_ip = [192,168,0,2]
            new_info.data_gateway = [192,168,0,1]
            ilidar.send_info(new_info)
            time.sleep(0.5)
            # STORE
            ilidar.send_store()
            print('Store present configuration')
            time.sleep(0.5)
            print('Please Manually Reboot Sensor and change IP address!')
            break

        # s: Setting ip as 192.168.5.X and STORE
        # need manual reboot of sensor and this script
        elif (key_input == ord('s')) or (key_input == ord('S')):
            already_input_timer = 100
            # SET IP
            new_info = copy.deepcopy(ilidar.info)
            new_info.data_sensor_ip = [192,168,5,10]
            new_info.data_dest_ip = [192,168,5,2]
            new_info.data_gateway = [192,168,5,1]
            ilidar.send_info(new_info)
            time.sleep(0.5)
            # STORE
            ilidar.send_store()
            print('Store present configuration')
            time.sleep(0.5)
            print('Please Manually Reboot Sensor and change IP address!')
            break

    if already_input_timer > 0:
        already_input_timer = already_input_timer - 1


ilidar.Try_exit()
cv2.destroyAllWindows()

