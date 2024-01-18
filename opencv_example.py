from ilidar import iTFS
import cv2
import time

# Script starts from here #
# Create the sensor
ilidar = iTFS()

# Wait for the sensor ready
while True:
    if (ilidar.Ready() == True):
        break
    else:
        time.sleep(0.1)

# Create opencv window
window_name = 'iTFS'
cv2.namedWindow(window_name, flags=cv2.WINDOW_NORMAL)

# Main loop starts from here #
while True:
    cv2.imshow(window_name, ilidar.Get_data())
    key_input = cv2.waitKey(10)
    if (key_input == 0x73) or (key_input == 0x53):
        # Exit when 's' or 'S' was pressed
        break

ilidar.Try_exit()
cv2.destroyAllWindows()

