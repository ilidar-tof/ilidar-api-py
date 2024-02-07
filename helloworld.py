from ilidar import iTFS
import time

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

# Main loop starts from here #
while True:
    str = input()

    # Exit when 's' or 'S' was pressed
    if str[0] == 's' or str[0] == 'S':
        break

ilidar.Try_exit()
