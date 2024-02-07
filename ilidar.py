from threading import Semaphore, Thread  # Thread related modules
import numpy as np  # array processing
import socket  # UDP communication
import struct  # Interpret bytes as packed binary data
import time  # To use time.sleep: Thread suspending function

######################################################################################
##	MIT License(MIT)																##
##																					##
##	Copyright(c) 2022 - Present HYBO Inc.											##
##																					##
##	Permission is hereby granted, free of charge, to any person obtaining a copy	##
##	of this software and associated documentation files(the "Software"), to deal	##
##	in the Software without restriction, including without limitation the rights	##
##	to use, copy, modify, merge, publish, distribute, sublicense, and /or sell		##
##	copies of the Software, and to permit persons to whom the Software is			##
##	furnished to do so, subject to the following conditions :						##
##																					##
##	The above copyright notice and this permission notice shall be included in all	##
##	copies or substantial portions of the Software.									##
##																					##
##	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR		##
##	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,		##
##	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE		##
##	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER			##
##	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,	##
##	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE	##
##	SOFTWARE.																		##
######################################################################################


class _PACKET:
    def __init__(self):
        """
		iTFS PACKET STRUCTURE DESCRIPTIONS

		idx	field	SIZE	VALUE
		0	STX0	1		0xA5
		1	STX1	1		0x5A
		2	ID		2		0x0000 for img_data, ...
		4	LEN		2		N
		6	PAYLOAD	N 	
		N+6	ETX0	1		0xA5
		N+7	ETX1	1		0x5A		 	
		TOTAL SIZE	N+8
		"""

        # Packet constant - general
        self.stx0 = 0xA5
        self.stx1 = 0x5A
        self.stx_len = 2
        self.id_len = 2
        self.len_len = 2
        self.header_len = 6
        self.etx0 = 0xA5
        self.etx1 = 0x5A
        self.overheader_len = 8

        # Packet constant - Sensor -> PC
        self.img_data_id = 0x0000
        self.img_data_len = 1282

        self.status_id = 0x0010
        self.status_len = 28

        self.status_full_id = 0x0011
        self.status_full_len = 312

        self.info_id = 0x0020
        self.info_len = 110

        # Packet constant - PC -> Sensor
        self.cmd_id = 0x0030
        self.cmd_len = 4

        self.cmd_sync = 0x0000
        self.cmd_select = 0x0001

        self.cmd_measure = 0x0100
        self.cmd_pause = 0x0101
        self.cmd_reboot = 0x0102
        self.cmd_store = 0x0103

        self.cmd_reset_factory = 0x0200
        self.cmd_reset_log = 0x0201

        self.cmd_read_info = 0x0300
        self.cmd_read_log = 0x0301
        self.cmd_read_depth = 0x0302
        self.cmd_read_tf = 0x0303

        self.cmd_redirect = 0x0400

        self.cmd_lock = 0x0500
        self.cmd_unlock = 0x0501

        # packet cmd request info
        self.cmd_read_info_pack = [
            0xA5,
            0x5A,
            0x30,
            0x00,
            0x04,
            0x00,
            0x00,
            0x03,
            0x00,
            0x00,
            0xA5,
            0x5A,
        ]

class _INFO:
    def __init__(self):
        self.sensor_sn = None
        self.sensor_hw_id = None
        self.sensor_fw_ver = None
        self.sensor_fw_date = None
        self.sensor_fw_time = None
        self.sensor_calib_id = None
        self.capture_mode = None # Valid value: 1,2,3
        self.capture_row = None
        self.capture_period = None
        self.capture_shutter = None # Max:[600,60,6,2,600] Min:[2,2,2,2,2]
        self.capture_limit = None
        self.data_output = None
        self.data_baud = None
        self.data_sensor_ip = None
        self.data_dest_ip = None
        self.data_subnet = None
        self.data_gateway = None
        self.data_port = None
        self.sync = None
        self.sync_delay = None
        self.arb = None
        self.arb_timeout = None
        self.lock = None
        
    def __eq__(self, other): # Equal to check configuration
        for key in self.__dict__:
            try:
                if getattr(self, key) != getattr(other, key):
                    return False
            except:
                return False
        return True

class iTFS:
    def __init__(self):
        # Constant
        self.max_col = 320
        self.max_row = 160
        self.gray_row = 240

        self.lidar_data_port = 4905
        self.lidar_config_port = 4906
        self.user_data_port = 7256
        self.user_config_port = 7257

        self.packet = _PACKET()

        # Variable
        self.sockData = None
        self.sockConfig = None

        self.read_is_ready = False  # [R] True when read socket is opened
        self.send_is_ready = False  # [R] True when send socket is opened
        self.send_is_busy = False  # [R] True when Send_run is in operation
        self.send_msg_box_is_empty = True  # [W] Make it True when you want to send message, automatically becomes False when Send is called.
        self.send_msg_box = ([])  # [W] Insert msg what you want to send, automatically becomes empty [] when Send is called.
        self.exit = False

        self.pc_has_sensor_info = False  # [R]

        self.broadcast_ip = "192.168.5.255"
        self.received_ip = ""  # IP address that current object listening
        self.listening_ip = ""  # IP address that current object listening
        self.listening_port = self.user_data_port

        self.data_semaphore = Semaphore(1)
        self.send_semaphore = Semaphore(1)

        self.status_capture_mode = 0
        self.status_capture_frame = 0
        self.status_sensor_sn = 0
        self.status_sensor_time_th = 0
        self.status_sensor_time_tl = 0
        self.status_sensor_frame_status = 0
        self.status_sensor_temp_rx = 0
        self.status_sensor_temp_core = 0
        self.status_sensor_vcsel_level = 0
        self.status_sensor_power_level = 0
        self.status_sensor_warning = 0
        
        self.info = _INFO()
        self.info.capture_row = self.max_row

        self.received_data = np.zeros((2 * self.max_row, self.max_col), dtype=np.uint16)
        self.output_data = np.zeros((2 * self.max_row, self.max_col), dtype=np.uint16)
        
        # Variables for DEV
        self.cumulated_row = 0 # cumulated receivced number of row, only for DEV to check whole successful shot without crash
        self.previous_frame = None # frame number just before capturing
        self.successful_frame_flag = False # Should be set true when cumulated row reach maximum row
        
        # Thread Section
        self.read_thread = Thread(target=self.Read_run)
        self.send_thread = Thread(target=self.Send_run)
        self.read_thread.start()
        self.send_thread.start()
        
        
        
        
    def Try_exit(self):
        self.exit = True

    def Copy_data(self):
        self.data_semaphore.acquire()
        self.output_data = np.copy(self.received_data)
        self.data_semaphore.release()

    def Get_data(self):
        self.data_semaphore.acquire()
        output = self.output_data
        self.data_semaphore.release()
        return output

    def Send(self):
        self.send_semaphore.acquire()
        self.sockConfig.sendto(bytearray(self.send_msg_box), (self.listening_ip, self.lidar_config_port))
        self.send_semaphore.release()
        self.send_msg_box = []
        self.send_msg_box_is_empty = True

    def Set_listening_IP(self, ip_address):
        self.listening_ip = ip_address

    def Read_run(self):
        # Create sensor->PC UDP sockets
        self.sockData = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockData.bind(("", self.user_data_port))
        self.sockData.settimeout(5.0)

        self.read_is_ready = True

        # Checking received IP address
        # !!! self.listening_ip MUST BE SETTED IN MAIN PROGRAM !!!
        while self.listening_ip == "" and self.exit is False:
            time.sleep(0.2)
            try:
                data, temp_ip = self.sockData.recvfrom(2000)
                valid = (
                    data[0] == self.packet.stx0
                    and data[1] == self.packet.stx1
                    and data[len(data) - 2] == self.packet.etx0
                    and data[len(data) - 1] == self.packet.etx1
                )
                if valid:
                    self.received_ip = temp_ip[0]
                    is_received = True
            except:
                is_received = False

        print("[MESSAGE] iTFS::LiDAR is ready. IP address is " + self.listening_ip)

        while self.exit == False:
            # Try to read UDP packet from the socket
            try:
                data, temp_ip = self.sockData.recvfrom(2000)
                is_received = True
            except:
                is_received = False

            if is_received == False:
                # Fail to receive UDP data #
                print(
                    "[MESSAGE] iTFS::LiDAR recvfrom: No data has been received in the last 5 seconds"
                )
            else:
                # Success to receive UDP data #

                # Check the data == ilidar_packet or not
                # img_data packet
                if (
                    (len(data) - self.packet.overheader_len) == self.packet.img_data_len
                ) and (self.pc_has_sensor_info == True):
                    row = data[6]
                    frame = data[7]
                    mode = (frame & 0xC0) >> 6
                    frame = frame & 0x3F
                    valid = data[0] == self.packet.stx0

                    if mode == 1:
                        if row < self.info.capture_row:
                            row_data = struct.unpack("<" + "H" * 640, data[8:1288])
                            row_data = np.reshape(row_data, (2, 320))

                            self.received_data[(2 * row) : (2 * row + 2), :] = row_data

                            if row == (self.info.capture_row - 1):
                                self.Copy_data()

                    elif mode == 2:
                        if row < (self.info.capture_row / 2):
                            row_data = struct.unpack("<" + "H" * 640, data[8:1288])
                            row_data = np.reshape(row_data, (2, 320))
                            kron_row_data = np.kron(
                                row_data, np.ones((2, 1), dtype=int)
                            )

                            self.received_data[
                                (4 * row) : (4 * row + 4), :
                            ] = kron_row_data

                            if row == (self.info.capture_row / 2 - 1):
                                self.Copy_data()

                    elif mode == 3:
                        if row < (self.info.capture_row / 4):
                            row_data = struct.unpack("<" + "H" * 640, data[8:1288])
                            row_data = np.reshape(row_data, (4, 160))
                            kron_row_data = np.kron(
                                row_data, np.ones((2, 2), dtype=int)
                            )

                            self.received_data[
                                (8 * row) : (8 * row + 8), :
                            ] = kron_row_data

                            if row == (self.info.capture_row / 4 - 1):
                                self.Copy_data()

                    else:
                        if row < self.gray_row:
                            row_data = struct.unpack("<" + "H" * 640, data[8:1288])
                            row_data = np.reshape(row_data, (2, 320))

                            self.received_data[(2 * row) : (2 * row + 2), :] = row_data

                            if row == (self.gray_row - 1):
                                self.Copy_data()
                                
                    # Check successful row
                    if self.previous_frame != frame:
                        self.successful_frame_flag = False
                        self.cumulated_row = 1
                    else:
                        self.cumulated_row += 1

                    if mode == 1 and self.cumulated_row == self.info.capture_row:
                        self.cumulated_row = 0 # zero to prevent constantly turn on flag
                        self.successful_frame_flag = True
                    elif mode == 2 and self.cumulated_row == self.info.capture_row/2:
                        self.cumulated_row = 0 # zero to prevent constantly turn on flag
                        self.successful_frame_flag = True
                    elif mode == 3 and self.cumulated_row == self.info.capture_row/4:
                        self.cumulated_row = 0 # zero to prevent constantly turn on flag
                        self.successful_frame_flag = True
                    self.previous_frame = frame

                # status_full packet
                elif (
                    len(data) - self.packet.overheader_len
                ) == self.packet.status_len and self.pc_has_sensor_info:
                    self.status_capture_mode = data[6]
                    self.status_capture_frame = data[7]
                    self.status_sensor_sn = int.from_bytes(
                        data[8:10], byteorder="little", signed=False
                    )
                    self.status_sensor_time_th = int.from_bytes(
                        data[10:18], byteorder="little", signed=False
                    )
                    self.status_sensor_time_tl = int.from_bytes(
                        data[18:20], byteorder="little", signed=False
                    )
                    self.status_sensor_frame_status = int.from_bytes(
                        data[20:22], byteorder="little", signed=False
                    )
                    self.status_sensor_temp_rx = int.from_bytes(
                        data[22:24], byteorder="little", signed=False
                    )
                    self.status_sensor_temp_core = int.from_bytes(
                        data[24:26], byteorder="little", signed=False
                    )
                    self.status_sensor_vcsel_level = int.from_bytes(
                        data[26:28], byteorder="little", signed=False
                    )
                    self.status_sensor_power_level = int.from_bytes(
                        data[28:30], byteorder="little", signed=False
                    )
                    self.status_sensor_warning = int.from_bytes(
                        data[30:34], byteorder="little", signed=False
                    )

                    print(
                        "[MESSAGE] iTFS::LiDAR status | mode ",
                        self.status_capture_mode,
                        " frame ",
                        self.status_capture_frame,
                        " time ",
                        self.status_sensor_time_th * 0.001,
                        " s temp ",
                        self.status_sensor_temp_core * 0.1,
                    )

                # status packet
                elif (
                    len(data) - self.packet.overheader_len
                ) == self.packet.status_full_len and self.pc_has_sensor_info:
                    self.status_capture_mode = data[6]
                    self.status_capture_frame = data[7]
                    self.status_sensor_sn = int.from_bytes(
                        data[8:10], byteorder="little", signed=False
                    )
                    self.status_sensor_time_th = int.from_bytes(
                        data[10:18], byteorder="little", signed=False
                    )
                    self.status_sensor_time_tl = int.from_bytes(
                        data[18:20], byteorder="little", signed=False
                    )
                    self.status_sensor_frame_status = int.from_bytes(
                        data[20:22], byteorder="little", signed=False
                    )
                    self.status_sensor_temp_rx = int.from_bytes(
                        data[22:24], byteorder="little", signed=False
                    )
                    self.status_sensor_temp_core = int.from_bytes(
                        data[24:26], byteorder="little", signed=False
                    )
                    self.status_sensor_vcsel_level = int.from_bytes(
                        data[34:36], byteorder="little", signed=False
                    )
                    self.status_sensor_power_level = int.from_bytes(
                        data[164:166], byteorder="little", signed=False
                    )
                    self.status_sensor_warning = int.from_bytes(
                        data[314:318], byteorder="little", signed=False
                    )

                    print(
                        "[MESSAGE] iTFS::LiDAR status | mode ",
                        self.status_capture_mode,
                        " frame ",
                        self.status_capture_frame,
                        " time ",
                        self.status_sensor_time_th * 0.001,
                        " s temp ",
                        self.status_sensor_temp_core * 0.1,
                    )

                # info packet
                elif (len(data) - self.packet.overheader_len) == self.packet.info_len:
                    self.info.sensor_sn = int.from_bytes(
                        data[6:8], byteorder="little", signed=False
                    )
                    self.info.sensor_hw_id = data[8:38]
                    self.info.sensor_fw_ver = data[38:41]
                    self.info.sensor_fw_date = data[41:53]
                    self.info.sensor_fw_time = data[53:62]
                    self.info.sensor_calib_id = int.from_bytes(
                        data[62:66], byteorder="little", signed=False
                    )

                    self.info.capture_mode = data[66]
                    self.info.capture_row = data[67]
                    self.info.capture_period = int.from_bytes(
                        data[68:70], byteorder="little", signed=False
                    )
                    self.info.capture_shutter = [
                        int.from_bytes(data[70:72], byteorder="little", signed=False),
                        int.from_bytes(data[72:74], byteorder="little", signed=False),
                        int.from_bytes(data[74:76], byteorder="little", signed=False),
                        int.from_bytes(data[76:78], byteorder="little", signed=False),
                        int.from_bytes(data[78:80], byteorder="little", signed=False),
                    ]
                    self.info.capture_limit = [
                        int.from_bytes(data[80:82], byteorder="little", signed=False),
                        int.from_bytes(data[82:84], byteorder="little", signed=False),
                    ]

                    self.info.data_output = data[84]

                    self.info.arb = data[85]

                    self.info.data_baud = int.from_bytes(
                        data[86:90], byteorder="little", signed=False
                    )
                    self.info.data_sensor_ip = data[90:94]
                    self.info.data_dest_ip = data[94:98]
                    self.info.data_subnet = data[98:102]
                    self.info.data_gateway = data[102:106]
                    self.info.data_port = int.from_bytes(
                        data[106:108], byteorder="little", signed=False
                    )

                    self.info.sync = data[108]

                    self.info.lock = data[109]

                    self.info.sync_delay = int.from_bytes(
                        data[110:112], byteorder="little", signed=False
                    )
                    self.info.arb_timeout = int.from_bytes(
                        data[112:116], byteorder="little", signed=False
                    )

                    print("[MESSAGE] iTFS::LiDAR info")
                    print(
                        "SN ",
                        self.info.sensor_sn,
                        " mode ",
                        self.info.capture_mode,
                        " rows ",
                        self.info.capture_row,
                        " period ",
                        self.info.capture_period,
                    )
                    print("shutter ", self.info.capture_shutter)
                    print("limit ", self.info.capture_limit)
                    print(
                        "ip ",
                        self.info.data_sensor_ip[0],
                        ".",
                        self.info.data_sensor_ip[1],
                        ".",
                        self.info.data_sensor_ip[2],
                        ".",
                        self.info.data_sensor_ip[3],
                        ".",
                    )
                    print(
                        "dest ",
                        self.info.data_dest_ip[0],
                        ".",
                        self.info.data_dest_ip[1],
                        ".",
                        self.info.data_dest_ip[2],
                        ".",
                        self.info.data_dest_ip[3],
                        ".",
                    )
                    print(
                        "sync ",
                        self.info.sync,
                        " syncBase ",
                        self.info.sync_delay,
                        " autoReboot ",
                        self.info.arb,
                        " autoRebootTick ",
                        self.info.arb_timeout,
                    )
                    print(
                        "FW version: V",
                        self.info.sensor_fw_ver[2],
                        ".",
                        self.info.sensor_fw_ver[1],
                        ".",
                        self.info.sensor_fw_ver[0],
                    )
                    print()

                    self.pc_has_sensor_info = True

        self.pc_has_sensor_info = False

        # Clean everything
        self.read_is_ready = False
        self.sockData.close()

    def Send_run(self):
        # Open PC->Sensor config(send) socket
        self.sockConfig = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockConfig.bind(("", self.user_config_port))
        self.sockConfig.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.send_is_ready = True
        self.send_msg_box_is_empty = True
        self.send_msg_box = []

        # Waiting until listening ip is determined
        while self.listening_ip == "" and self.exit == False:
            time.sleep(0.2)

        # Really running Send_run
        while self.exit == False:
            # Request sensor information every 1.0 second if there is no sensor info
            while (
                self.pc_has_sensor_info == False
                and self.exit == False
                and self.send_msg_box_is_empty == True
            ):
                self.send_msg_box = self.packet.cmd_read_info_pack
                self.send_msg_box_is_empty = False
                self.Send()
                
            if self.send_msg_box_is_empty == False:
                self.Send()
                self.successful_frame_flag = False
                self.cumulated_row = 1
            time.sleep(0.2)

        # Clean everything
        self.send_is_ready = False
        self.send_is_busy = False
        self.send_msg_box_is_empty = True
        self.send_msg_box = []
        self.sockConfig.close()

    def send_info(self, new_info):
        buffer = np.zeros(
            self.packet.overheader_len + self.packet.info_len, dtype=np.uint8
        )
        buffer[0] = self.packet.stx0
        buffer[1] = self.packet.stx1
        buffer[2] = (self.packet.info_id >> 0) & 0xFF
        buffer[3] = (self.packet.info_id >> 8) & 0xFF
        buffer[4] = (self.packet.info_len >> 0) & 0xFF
        buffer[5] = (self.packet.info_len >> 8) & 0xFF
        buffer[self.packet.header_len + self.packet.info_len] = self.packet.etx0
        buffer[self.packet.header_len + self.packet.info_len + 1] = self.packet.etx1

        offset = 6
        buffer[0 + offset] = self.status_sensor_sn % 256
        buffer[1 + offset] = self.status_sensor_sn / 256

        for _i in range(0, (30 + 3 + 12 + 9)):
            buffer[2 + _i + offset] = 0

        buffer[60 + offset] = new_info.capture_mode
        buffer[61 + offset] = new_info.capture_row
        buffer[62 + offset] = (new_info.capture_period >> 0) & 0xFF
        buffer[63 + offset] = (new_info.capture_period >> 8) & 0xFF
        buffer[64 + offset] = (new_info.capture_shutter[0] >> 0) & 0xFF
        buffer[65 + offset] = (new_info.capture_shutter[0] >> 8) & 0xFF
        buffer[66 + offset] = (new_info.capture_shutter[1] >> 0) & 0xFF
        buffer[67 + offset] = (new_info.capture_shutter[1] >> 8) & 0xFF
        buffer[68 + offset] = (new_info.capture_shutter[2] >> 0) & 0xFF
        buffer[69 + offset] = (new_info.capture_shutter[2] >> 8) & 0xFF
        buffer[70 + offset] = (new_info.capture_shutter[3] >> 0) & 0xFF
        buffer[71 + offset] = (new_info.capture_shutter[3] >> 8) & 0xFF
        buffer[72 + offset] = (new_info.capture_shutter[4] >> 0) & 0xFF
        buffer[73 + offset] = (new_info.capture_shutter[4] >> 8) & 0xFF
        buffer[74 + offset] = (new_info.capture_limit[0] >> 0) & 0xFF
        buffer[75 + offset] = (new_info.capture_limit[0] >> 8) & 0xFF
        buffer[76 + offset] = (new_info.capture_limit[1] >> 0) & 0xFF
        buffer[77 + offset] = (new_info.capture_limit[1] >> 8) & 0xFF

        buffer[78 + offset] = new_info.data_output

        buffer[79 + offset] = new_info.arb

        buffer[80 + offset] = (new_info.data_baud >> 0) & 0xFF
        buffer[81 + offset] = (new_info.data_baud >> 8) & 0xFF
        buffer[82 + offset] = (new_info.data_baud >> 16) & 0xFF
        buffer[83 + offset] = (new_info.data_baud >> 24) & 0xFF
        buffer[84 + offset] = new_info.data_sensor_ip[0]
        buffer[85 + offset] = new_info.data_sensor_ip[1]
        buffer[86 + offset] = new_info.data_sensor_ip[2]
        buffer[87 + offset] = new_info.data_sensor_ip[3]
        buffer[88 + offset] = new_info.data_dest_ip[0]
        buffer[89 + offset] = new_info.data_dest_ip[1]
        buffer[90 + offset] = new_info.data_dest_ip[2]
        buffer[91 + offset] = new_info.data_dest_ip[3]
        buffer[92 + offset] = new_info.data_subnet[0]
        buffer[93 + offset] = new_info.data_subnet[1]
        buffer[94 + offset] = new_info.data_subnet[2]
        buffer[95 + offset] = new_info.data_subnet[3]
        buffer[96 + offset] = new_info.data_gateway[0]
        buffer[97 + offset] = new_info.data_gateway[1]
        buffer[98 + offset] = new_info.data_gateway[2]
        buffer[99 + offset] = new_info.data_gateway[3]
        buffer[100 + offset] = (new_info.data_port >> 0) & 0xFF
        buffer[101 + offset] = (new_info.data_port >> 8) & 0xFF

        buffer[102 + offset] = new_info.sync
        buffer[103 + offset] = 0

        buffer[104 + offset] = (new_info.sync_delay >> 0) & 0xFF
        buffer[105 + offset] = (new_info.sync_delay >> 8) & 0xFF

        buffer[106 + offset] = (new_info.arb_timeout >> 0) & 0xFF
        buffer[107 + offset] = (new_info.arb_timeout >> 8) & 0xFF
        buffer[108 + offset] = (new_info.arb_timeout >> 16) & 0xFF
        buffer[109 + offset] = (new_info.arb_timeout >> 24) & 0xFF

        self.send_msg_box = buffer
        self.send_msg_box_is_empty = False
        
        
        # Set pc_has_sensor_info = False to check config is sent well
        # It will be automatically checked by read and send thread.
        self.pc_has_sensor_info = False


    def send_store(self):
        buffer = np.zeros(
            self.packet.overheader_len + self.packet.cmd_len, dtype=np.uint8
        )
        buffer[0] = self.packet.stx0
        buffer[1] = self.packet.stx1
        buffer[2] = (self.packet.cmd_id >> 0) & 0xFF
        buffer[3] = (self.packet.cmd_id >> 8) & 0xFF
        buffer[4] = (self.packet.cmd_len >> 0) & 0xFF
        buffer[5] = (self.packet.cmd_len >> 8) & 0xFF
        buffer[6] = (self.packet.cmd_store >> 0) & 0xFF
        buffer[7] = (self.packet.cmd_store >> 8) & 0xFF
        buffer[self.packet.header_len + self.packet.cmd_len] = self.packet.etx0
        buffer[self.packet.header_len + self.packet.cmd_len + 1] = self.packet.etx1

        self.send_msg_box = buffer
        self.send_msg_box_is_empty = False
        
        
        # Set pc_has_sensor_info = False to check config is sent well
        # It will be automatically checked by read and send thread.
        self.pc_has_sensor_info = False
        
        
    def send_reboot(self):
        buffer = np.zeros(
            self.packet.overheader_len + self.packet.cmd_len, dtype=np.uint8
        )
        buffer[0] = self.packet.stx0
        buffer[1] = self.packet.stx1
        buffer[2] = (self.packet.cmd_id >> 0) & 0xFF
        buffer[3] = (self.packet.cmd_id >> 8) & 0xFF
        buffer[4] = (self.packet.cmd_len >> 0) & 0xFF
        buffer[5] = (self.packet.cmd_len >> 8) & 0xFF
        buffer[6] = (self.packet.cmd_reboot >> 0) & 0xFF
        buffer[7] = (self.packet.cmd_reboot >> 8) & 0xFF
        buffer[self.packet.header_len + self.packet.cmd_len] = self.packet.etx0
        buffer[self.packet.header_len + self.packet.cmd_len + 1] = self.packet.etx1

        self.send_msg_box = buffer
        self.send_msg_box_is_empty = False
        
        
        # Set pc_has_sensor_info = False to check config is sent well
        # It will be automatically checked by read and send thread.
        self.pc_has_sensor_info = False

