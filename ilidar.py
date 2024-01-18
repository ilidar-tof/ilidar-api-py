from threading import Semaphore, Thread
import numpy as np
import socket
import struct

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

class iTFS:
    def __init__(self):
        # Constant
        self.max_col                        = 320
        self.max_row                        = 160
        self.gray_row                       = 240

        self.lidar_data_port		        = 4905
        self.lidar_config_port	            = 4906
        self.user_data_port		            = 7256
        self.user_config_port	            = 7257

        # Packet constant
        self.packet_stx0			        = 0xA5
        self.packet_stx1			        = 0x5A
        self.packet_stx_len		            = 2
        self.packet_id_len				    = 2
        self.packet_len_len				    = 2
        self.packet_header_len			    = 6
        self.packet_etx0				    = 0xA5
        self.packet_etx1				    = 0x5A
        self.packet_overheader_len		    = 8

        self.packet_img_data_id			    = 0x0000
        self.packet_img_data_len		    = 1282

        self.packet_status_id			    = 0x0010
        self.packet_status_len			    = 28

        self.packet_status_full_id		    = 0x0011
        self.packet_status_full_len		    = 312

        self.packet_info_id				    = 0x0020
        self.packet_info_len			    = 110

        self.packet_cmd_id				    = 0x0030
        self.packet_cmd_len				    = 4

        self.packet_cmd_sync			    = 0x0000
        self.packet_cmd_select			    = 0x0001

        self.packet_cmd_measure			    = 0x0100
        self.packet_cmd_pause			    = 0x0101
        self.packet_cmd_reboot			    = 0x0102
        self.packet_cmd_store			    = 0x0103

        self.packet_cmd_reset_factory	    = 0x0200
        self.packet_cmd_reset_log		    = 0x0201

        self.packet_cmd_read_info		    = 0x0300
        self.packet_cmd_read_log		    = 0x0301
        self.packet_cmd_read_depth		    = 0x0302
        self.packet_cmd_read_tf			    = 0x0303

        self.packet_cmd_redirect		    = 0x0400

        self.packet_cmd_lock			    = 0x0500
        self.packet_cmd_unlock			    = 0x0501

        self.packet_cmd_read_info_pack      = [0xA5, 0x5A, 0x30, 0x00, 0x04, 0x00, 0x00, 0x03, 0x00, 0x00, 0xA5, 0x5A]

        # Variable
        self.sockData                    = None
        self.sockConfig                  = None

        self.is_ready                    = False
        self.exit                        = False

        self.broadcast_ip                = '192.168.5.255'
        self.listening_port              = self.user_data_port

        self.data_semaphore              = Semaphore(1)
        self.send_semaphore              = Semaphore(1)

        self.ilidar_is_ready             = False

        self.status_capture_mode         = 0
        self.status_capture_frame        = 0
        self.status_sensor_sn            = 0
        self.status_sensor_time_th       = 0
        self.status_sensor_time_tl       = 0
        self.status_sensor_frame_status  = 0
        self.status_sensor_temp_rx       = 0
        self.status_sensor_temp_core     = 0
        self.status_sensor_vcsel_level   = 0
        self.status_sensor_power_level   = 0
        self.status_sensor_warning       = 0

        self.info_sensor_sn              = None
        self.info_sensor_hw_id           = None
        self.info_sensor_fw_ver          = None
        self.info_sensor_fw_date         = None
        self.info_sensor_fw_time         = None
        self.info_sensor_calib_id        = None
        self.info_capture_mode           = None
        self.info_capture_row            = self.max_row
        self.info_capture_period         = None
        self.info_capture_shutter        = None
        self.info_capture_limit          = None
        self.info_data_output            = None
        self.info_data_baud              = None
        self.info_data_sensor_ip         = None
        self.info_data_dest_ip           = None
        self.info_data_subnet            = None
        self.info_data_gateway           = None
        self.info_data_port              = None
        self.info_sync                   = None
        self.info_sync_delay             = None
        self.info_arb                    = None
        self.info_arb_timeout            = None
        self.info_lock                   = None

        self.received_data               = np.zeros((2 * self.max_row, self.max_col), dtype=np.uint16)
        self.output_data                 = np.zeros((2 * self.max_row, self.max_col), dtype=np.uint16)

        self.read_thread                 = None

        self.read_thread = Thread(target=self.Read_run)
        self.read_thread.start()

    def Ready(self):
        return self.is_ready

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
    
    def Send(self, send_data, sender):
        self.send_semaphore.acquire()
        self.sockConfig.sendto(bytearray(send_data), (sender, self.lidar_config_port))
        self.send_semaphore.release()

    def Read_run(self):
        # Create UDP sockets
        self.sockData = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockData.bind( ('', self.user_data_port))
        self.sockData.settimeout(5.0)

        self.sockConfig = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockConfig.bind( ('', self.user_config_port))
        self.sockConfig.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # Update ready status
        self.is_ready = True
        print('[MESSAGE] iTFS::LiDAR is ready.')

        while self.exit is False:
            # Try to read UDP packet from the socket
            try:
                data, sender = self.sockData.recvfrom(2000)
                is_received = True
            except:
                is_received = False

            if is_received is False:
                # Fail to receive UDP data #
                print('[MESSAGE] iTFS::LiDAR recvfrom: No data has been received in the last 5 seconds')
            else:
                # Success to receive UDP data #

                # Check the data is ilidar_packet or not
                # img_data packet
                if ((len(data) - self.packet_overheader_len) == self.packet_img_data_len) and (self.ilidar_is_ready):
                    row     = data[6]
                    frame   = data[7]
                    mode    = (frame & 0xC0) >> 6
                    frame   = frame & 0x3F

                    if mode == 1:
                        if row < self.info_capture_row:
                            row_data = struct.unpack('<' + 'H' * 640, data[8:1288])
                            row_data = np.reshape(row_data, (2, 320))

                            self.received_data[(2*row):(2*row + 2), :] = row_data

                            if row == (self.info_capture_row - 1):
                                self.Copy_data()

                    elif mode == 2:
                        if row < (self.info_capture_row / 2):
                            row_data = struct.unpack('<' + 'H' * 640, data[8:1288])
                            row_data = np.reshape(row_data, (2, 320))
                            kron_row_data = np.kron(row_data, np.ones((2, 1), dtype=int))

                            self.received_data[(4*row):(4*row + 4), :] = kron_row_data

                            if row == (self.info_capture_row/2 - 1):
                                self.Copy_data()
                                
                    elif mode == 3:
                        if row < (self.info_capture_row / 4):
                            row_data = struct.unpack('<' + 'H' * 640, data[8:1288])
                            row_data = np.reshape(row_data, (4, 160))
                            kron_row_data = np.kron(row_data, np.ones((2, 2), dtype=int))

                            self.received_data[(8*row):(8*row + 8), :] = kron_row_data

                            if row == (self.info_capture_row/4 - 1):
                                self.Copy_data()
                                
                    else:
                        if row < self.gray_row:
                            row_data = struct.unpack('<' + 'H' * 640, data[8:1288])
                            row_data = np.reshape(row_data, (2, 320))

                            self.received_data[(2*row):(2*row + 2), :] = row_data

                            if row == (self.gray_row - 1):
                                self.Copy_data()

                # status_full packet
                elif ((len(data) - self.packet_overheader_len) == self.packet_status_full_len):
                    self.status_capture_mode        = data[6]
                    self.status_capture_frame       = data[7]
                    self.status_sensor_sn           = int.from_bytes(data[8:10], byteorder='little', signed=False)
                    self.status_sensor_time_th      = int.from_bytes(data[10:18], byteorder='little', signed=False)
                    self.status_sensor_time_tl      = int.from_bytes(data[18:20], byteorder='little', signed=False)
                    self.status_sensor_frame_status = int.from_bytes(data[20:22], byteorder='little', signed=False)
                    self.status_sensor_temp_rx      = int.from_bytes(data[22:24], byteorder='little', signed=False)
                    self.status_sensor_temp_core    = int.from_bytes(data[24:26], byteorder='little', signed=False)
                    self.status_sensor_vcsel_level  = int.from_bytes(data[26:28], byteorder='little', signed=False)
                    self.status_sensor_power_level  = int.from_bytes(data[28:30], byteorder='little', signed=False)
                    self.status_sensor_warning      = int.from_bytes(data[30:34], byteorder='little', signed=False)

                    print('[MESSAGE] iTFS::LiDAR status | mode ', self.status_capture_mode, 
                          ' frame ', self.status_capture_frame,
                          ' time ', self.status_sensor_time_th * 0.001,
                          ' s temp ', self.status_sensor_temp_core * 0.1)
                    
                    if self.ilidar_is_ready is False:
                        self.send_semaphore.acquire()
                        self.sockConfig.sendto(bytearray(self.packet_cmd_read_info_pack), (sender[0], self.lidar_config_port))
                        self.send_semaphore.release()

                # status packet
                elif ((len(data) - self.packet_overheader_len) == self.packet_status_len):
                    self.status_capture_mode        = data[6]
                    self.status_capture_frame       = data[7]
                    self.status_sensor_sn           = int.from_bytes(data[8:10], byteorder='little', signed=False)
                    self.status_sensor_time_th      = int.from_bytes(data[10:18], byteorder='little', signed=False)
                    self.status_sensor_time_tl      = int.from_bytes(data[18:20], byteorder='little', signed=False)
                    self.status_sensor_frame_status = int.from_bytes(data[20:22], byteorder='little', signed=False)
                    self.status_sensor_temp_rx      = int.from_bytes(data[22:24], byteorder='little', signed=False)
                    self.status_sensor_temp_core    = int.from_bytes(data[24:26], byteorder='little', signed=False)
                    self.status_sensor_vcsel_level  = int.from_bytes(data[34:36], byteorder='little', signed=False)
                    self.status_sensor_power_level  = int.from_bytes(data[164:166], byteorder='little', signed=False)
                    self.status_sensor_warning      = int.from_bytes(data[314:318], byteorder='little', signed=False)

                    print('[MESSAGE] iTFS::LiDAR status | mode ', self.status_capture_mode, 
                          ' frame ', "{:.2d}".format(self.status_capture_frame),
                          ' time ', "{:.03f}".format(self.status_sensor_time_th * 0.001),
                          ' s temp ', "{:.03f}".format(self.status_sensor_temp_core * 0.1))
                    
                    if self.ilidar_is_ready is False:
                        self.send_semaphore.acquire()
                        self.sockConfig.sendto(bytearray(self.packet_cmd_read_info_pack), (sender[0], self.lidar_config_port))
                        self.send_semaphore.release()
                        
                # info packet
                elif ((len(data) - self.packet_overheader_len) == self.packet_info_len):
                    self.info_sensor_sn             = int.from_bytes(data[6:8], byteorder='little', signed=False)
                    self.info_sensor_hw_id          = data[8:38]
                    self.info_sensor_fw_ver         = data[38:41]
                    self.info_sensor_fw_date        = data[41:53]
                    self.info_sensor_fw_time        = data[53:62]
                    self.info_sensor_calib_id       = int.from_bytes(data[62:66], byteorder='little', signed=False)
                    
                    self.info_capture_mode          = data[66]
                    self.info_capture_row           = data[67]
                    self.info_capture_period        = int.from_bytes(data[68:70], byteorder='little', signed=False)
                    self.info_capture_shutter       = [int.from_bytes(data[70:72], byteorder='little', signed=False),
                                                       int.from_bytes(data[72:74], byteorder='little', signed=False),
                                                       int.from_bytes(data[74:76], byteorder='little', signed=False),
                                                       int.from_bytes(data[76:78], byteorder='little', signed=False),
                                                       int.from_bytes(data[78:80], byteorder='little', signed=False)]
                    self.info_capture_limit         = [int.from_bytes(data[80:82], byteorder='little', signed=False),
                                                       int.from_bytes(data[82:84], byteorder='little', signed=False)]
                    
                    self.info_data_output           = data[84]

                    self.info_arb                   = data[85]

                    self.info_data_baud             = int.from_bytes(data[86:90], byteorder='little', signed=False)
                    self.info_data_sensor_ip        = data[90:94]
                    self.info_data_dest_ip          = data[94:98]
                    self.info_data_subnet           = data[98:102]
                    self.info_data_gateway          = data[102:106]
                    self.info_data_port             = int.from_bytes(data[106:108], byteorder='little', signed=False)

                    self.info_sync                  = data[108]

                    self.info_lock                  = data[109]

                    self.info_sync_delay            = int.from_bytes(data[110:112], byteorder='little', signed=False)
                    self.info_arb_timeout           = int.from_bytes(data[112:116], byteorder='little', signed=False)
                    
                    print('[MESSAGE] iTFS::LiDAR info')
                    print('SN ', self.info_sensor_sn, ' mode ', self.info_capture_mode, ' rows ', self.info_capture_row, ' period ', self.info_capture_period)
                    print('shutter ', self.info_capture_shutter)
                    print('limit ', self.info_capture_limit)
                    print('ip ', self.info_data_sensor_ip)
                    print('dest ', self.info_data_dest_ip)
                    print('sync ', self.info_sync, ' syncBase ', self.info_sync_delay, ' autoReboot ', self.info_arb, ' autoRebootTick ', self.info_arb_timeout)
                    print('FW version: V', self.info_sensor_fw_ver[2], '.', self.info_sensor_fw_ver[1],  '.', self.info_sensor_fw_ver[0])
                    print()

                    self.ilidar_is_ready = True
        
        self.ilidar_is_ready = False
        
        # Clean everything
        self.sockConfig.close()
        self.sockData.close()

        self.is_ready = False

