# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#from detected_points import Detected_Points
import rclpy
from rclpy.node import Node
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField

import os
import time
import struct

import threading
import sys
import signal
import serial

import numpy as np

MSG_AZIMUT_STATIC_HEAT_MAP = 8

global shut_down
shut_down = 0

class TI_mmwave:
    def __init__(self, sdk_version=3.4,  cli_baud=115200,data_baud=921600, num_rx=4, num_tx=3,
                 verbose=False, connect=True, mode=0,cli_loc='COM4',data_loc='COM3', 
                 cfg_path=os.path.dirname(os.path.realpath(__file__)).replace("install/iwr6843aop_pub/lib/python3.8/site-packages/iwr6843aop_pub", "/src/iwr6843aop_pub/cfg_files") + "/" + "xwr68xx_profile_30Hz.cfg"):
        super(TI_mmwave, self).__init__()

        self.connected = False
        self.verbose = verbose
        self.mode = mode
        self.cfg_path = cfg_path

        if connect:
            self.cli_port = serial.Serial(cli_loc, cli_baud)
            self.data_port = serial.Serial(port=data_loc, baudrate=data_baud, timeout=None)
            self.connected = True

        self.sdk_version = sdk_version
        self.num_rx_ant = num_rx
        self.num_tx_ant = num_tx
        self.num_virtual_ant = num_rx * num_tx
        
        if mode == 0:
            self._initialize()
            time.sleep(1.0)
    
    def _configure_radar(self, config):
        for i in config:
            self.cli_port.write((i + '\n').encode())
            print(self.cli_port.port," send: ",i)
            print(self.cli_port.port,"receive: ",self.cli_port.readline())
            time.sleep(0.01)

    global cfg_path

    def _initialize(self):
        config = [line.rstrip('\r\n') for line in open(self.cfg_path)]
        if self.connected:
            self._configure_radar(config)

        self.config_params = {}  # Initialize an empty dictionary to store the configuration parameters

        for i in config:
            # Split the line
            split_words = i.split(" ")

            # Hard code the number of antennas, change if other configuration is used
            num_rx_ant = 4
            num_tx_ant = 3

            # Get the information about the profile configuration
            if "profileCfg" in split_words[0]:
                start_freq = int(split_words[2])
                idle_time = int(split_words[3])
                ramp_end_time = float(split_words[5])
                freq_slope_const = int(split_words[8])
                num_adc_samples = int(split_words[10])
                num_adc_samples_round_to2 = 1

                while num_adc_samples > num_adc_samples_round_to2:
                    num_adc_samples_round_to2 = num_adc_samples_round_to2 * 2

                dig_out_sample_rate = int(split_words[11])

            # Get the information about the frame configuration    
            elif "frameCfg" in split_words[0]:
                chirp_start_idx = int(split_words[1])
                chirp_end_idx = int(split_words[2])
                num_loops = int(split_words[3])
                num_frames = int(split_words[4])
                frame_periodicity = float(split_words[5])

        # Combine the read data to obtain the configuration parameters
        num_chirps_per_frame = (chirp_end_idx - chirp_start_idx + 1) * num_loops
        self.config_params["numDopplerBins"]        = num_chirps_per_frame / num_tx_ant
        self.config_params["numRangeBins"]          = num_adc_samples_round_to2
        self.config_params["rangeResolutionMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (2 * freq_slope_const * 1e12 * num_adc_samples)
        self.config_params["rangeIdxToMeters"]      = (3e8 * dig_out_sample_rate * 1e3) / (2 * freq_slope_const * 1e12 * self.config_params["numRangeBins"])
        self.config_params["dopplerResolutionMps"]  = 3e8 / (2 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * self.config_params["numDopplerBins"] * num_tx_ant)
        self.config_params["maxRange"]              = (300 * 0.9 * dig_out_sample_rate) / (2 * freq_slope_const * 1e3)
        self.config_params["maxVelocity"]           = 3e8 / (4 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * num_tx_ant)

    def stop(self):
        print("Shutting down sensor")
        self.cli_port.write('sensorStop\n'.encode())

    def close(self):
        """
        End connection between radar and machine

        Returns:
            None

        """
        print("Shutting down sensor")
        self.cli_port.write('sensorStop\n'.encode())
        self.cli_port.close()
        self.data_port.close()

    def _read_buffer(self):
        """
        Returns:

        """
        byte_buffer = self.data_port.read(self.data_port.in_waiting)

        return byte_buffer
        
    def _read_nsize_buffer(self,lenth):
        """
        Returns:

        """
        byte_buffer = self.data_port.read(lenth)

        return byte_buffer
        
    def _read_until_buffer(self,exp):
        """
        Returns:

        """
        byte_buffer = self.data_port.read_until(exp)

        return byte_buffer

    def _parse_header_frame(self, byte_buffer, idx):
        """
        Parses the byte buffer for the header of the data

        Args:
            byte_buffer: Buffer with TLV data
            idx: Current reading index of the byte buffer

        Returns:
            Tuple [Tuple (int), int]

        """
        (version, length, platform, frame_num, cpu_cycles, num_obj, num_tlvs, subframe_num), idx = self._unpack(byte_buffer=byte_buffer, idx=idx, items=8, form='I')

        return (version, length, platform, frame_num, cpu_cycles, num_obj, num_tlvs, subframe_num), idx
    
    def _parse_header_tlv(self, byte_buffer, idx):
        """ 
        Parses the byte buffer for the header of a tlv
        """
        (tlv_type, tlv_length), idx = self._unpack(byte_buffer=byte_buffer, idx=idx, items=2, form='I')
        return (tlv_type, tlv_length), idx

    def _parse_msg_detected_points(self, byte_buffer, idx):
        """ 
        Parses the information of the detected points message
        """
        (x,y,z,vel), idx = self._unpack(byte_buffer, idx, items=4, form='f')
       
        return (x,y,z,vel), idx

    def _parse_msg_detected_points_side_info(self,byte_buffer, idx):
        (snr,noise), idx = self._unpack(byte_buffer, idx, items=2, form='H')
        return (snr,noise),idx

    def _parse_msg_azimut_static_heat_map(self, byte_buffer, idx):
        """ 
        Parses the information of the azimuth heat map
        """
        (imag, real), idx = self._unpack(byte_buffer, idx, items=2, form='H')
        return (imag, real), idx
    
    @staticmethod
    def _unpack(byte_buffer, idx, order='', items=1, form='I'):
        """
        Helper function for parsing binary byte data

        Args:
            byte_buffer: Buffer with data
            idx: Curex in the buffer
            order: Little endian or big endian (for formart control string)
            items: Number of items to be extracted (also for formart control string)
            form: Data type to be extracted (for formart control string)

        Returns:rent ind
            Tuple [Tuple (object), int]

        """
        size = {'H': 2, 'h': 2, 'I': 4, 'Q': 8, 'f': 4} #uint16_t, int16_t, uint23_t, uint64_t, float
        try:
            data = struct.unpack(order + str(items) + form, byte_buffer[idx:idx + (items * size[form])])
            if len(data) == 1:
                data = data[0]
            return data, idx + (items * size[form])
        except:
            return None
#----------------------------------------------------------------------------#
pointcloud_data = []
sem_publish=threading.Semaphore(0)

class iwr6843_interface(object):
    def __init__(self, cfg_path, cli_port, data_port, _data_push_handle, _publish_unlock_handle):
        self.MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        self.ti=TI_mmwave(cli_loc=cli_port,data_loc=data_port,cfg_path=cfg_path)

        self.steam_buffer_n=[]
        self.steam_buffer_p=[]

        self.decode_point=[]
        self.decode_side_data=[]

        self.sem_read=threading.Semaphore(0)
        self.sem_process=threading.Semaphore(0)
        self.sem_swarp_1=threading.Semaphore(0)
        self.sem_swarp_2=threading.Semaphore(0)

        self.uart_read_thread = threading.Thread(target=self.uart_read_handle)
        self.frame_process_thread = threading.Thread(target=self.frame_process_handle)
        self.buff_swarp_thread = threading.Thread(target=self.buff_swarp_handle)

        self.data_push_handle=_data_push_handle
        self.publish_unlock_handle=_publish_unlock_handle

    def uart_read_handle(self):
        byte_buffer=self.ti._read_until_buffer(self.MAGIC_WORD) #read until magic word
        while 1:
            self.steam_buffer_n=self.ti._read_until_buffer(self.MAGIC_WORD)
            
            self.sem_swarp_1.release()
            self.sem_read.acquire()

            global shut_down
            if shut_down == 1:
                print("end1")
                self.ti.close()
                return

    def frame_process_handle(self):
        self.sem_swarp_2.release()
        while 1:
            self.sem_process.acquire()
            steam_buffer=self.steam_buffer_p
            self.sem_swarp_2.release()
            idx=0
            try:
                header_data, idx = self.ti._parse_header_frame(steam_buffer, idx) #read frame header without magic word

                num_tlv=header_data[6]
                num_points=header_data[5]
                
                self.decode_point=np.zeros((num_points,4),dtype=np.float)
                self.decode_side_data=np.zeros((num_points,2),dtype=np.float)

                for i in range(0,num_tlv): #tlv decode
                    (tlv_type, tlv_length), idx = self.ti._parse_header_tlv(steam_buffer, idx) #tlv header
                    
                    if tlv_type==1: #Detected Points
                        for i in range(num_points):
                            ( x, y, z,vel), idx = self.ti._parse_msg_detected_points(steam_buffer, idx)
                            self.decode_point[i][0]=x
                            self.decode_point[i][1]=y
                            self.decode_point[i][2]=z
                            self.decode_point[i][3]=vel
                        
                    elif tlv_type==7: #Side Info for Detected Points
                        for i in range(num_points):
                            (snr,noise), idx = self.ti._parse_msg_detected_points_side_info(steam_buffer, idx)
                            self.decode_side_data[i][0]=snr
                            self.decode_side_data[i][1]=noise

                    elif tlv_type>1051:
                        print("bad tlv:",tlv_type," lenth:",tlv_length,"  ",num_tlv)
                        break

                    else:
                        idx+=tlv_length
                
            except:
                pass
            
            global shut_down
            if shut_down == 1:
                print("end2")
                return

    def buff_swarp_handle(self):
        while 1:
            self.sem_swarp_1.acquire()
            self.sem_swarp_2.acquire()

            self.steam_buffer_p=self.steam_buffer_n
                        
            self.sem_process.release()
            self.sem_read.release()
            self.publish_unlock_handle()
            
            self.data_push_handle(np.column_stack((self.decode_point,self.decode_side_data))[:,:5])

            global shut_down
            if shut_down == 1:
                print("end3")
                
                self.sem_process.release()
                self.sem_read.release()
                self.publish_unlock_handle()
                return
    
    def start(self):
        self.uart_read_thread.start()
        self.frame_process_thread.start()
        self.buff_swarp_thread.start()

    def join(self):
        self.uart_read_thread.join()
        self.frame_process_thread.join()
        self.buff_swarp_thread.join()
            
    def close(self):
        self.ti.close()
#----------------------------------------------------------------------------#
class mmwave_Publisher(Node):
    def __init__(self, topic='iwr6843_pcl', frame_id='iwr6843_frame'):
        super().__init__('iwr6843_pcl_pub')
        
        self.frame_id=frame_id
        self.publisher_ = self.create_publisher(PointCloud2, topic, 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        global pointcloud_data, sem_publish
	
        sem_publish.acquire()
        
        if (not pointcloud_data.shape[0] == 0)  :     
            pointcloud_ = np.column_stack((pointcloud_data[:,0],pointcloud_data[:,1],pointcloud_data[:,2],pointcloud_data[:,3],pointcloud_data[:,4]))
            
            cloud_arr = np.asarray(pointcloud_).astype(np.float32) #  [[x,y,z],[x,y,z],[x,y,z]..]

            pcl_msg = PointCloud2()
            pcl_msg.header = std_msgs.msg.Header()
            pcl_msg.header.stamp = self.get_clock().now().to_msg()
            pcl_msg.header.frame_id = self.frame_id
            pcl_msg.height = 1                 # because unordered cloud
            pcl_msg.width = cloud_arr.shape[0] # number of points in cloud
            pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), # define interpretation of pointcloud message (offset is in bytes, float32 is 4 bytes)
                                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                                PointField(name='v', offset=12, datatype=PointField.FLOAT32, count=1),
                                PointField(name='SNR', offset=16, datatype=PointField.FLOAT32, count=1)]        
            pcl_msg.point_step = cloud_arr.dtype.itemsize*cloud_arr.shape[1] #size of 1 point (float32 * dimensions (3 when xyz))
            pcl_msg.row_step = pcl_msg.point_step*cloud_arr.shape[0] # only 1 row because unordered
            pcl_msg.is_dense = True
            pcl_msg.data = cloud_arr.tostring()

            self.publisher_.publish(pcl_msg)
            #self.get_logger().info('Publishing %s points' % cloud_arr.shape[0] )

#----------------------------------------------------------------------------#
def ctrlc_handler(signum, frame):        #ctrl+c handle
    global shut_down
    shut_down = 1
    time.sleep(0.25)
    print("Exiting")
    exit(1)
#----------------------------------------------------------------------------#
def push_pcl_sen(data):
    global pointcloud_data
    pointcloud_data = data

def pub_unlock_sen():
    global sem_publish
    sem_publish.release()
#----------------------------------------------------------------------------#
def main(argv=None):

    global cfg_path
    global data_port
    global cli_port
    global frame_id
    global topic

    orig_path = os.path.dirname(os.path.realpath(__file__))
    sep = "install/"
    separated = orig_path.split(sep, 1)[0]
    cfg_path = separated + "src/iwr6843aop_pub/cfg_files/xwr68xx_profile_30Hz.cfg"

    cli_port = '/dev/ttyMmr1Cmd'
    data_port = '/dev/ttyMmr1Data'
    frame_id='iwr6843_frame'
    topic='iwr6843_pcl'
    
    if len(sys.argv) > 1:
        cli_port = sys.argv[1]
    if len(sys.argv) > 2:
        data_port = sys.argv[2]
    if len(sys.argv) > 3:
        frame_id = sys.argv[3]
    if len(sys.argv) > 4:
        topic = sys.argv[4]
    #if len(sys.argv) > 5:
    #    cfg_path = sys.argv[5]

    print("cfg_path: ", cfg_path)
    
    signal.signal(signal.SIGINT, ctrlc_handler) #set ctrl-c handle 
    
    iwr6843_interface_node = iwr6843_interface(cfg_path, cli_port, data_port, push_pcl_sen, pub_unlock_sen)
    iwr6843_interface_node.start() 
    
    rclpy.init()
    minimal_publisher = mmwave_Publisher(topic,frame_id)   
    rclpy.spin(minimal_publisher)
    
    #shutdown
    iwr6843_interface_node.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

