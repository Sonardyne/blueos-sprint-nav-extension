# Copyright 2024 Sonardyne

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import constants
import logging
import math
import platform
from pymavlink import mavutil
from pyubx2 import UBXReader, NMEA_PROTOCOL, UBX_PROTOCOL
import serial
import socket
import threading
import time
from datetime import datetime, timezone

import message_service_pb2 as pb2

class MavlinkHelper():
        def __init__(self):
            self.connection = None
            self.timeSinceBoot = 0
            self.systemProcessor = platform.machine()
            self.correctProcessor = False
            self.isConnected = False
            self.hostIpAddress = ''

            if "arm" in self.systemProcessor:
                self.correctProcessor = True

            self.updateGPSThread = threading.Thread(target=self.updateGPS, args=())

            self.depth = 0
            self.gpsSerialPort = ""
            self.gpsBaudRate = 0
            self.ggaAddress = ""
            self.ggaPort= 0
            self.gpsSerial = None
            self.ggaClient = None
            self.gpsConnected = False
            self.ggaClientConnected = False
            self.exitGNSSEvent = threading.Event()
            self.gpsEpoch = datetime(1980, 1, 6, tzinfo=timezone.utc) # 06/01/1980 GPS epoch
            self.logger = logging.getLogger(__name__)
            logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
            logging.basicConfig(filename='/webui/logs/mavlinkHelper.log', encoding='utf-8', level=logging.INFO)

            file = open("/webui/ip_address.txt")
            for line in file:
                if constants.IP_ADDRESS_SEARCH_STRING in line:
                    split_items = line.split(constants.DELIMITER_EQUALS)
                    self.hostIpAddress = split_items[1].strip()
                    self.logger.info("Set host IP address to " + self.hostIpAddress)

        def __del__(self):
            # Switch GPS_TYPE back to Auto. If Mavlink conenction has been made
            if self.isConnected:
                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'GPS_TYPE',
                    1,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

            if(self.updateGPSThread.is_alive()):
                self.updateGPSThread.join()
        
        def connect(self):
            self.isConnected = False
            if self.correctProcessor:
                self.connection = mavutil.mavlink_connection('tcp:' + self.hostIpAddress +':5777')

                self.connection.wait_heartbeat()

                self.logger.info("Connected to Mavlink on " + self.hostIpAddress)

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'GPS_TYPE',
                    14,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                message = self.connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
   
                if (message['param_id'] == "GPS_TYPE" and  message['param_value'] == 14):
                    self.isConnected = True

                # Need to ensure the user GPS config is not overwritten
                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'GPS_SAVE_CFG',
                    0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'GPS_AUTO_CONFIG',
                    0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                # Enable Vision based messages
                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'VISO_TYPE',
                    1,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'EK3_ENABLE',
                    1,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'EK2_ENABLE',
                    0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'AHRS_EKF_TYPE',
                    3,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'EK3_SRC1_POSXY',
                    3,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'EK3_SRC1_VELXY',
                    3,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'EK3_SRC1_POSZ',
                    1,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                # Set PID parameters for POSITION HOLD mode
                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'PSC_POSXY_P',
                    2.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'PSC_POSZ_P',
                    1.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'PSC_VELXY_D',
                    0.2,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'PSC_VELXY_I',
                    0.02,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'PSC_VELXY_P',
                    4.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'PSC_VELZ_P',
                    5.0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'ACRO_YAW_P',
                    3.38,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'ATC_RAT_YAW_FLTE',
                    0,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                # Set yaw source preference to GPS, in this case the SPRINT-Nav
                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'EK3_SRC1_YAW',
                    2,
                    mavutil.mavlink.MAV_PARAM_TYPE_UINT8
                )

                self.connection.mav.param_set_send(
                    self.connection.target_system, self.connection.target_component,
                    b'ARHS_YAW_P',
                    0.4,
                    mavutil.mavlink.MAV_PARAM_EXT_TYPE_REAL64
                )


        def disconnect(self):
            if(self.isConnected):
                self.connection.close()

            if (self.gpsConnected):
                self.gpsSerial.close()

            if (self.ggaClientConnected):
                self.ggaClient.close()

        def sendPositionUpdate(self, hnavData):

            if self.correctProcessor and self.isConnected:
                self.depth = hnavData.depth_meters

                try:
                    yaw = hnavData.heading_degrees * 1E2

                    if (abs(yaw) < 1E2):
                        # 36000 must be used for North. 0 is not an available measurement value. 
                        # It is used to indicate that the yaw measurement is not to be used
                        yaw = 360 * 1E2

                    secondsSinceEpoch = hnavData.utc - self.gpsEpoch.timestamp()
                    gpsWeek = math.floor(secondsSinceEpoch / constants.WEEK_SECONDS)
                    msFromStartOfWeek = int((secondsSinceEpoch % constants.WEEK_SECONDS) * 1000)

                    self.connection.mav.gps_input_send(
                            int(hnavData.utc * 1E6),  # Timestamp since epoch in microseonds
                            0,  # GPS ID
                            # Flags indicating which fields to ignore 
                            (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_ALT |
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HDOP |
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP  |
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT | 
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                            mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY),
                            msFromStartOfWeek,  # GPS time (milliseconds from start of GPS week)
                            gpsWeek,  # GPS week number
                            3,  # 3-D Fix
                            int(hnavData.latitude_degrees * 1e7),  # Latitude, in degrees * 1E7
                            int(hnavData.longitude_degrees * 1e7),  # Longitude, in degrees * 1E7
                            0,  # Altitude above sea level (N/A)
                            65535,  # GPS HDOP horizontal dilution of position in m. Unknown so set to uint16 max
                            65535,  # GPS VDOP vertical dilution of position in m. Unknown so set to uint16 max
                            65535,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame. Ignored
                            65535,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame. Ignored
                            65535,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame. Ignored
                            65535,  # GPS speed accuracy in m/s. Need to convert from mm/s. Ignored
                            65535,  # GPS horizontal accuracy in m. Ignored
                            0,  # GPS vertical accuracy in m
                            12,   # Number of satellites visible
                            int(yaw) # Yaw, in degrees * 1E2
                        )
                except:
                    self.logger.error("Cannot send GPS Input message to ROV")

        def sendPositionDeltaUpdate(self, currentHnavData, previousHnavData):

            if self.correctProcessor and self.isConnected:
                timeDeltaMicroseconds = int((currentHnavData.utc - previousHnavData.utc) * 1E6) # Time difference in microseonds
                timeDeltaSeconds = currentHnavData.utc - previousHnavData.utc # Time difference in seconds
                pitchDelta = math.radians(currentHnavData.pitch_degrees - previousHnavData.pitch_degrees)
                rollDelta = math.radians(currentHnavData.roll_degrees - previousHnavData.roll_degrees)
                headingDelta = math.radians(currentHnavData.heading_degrees - previousHnavData.heading_degrees)
                rotationDelta = [pitchDelta, rollDelta, headingDelta]

                forwardPosDelta = currentHnavData.forward_velocity_meters_per_second *  timeDeltaSeconds
                starboardPosDelta = currentHnavData.starboard_velocity_meters_per_second * timeDeltaSeconds
                downwardPosDelta = currentHnavData.downward_velocity_meters_per_second * timeDeltaSeconds

                try:
                    self.connection.mav.vision_position_delta_send(
                        0,
                        timeDeltaMicroseconds, # Time difference in microseonds
                        [pitchDelta, rollDelta, headingDelta], # Change in X Y Z rotation,
                        [forwardPosDelta, starboardPosDelta, downwardPosDelta], # Change in X Y Z position,
                        100
                    )
                except Exception as e:
                    self.logger.error("Cannot send position delta message to ROV")

        def setGPS(self, port, baudRate):
            self.gpsSerialPort = port
            self.gpsBaudRate = baudRate
            success = False

            try:
                self.gpsSerial = serial.Serial(self.gpsSerialPort, self.gpsBaudRate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.1)
                success = True
                self.gpsConnected = True
            except:
                success = False
                self.gpsConnected = False
            
            return success

        def setGGA(self, address, port):
            self.ggaAddress = address
            self.ggaPort = port

            if (self.ggaClientConnected):
                self.ggaClient.close()

            self.ggaClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            success = False
            self.exitGNSSEvent.set()

            try:
                self.ggaClient.connect((self.ggaAddress, self.ggaPort))
                self.ggaClientConnected = True
                self.restartGPS()
                success = True
            except:
                success = False
                self.ggaClientConnected = False
            
            return success

        def startGPS(self):
            self.updateGPSThread.start()

        def restartGPS(self):
            if(self.updateGPSThread.is_alive()):
                self.updateGPSThread.join()

            self.updateGPSThread = threading.Thread(target=self.updateGPS, args=())
            self.startGPS()

        def updateGPS(self):
            if self.correctProcessor:            
                self.exitGNSSEvent = threading.Event()

                while (not self.exitGNSSEvent.is_set()):
                    try:
                        ubr = UBXReader(self.gpsSerial, False, protfilter=NMEA_PROTOCOL)
                        raw_data, parsed_data = ubr.read()

                        if parsed_data is not None:
                            line = raw_data.decode()
                            if line.startswith("$GPGGA") and parsed_data.numSV >= 5:
                                self.ggaClient.sendall(raw_data)
                            if line.startswith("$GPZDA"):
                                self.ggaClient.sendall(raw_data)
                    except:
                        self.logger.error("Cannot read GPS stream")