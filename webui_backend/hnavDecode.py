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
from crc import Calculator, Configuration
import logging
import struct
import threading

import message_service_pb2 as pb2

from utils import bitshift16, bitshift32, bitshift64, twosComplement
from threadSafeQueue import Queue

class HNavDecode():

    def __init__(self):
        self.hnavBytes = Queue()
        self.messageHeader = bytearray()
        self.messagePayload = bytearray()
        self.decodedMessages = Queue()
        self.logger = logging.getLogger(__name__)
        self.stopEvent = threading.Event()
        logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
        logging.basicConfig(filename='/webui/logs/hnavDecode.log', encoding='utf-8', level=logging.INFO)
        self.crcConfig = Configuration(
                width=16,
                polynomial=0x1021,
                init_value=0xFFFF,
                final_xor_value=0xFFFF,
                reverse_input=True,
                reverse_output=True,
            )
        self.crcCalculator = Calculator(self.crcConfig)

    def stop(self):
        self.stopEvent.set()

    def pushBytes(self, hnavBytes):
        self.hnavBytes._put(hnavBytes)

    def decodeBytes(self):
        decodeBuffer = bytearray()
        while (not self.stopEvent.is_set()):
            # Wait for a byte to appear in the queue, and then put it in the buffer
            try:
                decodeBuffer.append(self.hnavBytes._get(queueTimeout=0.1))
            except:
                pass

            # Ensure there are enough bytes available to decode a full HNav message
            if (len(decodeBuffer) >= constants.HEADER_SIZE + constants.MESSAGE_SIZE + constants.CRC_SIZE):
                success = self.tryDecodeHnavMessage(decodeBuffer)

                if (success):
                    del decodeBuffer[0:constants.HEADER_SIZE + constants.MESSAGE_SIZE + constants.CRC_SIZE]
                else:
                    del decodeBuffer[0]

    def tryDecodeHnavMessage(self, buffer):

        # Find the first two bytes of the message
        if (buffer[0] != constants.FIRST_BYTE or buffer[1] != constants.SECOND_BYTE):
            return False

        # Ensure that the protocol version is as expected
        if (buffer[2] != constants.PROTOCOL_VERSION):
            return False

        messageID = bitshift16(buffer[3], buffer[4])

        # Ensure that the message ID is as expected
        if (messageID != constants.MESSAGE_ID):
            return False

        messageSize = bitshift16(buffer[5], buffer[6])

        # Ensure that the message size is as expected
        if (messageSize != constants.MESSAGE_SIZE):
            return False

        # Check CRC for correctly received message
        messageCrc = self.crcCalculator.checksum(buffer[:(constants.HEADER_SIZE+constants.MESSAGE_SIZE)])
        expectedCrc = bitshift16(buffer[constants.HEADER_SIZE+constants.MESSAGE_SIZE], buffer[constants.HEADER_SIZE+constants.MESSAGE_SIZE+1])
        
        if (messageCrc == expectedCrc):
            self.messagePayload = buffer[constants.HEADER_SIZE:(constants.HEADER_SIZE + constants.MESSAGE_SIZE)]
            self.decodeHnav()
            return True
        else:
            return False

    def decodeHnav(self):
        utc = bitshift64(self.messagePayload[1],self.messagePayload[2],self.messagePayload[3],self.messagePayload[4],self.messagePayload[5],self.messagePayload[6],self.messagePayload[7],self.messagePayload[8])
        utc = utc * constants.UTC_SCALE

        latitude = bitshift32(self.messagePayload[9], self.messagePayload[10], self.messagePayload[11], self.messagePayload[12])
        latitude = twosComplement(latitude, 4) * constants.LATITUDE_SCALE

        longitude = bitshift32(self.messagePayload[13], self.messagePayload[14], self.messagePayload[15], self.messagePayload[16])
        longitude = twosComplement(longitude, 4) * constants.LONGITUDE_SCALE

        depth = bitshift32(self.messagePayload[17], self.messagePayload[18], self.messagePayload[19], self.messagePayload[20])
        depth = twosComplement(depth, 4) * constants.DEPTH_SCALE

        altitude = bitshift16(self.messagePayload[21], self.messagePayload[22])
        altitude = twosComplement(altitude, 2) * constants.ALTITUDE_SCALE

        roll = bitshift16(self.messagePayload[23], self.messagePayload[24])
        roll = twosComplement(roll, 2) * constants.ORIENTATION_SCALE

        pitch = bitshift16(self.messagePayload[25], self.messagePayload[26])
        pitch = twosComplement(pitch, 2) * constants.ORIENTATION_SCALE

        heading = bitshift16(self.messagePayload[27], self.messagePayload[28])
        heading = heading * constants.ORIENTATION_SCALE

        forwardVelocity = bitshift16(self.messagePayload[29], self.messagePayload[30])
        forwardVelocity = twosComplement(forwardVelocity, 2) * constants.VELOCITY_SCALE

        starboardVelocity = bitshift16(self.messagePayload[31], self.messagePayload[32])
        starboardVelocity = twosComplement(starboardVelocity, 2) * constants.VELOCITY_SCALE

        downwardVelocity = bitshift16(self.messagePayload[33], self.messagePayload[34])
        downwardVelocity = twosComplement(downwardVelocity, 2) * constants.VELOCITY_SCALE

        forwardAngularRate = bitshift16(self.messagePayload[35], self.messagePayload[36])
        forwardAngularRate = twosComplement(forwardAngularRate, 2) * constants.ANGULAR_VELOCITY_SCALE

        starboardAngularRate = bitshift16(self.messagePayload[37], self.messagePayload[38])
        starboardAngularRate = twosComplement(starboardAngularRate, 2) * constants.ANGULAR_VELOCITY_SCALE

        downwardAngularRate = bitshift16(self.messagePayload[39], self.messagePayload[40])
        downwardAngularRate = twosComplement(downwardAngularRate, 2) * constants.ANGULAR_VELOCITY_SCALE

        soundVelocity = bitshift16(self.messagePayload[41], self.messagePayload[42])
        soundVelocity = soundVelocity * constants.SOUND_VELOCITY_SCALE

        temperature = bitshift16(self.messagePayload[43], self.messagePayload[44])
        temperature = twosComplement(temperature, 2) * constants.TEMPERATURE_SCALE

        positionQuality = struct.unpack('f', bytearray([self.messagePayload[45], self.messagePayload[46], self.messagePayload[47], self.messagePayload[48]]))[0]
        positionQuality = positionQuality * constants.POSITION_QUALITY_SCALE

        headingQuality = bitshift16(self.messagePayload[49], self.messagePayload[50])
        headingQuality = twosComplement(headingQuality, 2) * constants.HEADING_QUALITY_SCALE

        velocityQuality = bitshift16(self.messagePayload[51], self.messagePayload[52])
        velocityQuality = velocityQuality * constants.VELOCITY_QUALITY_SCALE

        status = bitshift16(self.messagePayload[53], self.messagePayload[54])

        hnavResponse = pb2.HNavResponse(utc=utc, latitude_degrees=latitude, longitude_degrees=longitude, depth_meters=depth, altitude_meters=altitude, roll_degrees=roll, pitch_degrees=pitch, heading_degrees=heading, forward_velocity_meters_per_second=forwardVelocity, starboard_velocity_meters_per_second=starboardVelocity, downward_velocity_meters_per_second=downwardVelocity, forward_angular_rate_degrees_per_second=forwardAngularRate, starboard_angular_rate_degrees_per_second=starboardAngularRate, downward_angular_rate_degrees_per_second=downwardAngularRate, sound_velocity_meters_per_second=soundVelocity, temperature_degrees_celsius=temperature, position_quality_meters=positionQuality, heading_quality_degrees=headingQuality, velocity_quality_millimeters_per_second=velocityQuality, status=status)

        self.pushDecodedMessages(hnavResponse)

    def pushDecodedMessages(self, message):
        self.decodedMessages._put(message)

    def popDecodedMessages(self, timeout=0):
        return self.decodedMessages._get(timeout)
