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

import asyncio
from concurrent import futures
import grpc
import logging
import os
import serial
import serial.tools.list_ports
import threading
import time

import message_service_pb2 as pb2
import message_service_pb2_grpc as pb2_grpc 

import constants
from hnavClient import HNavClient
from mavlinkHelper import MavlinkHelper

class MessageService(pb2_grpc.MessageServiceServicer):

    def __init__(self):
        self.ipAddress = ""
        self.port = 0
        self.ethernetType = ""
        self.hnavClient = HNavClient()
        self.stopEvent = threading.Event()
        self.streamThread = None
        self.previousMessage = pb2.HNavResponse()
        self.currentMessage = pb2.HNavResponse()
        self.getCurrentMessageThread = threading.Thread(target=self.getCurrentMessage, args=())
        self.getCurrentMessageThread.start()
        self.mavlinkHelper = MavlinkHelper()
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
        logging.basicConfig(filename='/webui/logs/grpcServer.log', encoding='utf-8', level=logging.INFO)
        
    def __del__(self):
        self.hnavClient.setExitEvent()
        if(self.streamThread.is_alive()):
            self.streamThread.join()
        if(self.getCurrentMessageThread.is_alive()):
            self.getCurrentMessageThread.join()

    def GetHNavStream(self, request_iterator, context):
        try:
            while True:
                if (self.currentMessage != self.previousMessage):    
                    self.mavlinkHelper.sendPositionUpdate(self.currentMessage)
                    self.mavlinkHelper.sendPositionDeltaUpdate(self.currentMessage, self.previousMessage)
                    self.previousMessage = self.currentMessage
                    
                    yield self.currentMessage
                else:
                    time.sleep(0.1)
        except GeneratorExit:
            self.logger.error("Client disconnected while stream is active")
        finally:
            self.logger.error("Subscription to stream is closed")

    def SetHNavAddress(self, request, context):
        self.hnavClient.setExitEvent()
        self.streamThread = threading.Thread(target=self.hnavClient.stream, args=())
        self.ipAddress = request.ip_address
        self.port = request.port
        self.ethernetType = request.ethernet_type

        self.hnavClient.SetIpAddress(self.ipAddress)
        self.hnavClient.SetPort(self.port)
        self.hnavClient.SetEthernetType(self.ethernetType)

        success = self.hnavClient.connect()

        if (success):
            self.mavlinkHelper.connect()
            self.streamThread.start()
            
        else:
            self.hnavClient.setExitEvent()
            if(self.streamThread.is_alive()):
                self.streamThread.join()
            self.logger.error("Cannot connect to HNav Server")

        return pb2.EthernetResponse(success=success)

    def getCurrentMessage(self):
        while True:
            self.currentMessage = self.hnavClient.getCurrentMessage()

    def SetGPSAddress(self, request, context):
        success = self.mavlinkHelper.setGPS(request.port, request.baudrate)
        return pb2.EthernetResponse(success=success)

    def SetGGAAddress(self, request, context):
        success = self.mavlinkHelper.setGGA(request.ip_address, request.port)
        return pb2.EthernetResponse(success=success)

    def GetAvailableSerialPorts(self, request, context):
        availablePorts = []
        ports = list(serial.tools.list_ports.comports())
        
        for port in ports:
            availablePorts.append(port.device)

        availableBaudRates = constants.BAUDRATE_LIST
        return pb2.AvailableSerialResponse(success=True, baudrates=availableBaudRates, ports=availablePorts)

    def CloseConnections(self, request, context):
        self.hnavClient.disconnect()
        self.mavlinkHelper.disconnect()
        return pb2.RestartResponse(success=True)

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    pb2_grpc.add_MessageServiceServicer_to_server(MessageService(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':
    serve()
