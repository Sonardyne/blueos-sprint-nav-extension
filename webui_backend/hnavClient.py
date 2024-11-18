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

import logging
import socket
import sys
from hnavDecode import HNavDecode
import threading

class HNavClient():

    def __init__(self):
        self.ipAddress = ""
        self.port = 0
        self.ethernetType = ""
        self.hnavDecode = HNavDecode()
        self.decodeThread = threading.Thread(target=self.hnavDecode.decodeBytes, args=())
        self.decodeThread.start()
        self.client = None
        self.connected = False
        self.exitEvent = threading.Event()
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
        logging.basicConfig(filename='/webui/logs/hnavClient.log', encoding='utf-8', level=logging.INFO)

    def __del__(self):
        if(self.decodeThread.is_alive()):
            self.decodeThread.join()
    
    def SetIpAddress(self, ipAddress):
        self.ipAddress = ipAddress
            
    def SetPort(self, port):
        self.port = port
            
    def SetEthernetType(self, ethernetType):
        self.ethernetType = ethernetType

    def connect(self):
        success = False
        try:
            if (self.ethernetType == "UDP"):
                self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                self.client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.client.bind((self.ipAddress, self.port))
            else: 
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.connect((self.ipAddress, self.port))
            success = True
            self.connected = True
        except:
            success = False

        return success

    def disconnect(self):
        if (self.connected):
            self.client.close()
            self.connected = False

    def stream(self):
        self.exitEvent = threading.Event()

        while self.connected:
            if (self.exitEvent.is_set()):
                break

            data = self.client.recv(1024)
            if data:
                for byte in data:
                    self.hnavDecode.pushBytes(byte)

    def getCurrentMessage(self):
        return self.hnavDecode.popDecodedMessages()

    def setExitEvent(self):
        self.exitEvent.set()