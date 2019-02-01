from Node import Node
import time
from messages.jsdata_pb2 import JSMessage

class communication(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("comms initialized!")

    def send_msg(self, msg):
        m = JSMessage()
        m.ParseFromString(bytes(str(msg), 'utf8'))
        print("Transmitting: " + m)

    def loop(self):
        self.recv("comms", self.send_msg)

    def shutdown(self):
        print("properly overriden shutdown method")

