from Node import Node
import time

class communication(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("comms initialized!")

    def send_msg(self, msg):
        print("Transmitting: " + msg)

    def loop(self):
        self.recv("comms", self.send_msg)

    def shutdown(self):
        print("properly overriden shutdown method")

