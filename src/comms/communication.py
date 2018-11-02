from Node import Node
import time

class communication(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("comms initialized!")

    def run(self):

        def send_msg(msg):
            print("Transmitting: " + msg)

        while True:
            self.recv("comms", send_msg)
