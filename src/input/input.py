from Node import Node
import time
import random
from messages.jsdata_pb2 import JSMessage


class input(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("input initialized!")

    def loop(self):
        # self.get_input()
        # msg = self.inp
        msg = JSMessage()
        msg.id = random.randint(0, 10)
        self.send("comms", msg.SerializeToString())

    def shutdown(self):
        print("properly written shutdown method")

