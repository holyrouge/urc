from Node import Node
import time
import random

class input(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("input initialized!")

    def run(self):
        while True:
            msg = "input!"
            msg = msg + " " + str(time.time())
            self.send("comms", msg)
