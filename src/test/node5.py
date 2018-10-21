from Node import Node
import time

class test1(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("test 1 initialized")

    def run(self):
        num = 0
        while True:
            self.send("1", "PUB")
            num += 1
            time.sleep(1)

