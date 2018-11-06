from Node import  Node
import time


class test2(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("test 2 initialized")

    def run(self):
        num = 0
        while True:
            string = self.recv("1")
            x,y = string.split()
            print("Received ", x, y)
            num += 1
            time.sleep(1)

