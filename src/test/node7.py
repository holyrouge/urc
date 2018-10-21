from Node import Node
import time

class test3(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("test 3 initialized")

    def run(self):
        num = 0

        def test_cb(msg):
            print("Received reply: ", msg)
            
        while True:
            self.request("1", str(num), test_cb)
            num += 1



