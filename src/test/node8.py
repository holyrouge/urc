from Node import  Node
import time


class test4(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        print("test 4 initialized")

    def run(self):
        num = 0

        def test_cb(msg):
            return "Received %s by server" % msg

        while True:
            self.reply("1", test_cb)
            num += 1
            time.sleep(1)