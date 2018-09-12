from node import Node
from pubsub import r

class Sub1(Node):
    def __init__(self):
        super().__init__()
 
    def run(self):
        for x in range(3):
            r.publish('sub1:cmd1', 'move arm to pos (x, y, z)')