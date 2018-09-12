from node import Node
from pubsub import r

class Sub3(Node):
    def __init__(self):
        super().__init__()
 
    def run(self):
        for x in range(2):
            r.publish('sub3:cmd1', 'idk what I\'m doing honestly')