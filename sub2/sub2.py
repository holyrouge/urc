from node import Node
from pubsub import r

class Sub2(Node):
    def __init__(self):
        super().__init__()
 
    def run(self):
       for x in range(5):
            r.publish('sub2:cmd1', 'twiddle some bits')