from node import Node

class Sub3(Node):
    def __init__(self):
        super().__init__()
 
    def run(self):
        count = 0
        while count < 100:
            print(self.getName() + " is running")
            count += 1