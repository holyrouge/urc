from threading import Thread

class Node(Thread):
    def __init__(self):
        Thread.__init__(self, name="Thread-" + self.__class__.__name__)
