import json
from threading import Thread

class Node(Thread):
    def __init__(self, configPath):
        Thread.__init__(self, name=self.__class__.name)
        f = open(configPath)
        self.configData = json.load(f)
