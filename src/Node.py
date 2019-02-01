import json
import zmq
import traceback
from threading import Thread
from threading import Event
from status import Status


class Node(Thread):
    """This is the class that will be the base of every node object

    Each node will read a config file in JSON format to initialize zmq sockets.
    The sockets will be stored in a dict with the key being the topic and the value being the socket
    To create a node, make a config file and extend the run() method.
    Further functionality is on the way.
    """

    def __init__(self, configPath):
        Thread.__init__(self, name=self.__class__.name)
        f = open(configPath)
        self.configPath = configPath
        self.configData = json.load(f)
        f.close()
        self.id = self.configData['id']
        self.context = zmq.Context()
        self.topics = {}
        self.initzmq()
        self._stop_event = Event()
        self.status = Status.RUNNING

    def stop(self):

        """ This method sets the stop flag

        The node should safely shut down after this flag is set
        """

        self._stop_event.set()

    def stopped(self):

        """ Returns the value of the stopped flag

        """

        return self._stop_event.is_set()

    def run(self):

        """ This will be the node's main loop

        Everyone should override the loop and shutdown methods.
        """

        try:
            while True:

                if (self.stopped() == True):
                    print(self.id + " shutting down...")
                    self.stopzmq()
                    self.shutdown()
                    self.status = Status.STOPPED
                    break
                else:
                    self.loop()
        except Exception:
            self.status = Status.CRASHED
            traceback.print_exc()
            self.savedata()
            self.stopzmq()
            self.shutdown()

    def stopzmq(self):

        """ Shuts down all zmq stuff


        """

        self.context.destroy()

    def savedata(self):
        print("override this to save data on event of a crash")

    def loaddata(self):
        print("override this to reload data on the event of a restart")

    def restart(self):
        print("override this to do anything special for restarting the node")

    def shutdown(self):

        """ Gets called when the node is told to stop

        Everyone should override this for good practice
        """

        print(self.id + " needs an overridden shutdown method!")

    def loop(self):

        """ The main node code that gets executed every loop

        This is the method that should be overridden for the node to do stuff
        So help me God if anyone overrides this and puts a while true in there
        """

        print(self.id + " needs an overridden loop method")

    # TODO: allow more than one model to be used
    # TODO: look into using different protocols
    def initzmq(self):

        """This method initializes zmq sockets and places them in the topics dict

        It will throw exceptions if the JSON it was fed is not correct
        """

        if "model" not in self.configData:
            raise Exception("model not found in %s" % self.configPath)

        if "topics" not in self.configData:
            raise Exception("Topics or port not found in %s" % self.configPath)

        for topic in self.configData['topics']:
            for x, y in topic.items():
                if y == "pub" or y == "rep":
                    if y == "pub":
                        socket = self.context.socket(zmq.PUB)
                    else:
                        socket = self.context.socket(zmq.REP)
                    socket.bind(self.configData['protocol'] + "://*:%s" % self.configData['port'])
                    self.topics[x] = socket
                elif y == "sub" or y == "req":
                    if y == "sub":
                        socket = self.context.socket(zmq.SUB)
                    else:
                        socket = self.context.socket(zmq.REQ)
                    print(self.configData['protocol'] + "://localhost:%s" % self.configData['port'])
                    socket.connect(self.configData['protocol'] + "://localhost:%s" % self.configData['port'])
                    if y == "sub":
                        socket.setsockopt_string(zmq.SUBSCRIBE, x)
                    self.topics[x] = socket
                else:
                    raise Exception("Topic %s should be either pub or sub" % x)

    def send(self, topic, msg):

        """This method can be used to send messages for the pub pattern

        The first argument is the topic to send the message on and the second is the message body
        """
        self.topics[topic].send_string("%s %s" % (topic, msg))

    # TODO: implement a timeout
    def recv(self, topic, callback):

        """This method is used to receive messages for the sub pattern

        The first argument is the topic to look for messages on.
        The second argument is a function to be executed with the message received being passed to it as an argument
        NOT VERIFIED: This method is blocking, and will interrupt execution until a message is received
        """
        re = self.topics[topic].recv_string()
        callback(re)

    def request(self, topic, req, callback):

        """This method is used to send a request to a node

        The first argument is the topic(in this case, the node) to send a request to.
        The second argument is the request to send
        The third argument is a callback function to process the reply from the server. The reply will be a string
        """
        self.topics[topic].send_string(req)
        msg = self.topics[topic].recv_string()
        callback(msg)

    def reply(self, topic, callback):

        """This method is used to send a reply to a node

        The first argument is the topic(in this case, the node) to reply to
        The second argument is a callback that will handle the request sent to this node. It must return a string.
        The reply generated by the callback is sent as a reply to the node that sent a request
        """
        msg = self.topics[topic].recv_string()
        rep = callback(msg)
        self.topics[topic].send_string(str(rep))
