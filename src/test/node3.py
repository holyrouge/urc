import zmq
import random
import time


def testpub():
    port = "5556"

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:%s" % port)

    while True:
        topic = random.randrange(9999, 10005)
        messagedata = random.randrange(1, 215) - 80
        print("%d %d" % (topic, messagedata))
        socket.send(b"%d %d" % (topic, messagedata))
        time.sleep(1)
