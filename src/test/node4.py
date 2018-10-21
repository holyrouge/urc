import zmq

def testsub():
    port = "5556"

    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    print("connecting...")
    socket.connect("tcp://localhost:%s" % port)

    socket.setsockopt_string(zmq.SUBSCRIBE, "10001")

    while True:
        string = socket.recv()
        topic, messagedata = string.split()
        print("Recieved: ", topic, ", ", messagedata)

