import zmq
import sys

port = "5556"
if (len(sys.argv) > 1):
    port = sys.argv[1]
    int(port)

if (len(sys.argv) > 2):
    port1 = sys.argv[2]
    int(port1)

context = zmq.Context()
print("Connecting...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:%s" % port)

rnum = 0

while True:
    print("Sending request ", rnum, "...")
    socket.send_string("REQUEST!")
    message = socket.recv()
    print("received reply ", "[", message, "]")
