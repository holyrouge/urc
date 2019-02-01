from Node import Node
import serial
import time
from messages.jsdata_pb2 import JSMessage


class outbound(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        self.ser = serial.Serial(self.configData['comport'])
        self.ser.baudrate = self.configData['baudrate']
        if not self.ser.is_open:
            raise Exception
        print("outbound initialized")

    def transmit(self, msg):
        m = JSMessage()
        print(JSMessage.ParseFromString(msg))
        self.ser.write(msg.encode('utf-8'))

    def loop(self):
        self.recv("outbound", self.build_packet)

    def shutdown(self):
        print("sample shutdown")

    # def build_packet(self, msg):
    #     data = [255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 0, 0, 0, 0, 128, 128, 128, 128, 0,
    #             0]  # Making an array for the data.
    #     tmp = msg.split()
    #
    #     # data[11] is the B button
    #     # data[15] is left y axis
    #     # data[17] is right y axis
    #     print(tmp)
    #     if tmp[1] == '0':
    #         data[int(tmp[2]) + 14] = self.translate(int(tmp[3]), -32768, 32768, 0, 255)
    #         print("js: " + str(self.translate(int(tmp[3]), -32768, 32768, 0, 255)))
    #     elif tmp[1] == '1':
    #         data[11] = 1 if tmp[2] == '1' else 0
    #         print("button: " + str(tmp[2]) + ' ' + str(tmp[3]))
    #
    #     for x in data:
    #         if type(x) is float:
    #             x = int((x + 0.5) * 255)
    #             print('changed: ' + x)
    #             if x == 255:
    #                 x = 254
    #         if x == 255:
    #             x = 254
    #
    #     sum = 0
    #     for x in data:
    #         self.ser.write(bytes([x]))
    #         sum += x
    #     self.ser.write(bytes([sum % 255]))
    #     time.sleep(.0001)

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return int(rightMin + (valueScaled * rightSpan))
