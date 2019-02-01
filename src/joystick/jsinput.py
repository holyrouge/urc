import sdl2
import sdl2.ext
from Node import Node
from messages.jsdata_pb2 import JSMessage
import time


# sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
# gamepads = []
# for i in range(0, sdl2.SDL_NumJoysticks()):
#     if sdl2.SDL_IsGameController(i):
#         tmp = sdl2.SDL_GameControllerOpen(i)
#         gamepads.append(tmp)
#         print(str(i) + " is a gamepad")

# while True:
#     for event in sdl2.ext.get_events():
#         if event.type == sdl2.SDL_JOYAXISMOTION:
#             print([event.jaxis.which, event.jaxis.axis, event.jaxis.value])
#         elif event.type == sdl2.SDL_JOYBUTTONDOWN:
#             print([event.jbutton.which, event.jbutton.button, event.jbutton.state])
#         elif event.type == sdl2.SDL_JOYBUTTONUP:
#             print([event.jbutton.which, event.jbutton.button, event.jbutton.state])

class jsinput(Node):
    def __init__(self, configPath):
        Node.__init__(self, configPath)
        sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)
        self.gamepads = []
        for i in range(0, sdl2.SDL_NumJoysticks()):
            tmp = sdl2.SDL_JoystickOpen(i)
            self.gamepads.append(tmp)
        print("joystick initialized")

    def loop(self):

        for event in sdl2.ext.get_events():
            if event.type == sdl2.SDL_JOYAXISMOTION:
                # print([event.jaxis.which, event.jaxis.axis, event.jaxis.value])
                msg = str(0) + " " + str(event.jaxis.axis) + " " + str(event.jaxis.value)
                self.send("outbound", msg)
                sdl2.SDL_FlushEvents(sdl2.SDL_FIRSTEVENT, sdl2.SDL_LASTEVENT)
            elif event.type == sdl2.SDL_JOYBUTTONDOWN:
                # print([event.jbutton.which, event.jbutton.button, event.jbutton.state])
                msg = str(1) + " " + str(event.jbutton.button) + " " + str(event.jbutton.state)
                self.send("outbound", msg)
                sdl2.SDL_FlushEvents(sdl2.SDL_FIRSTEVENT, sdl2.SDL_LASTEVENT)
            elif event.type == sdl2.SDL_JOYBUTTONUP:
                # print([event.jbutton.which, event.jbutton.button, event.jbutton.state])
                msg = str(1) + " " + str(event.jbutton.button) + " " + str(event.jbutton.state)
                self.send("outbound", msg)
                sdl2.SDL_FlushEvents(sdl2.SDL_FIRSTEVENT, sdl2.SDL_LASTEVENT)
            time.sleep(0.0001)

        sdl2.SDL_Delay(50)

    def shutdown(self):
        print("joystick shutting down")
