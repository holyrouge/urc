import sdl2
import sdl2.ext
from messages.jsdata_pb2 import JSMessage

sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

j = sdl2.SDL_JoystickOpen(0)

prgend = 0

num_axes = sdl2.SDL_JoystickNumAxes(j)
num_buttons = sdl2.SDL_JoystickNumButtons(j)
num_hats = sdl2.SDL_JoystickNumHats(j)

num_axes = sdl2.SDL_JoystickNumAxes(j)
num_buttons = sdl2.SDL_JoystickNumButtons(j)
num_hats = sdl2.SDL_JoystickNumHats(j)

msg = JSMessage()

for i in range(0, num_axes):
    axis = msg.axes.add()
    axis.id = i
    axis.value = 0

for i in range(0, num_buttons):
    button = msg.buttons.add()
    button.id = i
    button.is_pressed = 0

for i in range(0, num_hats):
    hat = msg.hats.add()
    hat.id = i
    hat.value = 0

print(msg)

while prgend == 0:

    if sdl2.SDL_QuitRequested():
        prgend = 1

    for event in sdl2.ext.get_events():
        if event.type == sdl2.SDL_JOYAXISMOTION:
            print([event.jaxis.which, event.jaxis.axis, event.jaxis.value])
            msg = str(0) + " " + str(event.jaxis.axis) + " " + str(event.jaxis.value)
            sdl2.SDL_FlushEvents(sdl2.SDL_FIRSTEVENT, sdl2.SDL_LASTEVENT)
        elif event.type == sdl2.SDL_JOYBUTTONDOWN:
            print([event.jbutton.which, event.jbutton.button, event.jbutton.state])
            msg = str(1) + " " + str(event.jbutton.button) + " " + str(event.jbutton.state)
            sdl2.SDL_FlushEvents(sdl2.SDL_FIRSTEVENT, sdl2.SDL_LASTEVENT)
        elif event.type == sdl2.SDL_JOYBUTTONUP:
            print([event.jbutton.which, event.jbutton.button, event.jbutton.state])
            msg = str(1) + " " + str(event.jbutton.button) + " " + str(event.jbutton.state)
            sdl2.SDL_FlushEvents(sdl2.SDL_FIRSTEVENT, sdl2.SDL_LASTEVENT)

    print(msg)
