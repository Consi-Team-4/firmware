from threading import Thread, Event
from queue import Queue, Empty
import time

from inputs import get_gamepad, InputEvent

q = Queue()
stop = Event()


def read_controller(q:Queue, stop:Event):
    while not stop.is_set():
        events = get_gamepad()
        for event in events:
            q.put(event)


reader_thread = Thread(target=read_controller, args=(q, stop))
reader_thread.start()


state = {
    "BTN_SOUTH":    0,#A
    "BTN_EAST":     0,#B
    "BTN_WEST":     0,#X
    "BTN_NORTH":    0,#Y
    "BTN_THUMBL":   0,
    "BTN_THUMBR":   0,
    "BTN_TL":       0,
    "BTN_TR":       0,
    "BTN_SELECT":   0,#Start   (Yes, these are backwards)
    "BTN_START":    0,#Select/Back

    # Dpad (first=-1, released=0, second=1)
    "ABS_HAT0X":    0,#DPAD left/right
    "ABS_HAT0Y":    0,#DPAD up/down

    # Joysticks (first=-32768, middle=128?, second=32767)
    "ABS_X":        0,# Left Stick left/right
    "ABS_Y":        0,# Left Stick down/up
    "ABS_RX":       0,# Right Stick left/right
    "ABS_RY":       0,# Right Stick down/up

    # Triggers (released=0, fully pressed=255)
    "ABS_Z":        0,#Left Trigger
    "ABS_RZ":       0,#Right Trigger

    # No clue what this is, but it seems like it's always 0
    'SYN_REPORT':   0,
}

# class GamePadState:
#     def __init__(self):
#         A = False
#         B = False
#         X = False
#         Y = False

#         DpadX = 0
#         DpadY = 0

#         Start = False
#         Select = False

#         LBumper = False
#         RBumper = False
#         LThumb = False
#         RThumb = False

#         LTrigger = 0
#         RTrigger = 0
#         LX = 0
#         LY = 0
#         RX = 0
#         RY = 0


        



try:
    while True:
        time.sleep(0.5)
        try:
            while True:
                event:InputEvent = q.get(block=False)
                #if event.code in state:
                state[event.code] = event.state
        except Empty:
            pass
        print(state)
except KeyboardInterrupt:
    stop.set()
    print("Closing!")


reader_thread.join()