import threading
import EasyMCP2221
import time
from cellworld import Timer
mcp = EasyMCP2221.Device()
mcp.reset()
print(mcp)
mcp.set_pin_function(gp0="GPIO_OUT")
mcp.IOC_config(edge="rising")
j = False
t = Timer()
def int_read():
    global j
    global t
    while True:
        while j:
            pass
        j = True
        if mcp.IOC_read():
            print("Interrupt happened", t.to_seconds() * 1000)
            mcp.IOC_clear()
            time.sleep(.01)
        j = False
        time.sleep(.00001)

threading.Thread(target=int_read).start()
time.sleep(.1)

v = False
while True:
    while j:
        pass
    j = True
    mcp.GPIO_write(gp0=v)
    t.reset()
    j = False
    v = not v
    time.sleep(.5)

