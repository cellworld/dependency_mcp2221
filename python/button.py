import board
import digitalio
from time import sleep
button = digitalio.DigitalInOut(board.G1)
button.direction = digitalio.Direction.INPUT

led = digitalio.DigitalInOut(board.G0)
led.direction = digitalio.Direction.OUTPUT

v=False
while True:
    sleep(.25)
    print("PIN 0:", v)
    led.value = v
    sleep(.25)
    print("PIN 1:", button.value)
    v = not v
