# code to read block's digital photoresistor values for troubleshooting
# prints array of digital values
# 1 indicates bright, 0 indicates dark
# position in array 0-5 corresponds to block faces 1-6

from machine import Pin, Timer
photo1 = Pin(2, Pin.IN, Pin.PULL_UP)
photo2 = Pin(5, Pin.IN, Pin.PULL_UP)
photo3 = Pin(8, Pin.IN, Pin.PULL_UP)
photo4 = Pin(12, Pin.IN, Pin.PULL_UP)
photo5 = Pin(21, Pin.IN, Pin.PULL_UP)
photo6 = Pin(18, Pin.IN, Pin.PULL_UP)

timer = Timer()

def loop(timer):
    photo = [photo1.value(),photo2.value(),photo3.value(),photo4.value(),photo5.value(),photo6.value()]
    print(photo)

timer.init(freq=2, mode=Timer.PERIODIC, callback=loop)