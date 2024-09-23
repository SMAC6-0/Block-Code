# code to show how RGB LED is set up
# can also use this code to make new colors
from machine import Pin, PWM
ledb = Pin(22, Pin.OUT)
ledr = Pin(27, Pin.OUT)
ledg = Pin(26, Pin.OUT)

b = PWM(ledb)
r = PWM(ledr)
g = PWM(ledg)

# PWM frequency, probably doesn't matter as long as its above 50Hz
b.freq(500)
r.freq(500)
g.freq(500)

# duty cycle:
# RGB brightness levels (unsigned 16-bit value in the range 0 to 65535)
b.duty_u16(000)
r.duty_u16(000)
g.duty_u16(8000)

# b.deinit()
# r.deinit()
# g.deinit()


