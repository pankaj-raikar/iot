import RPi.GPIO as GPIO
import time

# Use BOARD numbering to match physical pins
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Physical pin numbers
RED = 11
GREEN = 13
BLUE = 15

GPIO.setup(RED, GPIO.OUT)
GPIO.setup(GREEN, GPIO.OUT)
GPIO.setup(BLUE, GPIO.OUT)

# PWM at 100 Hz
r = GPIO.PWM(RED, 100)
g = GPIO.PWM(GREEN, 100)
b = GPIO.PWM(BLUE, 100)

r.start(0)
g.start(0)
b.start(0)

def set_color(r_val, g_val, b_val):
    """
    r_val, g_val, b_val are 0–100 (duty cycle)
    If using common-anode LED, use 100-value
    """
    r.ChangeDutyCycle(r_val)
    g.ChangeDutyCycle(g_val)
    b.ChangeDutyCycle(b_val)

try:
    while True:
        set_color(100, 0, 0)   # red
        time.sleep(1)

        set_color(0, 100, 0)   # green
        time.sleep(1)

        set_color(0, 0, 100)   # blue
        time.sleep(1)

        set_color(100, 100, 0) # yellow
        time.sleep(1)

        set_color(0, 100, 100) # cyan
        time.sleep(1)

        set_color(100, 0, 100) # magenta
        time.sleep(1)

        set_color(100, 100, 100) # white
        time.sleep(1)

except KeyboardInterrupt:
    pass

r.stop()
g.stop()
b.stop()
GPIO.cleanup()
