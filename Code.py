import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

R, G, B = 12, 13, 19   # BCM GPIO numbers

GPIO.setup(R, GPIO.OUT)
GPIO.setup(G, GPIO.OUT)
GPIO.setup(B, GPIO.OUT)

def color(r, g, b):
    GPIO.output(R, r)
    GPIO.output(G, g)
    GPIO.output(B, b)

while True:
    color(1,0,0)  # Red
    sleep(1)
    color(0,1,0)  # Green
    sleep(1)
    color(0,0,1)  # Blue
    sleep(1)
    color(1,1,1)  # White
    sleep(1)
