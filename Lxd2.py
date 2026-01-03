import RPi.GPIO as GPIO
import time

SENSOR_PIN = 17  # GPIO17

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)

print("Light Sensor Test")
print("Cover the sensor to detect DARK")
print("-" * 30)

try:
    while True:
        sensor_state = GPIO.input(SENSOR_PIN)

        if sensor_state == 1:
            print("🌙 DARK detected")
        else:
            print("☀️ LIGHT detected")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram stopped")
    GPIO.cleanup()
