import RPi.GPIO as GPIO
import time

# Pin configuration
SENSOR_PIN = 17  # GPIO 17 (Physical pin 11)

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)

print("Light Sensor Test")
print("Adjust the blue potentiometer to set sensitivity")
print("-" * 40)

try:
    while True:
        sensor_state = GPIO.input(SENSOR_PIN)
        
        if sensor_state == 0:
            print("🌙 DARK detected")
        else:
            print("☀️  LIGHT detected")
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram stopped")
    GPIO.cleanup()
