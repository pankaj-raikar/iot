import RPi.GPIO as GPIO
import time

SENSOR_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)

print("=== DIAGNOSTIC MODE ===")
print("Cover and uncover the sensor while watching the output")
print("-" * 50)

try:
    last_state = None
    while True:
        current_state = GPIO.input(SENSOR_PIN)
        
        # Only print when state changes
        if current_state != last_state:
            timestamp = time.strftime("%H:%M:%S")
            print(f"[{timestamp}] State changed to: {current_state}")
            last_state = current_state
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nProgram stopped")
    GPIO.cleanup()
```

## Common Issues & Solutions

### 1. **Potentiometer at wrong position**
- The sensor has a threshold that might be set too high or too low
- **Solution**: Keep adjusting the blue potentiometer while covering/uncovering the sensor

### 2. **Module LED behavior**
- Watch the **LED on the module** itself:
  - If LED is always ON or always OFF → potentiometer needs adjustment
  - LED should turn ON/OFF as you cover the sensor

### 3. **Power supply issue**
- Try connecting **VCC to 5V** instead of 3.3V:
```
VCC → Pin 2 (5V)  # Instead of 3.3V
```

### 4. **Check wiring again**
Double-check these connections:
```
Sensor → Raspberry Pi
VCC → Pin 2 (5V)
GND → Pin 6 (GND)
DO  → Pin 11 (GPIO 17)
