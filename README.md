# GalleryGuard - IoT Project Plan

Winner concept and build plan for a fast, reliable demo.

## Table of Contents

- [1) Assumptions](#1-assumptions)
- [2) Winner & Rubric](#2-winner--rubric)
- [3) Product Spec (Winner)](#3-product-spec-winner)
- [4) System Architecture](#4-system-architecture)
- [5) Hardware Plan (Wiring That Won't Fail)](#5-hardware-plan-wiring-that-wont-fail)
- [6) Task Distribution for 8 People (Production-Style)](#6-task-distribution-for-8-people-production-style)
- [7) All Code (Copy-Paste Ready)](#7-all-code-copy-paste-ready)
- [8) Testing + Demo Playbook](#8-testing--demo-playbook)
- [9) Presentation Assets (Minimal, Apple-Clean)](#9-presentation-assets-minimal-apple-clean)

## 1) Assumptions

- Arduino Uno-class @ 5V
- Raspberry Pi 3/4 @ 3.3V
- HC-SR04 ultrasonic sensor
- SSD1306 I2C OLED 128x32 on Pi
- Sensors/actuators on Arduino
- Pi <-> Arduino via USB serial
- LED resistors available

## 2) Winner & Rubric

**Winner:** GalleryGuard - "Invisible buffer for priceless art."

**Why it wins (rubric):**

- **Reliability (30%)**: Simple distance threshold + stable climate ranges; easy to tune; very low mechanical complexity.
- **Wow factor <10s (25%)**: "Step too close" triggers immediate red + buzz + OLED warning.
- **Clear user value (20%)**: Protects art and preserves climate; instantly understandable.
- **Uses all components naturally (15%)**: Every part is essential to the protection story.
- **Speed (10%)**: Minimal UI, minimal logic, fast wiring and debugging.

## 3) Product Spec (Winner)

### Problem Statement

Museums and galleries need an unobtrusive way to keep visitors at a safe distance while also ensuring artwork preservation conditions. The solution must be touchless, reliable, and immediately readable by visitors and staff.

### User Journey

1. Device sits near a display.
2. Visitor approaches; device detects proximity.
3. If within safe buffer, a soft "active" signal shows.
4. If too close, device warns (red LED + short buzz + OLED message).
5. Ambient temperature/humidity always visible for staff.

### Key Behaviors

- Detect visitor distance and classify as safe / too close.
- Show climate status continuously on OLED.
- Only beep when the state changes to "Too Close" or "Error."
- LED semantics: green = ready/safe, blue = active/visitor present, red = alert.

### Edge Cases

- Ultrasonic spikes -> median filter + hysteresis.
- DHT11 spikes -> debounce + wide safe thresholds.
- Sensor disconnect -> "ERROR" state, red LED + periodic beep.
- Serial drop -> Arduino fallback LED pattern; Pi auto-reconnect.

### Success Metrics

- Detects approach within 1-2 seconds.
- No false buzzing more than once per minute during demo.
- OLED shows temperature/humidity and distance continuously.
- Recovery within 5 seconds after unplug/replug.

## 4) System Architecture

### Diagram (ASCII)

```text
[Ultrasonic]   [DHT11]
     \           /
      \         /
       +-- Arduino --(USB Serial)-- Raspberry Pi -- OLED
            |   \                          |
            |    \---- LEDs + Buzzer ------+
            |
       Breadboard + Jumpers
```

### Data Flow

- Arduino reads ultrasonic + DHT11 -> filters -> sends JSON lines to Pi.
- Pi runs state machine -> sends simple STATE commands to Arduino.
- Arduino drives LEDs + buzzer accordingly.
- Pi renders OLED + logs JSONL.

### State Machine (Simplified)

- IDLE (no visitor)
- APPROACH (visitor in safe range)
- TOO_CLOSE (distance < alert threshold)
- CLIMATE_ALERT (temp/humidity out of range for >15s)
- ERROR (stale sensor or serial loss)
- DEMO (manual mode)

### Transitions

- IDLE -> APPROACH when distance <= detect threshold.
- APPROACH -> TOO_CLOSE when distance < alert threshold.
- TOO_CLOSE -> APPROACH when distance > alert + 10cm.
- ANY -> CLIMATE_ALERT when climate out of range >15s.
- ANY -> ERROR when ultrasonic stale.
- ERROR -> IDLE when sensors recover.
- ANY <-> DEMO via software flag/command.

### Arduino <-> Pi Protocol

```text
Arduino -> Pi (JSON line):
{"ts":12345,"dist_cm":42.3,"temp_c":22.0,"hum":48,"ultra_ok":1,"dht_ok":1,"seq":57}

Pi -> Arduino (simple command line):
STATE,TOO_CLOSE,RED,200
STATE,APPROACH,BLUE,0
STATE,IDLE,GREEN,0
DEMO,ON
DEMO,OFF
```

## 5) Hardware Plan (Wiring That Won't Fail)

**Safety rule:** Nothing 5V touches Pi GPIO. Only Pi <-> Arduino via USB.

### Arduino Pin Mapping

| Component | Arduino Pin | Power | Notes |
|---|---|---|---|
| DHT11 data | D2 | 5V + GND | Use 10k pull-up if bare sensor |
| HC-SR04 Trig | D4 | 5V + GND | 10us trigger pulse |
| HC-SR04 Echo | D5 | 5V + GND | Read on Arduino only |
| LED Green | D9 | - | 220ohm in series to GND |
| LED Blue | D10 | - | 220ohm in series to GND |
| LED Red | D8 | - | 220ohm in series to GND |
| Buzzer | D6 | - | Active buzzer ok; else add resistor |

### OLED Wiring (Pi I2C)

- VCC -> 3.3V (Pin 1)
- GND -> GND (Pin 6)
- SDA -> GPIO2 (Pin 3)
- SCL -> GPIO3 (Pin 5)

### Physical Layout Tips

- Keep ultrasonic at the front edge of breadboard, facing the demo target.
- Route LED leads flat and tape them so they don't wiggle.
- Label each jumper with masking tape (D2, D4, D5, etc.).
- Bundle wires with a single zip tie to reduce accidental pulls.
- Use a small cardboard stand to keep sensors upright.

### Demo-Proofing Checklist

- Fresh USB cable and spare
- Tape every header connection
- Spare jumper wires + spare LED
- Mark "front" on the board
- Keep a printed pin map at the table

## 6) Task Distribution for 8 People (Production-Style)

### Roles + Deliverables

1. Product/Story + Demo Director: final story, demo choreography, judge flow.
2. Hardware Lead: breadboard wiring, strain relief, pin labels.
3. Arduino Firmware Lead: sensors + filtering + serial protocol + LED/buzzer control.
4. Raspberry Pi App Lead: serial handler + state machine + OLED + logging.
5. UX/Microcopy Lead: OLED text, LED semantics, sound timing.
6. Integration Engineer: end-to-end wiring + serial protocol testing.
7. QA/Test Engineer: test plan + soak test + failure drills.
8. Presentation/Docs Lead: slides + script + contingency notes.

### Hour-by-Hour Schedule (T-24 -> T-0)

- T-24 to T-22: Align on winner concept + thresholds + demo story.
- T-22 to T-18: Parallel build: wiring + Arduino firmware + Pi app skeleton.
- T-18 to T-16: Integration Checkpoint #1 (serial protocol + basic readings).
- T-16 to T-12: OLED layout, filtering, state machine tuning.
- T-12 to T-10: Integration Checkpoint #2 (full loop + LEDs + buzzer).
- T-10 to T-6: Reliability hardening, error handling, logging.
- T-6 to T-4: Integration Checkpoint #3 (demo rehearsal).
- T-4 to T-2: Polish microcopy, slides, demo script.
- T-2 to T-0: Final rehearsal + backup plan walk-through.

### Definition of Done (per role)

- Product/Story: 45-sec demo script + one-liner value.
- Hardware: wiring works after 3 power cycles with no re-seat.
- Arduino: stable JSON output + controlled LEDs/buzzer.
- Pi: reconnects after serial unplug, OLED stable.
- UX: text fits OLED; LED rules consistent.
- Integration: full loop works for 30 minutes.
- QA: tests executed + results logged.
- Presentation: 6 slides + timing practiced twice.

## 7) All Code (Copy-Paste Ready)

### Arduino (C++)

```cpp
#include <Arduino.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11

const int TRIG_PIN = 4;
const int ECHO_PIN = 5;

const int LED_R = 8;
const int LED_G = 9;
const int LED_B = 10;
const int BUZZER = 6;

#define DEMO_MODE_DEFAULT 0  // set to 1 for always-on demo mode

DHT dht(DHTPIN, DHTTYPE);

const int ULTRA_SAMPLES = 5;
float distBuf[ULTRA_SAMPLES];
int distIdx = 0;
bool distFilled = false;

unsigned long lastDhtMs = 0;
unsigned long lastUltraMs = 0;
unsigned long lastValidUltraMs = 0;
unsigned long lastValidDhtMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastCmdMs = 0;

float lastDist = -1.0;
float lastTemp = 0.0;
float lastHum = 0.0;

bool ultraOk = false;
bool dhtOk = false;

bool demoMode = DEMO_MODE_DEFAULT;

String cmdLine;

bool ledR = false, ledG = false, ledB = false;
bool buzzPending = false;
int buzzMs = 0;

float medianOf(float *arr, int n) {
  float tmp[ULTRA_SAMPLES];
  for (int i = 0; i < n; i++) tmp[i] = arr[i];
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }
  if (n == 0) return -1.0;
  if (n % 2 == 1) return tmp[n / 2];
  return (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5;
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms ~ 4m
  if (duration == 0) return -1.0;
  float dist = duration / 58.2; // cm
  if (dist < 2 || dist > 300) return -1.0;
  return dist;
}

void setLedState(bool r, bool g, bool b) {
  ledR = r;
  ledG = g;
  ledB = b;
  digitalWrite(LED_R, ledR ? HIGH : LOW);
  digitalWrite(LED_G, ledG ? HIGH : LOW);
  digitalWrite(LED_B, ledB ? HIGH : LOW);
}

void applyFallbackBlink() {
  // Blue blink if no recent command
  bool on = (millis() / 500) % 2 == 0;
  setLedState(false, false, on);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("DEMO,")) {
    if (line.endsWith("ON")) demoMode = true;
    if (line.endsWith("OFF")) demoMode = false;
    Serial.print("{\"event\":\"demo\",\"on\":");
    Serial.print(demoMode ? 1 : 0);
    Serial.println("}");
    return;
  }

  if (line.startsWith("STATE,")) {
    // Format: STATE,<state>,<led>,<buzz_ms>
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);

    String state = (p2 > 0) ? line.substring(p1 + 1, p2) : "";
    String led = (p3 > 0) ? line.substring(p2 + 1, p3) : "";
    String buzz = (p3 > 0) ? line.substring(p3 + 1) : "";

    led.toUpperCase();
    if (led == "GREEN") setLedState(false, true, false);
    else if (led == "BLUE") setLedState(false, false, true);
    else if (led == "RED") setLedState(true, false, false);
    else setLedState(false, false, false);

    int b = buzz.toInt();
    if (b > 0) {
      buzzMs = b;
      buzzPending = true;
    }
    lastCmdMs = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  dht.begin();

  setLedState(false, true, false); // ready on boot
  Serial.println("{\"event\":\"boot\",\"fw\":\"1.0\"}");
}

void loop() {
  // Read serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(cmdLine);
      cmdLine = "";
    } else if (c != '\r') {
      cmdLine += c;
      if (cmdLine.length() > 80) cmdLine = "";
    }
  }

  // DHT read every 2s
  unsigned long now = millis();
  if (now - lastDhtMs >= 2000) {
    lastDhtMs = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      lastHum = h;
      lastTemp = t;
      lastValidDhtMs = now;
      dhtOk = true;
    }
  }

  // Ultrasonic read every 100ms
  if (now - lastUltraMs >= 100) {
    lastUltraMs = now;
    float d = readUltrasonicCm();
    if (d > 0) {
      distBuf[distIdx] = d;
      distIdx = (distIdx + 1) % ULTRA_SAMPLES;
      if (distIdx == 0) distFilled = true;
      float med = medianOf(distBuf, distFilled ? ULTRA_SAMPLES : distIdx);
      if (med > 0) {
        lastDist = med;
        lastValidUltraMs = now;
        ultraOk = true;
      }
    }
  }

  // Demo mode overrides
  if (demoMode) {
    float phase = (now % 8000) / 8000.0 * 2.0 * PI;
    lastDist = 30 + 40 * (0.5 - 0.5 * cos(phase));
    lastTemp = 21 + 3 * sin(phase);
    lastHum = 45 + 8 * sin(phase + 1.5);
    ultraOk = true;
    dhtOk = true;
    lastValidUltraMs = now;
    lastValidDhtMs = now;
  }

  // Stale checks
  if (now - lastValidUltraMs > 2000) ultraOk = false;
  if (now - lastValidDhtMs > 6000) dhtOk = false;

  // Fallback LED pattern if no recent commands
  if (now - lastCmdMs > 3000) {
    applyFallbackBlink();
  }

  // Buzz once if pending
  if (buzzPending) {
    tone(BUZZER, 2000, buzzMs);
    buzzPending = false;
  }

  // Telemetry every 500ms
  if (now - lastTelemetryMs >= 500) {
    lastTelemetryMs = now;
    Serial.print("{\"ts\":");
    Serial.print(now);
    Serial.print(",\"dist_cm\":");
    if (ultraOk) Serial.print(lastDist, 1);
    else Serial.print("null");
    Serial.print(",\"temp_c\":");
    if (dhtOk) Serial.print(lastTemp, 1);
    else Serial.print("null");
    Serial.print(",\"hum\":");
    if (dhtOk) Serial.print(lastHum, 0);
    else Serial.print("null");
    Serial.print(",\"ultra_ok\":");
    Serial.print(ultraOk ? 1 : 0);
    Serial.print(",\"dht_ok\":");
    Serial.print(dhtOk ? 1 : 0);
    Serial.print("}");
    Serial.println();
  }
}
```

### Raspberry Pi (Python)

```python
#!/usr/bin/env python3
import os
import time
import json
import glob
import argparse
from datetime import datetime

import serial

OLED_OK = True
try:
    import board
    import busio
    import adafruit_ssd1306
    from PIL import Image, ImageDraw, ImageFont
except Exception:
    OLED_OK = False

BAUD = 115200
RECONNECT_SEC = 2.0

DIST_DETECT_CM = 120
DIST_ALERT_CM = 45
DIST_CLEAR_CM = 55  # hysteresis

TEMP_MIN = 18.0
TEMP_MAX = 26.0
HUM_MIN = 40.0
HUM_MAX = 60.0

STALE_ULTRA_MS = 2000
STALE_DHT_MS = 6000
CLIMATE_BAD_HOLD_SEC = 15

def find_port():
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    return None

def open_serial():
    port = find_port()
    if not port:
        return None
    try:
        return serial.Serial(port, BAUD, timeout=0.5)
    except Exception:
        return None

def init_oled():
    if not OLED_OK:
        return None, None, None
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)
        oled.fill(0)
        oled.show()
        image = Image.new("1", (oled.width, oled.height))
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default()
        return oled, draw, font
    except Exception:
        return None, None, None

def draw_oled(oled, draw, font, lines):
    if not oled:
        return
    draw.rectangle((0, 0, 127, 31), outline=0, fill=0)
    y = 0
    for line in lines[:4]:
        draw.text((0, y), line, font=font, fill=255)
        y += 8
    oled.image(draw.im)
    oled.show()

def ensure_logs_dir():
    os.makedirs("logs", exist_ok=True)

def log_json(entry):
    ensure_logs_dir()
    fname = datetime.now().strftime("logs/telemetry_%Y%m%d.jsonl")
    with open(fname, "a") as f:
        f.write(json.dumps(entry) + "\n")

def compute_state(data, now, climate_bad_since, last_state):
    if not data:
        return "ERROR", climate_bad_since

    ultra_ok = data.get("ultra_ok", 0) == 1
    dht_ok = data.get("dht_ok", 0) == 1

    if not ultra_ok:
        return "ERROR", climate_bad_since

    dist = data.get("dist_cm", None)
    if dist is None:
        return "ERROR", climate_bad_since

    # Climate check (if available)
    climate_ok = True
    if dht_ok:
        temp = data.get("temp_c", None)
        hum = data.get("hum", None)
        if temp is None or hum is None:
            climate_ok = True
        else:
            climate_ok = (TEMP_MIN <= temp <= TEMP_MAX) and (HUM_MIN <= hum <= HUM_MAX)

    if dht_ok and not climate_ok:
        if climate_bad_since == 0:
            climate_bad_since = now
        if now - climate_bad_since >= CLIMATE_BAD_HOLD_SEC:
            return "CLIMATE_ALERT", climate_bad_since
    else:
        climate_bad_since = 0

    # Distance states with hysteresis
    if last_state == "TOO_CLOSE":
        if dist > DIST_CLEAR_CM:
            return "APPROACH", climate_bad_since
        return "TOO_CLOSE", climate_bad_since

    if dist < DIST_ALERT_CM:
        return "TOO_CLOSE", climate_bad_since
    elif dist <= DIST_DETECT_CM:
        return "APPROACH", climate_bad_since
    else:
        return "IDLE", climate_bad_since

def state_to_led(state):
    if state == "IDLE":
        return "GREEN"
    if state == "APPROACH":
        return "BLUE"
    if state == "TOO_CLOSE":
        return "RED"
    if state == "CLIMATE_ALERT":
        return "RED"
    return "RED"

def state_label(state):
    return {
        "IDLE": "READY",
        "APPROACH": "ACTIVE",
        "TOO_CLOSE": "STEP BACK",
        "CLIMATE_ALERT": "CHECK AIR",
        "ERROR": "SENSORS?",
        "DEMO": "DEMO",
    }.get(state, state)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", help="force demo mode")
    args = parser.parse_args()

    oled, draw, font = init_oled()

    ser = None
    last_reconnect = 0
    last_state_sent = None
    last_state = "BOOT"
    climate_bad_since = 0
    last_rx_time = 0
    latest = {}

    if args.demo:
        latest["demo"] = True

    while True:
        now = time.time()

        if ser is None and (now - last_reconnect) > RECONNECT_SEC:
            last_reconnect = now
            ser = open_serial()
            if ser and args.demo:
                try:
                    ser.write(b"DEMO,ON\n")
                except Exception:
                    pass

        # Read serial line
        if ser:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    try:
                        data = json.loads(line)
                        latest = data
                        last_rx_time = now
                        log_json({"type": "telemetry", **data})
                    except json.JSONDecodeError:
                        pass
            except Exception:
                ser = None

        # If stale data, mark error
        if last_rx_time == 0 or (now - last_rx_time) * 1000 > STALE_ULTRA_MS:
            # Keep latest but treat as error
            pass

        # Compute state
        state, climate_bad_since = compute_state(latest, now, climate_bad_since, last_state)
        if args.demo:
            state = "DEMO"

        # Send state on change
        if ser and state != last_state_sent:
            led = state_to_led(state)
            buzz = 200 if state in ("TOO_CLOSE", "ERROR", "CLIMATE_ALERT") else 0
            cmd = f"STATE,{state},{led},{buzz}\n"
            try:
                ser.write(cmd.encode())
                last_state_sent = state
            except Exception:
                ser = None

        last_state = state

        # OLED update
        dist = latest.get("dist_cm")
        temp = latest.get("temp_c")
        hum = latest.get("hum")

        dist_txt = f"Dist {dist:>3.0f}cm" if isinstance(dist, (int, float)) else "Dist --"
        th_txt = "T -- H --"
        if isinstance(temp, (int, float)) and isinstance(hum, (int, float)):
            th_txt = f"T {temp:>2.0f}C H {hum:>2.0f}%"

        lines = [
            "GALLERY GUARD",
            state_label(state),
            dist_txt,
            th_txt,
        ]
        draw_oled(oled, draw, font, lines)

        time.sleep(0.1)

if __name__ == "__main__":
    main()
```

### Install / Run (Pi)

- Enable I2C: `sudo raspi-config` -> Interface Options -> I2C -> Enable
- Install deps:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-pil python3-smbus i2c-tools
pip3 install pyserial adafruit-circuitpython-ssd1306
```

- Run: `python3 gallery_guard.py`
- Optional demo mode: `python3 gallery_guard.py --demo`
- Serial permissions: `sudo usermod -a -G dialout $USER` then reboot

## 8) Testing + Demo Playbook

### Quick Test Checklist

- Arduino: DHT values update every ~2s; ultrasonic updates ~10 Hz.
- Serial: JSON lines appear in Pi terminal.
- LEDs: green/blue/red respond to state changes.
- Buzzer: only fires on TOO_CLOSE/ERROR transitions.
- OLED: renders 4 lines, no flicker.

### End-to-End Scenarios

1. Normal approach: Start far -> green; step within 1m -> blue; step close (<45cm) -> red + buzz.
2. Climate alert: Warm sensor with hand -> after 15s, OLED shows "CHECK AIR," red LED.
3. Disconnect test: Unplug ultrasonic -> "SENSORS?" with red LED; reconnect -> returns to green within 5s.

### Demo Choreography (Judge-Friendly)

- "This is GalleryGuard. It creates an invisible buffer around art and verifies climate."
- Judge walks in -> blue LED + OLED "ACTIVE."
- Judge steps closer -> red + buzz + OLED "STEP BACK 32cm."
- Step back -> green + OLED shows temperature/humidity.

### Fallback Plan

- OLED fails: Use LEDs + buzzer; Pi logs still prove data.
- Ultrasonic fails: Switch to demo mode on Pi (`--demo`).
- Serial drops: Arduino shows blue blink; Pi auto-reconnects.

## 9) Presentation Assets (Minimal, Apple-Clean)

### 6-Slide Outline

1. Problem - Art is fragile; distance and climate matter.
2. Solution - GalleryGuard: touchless buffer + climate glance.
3. How it works - Sensors -> Arduino -> Pi -> OLED + alerts.
4. Demo - Step in, step back, see the response.
5. Reliability - Filtering, hysteresis, reconnects.
6. Impact - Safer exhibits, calmer visitors.

### 20-Second Opening Line

"GalleryGuard is a small, silent sentinel that protects art. It watches distance and climate, and guides visitors instantly - no touch, no training."

### 10-Second Wow Line

"Watch this: step closer and it warns you - step back and it's calm again."

### 15-Second Close + Impact

"GalleryGuard makes every exhibit safer and every visitor clearer. Simple, reliable, and ready for real galleries."

### OLED Microcopy (Short, Elegant)

- Line 1: GALLERY GUARD
- Line 2: READY / ACTIVE / STEP BACK / CHECK AIR / SENSORS?
- Line 3: Dist  42cm
- Line 4: T 22C H 48%
