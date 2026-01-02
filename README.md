# HW039_Simple Library Documentation

## 📋 Overview

The `HW039_Simple` library for Arduino is designed to control the HW-039 motor driver (L298N compatible) **without using the STBY (Standby) pin**. The library provides a simple and intuitive interface for controlling the speed and direction of a DC motor.

## 🚀 Quick Start

### Installation

1. Create an `HW039_Simple` folder in your Arduino `libraries` directory
2. Place these two files inside:
   - `HW039_Simple.h`
   - `HW039_Simple.cpp`

### Basic Example

```cpp
#include "HW039_Simple.h"

// Create driver object (RPWM pin, LPWM pin)
HW039_Simple motor(5, 6);

void setup() {
    motor.begin();  // Initialize driver
}

void loop() {
    // Rotate forward at 75% speed
    motor.go(1, 192);  // 192/255 ≈ 75%
    delay(2000);
    
    // Stop
    motor.stop();
    delay(1000);
}
```

## 🔌 HW-039 Driver Connection

### Wiring Diagram

```
HW-039 Driver → Arduino UNO/Nano
─────────────────────────────────
RPWM         → PWM Digital Pin (3,5,6,9,10,11)
LPWM         → PWM Digital Pin
GND          → GND (common ground)
VCC          → 5V (logic power)
VM           → Motor power (6-12V, separate source)
OUT1, OUT2   → DC motor terminals
```

⚠️ **Important!** Motor power (VM) must be from a separate power supply. Do not connect powerful motors directly to Arduino pins!

## 📖 API Documentation

### Constructor

```cpp
HW039_Simple(uint8_t rpwm_pin, uint8_t lpwm_pin)
```

**Parameters:**
- `rpwm_pin` - Arduino pin number for RPWM signal (must support PWM)
- `lpwm_pin` - Arduino pin number for LPWM signal (must support PWM)

**Example:**
```cpp
// Using pins 5 and 6 on Arduino UNO
HW039_Simple motor(5, 6);
```

### Methods

#### `begin()`
Initializes the driver, configures pins, and stops the motor.

```cpp
void begin();
```

**Usage:**
```cpp
void setup() {
    motor.begin();
}
```

#### `go()`
Main motor control function.

```cpp
void go(int8_t direction, uint8_t speed);
```

**Parameters:**
- `direction` - rotation direction:
  - `1` or positive number = forward
  - `-1` or negative number = backward
  - `0` = stop
- `speed` - rotation speed (0-255):
  - `0` = stop
  - `255` = maximum speed

**Examples:**
```cpp
// Forward at half speed
motor.go(1, 128);

// Backward at full speed
motor.go(-1, 255);

// Stop
motor.go(0, 0);  // or simply motor.stop()
```

#### `goPercent()`
Control motor speed in percentages.

```cpp
void goPercent(int8_t speed_percent);
```

**Parameters:**
- `speed_percent` - speed in percentages (-100..100):
  - `100` = maximum forward
  - `-100` = maximum backward
  - `0` = stop

**Example:**
```cpp
// 50% speed forward
motor.goPercent(50);

// 75% speed backward
motor.goPercent(-75);
```

#### `goSmooth()`
Smooth speed change with ramp effect (acceleration/deceleration).

```cpp
void goSmooth(int8_t direction, uint8_t speed, uint16_t ramp_time = 500);
```

**Parameters:**
- `direction` - direction (1 = forward, -1 = backward)
- `speed` - target speed (0-255)
- `ramp_time` - smooth acceleration time in milliseconds (default 500ms)

**Example:**
```cpp
// Smooth acceleration to 200 over 1 second
motor.goSmooth(1, 200, 1000);
```

#### `stop()`
Immediate motor stop (zero PWM).

```cpp
void stop();
```

**Example:**
```cpp
motor.stop();
```

#### `brake()`
Active braking (short circuit motor windings).

```cpp
void brake();
```

**Note:** This method creates a short pulse for quick motor stopping.

**Example:**
```cpp
// Quick stop with braking
motor.brake();
```

#### `setMinSpeed()`
Set minimum speed (deadzone).

```cpp
void setMinSpeed(uint8_t min_speed);
```

**Parameters:**
- `min_speed` - minimum PWM value (0-255)

**Example:**
```cpp
// Set minimum speed to 30/255
motor.setMinSpeed(30);

// Now when calling go(1, 10), speed 30 will be used
motor.go(1, 10);  // Actual speed = 30
```

## 🎛️ Advanced Examples

### Example 1: Joystick Control

```cpp
#include "HW039_Simple.h"

HW039_Simple motor(5, 6);
const int joyPin = A0;  // Joystick on analog pin

void setup() {
    motor.begin();
    motor.setMinSpeed(50);  // Ignore very small values
    Serial.begin(9600);
}

void loop() {
    int joyValue = analogRead(joyPin);
    
    // Convert joystick value (-512..511) to percentages (-100..100)
    int speedPercent = map(joyValue, 0, 1023, -100, 100);
    
    // Filter noise (deadzone in center)
    if (abs(speedPercent) < 10) {
        motor.stop();
    } else {
        motor.goPercent(speedPercent);
    }
    
    delay(20);
}
```

### Example 2: Automatic Movement Sequence

```cpp
#include "HW039_Simple.h"

HW039_Simple motor(5, 6);

void setup() {
    motor.begin();
    Serial.begin(9600);
}

void loop() {
    Serial.println("Starting test sequence");
    
    // Smooth acceleration forward
    for (int speed = 0; speed <= 255; speed += 5) {
        motor.go(1, speed);
        delay(50);
    }
    delay(1000);
    
    // Smooth deceleration
    for (int speed = 255; speed >= 0; speed -= 5) {
        motor.go(1, speed);
        delay(50);
    }
    motor.stop();
    
    delay(2000);
    
    // Reversing with pause
    motor.goPercent(80);
    delay(1000);
    motor.brake();  // Quick stop
    delay(500);
    motor.goPercent(-80);
    delay(1000);
    motor.stop();
    
    delay(3000);
}
```

### Example 3: Two Motor Control (Robot)

```cpp
#include "HW039_Simple.h"

// Two motors for robot
HW039_Simple motorLeft(5, 6);   // Left motor
HW039_Simple motorRight(9, 10); // Right motor

void setup() {
    motorLeft.begin();
    motorRight.begin();
}

void robotMove(int leftSpeed, int rightSpeed) {
    // Left motor
    if (leftSpeed > 0) {
        motorLeft.go(1, leftSpeed);
    } else if (leftSpeed < 0) {
        motorLeft.go(-1, -leftSpeed);
    } else {
        motorLeft.stop();
    }
    
    // Right motor
    if (rightSpeed > 0) {
        motorRight.go(1, rightSpeed);
    } else if (rightSpeed < 0) {
        motorRight.go(-1, -rightSpeed);
    } else {
        motorRight.stop();
    }
}

void loop() {
    // Forward
    robotMove(200, 200);
    delay(2000);
    
    // Turn left
    robotMove(100, 200);
    delay(1000);
    
    // Turn right
    robotMove(200, 100);
    delay(1000);
    
    // Rotate in place
    robotMove(150, -150);
    delay(800);
    
    // Stop
    robotMove(0, 0);
    delay(2000);
}
```

## ⚙️ Technical Details

### PWM Frequency

The library automatically configures optimal PWM frequency for different boards:

- **Arduino UNO/Nano:**
  - Pins 5,6: ~980 Hz
  - Other PWM pins: ~490 Hz

- **ESP32/ESP8266:**
  - Automatically set to 20 kHz
  - Frequency beyond audible range

### Operation Algorithm

```
direction > 0 (forward):
  RPWM = speed (0-255)
  LPWM = 0

direction < 0 (backward):
  RPWM = 0
  LPWM = speed (0-255)

direction = 0 (stop):
  RPWM = 0
  LPWM = 0
```

### Implementation Features

1. **Overload Protection:** All speed values limited to 0-255 range
2. **Minimum Speed:** Deadzone implemented to prevent motor humming
3. **Soft Start:** `goSmooth()` method prevents abrupt starts
4. **Cross-platform:** Supports Arduino, ESP32, ESP8266

## 🔧 Troubleshooting

### Motor Not Rotating
1. Check power connection (VM = 6-12V)
2. Ensure common ground (GND) is connected
3. Verify pins support PWM (3,5,6,9,10,11 on UNO)

### Motor Humming But Not Rotating
1. Increase minimum speed: `motor.setMinSpeed(50);`
2. Check motor power supply adequacy
3. Increase power supply voltage (VM)

### Driver Overheating
1. Ensure heatsink is attached to the IC
2. Check motor current draw (max 2A per channel)
3. Use active cooling if necessary

### Noise During Operation (High-pitched Whine)
1. On ESP, increase PWM frequency: `analogWriteFreq(25000);`
2. Add 100nF ceramic capacitor between VCC and GND of driver

## 📈 Additional Features

### Custom PWM Frequency

```cpp
// For ESP boards you can change frequency
void setup() {
    motor.begin();
    #if defined(ESP32) || defined(ESP8266)
        analogWriteFreq(25000); // 25 kHz
    #endif
}
```

### Use with Encoders

```cpp
// Example with speed measurement
HW039_Simple motor(5, 6);
volatile int encoderCount = 0;

void encoderISR() {
    encoderCount++;
}

void setup() {
    motor.begin();
    attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
}

void loop() {
    // Speed PID controller
    motor.goPercent(calculateSpeedPID(encoderCount));
}
```

## 📄 License and Compatibility

**Compatibility:** Arduino UNO, Nano, Mega, ESP32, ESP8266

**Supported Drivers:** HW-039, L298N, and other dual H-bridges

**License:** MIT License - free to use and modify

## 🎯 Quick Reference Cheat Sheet

```cpp
// Initialization
HW039_Simple motor(RPWM_pin, LPWM_pin);
motor.begin();

// Basic Commands
motor.go(direction, speed);        // 1/-1, 0-255
motor.goPercent(percent);          // -100..100
motor.goSmooth(direction, speed);  // Soft start
motor.stop();                      // Stop
motor.brake();                     // Braking

// Settings
motor.setMinSpeed(min_speed);      // Deadzone
```

## 🤝 Contributing

Found a bug or have suggestions for improvement? Create an issue or pull request in the project repository.

---

**Version:** 1.0.0  
**Author:** GLEB
**Date:** 2026
