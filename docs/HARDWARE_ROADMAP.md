# Hardware Integration Roadmap

## Current Status
✅ **Clean slate** - All TMC5160 stepper motor code removed  
✅ **Base system** - UART console, logging, command processing intact  
✅ **MCU** - STM32F303K8 (64MHz, 12KB RAM, 64KB Flash)

## Target Hardware

### 1. MPU9250 - 9-Axis IMU
**Purpose:** Balance sensing (accelerometer + gyroscope)  
**Interface:** I2C (primary) or SPI (faster)  
**Pins Required:**
- I2C1: PB6 (SCL), PB7 (SDA)
- Or SPI1: PA5 (SCK), PA6 (MISO), PA7 (MOSI), PA4 (CS)
- INT pin for motion detection (optional)

**Implementation Steps:**
1. Initialize I2C/SPI communication
2. Read WHO_AM_I register to verify connection
3. Configure accelerometer and gyroscope ranges
4. Implement complementary or Kalman filter for angle estimation
5. Calibration routine for bias removal

### 2. NRF24L01 - 2.4GHz Wireless Transceiver
**Purpose:** Remote control and telemetry  
**Interface:** SPI  
**Pins Required:**
- SPI1 (shared with MPU9250 if using I2C for IMU)
- CE: GPIO output (chip enable)
- CSN: GPIO output (SPI chip select)
- IRQ: GPIO input (interrupt, optional)

**Implementation Steps:**
1. Configure SPI for NRF24L01 timing
2. Initialize radio (channel, address, data rate)
3. Set up receive/transmit pipelines
4. Implement packet protocol for control commands
5. Add telemetry data transmission

### 3. DRV8256E - Dual DC Motor Drivers
**Purpose:** Drive 2x DC motors with encoders  
**Interface:** PWM (speed) + GPIO (direction)  
**Pins Required per motor:**
- PH (Phase/Direction): GPIO output
- EN (Enable/PWM): Timer PWM output
- SLEEP: GPIO output (tie HIGH for normal operation)
- FAULT: GPIO input (error detection, optional)

**Motor 1:**
- PH: PA0, EN: TIM2_CH1 (PA0 alternate) or separate
- Adjust pinout based on timer availability

**Motor 2:**
- PH: PA1, EN: TIM2_CH2 (PA1 alternate)

**Implementation Steps:**
1. Configure timers for PWM generation (20kHz recommended)
2. Set up GPIO for direction control
3. Implement motor control class with speed/direction methods
4. Add current limiting via VREF (hardware or software)

### 4. Quadrature Encoders
**Purpose:** Closed-loop motor position/velocity feedback  
**Interface:** Timer in Encoder Mode  
**Pins Required per encoder:**
- Channel A: Timer input
- Channel B: Timer input

**Encoder 1:**
- TIM3: PA6 (CH1), PA7 (CH2)

**Encoder 2:**
- TIM4: PB6 (CH1), PB7 (CH2)

**Implementation Steps:**
1. Configure timers in encoder mode
2. Set up counter for position tracking
3. Calculate velocity from position delta
4. Implement overflow handling for continuous rotation

## Pin Allocation Summary

| Peripheral | Pin(s) | Function |
|------------|--------|----------|
| UART2 | PA2, PA3 | Console/Debug |
| I2C1 | PB6, PB7 | MPU9250 (option 1) |
| SPI1 | PA5, PA6, PA7 | MPU9250 (option 2) or NRF24L01 |
| TIM2 | PA0, PA1 | Motor PWM (EN signals) |
| TIM3 | PA6, PA7 | Encoder 1 |
| TIM4 | PB6, PB7 | Encoder 2 |
| GPIO | Various | Motor DIR, NRF CE/CSN, etc. |

⚠️ **Note:** Some pin conflicts exist. Final assignment depends on MPU9250 interface choice (I2C vs SPI).

## Software Architecture

### Control Loop Structure
```
Main Loop (1ms):
├── Read IMU data (MPU9250)
├── Read encoder positions/velocities
├── Calculate tilt angle (sensor fusion)
├── Run PID controller (balance + velocity)
├── Update motor PWM outputs
├── Process wireless commands (NRF24L01)
└── Send telemetry data
```

### PID Controllers Needed
1. **Balance PID** - Keep robot upright (angle → motor torque)
2. **Velocity PID** - Control forward/backward speed (optional)
3. **Position PID** - Navigate to target position (optional)

### Key Parameters to Tune
- PID gains (Kp, Ki, Kd)
- Sensor filter coefficients
- Motor response curves
- Loop timing and frequencies

## Next Steps
1. ✅ Clean up TMC5160 code
2. ⏳ Wire MPU9250 and test I2C communication
3. ⏳ Implement angle calculation and filtering
4. ⏳ Wire DRV8256E and test PWM motor control
5. ⏳ Wire encoders and verify position tracking
6. ⏳ Implement basic balance controller
7. ⏳ Add NRF24L01 for wireless control
8. ⏳ Tune PID and test balancing

## Lessons Learned from TMC5160
- ✅ SPI implementation is solid and reusable
- ✅ Diagnostic logging extremely valuable for debugging
- ✅ Breadboards unreliable for high-current motor connections
- ✅ Always verify power supply voltages for different chips
- ✅ Hardware protection features (like open load detection) work as designed

---
**Last Updated:** 2025-11-16  
**Project:** Self-Balancing Robot  
**MCU:** STM32F303K8 Nucleo-32
