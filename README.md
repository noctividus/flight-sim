# Flight Simulator Firmware

This firmware turns a **SparkFun Pro Micro (ATmega32U4)** into a USB flight simulator HID for elevator, aileron, rudder, air brakes, and wheel brake (digital), plus a few buttons. Analog axes are read via an **ADS1015** I²C ADC to maximize resolution.

## Required Libraries

- **Arduino Joystick Library** (Matthew Heironimus) — exposes HID joystick
  - <https://github.com/MHeironimus/ArduinoJoystickLibrary>
- **SparkFun ADS1015 Arduino Library** — external 12‑bit ADC
  - <https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library>
- **Encoder** (PJRC) — quadrature encoder for wheel brake
  - <https://www.pjrc.com/teensy/td_libs_Encoder.html>
- **Wire** — built‑in I²C

> Libraries referenced in code: `Joystick.h`, `SparkFun_ADS1015_Arduino_Library.h`, `Wire.h`, `Encoder.h`.

## MCU & Peripherals

> See the [Hardware Readme](/hardware/README.md) for more info.

- **MCU:** SparkFun Pro Micro (5V/16 MHz or 3.3V/8 MHz)
- **ADC:** SparkFun **ADS1015** (I²C)
  - Default gain set to `ADS1015_CONFIG_PGA_1` to avoid saturation.
- **Brake:** Quadrature **encoder** (CLK/DT)
- **Buttons:** Joystick pushbutton (active‑LOW), Brake button, Release switch

### I²C Wiring (ADS1015 ↔ Pro Micro)
> This is best accomplished using the sparkfun qwic connector.
> The hall effect sensors are 3.3V, so you must use the power off the QWIIC.

| ADS1015 | Pro Micro | Notes |
|---|---|---|
| VCC | VCC (3.3 V or 5 V)* | Match board voltage |
| GND | GND | Common ground |
| **SDA** | **SDA** (labeled) | I²C data |
| **SCL** | **SCL** (labeled) | I²C clock |

\* The ADS1015 accepts 2–5.5 V; use the same voltage as your sensors/pots.

### Axis & Input Mapping

Analog axes are read from the ADS1015 single‑ended channels:

| Function | Code Symbol | ADS1015 Channel | Notes |
|---|---|---:|---|
| **Elevator (Pitch)** | `elevatorPinADC` | **A0 (0)** | Hall Effect Sensor → A0, other ends to VCC/GND |
| **Aileron (Roll)** | `aileronPinADC` | **A1 (1)** | Hall Effect Sensor → A1 |
| **Rudder (Yaw)** | `RUDDER_ADC_PIN` | **A2 (2)** | Hall Effect Sensor → A2 |

Digital inputs on the Pro Micro:

| Function | Code Symbol | Pro Micro Pin | Mode | Active Level |
|---|---|---:|---|---|
| **Joystick Button** | `JOYSTICK_BTN_PIN` | **D4** | `INPUT_PULLUP` | **LOW** |
| **Towhook Release** | `RELEASE_PIN` | **D8** | `INPUT_PULLUP` | **LOW** |
| **Wheel Brake** | `BRAKE_BTN_PIN` | **D9** | `INPUT` | External pull‑up/down required |
| **Air Brake Encoder CLK** | `BRAKE_CLK_PIN` | **D0** | — | Quadrature A |
| **Air Brake Encoder DT** | `BRAKE_DT_PIN` | **D1** | — | Quadrature B |
| *(Optional LED)* | `JOYSTICK_LED_PIN` | **D5** | — | Defined, not used in code |

> Note: D0/D1 are also hardware Serial (RX/TX) on the 32U4. The sketch uses `Serial`, so avoid connecting external serial devices on these pins during normal operation.


## Features

- USB HID joystick with Pitch, Roll, Rudder, Throttle/Brake and buttons
- On‑device **calibration**, persisted to EEPROM
- Adjustable encoder‑based brake input

## Build & Upload

1. Install the required libraries (above) via Library Manager or from GitHub.
2. Board: **SparkFun Pro Micro** (ATmega32U4). Select correct voltage/clock. (5V)
3. Build and Upload via PlatformIO
4. Verify in Windows/Mac game controller panel that axes/buttons respond.

## Troubleshooting

- **No axis movement:** Check ADS1015 power and I²C (SDA/SCL continuity). Ensure pots are wired as voltage dividers.
- **Axis saturates/clips:** Confirm gain = 1 in ADS1015 setup and sensor voltage matches VCC.
- **Encoder erratic:** Keep short, shielded wires; avoid using D0/D1 for other peripherals.

## Development Plan
 - Add Flaps

---

**Author:** Rich Mayfield  
**Target Board:** SparkFun Pro Micro 5V
