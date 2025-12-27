
# Victron VE.Direct HEX Protocol Python Library

![Victron Energy](https://img.shields.io/badge/Victron-Energy-blue) ![Python Version](https://img.shields.io/badge/python-3.8+-yellow) ![Protocol](https://img.shields.io/badge/Protocol-VE.Direct%20HEX-orange)

A Python script for communicating with **Victron VE.Direct** devices (such as BlueSolar and SmartSolar MPPT charge controllers) using the **HEX** protocol via a UART (serial) connection. This implementation is based on the official documentation: `BlueSolar-HEX-protocol.pdf` (Rev 18).

---

## In this Article
* [Features](#features)
* [Requirements](#requirements)
* [Quick Start](#quick-start)
* [Supported Registers](#supported-registers)
* [Technical Details](#technical-details)

---

## Features

* **Construct Library:** Utilizes the `construct` tool for declarative building and parsing of binary data frames.
* **HEX Validation:** Handles frame formatting, automatic checksum calculation, and response verification.
* **Register Abstraction:**
    * `get_register()`: Automatically handles decoding, scaling, and value mapping (e.g., 3 -> "Bulk").
    * `set_register()`: Safely writes values to the device with full error code handling.

---

## Requirements

Install the required dependencies using the pip package manager:

```bash
pip install pyserial construct
```
## Quick Start
The library offers an extended class called vedirectDEV, which includes pre-built methods for the most common MPPT device parameters.

### Example: Reading Data

```python
from vedirect import vedirectDEV

# Initialize the device (specify the correct serial port)
mppt = vedirectDEV("/dev/ttyAMA0", baudrate=19200, debug=False)

try:
    print(f"Model: {mppt.get_model_name()}")
    print(f"Charger Voltage: {mppt.get_charger_voltage()} V")
    print(f"Panel Power: {mppt.get_panel_power()} W")
    print(f"Current State: {mppt.get_device_state()}")
finally:
    mppt.disconnect()
```

### Example: Controlling the DevicePython
```python
# Turn on the Load Output
mppt.set_load_output_control("ON")

# Set External Control mode (for remote voltage/current control)
mppt.set_device_in_external_control_mode()
```
### Supported Registers

Below is a table of selected registers supported by the module:

| Register Key     | ID (HEX) | Scale | Description                                   |
|------------------|----------|-------|-----------------------------------------------|
| PANEL_VOLTAGE    | 0xEDBB   | 0.01  | Photovoltaic panel voltage (V)                |
| CHARGER_VOLTAGE  | 0xEDD5   | 0.01  | Battery charging voltage (V)                  |
| PANEL_POWER      | 0xEDBC   | 0.01  | Current panel power (W)                       |
| DEVICE_STATE     | 0x0201   | 1     | Current operating state (e.g., Bulk, Float)   |
| YIELD_TODAY      | 0xEDD3   | 0.01  | Today's energy yield (kWh)                    |


## Technical Details
### Checksum Validation
The device verifies every frame for consistency. The sum of all binary bytes (command + data + checksum) modulo 256 must equal $0x55$

$$\left( \sum_{i=1}^{n} \text{byte}_i + \text{checksum} \right) \bmod 256 = 0x55$$

### Asynchronous Data

> [!NOTE]
> VE.Direct MPPT devices broadcast text data (Text Mode) continuously. This library automatically ignores the text stream and   correctly captures HEX responses, which start with the ```:``` character. 

### Safety

> [!WARNING]
> Writing incorrect values to registers (e.g., BATTERY_VOLTAGE_SETTING) can permanently damage your batteries. Always verify the manufacturer's allowed ranges before sending a command.
