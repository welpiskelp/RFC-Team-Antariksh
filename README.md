# `RFCLast` Flight Computer

**Project Author:** Team Antariksh

## Overview

This repository contains the flight computer (FC) software for the `RFCLast` rocket. It is an Arduino-based (Teensy) project designed to manage all aspects of the flight, including sensor data acquisition, flight state determination, data logging, and the deployment of recovery and payload systems.

The system is built to be robust, featuring state recovery via EEPROM to handle in-flight resets and a multi-interface logging system that transmits data via LoRa, saves to an SD card, and prints to USB Serial.

## Key Features

* **Finite State Machine (FSM):** The rocket's logic is driven by a state machine that progresses from `INITIALIZING` to `STANDBY`, `LIFTOFF_CONFIRMED`, `APOGEE_REACHED`, `DESCENT_STARTED`, and `TOUCHDOWN_CONFIRMED`.
* **State Recovery:** The current flight state and flags are saved to EEPROM on every change, allowing the FC to resume its logic from the correct state if a power cycle or reset occurs .
* **Telecommand System:** Before launch, the FC waits in `STANDBY` mode for LoRa-based commands. It accepts "START" (to begin flight logic), "ERASE" (to wipe the EEPROM), and "CALIBRATE" (to re-run sensor calibration).
* **Modular Logging:** A central logger module (`logger.cpp`) routes telemetry data to multiple destinations:
    * `com_logger`: For live telemetry via USB Serial .
    * `sdcard_logger`: For high-fidelity logging to an onboard SD card .
    * `lora_logger`: For long-range radio transmission to the ground station .
* **Redundant Apogee Detection:** Apogee is primarily detected by analyzing altitude changes . A redundant, time-based check (`FLIGHT_TIME_THRESHOLD`) also triggers apogee events as a failsafe.
* **Pyro Channel Control:** Manages five separate pyro channels for:
    * Payload Deployment (x2 channels)
    * Main Parachute Deployment (x2 channels, one redundant)
    * Reefing Line Cut

## Hardware Configuration

This software is configured for the following hardware:

* **IMU:** Adafruit LSM6DSO32 (on SPI1)
* **Barometer:** Adafruit BMP390 (on SPI1)
* **GPS:** TinyGPS++ compatible module (on `Serial8`)
* **Radio:** LoRa Module (on SPI1)
* **Storage:** Onboard SD Card (`BUILTIN_SDCARD`) & EEPROM
* **Audio:** Passive Buzzer

### Pinout

| Function | Pin |
| :--- | :--- |
| **SPI1 SCK** | 27 | 
| **SPI1 MISO** | 1 |
| **SPI1 MOSI** | 26 |
| **BMP390 CS** | 28 |
| **LSM6DSO32 CS** | 5 |
| **LoRa CS** | 10 |
| **LoRa RST** | 9 |
| **LoRa IRQ** | 2 |
| **Buzzer** | 23 |
| **Battery Voltage** | 41 |
| **Pyro: Main 1** | 14 |
| **Pyro: Main 2** | 0 |
| **Pyro: Reefing** | 28 |
| **Pyro: Payload 1** | 31 |
| **Pyro: Payload 2** | 33 |

## Software Architecture

The code is organized into several key modules:

* **`RFCLast.ino`:** The main Arduino sketch. It contains the `setup()` and `loop()` functions, the primary state machine logic, sensor reading, and hardware initialization routines.
* **`state.h/.cpp`:** Defines the core `state_t` enum (e.g., `STATE_LIFTOFF_CONFIRMED`) and the `update_state` function that manages transitions between states .
* **`logger.h/.cpp`:** A central logging system that formats data into the standard telemetry packet. It holds a registry of logging interfaces and dispatches data to them. It also manages EEPROM logging logic .
* **`com_logger.h/.cpp`:** A logger interface for writing telemetry to the USB `Serial` port .
* **`sdcard_logger.h/.cpp`:** A logger interface for writing telemetry to a file (`LOGNEW.TXT`) on the SD card .
* **`lora_logger.h/.cpp`:** A logger interface for transmitting telemetry packets via LoRa .
* **`buzzer.h/.cpp`:** Manages the passive buzzer, providing audible feedback for events like initialization success, failure, and landing .

## Flight Logic (Mission Profile)

1.  **Power On (STATE_INITIALIZING):**
    * The FC initializes all sensors, pyro channels, and logging systems .
    * It attempts to recover its last known state from EEPROM.
    * If no valid state is found (fresh boot), it performs sensor calibration and proceeds to `STATE_STANDBY`.
    * If a state is recovered, it skips `STANDBY` and resumes logic from its previous state (e.g., `STATE_LIFTOFF_CONFIRMED`) .

2.  **Ready for Launch (STATE_STANDBY):**
    * The FC enters LoRa receive mode and waits for a telecommand.
    * Upon receiving "START", it exits this state and prepares for liftoff detection .

3.  **Boost (STATE_LIFTOFF_CONFIRMED):**
    * This state is entered when acceleration exceeds 20.0 m/s² and altitude is above 50.0 m.
    * The FC monitors altitude to detect apogee.
    * A redundant timer starts, and if apogee is not detected by `FLIGHT_TIME_THRESHOLD` (50 seconds), it will force an apogee event[cite: 464].

4.  **Apogee (STATE_APOGEE_REACHED):**
    * Triggered by either altitude-based apogee detection or the time-based failsafe .
    * The FC immediately fires both Payload pyro channels and the Main Chute 1 pyro channel .
    * A 7-second timer (`REDUNDANT_PARACHUTE_TIME_THRESHOLD`) is started for the redundant main chute.

5.  **Descent (STATE_MAIN_EJECTED / STATE_DESCENT_STARTED):**
    * The FC fires the Main Chute 2 pyro channel after the 7-second timer expires.
    * It monitors altitude during descent.
    * When the rocket passes below 350.0 m (`MAIN_CHUTE_ALTITUDE`), the Reefing pyro channel is fired.

6.  **Landing (STATE_TOUCHDOWN_CONFIRMED):**
    * *Note: The logic for detecting touchdown is not yet implemented* .
    * Once this state is entered, all pyro channels are shut down.
    * The FC flushes any remaining data from EEPROM to the SD card (if the flag `eep_sd_flag` was set).

---

