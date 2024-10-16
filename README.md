# AlphaController - Microcontroller Implementation

## Overview

This repository contains the microcontroller firmware for the **[AlphaController](https://drive.google.com/file/d/19e3JAtkkvVFHQCaV6ak2ICiuk-VxShHx/view?usp=share_link)** project, developed as part of a diploma thesis at **HTBLuVA Salzburg** by **Yll Kryeziu** and **Marvin Winkler**.

The AlphaController is a power control system designed to regulate the temperature of electrical heaters. It improves on existing PWM-based systems by using phase control and burst fire control for more precise power adjustments and to eliminate temperature fluctuations. The microcontroller controls the heaters via Solid State Relays (SSRs) based on user-defined parameters.

### Key Features

- **Phase Control** and **Burst Fire Control** for efficient power regulation.
- Support for 7 operational modes, including:
  - Permanent ON/OFF
  - Uncorrected and corrected phase control (voltage and power-based)
  - Optimized burst fire control using Delta-Sigma modulation.
- **Zero Crossing Detection** for accurate synchronization with the AC power supply.
- Real-time control through UART communication with a user interface (configured via Delphi application).
- Multi-channel control for up to 3 heaters (including support for 3-phase operation).

## Hardware Requirements

- **Microcontroller**: ATmega644P used to generate control signals for the Solid State Relays (SSRs).
- **Zero-Crossing Detector**: Synchronizes the control signals with the AC mains.
- **Solid State Relays (SSRs)**: Used for switching the heaters on and off.
- **UART Communication**: Allows communication between the microcontroller and the user interface.

## Installation and Setup

1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/AlphaController-AVR.git
    ```

2. Open the project in your preferred **AVR development environment** (e.g., Atmel Studio, Arduino IDE).

3. Compile and upload the firmware to the **ATmega644P** microcontroller.

4. Connect the microcontroller to the external hardware (Solid State Relays, Zero Crossing Logic, etc.) as specified in the documentation.

5. The microcontroller will communicate with the **Delphi application** over UART to receive user-configured parameters and control the heaters accordingly.

## Modes of Operation

The AlphaController supports seven operational modes:
1. **Permanent ON**
2. **Permanent OFF**
3. **Phase Control (Uncorrected)**
4. **Phase Control (Voltage-Corrected)**
5. **Phase Control (Power-Corrected)**
6. **Burst Fire Control**
7. **Optimized Burst Fire Control** (using a Delta-Sigma modulator)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors

- **Yll Kryeziu** - Developer, Microcontroller Software
- **Marvin Winkler** - Developer, Hardware Design
