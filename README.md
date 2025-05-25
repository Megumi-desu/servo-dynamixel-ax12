# Servo Dynamixel AX12 Library for STM32 HAL

![Image](https://github.com/user-attachments/assets/60799ba2-2870-4c05-a430-3b95ad65013d))

A comprehensive STM32 HAL-based library for controlling Dynamixel AX12/AX12A servo motors. This library provides a simple and efficient interface for controlling these high-performance servo motors in your robotics projects.

## Features

- Full support for Dynamixel AX12/AX12A servo motors
- Built on STM32 HAL for easy integration with STM32 microcontrollers
- UART communication with direction pin control for half-duplex communication
- Functions for all common servo operations:
  - Position control with speed adjustment
  - Reading current position
  - Setting compliance slope (stiffness control)
  - Setting maximum torque
  - Setting punch (minimum current)
  - EEPROM locking for parameter protection
  - Servo reset capabilities
  - Servo ping for connection verification
- Synchronous control of multiple servos for efficient, coordinated movements

## Hardware Requirements

- STM32 microcontroller (tested on STM32F4 series)
- UART port (default: UART2)
- One GPIO pin for direction control
- Logic level converter/buffer (SN74HC241N recommended)
- Dynamixel AX12/AX12A servo motor(s)
- Power supply appropriate for Dynamixel servos (7~12V)

## Hardware Connection

The Dynamixel AX12 uses a half-duplex UART interface, which requires a direction control signal. The following diagram shows the basic connection setup:

```
STM32                     Logic Buffer           Dynamixel AX12
            +-------------+ (SN74HC241N)  +-------------+
            |             |               |             |
TX (UART2) -+-------------+---------------+ Data        |
            |             |               |             |
RX (UART2) -+-------------+               |             |
            |             |               |             |
GPIO PA0   -+- Direction -+               |             |
            |             |               |             |
            +-------------+               +-------------+
```

The direction pin toggles between transmit mode (HIGH) and receive mode (LOW).

## Installation

1. Clone this repository into your project directory:
   ```
   git clone https://github.com/Megumi-desu/servo-dynamixel-ax12.git
   ```

2. Include the library files in your STM32 project:
   - Add `dynamixel_ax12.c` to your source files
   - Add `dynamixel_ax12.h` to your include path

3. Configure your STM32CubeIDE/CubeMX project:
   - Configure UART2 (or modify the code to use your preferred UART)
   - Configure a GPIO pin for direction control (default: PA0)
   - Make sure to enable the necessary clocks and peripherals

## Basic Usage

Here's a quick example of how to use the library:

```c
#include "dynamixel_ax12.h"

void main(void) {
  // Initialize HAL and peripherals first
  /* ... */
  
  // Initialize Dynamixel communication
  Dynamixel_Init();
  
  // Ping servo to verify connection
  if (Dynamixel_Ping(1) == HAL_OK) {
    // Set max torque
    Dynamixel_SetMaxTorque(1, 512);
    
    // Set servo to position 150 degrees with speed 300
    Dynamixel_SetAngle(1, 150, 300);
    
    // Read current position
    uint16_t position;
    Dynamixel_ReadPosition(1, &position);
  }
}
```

## API Reference

### Initialization

```c
void Dynamixel_Init(void);
```
Initializes the Dynamixel communication interface.

### Basic Control Functions

```c
// Move servo to a position (0-1023) with speed (0-1023)
HAL_StatusTypeDef Dynamixel_MoveSpeed(uint8_t ID, uint16_t Position, uint16_t Speed);

// Move servo to an angle (0-300 degrees) with speed (0-1023)
HAL_StatusTypeDef Dynamixel_SetAngle(uint8_t ID, uint16_t Angle, uint16_t Speed);

// Read the current position (0-1023)
HAL_StatusTypeDef Dynamixel_ReadPosition(uint8_t ID, uint16_t *Position);
```

### Advanced Control Functions

```c
// Set the compliance slope (stiffness) of the servo
HAL_StatusTypeDef Dynamixel_SetSlope(uint8_t ID, uint8_t CW_Slope, uint8_t CCW_Slope);

// Set the punch (minimum current) of the servo
HAL_StatusTypeDef Dynamixel_SetPunch(uint8_t ID, uint16_t Punch);

// Set the maximum torque of the servo
HAL_StatusTypeDef Dynamixel_SetMaxTorque(uint8_t ID, uint16_t MaxTorque);

// Lock the EEPROM to protect settings
HAL_StatusTypeDef Dynamixel_LockEEPROM(uint8_t ID);

// Reset servo to factory settings
HAL_StatusTypeDef Dynamixel_Reset(uint8_t ID);

// Check if servo is connected
HAL_StatusTypeDef Dynamixel_Ping(uint8_t ID);
```

### Synchronous Control

```c
// Send synchronized position and speed commands to multiple servos
HAL_StatusTypeDef Start_Pose_SYNC(void);
```
This function sends position and speed commands to multiple servos simultaneously for coordinated movements. It uses the global `posArray` and `speedArray` arrays to store the target positions and speeds for each servo.

## Configuration

The library uses the following default configuration, which can be modified in `dynamixel_ax12.h`:

- UART: UART2
- Direction pin: PA0
- Baud rate: 1,000,000 bps
- TX delay: 20 µs

## Notes on Performance

- The Dynamixel AX12 requires precise timing for successful communication. The library includes delay functions to ensure proper timing.
- For high-performance applications, consider using DMA for UART communication.
- The sync write function can significantly reduce communication overhead when controlling multiple servos.

## Troubleshooting

Common issues and their solutions:

1. **No response from servo**
   - Check power supply (7~12V required)
   - Verify UART connections
   - Ensure direction pin is working correctly
   - Check servo ID (default is 1)

2. **Erratic movement**
   - Verify proper voltage level conversion
   - Check for communication errors
   - Ensure timing delays are appropriate

3. **Communication errors**
   - Verify baud rate settings
   - Check for proper line termination
   - Ensure buffer IC is functioning correctly

## Contributing

Contributions to improve the library are welcome! Please feel free to submit a pull request or open an issue.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Developed by [Megumi-desu](https://github.com/Megumi-desu)
- Special thanks to Robotis for the Dynamixel servo documentation
