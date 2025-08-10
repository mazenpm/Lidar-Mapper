# LIDAR Scanning System - 2DX Final Spatial Mapping Project

## Project Overview

The 2DX Final Spatial Mapping project is a 3D scanning device that captures distances in several 360-degree planes along an orthogonal axis, enabling three-dimensional environment mapping. The system consists of three main components: a microcontroller, a time-of-flight sensor, and a stepper motor.

### System Architecture

The system operates through the following workflow:
1. **Microcontroller Management**: The MSP432E401Y manages all system operations, power delivery, and component configuration
2. **Stepper Motor Control**: Provides 360-degree rotation for the mounted sensor
3. **Distance Sensing**: VL53L1X time-of-flight sensor captures measurements every 45 degrees
4. **Data Transmission**: All data is transmitted via UART to a PC for visualization
5. **3D Visualization**: Python scripts convert the data into 3D coordinate representations

## Hardware Components

### Core Components

| Component | Model | Specifications |
|-----------|-------|----------------|
| **Microcontroller** | Texas Instruments MSP432E401Y | ARM Cortex-M4F, 40MHz, 4 user LEDs, 2 user switches |
| **Stepper Motor** | MOT-28BYJ48 w/ ULN2003 Driver | 5-12V, 512 steps/360°, LED state indicators |
| **Time-of-Flight Sensor** | VL53L1X | 2.65-3.5V, up to 4m range, I2C communication |
| **External Button** | Push Button | GPIO connected, stops data acquisition |

### Communication Interfaces

- **I2C**: Between MSP432E401Y and VL53L1X sensor
- **UART**: Between MSP432E401Y and PC (115200 baud rate)
- **USB**: PC connection for data visualization

## Software Requirements

### Python Dependencies
- **Python Version**: 3.6-3.9 (PySerial support)
- **Open3D**: 3D data processing and visualization (Python 3.9)
- **NumPy**: Numerical computing
- **PySerial**: Serial communication

### Development Environment
- **IDE**: Keil uVision (for MSP432E401Y development)
- **Python IDE**: Python IDLE or any compatible editor

## Project Structure

```
Lidar_Mapper/
├── Keil_MSP432E401Y/          # Microcontroller firmware
│   ├── 2dx_studio_8c.c        # Main application code
│   ├── onboardLEDs.c/h        # LED control functions
│   ├── PLL.c/h                # Phase-locked loop configuration
│   ├── SysTick.c/h            # System timer functions
│   ├── uart.c/h               # UART communication
│   ├── vl53l1_platform_2dx4.c/h  # VL53L1X platform interface
│   └── VL53L1X_api.c/h       # VL53L1X sensor API
├── PC_Visualization/           # Python visualization scripts
│   ├── PC_maamonm.py         # Main visualization script
│   └── demo2dx.xyz           # Sample 3D coordinate data
└── docs/                      # Documentation and datasheets
    ├── VL53L1X Parameter documentation.pdf
    ├── VL53L1X_ULD_API_UM2510_Rev2.pdf
    └── vl53l1x.pdf
```

## Setup Instructions

### 1. Hardware Assembly
1. Connect the MSP432E401Y microcontroller to the stepper motor driver
2. Mount the VL53L1X sensor to the stepper motor
3. Connect the external push button to the designated GPIO pin
4. Ensure proper power supply connections (5V for motor, 3.3V for sensor)

### 2. Firmware Development
1. Open the project in Keil uVision
2. Configure the target device as MSP432E401Y
3. Build and flash the firmware to the microcontroller
4. Verify all components are properly initialized

### 3. Python Environment Setup
```bash
# Install required Python packages
pip install pyserial numpy open3d

# For Python 3.9 specifically (Open3D compatibility)
pip install open3d==0.17.0
```

## Usage Instructions

### 1. System Operation
1. Power on the MSP432E401Y microcontroller
2. Press the onboard button to initiate scanning
3. The stepper motor will rotate 360° in 45° increments
4. Distance measurements are captured at each position
5. Data is transmitted via UART to the connected PC

### 2. Data Visualization
1. Ensure the PC is connected via USB and recognizes COM3 (or appropriate port)
2. Run the Python visualization script:
   ```bash
   cd PC_Visualization
   python PC_maamonm.py
   ```
3. The script will read the serial data and generate a `.xyz` file
4. 3D visualization will display the scanned environment

### 3. Data Format
The system generates 24 total measurements per scan:
- **3 complete scans** × **8 measurements per scan** (45° intervals)
- Output format: `.xyz` file with x, y, z coordinates
- Sample data structure:
  ```
  x1 y1 z1
  x2 y2 z2
  ...
  x24 y24 z24
  ```

## System Specifications

### Performance Characteristics
- **Scan Resolution**: 45° increments (8 measurements per 360° rotation)
- **Range**: Up to 4 meters (VL53L1X sensor limit)
- **Communication Speed**: 115200 baud UART
- **Motor Precision**: 512 steps per full rotation

### Power Requirements
- **Microcontroller**: 3.3V supply
- **Stepper Motor**: 5-12V supply
- **VL53L1X Sensor**: 2.65-3.5V supply

---

