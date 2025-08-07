# STM32L4R9AI-MPU6050-Driver
MPU6050 6-axis IMU driver for STM32L4R9AI Discovery board using I2C3 interface. Reads accelerometer, gyroscope, and temperature data every 500ms via UART2 serial output.

# MPU6050 Driver for STM32L4R9AI Discovery Board

A complete MPU6050 6-axis IMU (Inertial Measurement Unit) driver implementation for the STM32L4R9AI Discovery board using STM32CubeIDE and HAL library.

## Features

- ✅ Read 3-axis accelerometer data (±2g range)
- ✅ Read 3-axis gyroscope data (±250°/s range)
- ✅ Temperature sensor reading
- ✅ I2C3 communication interface
- ✅ UART2 serial output for data monitoring
- ✅ 500ms sampling rate
- ✅ Real-time data display via serial terminal

## Hardware Requirements

### Components
- STM32L4R9AI Discovery Board
- MPU6050 6-axis IMU sensor module
- Jumper wires for connections

### Pin Connections

| MPU6050 Pin | STM32L4R9AI Pin | Arduino Pin | Function |
|-------------|-----------------|-------------|----------|
| VCC         | 3.3V           | -           | Power Supply |
| GND         | GND            | -           | Ground |
| SDA         | PG8            | ARD14       | I2C Data |
| SCL         | PG7            | ARD15       | I2C Clock |
| AD0         | GND            | -           | Address Select (0x68) |

## Software Configuration

### STM32CubeMX Settings

1. **I2C3 Configuration:**
   - Mode: I2C
   - I2C Speed: Standard Mode (100 KHz)
   - SDA Pin: PG8 (ARD14)
   - SCL Pin: PG7 (ARD15)

2. **UART2 Configuration:**
   - Mode: Asynchronous
   - Baud Rate: 115200
   - Word Length: 8 bits
   - Stop Bits: 1
   - Parity: None

3. **GPIO Configuration:**
   - Enable GPIO ports for I2C3 and UART2
   - Configure pins as alternate function

## Installation & Usage

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/STM32L4R9AI-MPU6050-Driver.git
cd STM32L4R9AI-MPU6050-Driver
```

### 2. STM32CubeIDE Setup
1. Open STM32CubeIDE
2. Import the project: `File > Import > Existing Projects into Workspace`
3. Select the cloned repository folder
4. Build the project (`Ctrl + B`)

### 3. Hardware Setup
1. Connect MPU6050 to STM32L4R9AI according to the pin table above
2. Ensure proper power supply (3.3V) to MPU6050
3. Connect ST-Link debugger/programmer

### 4. Programming & Monitoring
1. Flash the program to STM32L4R9AI board
2. Open serial terminal (PuTTY, Tera Term, or Arduino Serial Monitor)
3. Configure: 115200 baud, 8N1
4. Connect to the appropriate COM port
5. Reset the board to start data transmission

## Sample Output

```
MPU6050 Initialization Successful!
Starting MPU6050 data reading...
Acc: X=0.02, Y=0.01, Z=1.00 | Gyro: X=-0.45, Y=0.23, Z=0.12 | Temp=25.67°C
Acc: X=0.03, Y=0.02, Z=0.99 | Gyro: X=-0.43, Y=0.21, Z=0.10 | Temp=25.68°C
Acc: X=0.01, Y=0.00, Z=1.01 | Gyro: X=-0.41, Y=0.25, Z=0.08 | Temp=25.69°C
```

## Code Structure

```
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Main header file
│   │   └── stm32l4xx_hal_conf.h
│   └── Src/
│       ├── main.c              # Main application with MPU6050 driver
│       ├── stm32l4xx_hal_msp.c
│       └── stm32l4xx_it.c
├── Drivers/                    # STM32 HAL drivers
└── README.md
```

## API Functions

### Initialization
```c
void MPU6050_Init(void);
```
Initializes MPU6050 with default settings and verifies connection.

### Data Reading
```c
void MPU6050_Read_All(void);    // Read all sensor data at once
void MPU6050_Read_Accel(void);  // Read only accelerometer
void MPU6050_Read_Gyro(void);   // Read only gyroscope  
void MPU6050_Read_Temp(void);   // Read only temperature
```

### Data Structure
```c
typedef struct {
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    double Ax, Ay, Az;                    // Acceleration in g
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
    double Gx, Gy, Gz;                   // Angular velocity in °/s
    float Temperature;                   // Temperature in °C
} MPU6050_t;
```

## Configuration Options

### Accelerometer Ranges
| FS_SEL | Range | Sensitivity |
|--------|-------|-------------|
| 0      | ±2g   | 16384 LSB/g |
| 1      | ±4g   | 8192 LSB/g  |
| 2      | ±8g   | 4096 LSB/g  |
| 3      | ±16g  | 2048 LSB/g  |

### Gyroscope Ranges  
| FS_SEL | Range    | Sensitivity   |
|--------|----------|---------------|
| 0      | ±250°/s  | 131 LSB/°/s   |
| 1      | ±500°/s  | 65.5 LSB/°/s  |
| 2      | ±1000°/s | 32.8 LSB/°/s  |
| 3      | ±2000°/s | 16.4 LSB/°/s  |

## Troubleshooting

### Common Issues

1. **"MPU6050 Initialization Failed!"**
   - Check wiring connections
   - Verify 3.3V power supply
   - Ensure I2C pullup resistors (usually built-in on modules)

2. **No Serial Output**
   - Check UART2 connections and configuration
   - Verify baud rate (115200)
   - Ensure correct COM port selection

3. **Incorrect Readings**
   - Verify sensor orientation
   - Check for loose connections
   - Consider calibration for precise applications

### Debug Tips
- Use STM32CubeIDE debugger to step through initialization
- Check I2C signals with oscilloscope if available
- Verify WHO_AM_I register returns 0x68 (104 decimal)

## Development Environment

- **IDE:** STM32CubeIDE 1.x
- **Framework:** STM32 HAL Library
- **Target:** STM32L4R9AIIx microcontroller
- **Board:** STM32L4R9AI Discovery Kit

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

- **Your Name** - [Thirumalesh](https://github.com/thirumalesh-developer)

## Acknowledgments

- STMicroelectronics for STM32 HAL library
- InvenSense for MPU6050 documentation
- STM32 community for reference implementations

---

**⭐ If this project helped you, please star the repository!**