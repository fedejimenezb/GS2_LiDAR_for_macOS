# GS2 LiDAR Visualizer - Distribution Package

## Overview
This package contains everything needed to run the YDLidar GS2 LiDAR sensor with an ESP32-C6 and visualize the data in real-time.

## Contents

### Arduino/GS2_LiDAR/
- `GS2_LiDAR.ino` - Main Arduino sketch for ESP32-C6
- `MyGS2.h` - Custom GS2 LiDAR library header
- `MyGS2.cpp` - Custom GS2 LiDAR library implementation
- Supporting header files (defines, structs, enums)

### Visualizer/
- `visualizer.py` - Real-time polar plot visualization script
- `requirements.txt` - Python dependencies

### Documentation/
- `SETUP.md` - Complete setup instructions
- `TROUBLESHOOTING.md` - Common issues and solutions
- `CALIBRATION.md` - Information about LiDAR calibration

## Quick Start

### Hardware Setup
1. **ESP32-C6 Connections:**
   - LiDAR RX → ESP32 GPIO 5 (TX)
   - LiDAR TX → ESP32 GPIO 4 (RX)
   - LiDAR 5V → ESP32 5V
   - LiDAR GND → ESP32 GND

2. **Upload Arduino Code:**
   - Open `Arduino/GS2_LiDAR/GS2_LiDAR.ino` in Arduino IDE
   - Select Board: "ESP32C6 Dev Module"
   - Select Port: Your ESP32-C6 port
   - Upload

### Software Setup
1. **Install Python Dependencies:**
   ```bash
   cd Visualizer
   pip install -r requirements.txt
   ```

2. **Run Visualizer:**
   ```bash
   python visualizer.py
   ```
   - Select your ESP32 serial port when prompted
   - The visualization window will open automatically

## Features

### Automatic Calibration
- The ESP32 automatically retrieves calibration coefficients from the LiDAR on startup
- No manual configuration needed
- Coefficients are device-specific for optimal accuracy

### Real-time Visualization
- 90-degree field of view polar plot
- Distance range: 0-100cm
- Updates in real-time as the LiDAR scans

### Debug Output
- Handshake status messages
- Calibration coefficient display
- Connection diagnostics

## Technical Details

### LiDAR Specifications
- **Model:** YDLidar GS2
- **FOV:** 90 degrees
- **Range:** 12cm - 8m
- **Scan Rate:** ~4500 points/sec
- **Baud Rate:** 921600

### ESP32-C6 Configuration
- **Serial Monitor:** 921600 baud
- **LiDAR Serial:** Serial1 (GPIO 4 RX, GPIO 5 TX)
- **Output Format:** CSV (Angle,Distance)

### Angle Calculation
The library uses a sophisticated dual-sensor coordinate transformation:
1. Retrieves device-specific K, B, and bias coefficients
2. Applies atan or linear model based on B coefficient value
3. Performs rotation matrix transformations
4. Compensates for physical sensor offset (Px, Py)
5. Uses atan2() for proper quadrant handling

## Troubleshooting

### "Handshake FAILED"
- Check wiring connections
- Verify LiDAR is powered (5V)
- Ensure correct GPIO pins (4=RX, 5=TX)

### No Visualization
- Close Arduino Serial Monitor before running visualizer
- Check that ESP32 is outputting CSV data
- Verify correct serial port selected

### Double Lines / Gaps
- This indicates calibration coefficient issues
- Check Serial Monitor for "[COEFFICIENTS]" output
- Verify handshake succeeded

## Version History
- **v1.0** - Initial release with automatic calibration and atan2 bug fix

## License
This code is provided as-is for educational and personal use.

## Support
For issues or questions, refer to the Documentation folder.
