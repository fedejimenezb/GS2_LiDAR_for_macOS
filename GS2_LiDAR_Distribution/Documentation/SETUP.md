# Setup Instructions

## Prerequisites

### Hardware
- ESP32-C6 Development Board
- YDLidar GS2 LiDAR Sensor
- USB Cable (for ESP32)
- Jumper wires (4x)

### Software
- Arduino IDE 2.x or later
- Python 3.8 or later
- pip (Python package manager)

## Step-by-Step Setup

### 1. Install Arduino IDE
1. Download from https://www.arduino.cc/en/software
2. Install ESP32 board support:
   - Open Arduino IDE
   - Go to File → Preferences
   - Add to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to Tools → Board → Boards Manager
   - Search for "esp32"
   - Install "esp32" by Espressif Systems

### 2. Hardware Connections

**IMPORTANT:** Double-check these connections before powering on!

| LiDAR Pin | ESP32-C6 Pin | Notes |
|-----------|--------------|-------|
| 5V | 5V | Power supply |
| GND | GND | Ground |
| TX | GPIO 4 | LiDAR transmit → ESP32 receive |
| RX | GPIO 5 | LiDAR receive → ESP32 transmit |

**Note:** The LiDAR's TX goes to ESP32's RX (GPIO 4), and LiDAR's RX goes to ESP32's TX (GPIO 5).

### 3. Upload Arduino Code

1. Open `Arduino/GS2_LiDAR/GS2_LiDAR.ino`
2. Select your board:
   - Tools → Board → esp32 → ESP32C6 Dev Module
3. Select your port:
   - Tools → Port → (your ESP32 port)
4. Click Upload (→ button)
5. Wait for "Done uploading" message

### 4. Verify Arduino Upload

1. Open Serial Monitor (Tools → Serial Monitor)
2. Set baud rate to **921600**
3. Press ESP32 reset button
4. You should see:
   ```
   --- ESP32-C6 GS2 LiDAR Deep Debug ---
   [INIT] Stopping any existing scan...
   [INIT] Clearing buffer...
   [INIT] Pinging LiDAR...
   [INIT] Found 1 LiDAR(s)
   [INIT] Requesting parameters...
   [HANDSHAKE] Available bytes: 18
   [HANDSHAKE] Raw response:
   ...
   [INIT] SUCCESS - Got calibration coefficients!
   Handshake: SUCCESS
   ```
5. Then you should see CSV data:
   ```
   45.23,523.00
   44.67,521.50
   ...
   ```

### 5. Install Python Dependencies

Open a terminal/command prompt:

**macOS/Linux:**
```bash
cd /path/to/GS2_LiDAR_Distribution/Visualizer
pip3 install -r requirements.txt
```

**Windows:**
```cmd
cd C:\path\to\GS2_LiDAR_Distribution\Visualizer
pip install -r requirements.txt
```

### 6. Run Visualizer

**IMPORTANT:** Close the Arduino Serial Monitor first!

**macOS/Linux:**
```bash
python3 visualizer.py
```

**Windows:**
```cmd
python visualizer.py
```

1. The script will list available serial ports
2. Enter the number for your ESP32 port
3. A visualization window will open showing real-time LiDAR data

## Expected Results

### Successful Handshake
- Serial Monitor shows "[INIT] SUCCESS - Got calibration coefficients!"
- Coefficient values are displayed (K0, B0, K1, B1, bias)
- CSV data streams continuously

### Successful Visualization
- Polar plot window opens
- Green dots appear showing detected objects
- Points update in real-time
- Pointing at a flat surface shows a continuous arc

## Next Steps

- See `TROUBLESHOOTING.md` if you encounter issues
- See `CALIBRATION.md` to understand the calibration process
- Experiment with different objects and distances!
