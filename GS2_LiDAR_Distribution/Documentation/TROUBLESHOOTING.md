# Troubleshooting Guide

## Handshake Issues

### Problem: "[HANDSHAKE] TIMEOUT - No response from LiDAR"

**Possible Causes:**
1. LiDAR not powered
2. Wrong GPIO pins
3. Crossed TX/RX wires
4. Faulty connections

**Solutions:**
1. **Check Power:**
   - Verify 5V connection
   - Check GND connection
   - LiDAR should have a small LED indicator (if equipped)

2. **Verify Wiring:**
   - LiDAR TX → ESP32 GPIO 4 (RX)
   - LiDAR RX → ESP32 GPIO 5 (TX)
   - Use a multimeter to check continuity

3. **Test Serial Communication:**
   - Add this to `setup()` to test:
     ```cpp
     Serial1.println("TEST");
     ```
   - If LiDAR responds, wiring is correct

### Problem: "Handshake: FAILED Code 0"

**Cause:** LiDAR was already scanning when handshake attempted

**Solution:**
- This is now fixed in the current code
- The code stops any existing scan before handshake
- If still failing, try power cycling the LiDAR

### Problem: Coefficients show as 0.0000

**Cause:** Failed to parse parameter response

**Solutions:**
1. Check Serial Monitor for "[HANDSHAKE] Raw response:"
2. Verify bytes 8-16 are not all zeros
3. If all zeros, LiDAR may need firmware update

## Visualization Issues

### Problem: "No module named 'matplotlib'"

**Solution:**
```bash
pip install matplotlib pyserial numpy
```

Or use the requirements file:
```bash
pip install -r requirements.txt
```

### Problem: "Port already in use" or "Permission denied"

**Cause:** Arduino Serial Monitor is still open

**Solution:**
1. Close Arduino IDE Serial Monitor
2. On Linux/Mac, you may need sudo:
   ```bash
   sudo python3 visualizer.py
   ```
3. Or add user to dialout group (Linux):
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   (logout and login required)

### Problem: Visualization window opens but no data appears

**Possible Causes:**
1. ESP32 not sending CSV data
2. Wrong serial port selected
3. Baud rate mismatch

**Solutions:**
1. **Verify ESP32 Output:**
   - Open Arduino Serial Monitor (921600 baud)
   - You should see lines like: `45.23,523.00`
   - If not, re-upload Arduino code

2. **Check visualizer.py Port:**
   - The script lists available ports
   - Make sure you selected the correct one
   - On Mac: usually `/dev/cu.usbserial-*`
   - On Linux: usually `/dev/ttyUSB*` or `/dev/ttyACM*`
   - On Windows: usually `COM*`

3. **Verify Baud Rate:**
   - visualizer.py uses 921600 (hardcoded)
   - ESP32 Serial.begin() must match

### Problem: Points appear but in wrong locations

**Cause:** Calibration coefficients not retrieved

**Solution:**
1. Check Serial Monitor for:
   ```
   [INIT] SUCCESS - Got calibration coefficients!
   ```
2. If handshake failed, coefficients will use fallback values
3. Fix handshake issue first (see above)

## Data Quality Issues

### Problem: Double lines when pointing at flat surface

**Cause:** This was a bug in earlier versions (atan vs atan2)

**Solution:**
- Make sure you're using the latest code
- The current version uses `atan2()` which fixes this
- If still seeing double lines, check that B coefficients are < 1

### Problem: Gaps in the scan

**Possible Causes:**
1. Physical obstruction
2. Surface too reflective/absorptive
3. Out of range (too close or too far)

**Solutions:**
1. **Check Range:**
   - GS2 minimum range: ~12cm
   - GS2 maximum range: ~8m
   - Objects outside this range won't be detected

2. **Surface Properties:**
   - Very dark surfaces absorb IR light
   - Very shiny surfaces reflect away from sensor
   - Try a matte, light-colored surface for testing

3. **Physical Setup:**
   - Ensure LiDAR is stable and not vibrating
   - Check that nothing is blocking the sensor window

### Problem: Noisy/jittery data

**Possible Causes:**
1. Electrical interference
2. Poor power supply
3. Reflective surfaces

**Solutions:**
1. **Power Quality:**
   - Use a good quality USB cable
   - Try a different USB port
   - Add a capacitor (100µF) across 5V and GND

2. **Reduce Interference:**
   - Keep LiDAR away from motors/servos
   - Use shielded cables if possible

## Compilation Errors

### Problem: "MyGS2.h: No such file or directory"

**Cause:** Files not in same folder as .ino

**Solution:**
- All files must be in `GS2_LiDAR/` folder:
  - GS2_LiDAR.ino
  - MyGS2.h
  - MyGS2.cpp
  - YDLiDar_defines.h
  - YDLiDar_structs.h
  - YDLiDar_enums.h

### Problem: "ESP32C6 Dev Module not found"

**Cause:** ESP32 board support not installed

**Solution:**
1. Go to Tools → Board → Boards Manager
2. Search "esp32"
3. Install "esp32" by Espressif Systems
4. Restart Arduino IDE

## Performance Issues

### Problem: Slow update rate in visualization

**Possible Causes:**
1. Computer too slow for matplotlib
2. Too many points being plotted

**Solutions:**
1. Close other applications
2. Reduce matplotlib figure size
3. Use a faster computer

### Problem: ESP32 resets randomly

**Possible Causes:**
1. Power supply insufficient
2. Brownout detection triggered

**Solutions:**
1. Use powered USB hub
2. Add bulk capacitor to 5V rail
3. Check USB cable quality

## Still Having Issues?

If none of these solutions work:

1. **Capture Debug Output:**
   - Copy all Serial Monitor output
   - Note exact error messages

2. **Check Hardware:**
   - Test LiDAR with official YDLidar software
   - Test ESP32 with simple blink sketch

3. **Verify Software Versions:**
   - Arduino IDE version
   - ESP32 board package version
   - Python version
   - Library versions

4. **Try Minimal Test:**
   - Upload a simple Serial1 echo sketch
   - Verify basic UART communication works
