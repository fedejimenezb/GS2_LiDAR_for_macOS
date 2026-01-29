# LiDAR Calibration Information

## Overview

The YDLidar GS2 uses a dual-sensor architecture that requires precise calibration coefficients for accurate angle calculation. These coefficients are unique to each LiDAR unit and stored in the device's firmware.

## Calibration Coefficients

### K Coefficients (K0, K1)
- **Purpose:** Degrees per pixel scaling factor
- **Typical Range:** 0.01 - 0.60
- **Your LiDAR:** K0 = K1 ≈ 0.015

The K coefficient determines how much the angle changes per pixel in the sensor array.

### B Coefficients (B0, B1)
- **Purpose:** Angle offset for each sensor
- **Typical Range:** 0.0 - 2.0
- **Your LiDAR:** B0 ≈ 0.607, B1 ≈ 0.608

The B coefficient shifts the starting angle for each sensor half. When B < 1, the atan (arctangent) formula is used; when B > 1, a linear formula is used.

### Bias Coefficient
- **Purpose:** Global angle offset
- **Typical Range:** -5.0 to +5.0
- **Your LiDAR:** bias ≈ 1.7

The bias shifts the entire field of view to align with the physical mounting.

## How Calibration Works

### 1. Automatic Retrieval
When the ESP32 starts up, it:
1. Stops any existing scan
2. Sends a `GET_PARAMETERS` command (0x61)
3. Receives a response with calibration data
4. Parses the coefficients from the response
5. Stores them in memory for use during scanning

### 2. Coefficient Format
The LiDAR sends coefficients as 16-bit integers that must be scaled:
- K and B values: `raw_value / 10000.0`
- Bias value: `raw_value / 10.0`

Example from your LiDAR:
```
Raw K0: 0x0096 (150) → 150 / 10000 = 0.0150
Raw B0: 0x17B8 (6072) → 6072 / 10000 = 0.6072
Raw bias: 0x11 (17) → 17 / 10 = 1.7
```

### 3. Angle Calculation Algorithm

For each pixel `n` (0-159):

**Left Sensor (n < 80):**
```
pixelU = 80 - n
if (B0 > 1):
    tempTheta = K0 × pixelU - B0
else:
    tempTheta = atan(K0 × pixelU - B0) × 180/π

tempDist = (raw_distance - Px) / cos((PANGLE + bias - tempTheta) × π/180)
```

**Right Sensor (n ≥ 80):**
```
pixelU = 160 - n
if (B1 > 1):
    tempTheta = K1 × pixelU - B1
else:
    tempTheta = atan(K1 × pixelU - B1) × 180/π

tempDist = (raw_distance - Px) / cos((PANGLE + bias + tempTheta) × π/180)
```

Then both sensors apply coordinate transformation:
```
tempX = cos(±(PANGLE + bias)) × tempDist × cos(tempTheta) + ...
tempY = -sin(±(PANGLE + bias)) × tempDist × cos(tempTheta) + ...
tempX += Px
tempY ± Py

final_distance = sqrt(tempX² + tempY²)
final_angle = atan2(tempY, tempX) × 180/π
```

## Physical Constants

These are hardcoded in the library and should match the GS2 specifications:

- **ANGLE_PX:** 1.22 mm (X offset between sensors)
- **ANGLE_PY:** 5.315 mm (Y offset between sensors)
- **ANGLE_PANGLE:** 22.5° (mounting angle of each sensor)

## Why Calibration Matters

### Without Proper Calibration:
- ❌ Double lines when scanning flat surfaces
- ❌ Gaps in the field of view
- ❌ Incorrect distance measurements
- ❌ Angular distortion

### With Proper Calibration:
- ✅ Smooth, continuous scans
- ✅ Accurate angles across full 90° FOV
- ✅ Correct distance measurements
- ✅ Seamless blending of dual sensors

## Fallback Values

If the handshake fails, the code uses fallback values based on your specific LiDAR:

```cpp
K0 = 0.0150
B0 = 0.6072
K1 = 0.0150
B1 = 0.6080
bias = 1.7
```

**Note:** These are specific to your LiDAR unit. Other GS2 units will have different values!

## Verifying Calibration

To check if calibration was successful:

1. Open Serial Monitor (921600 baud)
2. Look for:
   ```
   [INIT] SUCCESS - Got calibration coefficients!
   [COEFFICIENTS] LiDAR 1: K0=0.0150 B0=0.6072 K1=0.0150 B1=0.6080 bias=1.70
   ```
3. If you see "Handshake: FAILED", calibration did not work

## Manual Calibration (Advanced)

If automatic calibration fails repeatedly, you can hardcode the coefficients:

1. Note the fallback values from a successful handshake
2. Edit `MyGS2.cpp`, function `getMeasurements()`
3. Replace the fallback values with your LiDAR's values
4. This bypasses the handshake but uses correct coefficients

**Warning:** Only do this if you've successfully retrieved coefficients at least once and recorded them!

## Understanding the Math

The dual-sensor design means:
- Left sensor (pixels 0-79) scans one half of the FOV
- Right sensor (pixels 80-159) scans the other half
- Each sensor has its own optical axis at ±22.5° from center
- Physical separation (Py = 5.315mm) creates a parallax effect
- Rotation matrices transform from sensor-local to global coordinates
- The atan2() function ensures correct angle quadrants

This complex transformation is why device-specific calibration is essential!
