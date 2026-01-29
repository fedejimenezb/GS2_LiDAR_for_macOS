# GS2 LiDAR Distribution Package - File Inventory

## Package Structure

```
GS2_LiDAR_Distribution/
├── README.md                           # Main documentation
├── Arduino/
│   └── GS2_LiDAR/
│       ├── GS2_LiDAR.ino              # Main Arduino sketch
│       ├── MyGS2.h                     # Library header
│       ├── MyGS2.cpp                   # Library implementation
│       ├── YDLiDar_defines.h          # Constants and definitions
│       ├── YDLiDar_structs.h          # Data structures
│       └── YDLiDar_enums.h            # Enumerations
├── Visualizer/
│   ├── visualizer.py                   # Real-time visualization script
│   └── requirements.txt                # Python dependencies
└── Documentation/
    ├── SETUP.md                        # Setup instructions
    ├── TROUBLESHOOTING.md              # Troubleshooting guide
    └── CALIBRATION.md                  # Calibration information
```

## File Descriptions

### Root Level
- **README.md** - Overview, quick start, features, and specifications

### Arduino/GS2_LiDAR/
- **GS2_LiDAR.ino** - ESP32-C6 main sketch with handshake and CSV output
- **MyGS2.h** - Custom GS2 library header with class declaration
- **MyGS2.cpp** - Library implementation with angle calculation and calibration
- **YDLiDar_defines.h** - Physical constants (Px, Py, PANGLE) and command bytes
- **YDLiDar_structs.h** - Data structures for scan data
- **YDLiDar_enums.h** - Error codes and status enums

### Visualizer/
- **visualizer.py** - Python script for real-time polar plot visualization
- **requirements.txt** - Python package dependencies (matplotlib, pyserial, numpy)

### Documentation/
- **SETUP.md** - Complete step-by-step setup instructions
- **TROUBLESHOOTING.md** - Common issues and solutions
- **CALIBRATION.md** - Technical details about calibration system

## Version Information
- **Package Version:** 1.0
- **Date Created:** 2026-01-29
- **Compatible Hardware:** ESP32-C6, YDLidar GS2
- **Python Version:** 3.8+
- **Arduino IDE:** 2.x+

## Key Features Implemented
✅ Automatic calibration coefficient retrieval
✅ Fixed handshake (stops scan before parameter request)
✅ atan2() for correct quadrant handling
✅ Real-time CSV data streaming
✅ Polar plot visualization
✅ Comprehensive error handling and debug output

## Distribution Notes
This package is self-contained and includes everything needed to:
1. Upload code to ESP32-C6
2. Connect to YDLidar GS2
3. Visualize LiDAR data in real-time
4. Troubleshoot common issues
5. Understand the calibration system

No additional downloads or external dependencies required (except Python packages via pip).
