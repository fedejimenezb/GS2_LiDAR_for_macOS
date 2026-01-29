import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time
import argparse
import sys

# --- Data Store ---
max_points = 1000  # Decrease if laggy
angles = deque(maxlen=max_points)
distances = deque(maxlen=max_points)

def select_serial_port():
    """Lists available serial ports and asks user to select one."""
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("\n❌ No serial ports found! Check your connection.")
        return None
    
    print("\n🔎 Available Serial Ports:")
    print("-" * 40)
    for i, p in enumerate(ports):
        print(f" [{i}] {p.device}  ({p.description})")
    print("-" * 40)
        
    while True:
        selection = input("\n👉 Select port index (or 'q' to quit): ").strip()
        if selection.lower() == 'q':
            return None
        
        try:
            index = int(selection)
            if 0 <= index < len(ports):
                return ports[index].device
            print("⚠️ Invalid index. Please try again.")
        except ValueError:
            print("⚠️ Invalid input. Enter a number.")

def read_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"\n✅ Connected to {port} at {baud} baud.")
        print("Listening for LiDAR data... (Ctrl+C to stop)")
    except Exception as e:
        print(f"\n❌ Error opening serial port: {e}")
        return

    while True:
        try:
            # Read line, decode, and strip whitespace
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue
            
            # Expected format: "Angle,Distance" (e.g., "120.50,1500")
            parts = line.split(',')
            if len(parts) == 2:
                try:
                    ang = float(parts[0])
                    dist = float(parts[1])
                    if dist > 0:
                        angles.append(np.radians(ang))
                        distances.append(dist)
                except ValueError:
                    pass
        except Exception as e:
            print(f"Serial read error: {e}")
            break

def update_plot(frame, ax):
    ax.clear()
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1) # Clockwise
    ax.set_rmax(2000) # Max range 2000mm, adjust as needed
    ax.grid(True, alpha=0.3)
    ax.set_title("GS2 LiDAR Real-time Scan", color='white')
    
    # Styling
    ax.set_facecolor('#1e1e1e')
    
    if angles:
        ax.scatter(list(angles), list(distances), s=5, c='#00ff00', alpha=0.8)
    
    return ax,

def main():
    parser = argparse.ArgumentParser(description="GS2 LiDAR Visualizer")
    parser.add_argument("port", nargs='?', help="Serial port of ESP32 (optional)")
    args = parser.parse_args()

    port = args.port
    
    # If no port provided, ask interactively
    if not port:
        port = select_serial_port()
        if not port:
            print("Exiting.")
            sys.exit(0)

    # Start Serial Thread
    t = threading.Thread(target=read_serial, args=(port, 921600))
    t.daemon = True
    t.start()

    # Setup Plot
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(8, 8))
    fig.canvas.manager.set_window_title('GS2 LiDAR Visualizer')
    ax = fig.add_subplot(111, projection='polar')
    
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update_plot, fargs=(ax,), interval=30) 
    
    try:
        plt.show()
    except KeyboardInterrupt:
        sys.exit(0)

if __name__ == "__main__":
    main()
