#include "MyGS2.h"

// --- ESP32-C6 Pin Configuration ---
#define LIDAR_RX_PIN 4
#define LIDAR_TX_PIN 5

// Initialize the GS2 LiDAR
// Use Serial1 for the ESP32-C6 connection to the LiDAR
// RX Pin: 4, TX Pin: 5
MyGS2 lidar(&Serial1, GS_LIDAR_BAUDRATE_921600, SERIAL_8N1, 4, 5);

void setup() {
  Serial.begin(921600);
  delay(3000);
  Serial.println("\n--- ESP32-C6 GS2 LiDAR Deep Debug ---");

  Serial1.begin(921600, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Try handshake once
  if (lidar.initialize() == GS_OK) {
    Serial.println("Handshake: SUCCESS");
    lidar.startScanning();
  } else {
    lidar.stopScanning();
    Serial.println("Handshake: FAILED. Forcing Scan...");
    lidar.startScanning();
  }
}

void loop() {
  // 1. Try to read via library
  iter_Scan scan = lidar.iter_scans();

  bool validPoints = false;
  for (int i = 0; i < 160; i++) {
    if (scan.valid[i]) {
      // Print CSV
      Serial.printf("%.2f,%.2f\n", scan.angle[i], (float)scan.distance[i]);
      validPoints = true;
    }
  }

  // 2. If NO valid points, peeking into the buffer directly!
  // This helps us see if the library is consuming data but returning bad
  // values, or if the data itself is just zeros.
  if (!validPoints) {
    if (Serial1.available()) {
      // Peek at the next few bytes
      // Note: we can't read() too much or we break the library's parser.
      // But if the library returned no valid points, maybe it's stuck.

      // Let's just print that we have data available
      // Serial.printf("Bytes Avail: %d\n", Serial1.available());
    }
  }
}