// Example: Connected LoRa Device (e.g., Robot Node)
// This code runs on a separate ESP32/ESP8266 board with an SX1278 LoRa module.

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa Module Pin Definitions (adjust for your specific board/wiring)
// These should match the pins on your main ESP32-C3 subsystem IF you were running
// a LoRa-to-LoRa direct communication scenario, which is usually not the case.
// For two distinct LoRa nodes, their pins are independent.
#define LORA_SS    5    // LoRa NSS pin
#define LORA_RST   6    // LoRa RESET pin
#define LORA_DIO0  7    // LoRa DIO0 pin (for interrupt callback)
#define LORA_FREQ  433E6 // LoRa frequency, MUST MATCH the main ESP32-C3

// Buffer for received data
char receivedLoRaData[256];

void onLoRaReceive(int packetSize) {
  if (packetSize == 0) return; // No packet received

  int bytesRead = 0;
  // Read packet
  for (int i = 0; i < packetSize; i++) {
    if (bytesRead < sizeof(receivedLoRaData) - 1) { // -1 for null terminator
      receivedLoRaData[bytesRead++] = (char)LoRa.read();
    } else {
      LoRa.read(); // Read and discard if buffer is full
    }
  }
  receivedLoRaData[bytesRead] = '\0'; // Null-terminate the string

  Serial.print("LoRa Robot: Received packet '");
  Serial.print(receivedLoRaData);
  Serial.print("' with RSSI ");
  Serial.print(LoRa.packetRssi());
  Serial.print(", SNR ");
  Serial.print(LoRa.packetSnr());
  Serial.println();

  // --- Send data back to the Controller ESP32-C3 Subsystem ---
  String response = "ACK from Robot! (Rx '" + String(receivedLoRaData) + "')";
  Serial.print("LoRa Robot: Sending response: ");
  Serial.println(response);

  LoRa.beginPacket();
  LoRa.print(response); // Send the string
  LoRa.endPacket(true); // true for async sending, non-blocking
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Robot Node Starting...");

  // Initialize LoRa
  // Make sure SPI pins are correct for your ESP32/ESP8266 board
  SPI.begin(LORA_SS, -1, -1, -1); // For ESP32/ESP8266, check SPI default pins, e.g., ESP32: HSPI_SCK(14), HSPI_MISO(12), HSPI_MOSI(13)
                                  // NSS is passed to LoRa.setPins()
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa Robot: Starting LoRa failed!");
    while (true);
  }

  // Set callback for packet reception
  LoRa.onReceive(onLoRaReceive);
  LoRa.receive(); // Put the radio into receive mode

  Serial.println("LoRa Robot Node ready to receive.");
}

void loop() {
  // LoRa reception is interrupt-driven, so loop can be used for other tasks.
  // We explicitly call LoRa.receive() after sending to put it back in RX mode.
}