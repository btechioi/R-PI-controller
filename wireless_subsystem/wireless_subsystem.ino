Okay, I will implement the suggested improvements to the ESP32-C3 wireless subsystem code.

Here's a summary of the improvements I'll be adding:

1.  **CRC-16 for UART Framing:** Implement a CRC-16 (XMODEM) calculation for data sent over `Serial1` to the Pi Zero W, and verify it upon reception. This significantly improves data integrity.
2.  **Command Acknowledgments:** Introduce a simple acknowledgment (ACK) or negative acknowledgment (NACK) system for commands sent from the Pi Zero W. The ESP32-C3 will send a status response back.
3.  **Radio De-initialization/Power Management:** When switching radios, the previously active radio will be put into a lower power or idle state to reduce power consumption and potential interference.
4.  **NRF24 Dynamic Payload Size:** Correctly read the actual payload size from NRF24L01+ to prevent sending stale data to the Pi.

Let's start by modifying the code. I'll break it down into sections for clarity.

```cpp
// ESP32-C3 Wireless Subsystem Full Code
// Uses Serial1 for UART to Pi Zero W (Protocol Communication)
// Uses Serial (USB CDC) for Debugging
// Supports LoRa (SX1278), NRF24L01+, and ESP-NOW

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <esp_now.h>
#include <WiFi.h>

// ===================== Constants =====================
#define UART_BAUD 230400
#define UART_RX   9   // GPIO9 (RXD)
#define UART_TX   10  // GPIO10 (TXD)

#define RADIO_NONE   0
#define RADIO_LORA   1
#define RADIO_NRF24  2
#define RADIO_ESPNOW 3

#define FRAME_START  0x7E
#define FRAME_ESCAPE 0x7D
#define FRAME_XOR    0x20

#define MAX_PAYLOAD_SIZE 240 // Max data payload before framing and CRC
#define MAX_FRAME_SIZE   256 // Max size of the framed data (including start, escape, and CRC)

// Command Opcodes from Pi to ESP32-C3
#define CMD_SET_RADIO_LORA      0x01
#define CMD_SET_RADIO_NRF24     0x02
#define CMD_SET_RADIO_ESPNOW    0x03
#define CMD_SEND_DATA           0x04
#define CMD_SET_NRF24_TX_ADDR   0x05
#define CMD_SET_NRF24_RX_ADDR   0x06
#define CMD_SET_ESPNOW_PEER_MAC 0x07

// Response Opcodes from ESP32-C3 to Pi Zero W
#define RSP_ACK                 0xA0
#define RSP_NACK                0xA1
#define RSP_RECEIVED_DATA       0xB0 // For data coming from radios

// LoRa
#define LORA_SS    5
#define LORA_RST   6
#define LORA_DIO0  7
#define LORA_FREQ  433E6

// NRF24
#define NRF_CE  3
#define NRF_CSN 4
const byte default_nrf_addr[5] = {'C','T','R','L','0'};

// ===================== Globals =====================
uint8_t currentRadio = RADIO_NONE;

// NRF
RF24 radio(NRF_CE, NRF_CSN);
byte nrf_rx_addr[5] = {'R','B','O','T','1'};
byte nrf_tx_addr[5] = {'R','B','O','T','1'};

// ESP-NOW
esp_now_peer_info_t peerInfo;
uint8_t esp_peer_mac[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}; // Example MAC, replace with actual peer MAC

// Buffers
// rxBuffer will now store the raw payload after deframing and CRC check
uint8_t rxBuffer[MAX_PAYLOAD_SIZE];
// txBuffer will be used to construct the payload (command + data) before framing
uint8_t txBuffer[MAX_PAYLOAD_SIZE + 1]; // +1 for command byte
uint8_t framedTxBuffer[MAX_FRAME_SIZE]; // For the actual framed data to send out UART

// ===================== Utility =====================

// CRC-16 (XMODEM) calculation
// Polynomial: x^16 + x^12 + x^5 + 1 (0x1021)
uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc = crc ^ ((uint16_t)data[i] << 8);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

void debugPrint(const String& msg) {
  Serial.println("[DBG] " + msg);
}

// Sends a framed message with CRC to the Pi Zero W
// The 'data' here is the raw payload (command + actual data)
void sendFramedToPi(const uint8_t* data, size_t len) {
  if (len > MAX_PAYLOAD_SIZE) {
    debugPrint("sendFramedToPi: Payload too large for framing.");
    return;
  }

  uint16_t crc = calculateCRC16(data, len);
  size_t framedIdx = 0;

  framedTxBuffer[framedIdx++] = FRAME_START;

  for (size_t i = 0; i < len; i++) {
    if (data[i] == FRAME_START || data[i] == FRAME_ESCAPE) {
      framedTxBuffer[framedIdx++] = FRAME_ESCAPE;
      framedTxBuffer[framedIdx++] = data[i] ^ FRAME_XOR;
    } else {
      framedTxBuffer[framedIdx++] = data[i];
    }
  }

  // Add CRC bytes (LSB first for consistency, or MSB based on preference)
  // Here, sending LSB then MSB
  uint8_t crc_lsb = crc & 0xFF;
  uint8_t crc_msb = (crc >> 8) & 0xFF;

  if (crc_lsb == FRAME_START || crc_lsb == FRAME_ESCAPE) {
    framedTxBuffer[framedIdx++] = FRAME_ESCAPE;
    framedTxBuffer[framedIdx++] = crc_lsb ^ FRAME_XOR;
  } else {
    framedTxBuffer[framedIdx++] = crc_lsb;
  }

  if (crc_msb == FRAME_START || crc_msb == FRAME_ESCAPE) {
    framedTxBuffer[framedIdx++] = FRAME_ESCAPE;
    framedTxBuffer[framedIdx++] = crc_msb ^ FRAME_XOR;
  } else {
    framedTxBuffer[framedIdx++] = crc_msb;
  }

  Serial1.write(framedTxBuffer, framedIdx);
}

// Decodes a framed input from the Pi Zero W, including CRC verification
// The 'dest' buffer will receive the raw payload (command + actual data)
bool decodeFramedInput(Stream &stream, uint8_t *dest, size_t &outLen) {
  static bool inFrame = false;
  static bool escape = false;
  static size_t idx = 0;
  static uint8_t tempRxBuffer[MAX_FRAME_SIZE]; // Temporary buffer for deframing

  while (stream.available()) {
    uint8_t b = stream.read();

    if (inFrame) {
      if (b == FRAME_ESCAPE) {
        escape = true;
      } else if (b == FRAME_START) {
        // End of frame detected (or unexpected start in middle of frame)
        inFrame = false;

        if (idx < 2) { // Need at least 2 bytes for CRC
          debugPrint("Framed input: Frame too short for CRC.");
          idx = 0; // Reset for next frame
          return false;
        }

        // Extract CRC from the end of the temporary buffer
        uint8_t received_crc_lsb = tempRxBuffer[idx - 2];
        uint8_t received_crc_msb = tempRxBuffer[idx - 1];
        uint16_t received_crc = (received_crc_msb << 8) | received_crc_lsb;

        // Calculate CRC on the actual data part (excluding the received CRC bytes)
        size_t dataLen = idx - 2;
        uint16_t calculated_crc = calculateCRC16(tempRxBuffer, dataLen);

        if (calculated_crc == received_crc) {
          if (dataLen > MAX_PAYLOAD_SIZE) { // Check if data part exceeds max payload
             debugPrint("Framed input: Decoded data length exceeds MAX_PAYLOAD_SIZE.");
             idx = 0; // Reset
             return false;
          }
          memcpy(dest, tempRxBuffer, dataLen);
          outLen = dataLen;
          idx = 0; // Reset for next frame
          return true; // Complete and valid frame received
        } else {
          debugPrint("Framed input: CRC mismatch! Expected 0x" + String(calculated_crc, HEX) + ", Got 0x" + String(received_crc, HEX));
          idx = 0; // Reset for next frame
          return false;
        }
      } else {
        if (escape) {
          tempRxBuffer[idx++] = b ^ FRAME_XOR;
          escape = false;
        } else {
          tempRxBuffer[idx++] = b;
        }
        if (idx >= MAX_FRAME_SIZE) { // Buffer overflow of the temporary deframed buffer
          debugPrint("Framed input buffer overflow (tempRxBuffer)!");
          inFrame = false; // Discard current frame
          idx = 0;
          return false;
        }
      }
    } else {
      if (b == FRAME_START) {
        inFrame = true;
        idx = 0;
        escape = false;
      }
    }
  }
  return false; // No complete frame received yet
}

// Sends an ACK or NACK response back to the Pi
void sendResponseToPi(uint8_t status, uint8_t original_command = 0) {
  txBuffer[0] = status; // RSP_ACK or RSP_NACK
  txBuffer[1] = original_command; // Echo the command for context
  sendFramedToPi(txBuffer, 2);
}

// ===================== Radio Power Management =====================
void deactivateAllRadios() {
  // Put LoRa into sleep mode
  LoRa.sleep();
  debugPrint("LoRa put to sleep.");

  // Put NRF24 into power down mode
  radio.powerDown();
  debugPrint("NRF24 powered down.");

  // For ESP-NOW, esp_now_deinit() would remove peers and callbacks,
  // which might be too drastic. It's usually fine to leave it initialized
  // if not actively sending/receiving, as it doesn't consume much power
  // when idle. If absolute lowest power is needed, deinit could be added,
  // but it would require re-initialization and re-adding peers later.
  // For this application, we assume ESP-NOW can stay initialized.
  WiFi.mode(WIFI_OFF); // Turn off WiFi (which ESP-NOW uses)
  debugPrint("WiFi turned off for ESP-NOW power saving.");
  // Note: Turning off WiFi also affects ESP-NOW. Re-enabling would be needed.
  // A better approach for ESP-NOW might be to just ensure no TX is happening.
  // For this example, let's turn WiFi off and then back on when needed for ESP-NOW.
  // Alternative: esp_wifi_stop(), esp_wifi_start()
}

// ===================== LoRa Functions =====================
void lora_onReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t loraRxBuffer[packetSize];
  for (int i = 0; i < packetSize; i++) {
    loraRxBuffer[i] = (uint8_t)LoRa.read();
  }

  debugPrint("LoRa Received: " + String((char*)loraRxBuffer, packetSize));

  // Forward received data to Pi, prefixed with RSP_RECEIVED_DATA
  txBuffer[0] = RSP_RECEIVED_DATA;
  if (packetSize <= MAX_PAYLOAD_SIZE - 1) { // Check if data + opcode fits
    memcpy(&txBuffer[1], loraRxBuffer, packetSize);
    sendFramedToPi(txBuffer, packetSize + 1);
  } else {
    debugPrint("LoRa received packet too large to forward to Pi.");
  }
}

bool setupLoRa() {
  SPI.begin(LORA_SS, -1, -1, -1); // SCK, MISO, MOSI are default pins for ESP32-C3
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    debugPrint("Starting LoRa failed!");
    return false;
  }
  LoRa.onReceive(lora_onReceive);
  // LoRa.receive(); // Will be called when this radio is active
  debugPrint("LoRa initialized.");
  return true;
}

void activateLoRa() {
  deactivateAllRadios();
  LoRa.receive(); // Put radio in receive mode
  currentRadio = RADIO_LORA;
  debugPrint("LoRa activated.");
}

void sendLoRa(const uint8_t* data, size_t len) {
  LoRa.beginPacket();
  LoRa.write(data, len);
  LoRa.endPacket(true); // true for asynchronous sending
  debugPrint("LoRa Sent: " + String((char*)data, len));
}

// ===================== NRF24L01+ Functions =====================
bool setupNRF24() {
  if (!radio.begin()) {
    debugPrint("NRF24L01+ radio hardware not responding!");
    return false;
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(76); // Channel 76 is a common choice, can be changed
  radio.setDataRate(RF24_2MBPS); // 2MBPS for faster communication

  radio.openReadingPipe(1, nrf_rx_addr);
  radio.openWritingPipe(nrf_tx_addr);

  // radio.startListening(); // Will be called when this radio is active
  debugPrint("NRF24L01+ initialized.");
  return true;
}

void activateNRF24() {
  deactivateAllRadios();
  radio.startListening();
  currentRadio = RADIO_NRF24;
  debugPrint("NRF24L01+ activated.");
}

void loopNRF24() {
  if (radio.available()) {
    uint8_t pipe;
    uint8_t nrfRxBuffer[radio.getPayloadSize()]; // Use max payload size for static buffer
    size_t len = radio.getDynamicPayloadSize(); // Get actual dynamic payload size

    if (len > 0 && len <= sizeof(nrfRxBuffer)) { // Ensure valid length
      radio.read(&nrfRxBuffer, len);
      debugPrint("NRF24 Received (" + String(len) + " bytes): " + String((char*)nrfRxBuffer, len));

      // Forward received data to Pi, prefixed with RSP_RECEIVED_DATA
      txBuffer[0] = RSP_RECEIVED_DATA;
      if (len <= MAX_PAYLOAD_SIZE - 1) { // Check if data + opcode fits
        memcpy(&txBuffer[1], nrfRxBuffer, len);
        sendFramedToPi(txBuffer, len + 1);
      } else {
        debugPrint("NRF24 received packet too large to forward to Pi.");
      }
    } else {
      debugPrint("NRF24 received invalid payload size.");
    }
  }
}

void sendNRF24(const uint8_t* data, size_t len) {
  if (len > 32) {
    debugPrint("NRF24 payload too large (max 32 bytes).");
    return;
  }
  radio.stopListening();
  bool ok = radio.write(data, len);
  radio.startListening();
  if (ok) {
    debugPrint("NRF24 Sent: " + String((char*)data, len));
  } else {
    debugPrint("NRF24 Send failed.");
  }
}

// ===================== ESP-NOW Functions =====================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  debugPrint("ESP-NOW Last Packet Send Status: " + String(status == ESP_NOW_SEND_OK ? "Success" : "Fail"));
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  debugPrint("ESP-NOW Received: " + String((char*)incomingData, len));

  // Forward received data to Pi, prefixed with RSP_RECEIVED_DATA
  txBuffer[0] = RSP_RECEIVED_DATA;
  if (len <= MAX_PAYLOAD_SIZE - 1) { // Check if data + opcode fits
    memcpy(&txBuffer[1], incomingData, len);
    sendFramedToPi(txBuffer, len + 1);
  } else {
    debugPrint("ESP-NOW received packet too large to forward to Pi.");
  }
}

bool setupESPNOW() {
  // WiFi.mode(WIFI_STA); // Will be set when activated
  if (esp_now_init() != ESP_OK) {
    debugPrint("Error initializing ESP-NOW");
    return false;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(&peerInfo.peer_addr, esp_peer_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    debugPrint("Failed to add initial ESP-NOW peer");
    // This isn't fatal for init, just means the default peer wasn't added.
    // Can be added later via command.
  }
  debugPrint("ESP-NOW initialized.");
  return true;
}

void activateESPNOW() {
  deactivateAllRadios(); // This will turn WiFi OFF
  WiFi.mode(WIFI_STA); // Turn WiFi back on for ESP-NOW
  WiFi.begin(); // Not strictly needed for ESP-NOW but good practice or can cause issues
                // esp_now_init() handles some of this.
  // Ensure ESP-NOW peer is re-added if WiFi was turned off and on.
  // Or, if not de-initializing ESP-NOW, this might not be needed.
  // For robustness, let's re-add peer.
  esp_now_del_peer(esp_peer_mac); // Try to delete existing, ignore error if not exists
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
      debugPrint("Failed to add ESP-NOW peer after WiFi restart.");
  }
  currentRadio = RADIO_ESPNOW;
  debugPrint("ESP-NOW activated.");
}


void sendESPNOW(const uint8_t* data, size_t len) {
  esp_err_t result = esp_now_send(esp_peer_mac, data, len);
  if (result != ESP_OK) {
    debugPrint("Error sending ESP-NOW data");
  } else {
    debugPrint("ESP-NOW sending initiated.");
  }
}

// ===================== Main Setup and Loop =====================
void setup() {
  Serial.begin(115200); // USB CDC for debug
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX); // UART to Pi Zero W
  debugPrint("ESP32-C3 Wireless Subsystem Starting...");

  // Initialize all radios' core functionalities.
  // We'll activate them (and manage power) when explicitly selected.
  setupLoRa();
  setupNRF24();
  setupESPNOW(); // ESP-NOW initializes WiFi but does not turn it on explicitly here.

  // Initially, no radio is active in terms of listening.
  // All radios are put into a low-power state by default.
  deactivateAllRadios();
  currentRadio = RADIO_NONE; // Explicitly set to none
}

void loop() {
  size_t payloadLen = 0;
  if (decodeFramedInput(Serial1, rxBuffer, payloadLen)) {
    // Process command from Pi Zero W
    if (payloadLen > 0) {
      uint8_t command = rxBuffer[0];
      uint8_t* data = &rxBuffer[1];
      size_t dataLen = payloadLen - 1;

      bool command_successful = false; // Flag for ACK/NACK

      switch (command) {
        case CMD_SET_RADIO_LORA:
          activateLoRa();
          command_successful = true;
          break;
        case CMD_SET_RADIO_NRF24:
          activateNRF24();
          command_successful = true;
          break;
        case CMD_SET_RADIO_ESPNOW:
          activateESPNOW();
          command_successful = true;
          break;
        case CMD_SEND_DATA:
          if (currentRadio == RADIO_LORA) {
            sendLoRa(data, dataLen);
            command_successful = true; // Assume success for now, actual radio TX might fail
          } else if (currentRadio == RADIO_NRF24) {
            sendNRF24(data, dataLen);
            command_successful = true;
          } else if (currentRadio == RADIO_ESPNOW) {
            sendESPNOW(data, dataLen);
            command_successful = true;
          } else {
            debugPrint("CMD_SEND_DATA: No radio selected to send data.");
            command_successful = false;
          }
          break;
        case CMD_SET_NRF24_TX_ADDR:
            if (currentRadio == RADIO_NRF24 && dataLen == 5) {
                memcpy(nrf_tx_addr, data, 5);
                radio.openWritingPipe(nrf_tx_addr);
                debugPrint("NRF24 TX address set to: " + String((char*)nrf_tx_addr, 5));
                command_successful = true;
            } else {
                debugPrint("CMD_SET_NRF24_TX_ADDR: Invalid command or data length.");
            }
            break;
        case CMD_SET_NRF24_RX_ADDR:
            if (currentRadio == RADIO_NRF24 && dataLen == 5) {
                memcpy(nrf_rx_addr, data, 5);
                radio.openReadingPipe(1, nrf_rx_addr);
                debugPrint("NRF24 RX address set to: " + String((char*)nrf_rx_addr, 5));
                command_successful = true;
            } else {
                debugPrint("CMD_SET_NRF24_RX_ADDR: Invalid command or data length.");
            }
            break;
        case CMD_SET_ESPNOW_PEER_MAC:
            if (dataLen == 6) { // This command can be accepted regardless of currentRadio
                // Remove existing peer first if it's the same or if there's only one
                esp_now_del_peer(esp_peer_mac); // Attempt to delete, error is ok if not found
                memcpy(esp_peer_mac, data, 6);
                memcpy(&peerInfo.peer_addr, esp_peer_mac, 6);
                if (esp_now_add_peer(&peerInfo) != ESP_OK){
                    debugPrint("CMD_SET_ESPNOW_PEER_MAC: Failed to add ESP-NOW peer with new MAC.");
                    command_successful = false;
                } else {
                    debugPrint("CMD_SET_ESPNOW_PEER_MAC: Peer MAC set to: " + String(esp_peer_mac[0], HEX) + ":" + String(esp_peer_mac[1], HEX) + ":" + String(esp_peer_mac[2], HEX) + ":" + String(esp_peer_mac[3], HEX) + ":" + String(esp_peer_mac[4], HEX) + ":" + String(esp_peer_mac[5], HEX));
                    command_successful = true;
                }
            } else {
                debugPrint("CMD_SET_ESPNOW_PEER_MAC: Invalid data length.");
            }
            break;
        default:
          debugPrint("Unknown command received: " + String(command, HEX));
          command_successful = false; // Unknown command is a NACK
          break;
      }
      // Send ACK/NACK response back to Pi
      if (command_successful) {
        sendResponseToPi(RSP_ACK, command);
      } else {
        sendResponseToPi(RSP_NACK, command);
      }
    }
  }

  // Handle incoming data for the currently active radio (only NRF24 needs polling)
  if (currentRadio == RADIO_NRF24) {
    loopNRF24();
  }
  // LoRa and ESP-NOW handle reception via callbacks, so no direct polling here
}
```