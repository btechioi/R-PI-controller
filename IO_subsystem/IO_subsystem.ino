#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h> // Make sure this library is installed (e.g., from Adafruit or I2CDev)

// --- Define Pico Specific SPI Pins ---
// Default SPI on Pico:
// SCK: GP18
// MOSI: GP19
// MISO: GP16
// CS: Usually assigned by user. We will use GP1 for CS.
#define SPI_SCK_PIN     18
#define SPI_MOSI_PIN    19
#define SPI_MISO_PIN    16
#define SPI_CS_PIN      1   // *** CHANGED: Dedicated CS pin for SPI slave to avoid conflict with enc2B (GP17) ***

// --- Constants ---
// Calculated Payload Length (8 analog * 2 bytes/val + 6 buttons + 6 switches + 2 encoders * 2 bytes/enc + 3 accel axes * 2 bytes/axis)
#define ANALOG_COUNT    8
#define BUTTON_COUNT    6
#define SWITCH_COUNT    6
#define ENCODER_COUNT   2
#define ACCEL_AXES      3

#define PAYLOAD_LEN     (ANALOG_COUNT * 2 + BUTTON_COUNT + SWITCH_COUNT + ENCODER_COUNT * 2 + ACCEL_AXES * 2) // 16+6+6+4+6 = 38 bytes

// Frame Structure: START_BYTE (1) + PAYLOAD (38) + CHECKSUM (1) + END_BYTE (1) + PADDING (1)
#define FRAME_LEN       (1 + PAYLOAD_LEN + 1 + 1 + 1) // 42 bytes

// --- Input Pins ---
// Note: Analog pins A0-A7 typically map to GPIO 26-28 on Pico W, and A0-A3 (GPIO 26-29) on standard Pico.
// Ensure these map to physical pins you connect to ADCs.
const uint8_t analogPins[ANALOG_COUNT] = {26, 27, 28, 29, 22, 21, 20, 19}; // Example: using available GPIOs as analog inputs if A0-A7 are not direct labels. Adjust as per your board.
const uint8_t buttonPins[BUTTON_COUNT] = {2, 3, 4, 5, 6, 7};
const uint8_t switchPins[SWITCH_COUNT] = {8, 9, 10, 11, 12, 13};

// Rotary encoders
const uint8_t enc1A = 14, enc1B = 15;
const uint8_t enc2A = 16, enc2B = 17; // enc2B (GP17) no longer conflicts with SPI CS

volatile int16_t enc1Count = 0, enc2Count = 0;

// MPU6050
MPU6050 mpu;
int16_t accX = 0, accY = 0, accZ = 0;

// --- Function Prototypes ---
void setupInputs();
void setupMPU();
uint8_t calculateChecksum(uint8_t* data, uint8_t len);
void buildPayload(uint8_t* payload);

// --- Interrupt Service Routines (ISRs) for Encoders ---
// IRAM_ATTR is removed as it's not applicable to RP2040
void enc1ISR() {
  if (digitalRead(enc1A) == digitalRead(enc1B)) enc1Count++;
  else enc1Count--;
}

void enc2ISR() {
  if (digitalRead(enc2A) == digitalRead(enc2B)) enc2Count++;
  else enc2Count--;
}

// --- Setup Functions ---
void setupInputs() {
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  for (uint8_t i = 0; i < SWITCH_COUNT; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }
  
  pinMode(enc1A, INPUT_PULLUP);
  pinMode(enc1B, INPUT_PULLUP);
  pinMode(enc2A, INPUT_PULLUP);
  pinMode(enc2B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(enc1A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2A), enc2ISR, CHANGE);
}

void setupMPU() {
  Wire.begin(); // Default I2C pins on Pico: SDA=GP4, SCL=GP5
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected or communication failed!");
  } else {
    Serial.println("MPU6050 connection successful.");
  }
}

// --- Utility Functions ---
uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void buildPayload(uint8_t* payload) {
  uint8_t index = 0;
  
  // Joysticks (Analog Inputs)
  for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
    uint16_t val = analogRead(analogPins[i]);
    payload[index++] = val >> 8; // MSB
    payload[index++] = val & 0xFF; // LSB
  }

  // Buttons (Digital Inputs)
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    // Buttons are INPUT_PULLUP, so HIGH means not pressed, LOW means pressed (0)
    payload[index++] = digitalRead(buttonPins[i]) ? 1 : 0; // 1 if not pressed, 0 if pressed
  }

  // Switches (Digital Inputs)
  for (uint8_t i = 0; i < SWITCH_COUNT; i++) {
    // Switches are INPUT_PULLUP, so HIGH means open, LOW means closed (0)
    payload[index++] = digitalRead(switchPins[i]) ? 1 : 0; // 1 if open, 0 if closed
  }

  // Encoders
  noInterrupts(); // Disable interrupts while reading volatile variables
  int16_t e1 = enc1Count;
  int16_t e2 = enc2Count;
  interrupts(); // Re-enable interrupts
  
  payload[index++] = e1 >> 8; // MSB
  payload[index++] = e1 & 0xFF; // LSB
  payload[index++] = e2 >> 8; // MSB
  payload[index++] = e2 & 0xFF; // LSB

  // MPU6050 Accelerometer
  mpu.getAcceleration(&accX, &accY, &accZ);
  payload[index++] = accX >> 8; // MSB
  payload[index++] = accX & 0xFF; // LSB
  payload[index++] = accY >> 8; // MSB
  payload[index++] = accY & 0xFF; // LSB
  payload[index++] = accZ >> 8; // MSB
  payload[index++] = accZ & 0xFF; // LSB
}

// --- Main Setup ---
void setup() {
  Serial.begin(115200); // USB CDC for debugging
  Serial.println("I/O Subsystem Starting on Pico...");

  setupInputs();
  setupMPU();

  // --- SPI Slave Init for Pico ---
  // Using specific pins for SPI on Pico: SCK, MISO, MOSI, CS
  // The SPI.begin(SCK, MISO, MOSI, SS_PIN) method is common for master/slave setup on RP2040.
  // We explicitly set SlaveMode.
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  SPI.setSlaveMode(true); // Set the Pico as an SPI slave
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0); // Most common mode, ensure master matches

  // The CS pin needs to be an input and interrupt-capable for typical slave sensing
  pinMode(SPI_CS_PIN, INPUT_PULLUP); // Or INPUT_PULLDOWN, depending on master's CS polarity

  Serial.println("SPI Slave initialized.");
  Serial.print("Payload size: "); Serial.print(PAYLOAD_LEN); Serial.println(" bytes.");
  Serial.print("Frame size: "); Serial.print(FRAME_LEN); Serial.println(" bytes.");
}

// --- Main Loop ---
void loop() {
  // SPI Slave Polling Loop
  // In SPI slave mode on Pico, you typically use SPI.transfer() to both send and receive
  // within the same transaction. The master initiates the clock.
  // We'll continuously transfer a dummy byte (0x00) and check what we receive.
  
  uint8_t receivedByte = SPI.transfer(0x00); // Send 0x00, receive whatever master sends
  
  // Check if CS is LOW (active), indicating a transaction might be in progress.
  // This is a common pattern to ensure you're in an active transaction.
  // Some slave libraries might handle this implicitly, but explicit check is safer.
  if (digitalRead(SPI_CS_PIN) == LOW) { // Assuming active LOW CS
      if (receivedByte == 0xA1) {
        // Master requested data!
        Serial.println("Received 0xA1 command via SPI.");
        
        uint8_t payload[PAYLOAD_LEN]; // Corrected size
        buildPayload(payload);
        uint8_t checksum = calculateChecksum(payload, PAYLOAD_LEN); // Use correct length

        uint8_t frame[FRAME_LEN]; // Corrected size
        
        frame[0] = 0xAA; // Start byte
        memcpy(&frame[1], payload, PAYLOAD_LEN); // Copy payload
        frame[1 + PAYLOAD_LEN] = checksum; // Checksum at the correct index
        frame[1 + PAYLOAD_LEN + 1] = 0x55; // End byte
        frame[1 + PAYLOAD_LEN + 1 + 1] = 0x00; // Padding

        // Transfer the entire frame back to the master
        // The master needs to perform FRAME_LEN transfers to receive all data.
        for (uint8_t i = 0; i < FRAME_LEN; i++) {
          SPI.transfer(frame[i]);
        }
        Serial.println("Sent sensor data frame via SPI.");
      }
  }
  
  // Add a small delay if needed, but not strictly necessary for SPI polling
  // as SPI transactions are fast. Be mindful of how often the master polls.
  delay(1); // Small delay to yield to other tasks if any, and not busy-loop too aggressively
            // Can be removed if the master polls very frequently.
}