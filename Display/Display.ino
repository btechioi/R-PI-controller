#include <SPI.h>
#include <TFT_eSPI.h>               // ILI9341 display driver
#include <XPT2046_Touchscreen.h>    // Resistive touchscreen driver

// --- Pin Definitions ---
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define SPI_MISO   19
#define SPI_MOSI   23
#define SPI_SCLK   18
#define SPI_CS     5   // SPI slave CS from master

// --- Global Objects ---
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

// --- Shared UI state ---
String robotMode = "Idle";
int controllerBattery = 92;
int robotBattery = 85;
bool connected = false;

// Robot telemetry placeholders
int robotTemp = 38;         // Celsius
float robotVoltage = 7.2f;  // Volts
int robotRSSI = -65;        // dBm
int robotRPM = 120;         // Motor speed

// SPI buffers and flags
#define MAX_FRAME 64
volatile uint8_t spi_rx_buf[MAX_FRAME];
volatile uint8_t spi_tx_buf[MAX_FRAME];
volatile bool data_ready = false;

// --- SPI slave interrupt handler ---
void IRAM_ATTR onSPIReceive() {
  static uint8_t i = 0;
  while (SPI.available()) {
    if (i < MAX_FRAME) {
      spi_rx_buf[i++] = SPI.transfer(0x00);
    }
  }
  data_ready = true;
  i = 0;
}

// --- Battery Icon Drawer ---
void drawBatteryIcon(int x, int y, int level, const char* label) {
  int barWidth = 8;
  int barHeight = 20;
  int outlineWidth = 40;
  int outlineHeight = 24;
  int levelBars = map(level, 0, 100, 0, 4);

  // Outline
  tft.drawRect(x, y, outlineWidth, outlineHeight, TFT_WHITE);
  tft.drawRect(x + outlineWidth, y + 6, 4, 12, TFT_WHITE); // battery tip

  // Label
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(x, y - 10);
  tft.print(label);

  // Bars inside battery
  for (int i = 0; i < 4; i++) {
    int bx = x + 4 + i * (barWidth + 2);
    uint16_t color = (i < levelBars)
        ? (level < 20 ? TFT_RED : (level < 60 ? TFT_YELLOW : TFT_GREEN))
        : TFT_DARKGREY;
    tft.fillRect(bx, y + 2, barWidth, barHeight, color);
  }

  // Percentage text
  tft.setCursor(x, y + outlineHeight + 2);
  tft.print(level);
  tft.print("%");
}

// --- Draw Robot Telemetry Stats ---
void drawStatsPanel() {
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 120);
  tft.printf("Temp: %dC   ", robotTemp);
  tft.setCursor(10, 145);
  tft.printf("Volt: %.2fV   ", robotVoltage);
  tft.setCursor(10, 170);
  tft.printf("RSSI: %ddBm   RPM: %d", robotRSSI, robotRPM);
}

// --- Draw Buttons ---
void drawButtons() {
  // Start Mission
  tft.fillRect(10, 200, 90, 35, TFT_GREEN);
  tft.setCursor(20, 210);
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.print("Start");

  // Reset
  tft.fillRect(110, 200, 90, 35, TFT_ORANGE);
  tft.setCursor(120, 210);
  tft.print("Reset");

  // Stop
  tft.fillRect(210, 200, 90, 35, TFT_RED);
  tft.setCursor(220, 210);
  tft.setTextColor(TFT_WHITE);
  tft.print("STOP");

  // Change Mode
  tft.fillRect(10, 240, 140, 35, TFT_BLUE);
  tft.setCursor(20, 250);
  tft.setTextColor(TFT_WHITE);
  tft.print("Change Mode");

  // Ping
  tft.fillRect(160, 240, 140, 35, TFT_DARKGREY);
  tft.setCursor(170, 250);
  tft.print("Ping");
}

// --- Draw Entire UI ---
void drawUI() {
  tft.fillScreen(TFT_BLACK);
  drawBatteryIcon(10, 10, controllerBattery, "Controller");
  drawBatteryIcon(150, 10, robotBattery, "Robot");

  tft.setCursor(10, 50);
  tft.setTextSize(2);
  tft.setTextColor(connected ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print("Status: ");
  tft.print(connected ? "Connected" : "Disconnected");

  tft.setCursor(10, 90);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Mode: ");
  tft.print(robotMode);

  drawStatsPanel();
  drawButtons();
}

// --- Send UI event to Pi Zero over SPI ---
void sendTouchEvent(uint8_t event_id) {
  digitalWrite(SPI_CS, LOW);
  SPI.transfer(0xAA); // Start byte
  SPI.transfer(0x01); // UI event command
  SPI.transfer(0x01); // Payload length
  SPI.transfer(event_id);
  SPI.transfer(0xAA ^ 0x01 ^ 0x01 ^ event_id); // simple checksum
  digitalWrite(SPI_CS, HIGH);
}

// --- Handle touch input and map to buttons ---
void handleTouch() {
  if (touch.touched()) {
    TS_Point p = touch.getPoint();

    // Map touchscreen coordinates (calibrate if needed)
    int x = map(p.x, 3800, 100, 0, 320);
    int y = map(p.y, 100, 3800, 0, 240);

    // Check buttons bounding boxes
    if (y >= 200 && y <= 235) {
      if (x >= 10 && x <= 100) sendTouchEvent(0x10);        // Start
      else if (x >= 110 && x <= 200) sendTouchEvent(0x20);  // Reset
      else if (x >= 210 && x <= 300) sendTouchEvent(0x30);  // Stop
    } else if (y >= 240 && y <= 275) {
      if (x >= 10 && x <= 150) sendTouchEvent(0x40);        // Change Mode
      else if (x >= 160 && x <= 300) sendTouchEvent(0x50);  // Ping
    }
  }
}

// --- Parse telemetry packet from Pi Zero ---
void parseTelemetry(uint8_t *buf, size_t len) {
  if (len < 9) return;
  if (buf[0] != 0xAA || buf[1] != 0x02) return; // Check start + cmd

  controllerBattery = buf[2];
  robotBattery = buf[3];
  connected = buf[4] != 0 ? true : false;
  robotTemp = buf[5];
  robotVoltage = buf[6] / 10.0f;
  robotRSSI = (int8_t)buf[7];
  robotRPM = buf[8];
  robotMode = String((char *)&buf[9]);

  drawUI();
}

// --- Poll SPI for data ---
void pollSPI() {
  if (data_ready) {
    data_ready = false;
    parseTelemetry((uint8_t *)spi_rx_buf, MAX_FRAME);
  }
}

// --- Setup and Main Loop ---
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);

  touch.begin();

  drawUI();

  pinMode(SPI_CS, INPUT_PULLUP);
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
  attachInterrupt(digitalPinToInterrupt(SPI_CS), onSPIReceive, FALLING);
}

void loop() {
  handleTouch();
  pollSPI();
}
