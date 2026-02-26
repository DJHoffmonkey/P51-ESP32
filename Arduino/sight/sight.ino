#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

/* --- MASTER AVIONICS (ESP32-C3 #1) ---
 * Role: MSP Parsing, K-14 HUD (0.49" 72x40), Serial Bridge to Slave
 * Pins: 
 * - HUD: 5 (SDA), 6 (SCL)
 * - FC: 20 (RX)
 * - Slave Link: 21 (TX)
 * - Button: 9
 * - Blue LED: 8
 */

// HUD Constructor (72x40 0.49" OLED)
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 6, /* data=*/ 5);

// --- MISSION CONSTANTS ---
const int BOOT_BUTTON = 9;   
const int LED_BLUE = 8;      
const int ARM_RC_CHANNEL = 13; 
const float LEAD_SENSITIVITY = 6.0;
const float ACCEL_1G = 512.0;
const float VOLT_THRESHOLD = 14.0;
const unsigned long BATT_DEBOUNCE = 2000;

// --- SYSTEM VARIABLES ---
int manualContrast = 20; 
float currentContrast = 20.0; 
bool manualMode = false, insaneModeActive = false, txMax = false, lastBootState = HIGH, isLongPress = false;
unsigned long buttonDownTime = 0, lastRequest = 0, lastDataTime = 0, lowVoltTimer = 0, lastBroadcast = 0;

// Flight Data
float vBat = 16.0, currentG = 1.0, roll = 0, pitch = 0, lastPitch = 0, lastRoll = 0, heading = 0;
float filteredX = 0, filteredY = 0;
uint16_t armSwitchValue = 1000;
bool sessionHasMSP = false, currentlyReceiving = false, showLowBatText = false;

// --- HARDWARE BOOST (STARK HACK) ---
void applyHardwareBoost(bool enable) {
  if (enable) {
    u8g2.sendF("c", 0xD9); u8g2.sendF("c", 0xF1);
    u8g2.sendF("c", 0xDB); u8g2.sendF("c", 0x40);
    u8g2.sendF("c", 0x8D); u8g2.sendF("c", 0x14);
    insaneModeActive = true;
  } else {
    u8g2.sendF("c", 0xD9); u8g2.sendF("c", 0x22);
    u8g2.sendF("c", 0xDB); u8g2.sendF("c", 0x20);
    u8g2.sendF("c", 0x8D); u8g2.sendF("c", 0x14);
    insaneModeActive = false;
  }
}

void sendMSPRequest(uint8_t cmd) { 
  uint8_t req[] = {'$', 'M', '<', 0, cmd, cmd}; 
  Serial1.write(req, 6); 
}

void readMSPResponse() {
  while (Serial1.available() >= 6) {
    if (Serial1.peek() != '$') { Serial1.read(); continue; }
    if (Serial1.read() == '$' && Serial1.read() == 'M' && Serial1.read() == '>') {
      uint8_t size = Serial1.read(); uint8_t cmd = Serial1.read();
      sessionHasMSP = true; lastDataTime = millis();

      if (cmd == 108) { // ATTITUDE + HEADING
        int16_t angX = Serial1.read() | (Serial1.read() << 8); // Roll
        int16_t angY = Serial1.read() | (Serial1.read() << 8); // Pitch
        int16_t head = Serial1.read() | (Serial1.read() << 8); // Heading
        roll = angX / 10.0; pitch = angY / 10.0; heading = (float)head;
        for (int i = 0; i < size - 6; i++) Serial1.read();
      } 
      else if (cmd == 102) { // RAW IMU
        for (int i = 0; i < 4; i++) Serial1.read();
        int16_t az = Serial1.read() | (Serial1.read() << 8); 
        currentG = (((float)az / ACCEL_1G) * 0.1) + (currentG * 0.9);
        for (int i = 0; i < size - 6; i++) Serial1.read();
      } 
      else if (cmd == 110) { // ANALOG VOLTS
        vBat = ((Serial1.read() / 10.0) * 0.2) + (vBat * 0.8);
        for (int i = 0; i < size - 1; i++) Serial1.read();
      } 
      else if (cmd == 105) { // RC CHANNELS
        for (int i = 0; i < 8; i++) Serial1.read();
        for (int i = 4; i < ARM_RC_CHANNEL; i++) { Serial1.read(); Serial1.read(); }
        armSwitchValue = Serial1.read() | (Serial1.read() << 8); 
        int bytesConsumed = 8 + ((ARM_RC_CHANNEL - 4) * 2) + 2; 
        for (int i = 0; i < (size - bytesConsumed); i++) Serial1.read(); 
      } 
      else { for (int i = 0; i < size; i++) Serial1.read(); }
    }
  }
}

void setup() {
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH); 
  
  u8g2.begin();
  u8g2.setContrast(20);
  applyHardwareBoost(false);
}

void loop() {
  // 1. BUTTON / CONTRAST OVERRIDE
  bool currentBootState = digitalRead(BOOT_BUTTON);
  if (currentBootState == LOW && lastBootState == HIGH) { buttonDownTime = millis(); isLongPress = false; }
  else if (currentBootState == HIGH && lastBootState == LOW && !isLongPress) {
    if (manualMode) { manualContrast = (manualContrast >= 255) ? 20 : manualContrast + 40; } else { txMax = !txMax; }
  } else if (currentBootState == LOW && (millis() - buttonDownTime > 1000) && !isLongPress) { manualMode = !manualMode; isLongPress = true; }
  lastBootState = currentBootState;

  // 2. DATA POLLING
  if (millis() - lastRequest > 50) { 
    sendMSPRequest(108); sendMSPRequest(105); 
    sendMSPRequest(102); sendMSPRequest(110); 
    lastRequest = millis(); 
  }
  readMSPResponse();

  // 3. BRIGHTNESS LOGIC
  bool isArmed = (armSwitchValue > 1800);
  bool warActive = isArmed || txMax;
  
  if (isArmed && vBat < VOLT_THRESHOLD) {
    if (lowVoltTimer == 0) lowVoltTimer = millis();
    if (millis() - lowVoltTimer > BATT_DEBOUNCE) showLowBatText = true;
  } else { lowVoltTimer = 0; showLowBatText = false; }

  int targetContrast = manualMode ? manualContrast : (warActive ? 255 : 20);
  if (abs(currentContrast - targetContrast) > 1) {
    currentContrast += (targetContrast - currentContrast) * ((targetContrast > currentContrast) ? 0.10 : 0.25); 
    u8g2.setContrast((int)currentContrast);
    if (currentContrast > 210 && !insaneModeActive) applyHardwareBoost(true);
    if (currentContrast < 90 && insaneModeActive) applyHardwareBoost(false);
  }

  // 4. PHYSICS & GHOST SIMULATION
  currentlyReceiving = (millis() - lastDataTime < 1000);
  if (!sessionHasMSP || !currentlyReceiving) {
    float t = millis() / 1000.0;
    filteredX = sin(t * 0.7) * 12.0; filteredY = cos(t * 0.5) * 10.0;
    currentG = 1.0 + (sin(t * 1.2) * 0.1); vBat = 16.2 + (sin(t * 0.3) * 0.2);
    pitch = sin(t * 0.5) * 20.0; roll = cos(t * 0.8) * 45.0;
    heading += 0.5; if (heading >= 360) heading = 0;
  } else {
    float pDelta = (pitch - lastPitch); float rDelta = (roll - lastRoll);
    filteredX = ((rDelta * -LEAD_SENSITIVITY) * 0.12) + (filteredX * 0.88);
    filteredY = ((pDelta * LEAD_SENSITIVITY) * 0.12) + (filteredY * 0.88);
    lastPitch = pitch; lastRoll = roll;
  }

  // 5. SERIAL BROADCAST TO SLAVE (V, G, P, R, H)
  if (millis() - lastBroadcast > 50) {
    Serial1.print(vBat); Serial1.print(",");
    Serial1.print(currentG); Serial1.print(",");
    Serial1.print(pitch); Serial1.print(",");
    Serial1.print(roll); Serial1.print(",");
    Serial1.println(heading);
    lastBroadcast = millis();
  }

  // 6. HUD RENDERING
  float distSq = (filteredX * filteredX) + (filteredY * filteredY);
  bool ledOn = (distSq < 1.5) || (showLowBatText && (millis() % 200 < 100));
  digitalWrite(LED_BLUE, ledOn ? LOW : HIGH);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_04b_03_tr);
    const char* sym = currentlyReceiving ? (warActive ? "*" : "+") : "-";
    u8g2.drawStr(2, 5, sym); u8g2.drawStr(66, 5, sym);
    u8g2.drawStr(2, 39, sym); u8g2.drawStr(66, 39, sym);

    // K-14 Sight Reticle
    u8g2.drawCircle(36, 20, 17); u8g2.drawCircle(36, 20, 18);
    int curX = 36 + (int)filteredX; int curY = 20 + (int)filteredY;
    u8g2.drawBox(curX - 1, curY - 1, 3, 3);
    for (int i = 0; i < 6; i++) {
      float a = i * 1.047;
      u8g2.drawBox(curX + (cos(a) * 12) - 1, curY + (sin(a) * 12) - 1, 2, 2);
    }
    
    // G-Meter
    u8g2.drawLine(0, 10, 0, 30);
    u8g2.drawHLine(0, constrain(20 - (int)((currentG - 1.0) * 8.0), 10, 30), 3);

    if (showLowBatText) {
      char vBuf[6]; dtostrf(vBat, 4, 1, vBuf);
      u8g2.drawStr(25, 39, "LOW "); u8g2.drawStr(45, 39, vBuf);
    }
    if (manualMode) u8g2.drawStr(28, 5, "MAN");
  } while (u8g2.nextPage());
}