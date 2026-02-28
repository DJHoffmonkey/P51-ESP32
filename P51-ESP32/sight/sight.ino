#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_log.h>

/* --- MASTER AVIONICS (ESP32-C3 #1) ---
 * Role: MSP Parsing, K-14 HUD, 3-Char Tagged Serial Broadcast
 */

// --- MISSION CONSTANTS ---
const int BOOT_BUTTON = 9;   
const int LED_BLUE = 8;
const int BUILT_IN_LED_SCL = 6;
const int BUILT_IN_LED_SDA = 5;     
const int SLAVE_TX = 2;
const int SLAVE_RX = 3;
const int RX_FROM_FC = 20;
const int TX_TO_FC = 21;
const int ARM_RC_CHANNEL = 13; 
const float LEAD_SENSITIVITY = 6.0;
const float ACCEL_1G = 512.0;
const float VOLT_THRESHOLD = 14.0;
const unsigned long BATT_DEBOUNCE = 2000;

// HUD Constructor (72x40 0.42" OLED)
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2_builtin(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, BUILT_IN_LED_SCL, BUILT_IN_LED_SDA);

// --- SYSTEM VARIABLES ---
int manualContrast = 20; 
float currentContrast = 20.0; 
bool manualMode = false, insaneModeActive = false, txMax = false, lastBootState = HIGH, isLongPress = false;
unsigned long buttonDownTime = 0, lastRequest = 0, lastDataTime = 0, lowVoltTimer = 0, lastBroadcast = 0;

// Flight Data
float vBat = 16.0, currentG = 1.0, roll = 0, pitch = 0, lastPitch = 0, lastRoll = 0, heading = 0, altitude = 0;
float filteredX = 0, filteredY = 0;
uint16_t armSwitchValue = 1000;
bool sessionHasMSP = false, currentlyReceiving = false, showLowBatText = false;

// --- HARDWARE BOOST ---
void applyHardwareBoost(bool enable) {
  if (enable) {
    u8g2_builtin.sendF("c", 0xD9); 
    u8g2_builtin.sendF("c", 0xF1);
    u8g2_builtin.sendF("c", 0xDB); 
    u8g2_builtin.sendF("c", 0x40);
    u8g2_builtin.sendF("c", 0x8D); 
    u8g2_builtin.sendF("c", 0x14);
    insaneModeActive = true;
  } else {
    u8g2_builtin.sendF("c", 0xD9); 
    u8g2_builtin.sendF("c", 0x22);
    u8g2_builtin.sendF("c", 0xDB); 
    u8g2_builtin.sendF("c", 0x20);
    u8g2_builtin.sendF("c", 0x8D); 
    u8g2_builtin.sendF("c", 0x14);
    insaneModeActive = false;
  }
}

void sendMSPRequest(uint8_t cmd) { 
  uint8_t req[] = {'$', 'M', '<', 0, cmd, cmd}; 
  Serial1.write(req, 6); 
}

void readMSPResponse() {
  while (Serial1.available() >= 6) {
    if (Serial1.peek() != '$') { 
      Serial1.read(); 
      continue; 
    }
    if (Serial1.read() == '$' && Serial1.read() == 'M' && Serial1.read() == '>') {
      uint8_t size = Serial1.read(); 
      uint8_t cmd = Serial1.read();
      sessionHasMSP = true; 
      lastDataTime = millis();

      if (cmd == 108) { // ATTITUDE + HEADING
        int16_t angX = Serial1.read() | (Serial1.read() << 8); 
        int16_t angY = Serial1.read() | (Serial1.read() << 8); 
        int16_t head = Serial1.read() | (Serial1.read() << 8); 
        roll = angX / 10.0; 
        pitch = angY / 10.0; 
        heading = (float)head;
        for (int i = 0; i < size - 6; i++) {
          Serial1.read();
        }
      } 
      else if (cmd == 109) { // ALTITUDE
        int32_t altCm = Serial1.read() | (Serial1.read() << 8) | (Serial1.read() << 16) | (Serial1.read() << 24);
        altitude = altCm * 0.0328084; 
        for (int i = 0; i < size - 4; i++) {
          Serial1.read();
        }
      }
      else if (cmd == 102) { // RAW IMU
        for (int i = 0; i < 4; i++) {
          Serial1.read();
        }
        int16_t az = Serial1.read() | (Serial1.read() << 8); 
        currentG = (((float)az / ACCEL_1G) * 0.1) + (currentG * 0.9);
        for (int i = 0; i < size - 6; i++) {
          Serial1.read();
        }
      } 
      else if (cmd == 110) { // ANALOG VOLTS
        vBat = ((Serial1.read() / 10.0) * 0.2) + (vBat * 0.8);
        for (int i = 0; i < size - 1; i++) {
          Serial1.read();
        }
      } 
      else if (cmd == 105) { // RC CHANNELS
        for (int i = 0; i < 8; i++) {
          Serial1.read();
        }
        for (int i = 4; i < ARM_RC_CHANNEL; i++) { 
          Serial1.read(); 
          Serial1.read(); 
        }
        armSwitchValue = Serial1.read() | (Serial1.read() << 8); 
        int bytesConsumed = 8 + ((ARM_RC_CHANNEL - 4) * 2) + 2; 
        for (int i = 0; i < (size - bytesConsumed); i++) {
          Serial1.read(); 
        }
      } 
      else { 
        for (int i = 0; i < size; i++) {
          Serial1.read(); 
        }
      }
    }
  }
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);

  // LINK TO COMPUTER (USB Debugging)
  // Ensure "USB CDC On Boot" is ENABLED in the Arduino Tools menu
  Serial.begin(115200); 

  // LINK TO FLIGHT CONTROLLER (Pins 20/21)
  // This is your existing working FC connection
  Serial1.begin(115200, SERIAL_8N1, RX_FROM_FC, TX_TO_FC); 

  // LINK TO SLAVE (Cockpit Screens)
  // We use Serial0 but re-route it to different Pins
  // This prevents it from clashing with the FC on pins 20/21
  Serial0.begin(115200, SERIAL_8N1, SLAVE_RX, SLAVE_TX);

  // Hardware Initialization
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH); 
  
  u8g2_builtin.begin();
  u8g2_builtin.setContrast(20);
  applyHardwareBoost(false);

  Serial.println("MASTER ONLINE - CHECKING FC...");
  unsigned long bootCheckStart = millis();
  
  // Try for 2 seconds to find an FC
  while (millis() - bootCheckStart < 2000) {
    sendMSPRequest(108);
    delay(100);
    readMSPResponse();
    if (sessionHasMSP) {
      break;
    }
  }

  if (sessionHasMSP) {
    isBenchMode = false;
    Serial.println("MODE: FLIGHT (MSP DETECTED)");
  } else {
    isBenchMode = true;
    Serial.println("MODE: BENCH (SIMULATING)");
  }
}

void loop() {
  // INPUT HANDLING
  bool currentBootState = digitalRead(BOOT_BUTTON);
  
  if (currentBootState == LOW && lastBootState == HIGH) { 
    buttonDownTime = millis(); 
    isLongPress = false; 
  }
  else if (currentBootState == HIGH && lastBootState == LOW && !isLongPress) {
    if (manualMode) { 
      manualContrast = (manualContrast >= 255) ? 20 : manualContrast + 40; 
    } else { 
      txMax = !txMax; 
    }
  } 
  else if (currentBootState == LOW && (millis() - buttonDownTime > 1000) && !isLongPress) { 
    manualMode = !manualMode; 
    isLongPress = true; 
  }
  lastBootState = currentBootState;

  // DATA ACQUISITION (LOCKED MODE)
  if (isBenchMode) {
    float t = millis() / 1000.0;
    pitch = sin(t * 0.5) * 20.0; 
    roll = cos(t * 0.8) * 45.0; 
    altitude = 500 + (sin(t * 0.2) * 100);
    heading += 0.5; 
    if (heading >= 360) {
      heading = 0;
    }
    vBat = 16.2;
    currentG = 1.0;
    currentlyReceiving = false; 
  } 
  else {
    if (millis() - lastRequest > 50) { 
      sendMSPRequest(108); 
      sendMSPRequest(109); 
      sendMSPRequest(105); 
      sendMSPRequest(102); 
      sendMSPRequest(110); 
      lastRequest = millis(); 
    }
    readMSPResponse();
    currentlyReceiving = (millis() - lastDataTime < 1000);
  }

  // SHARED PHYSICS
  bool isArmed = (armSwitchValue > 1800);
  bool warActive = isArmed || txMax;
  
  float pDelta = (pitch - lastPitch); 
  float rDelta = (roll - lastRoll);
  
  filteredX = ((rDelta * -LEAD_SENSITIVITY) * 0.12) + (filteredX * 0.88);
  filteredY = ((pDelta * LEAD_SENSITIVITY) * 0.12) + (filteredY * 0.88);
  
  lastPitch = pitch; 
  lastRoll = roll;

  // POWER & CONTRAST
  if (isArmed && vBat < VOLT_THRESHOLD) {
    if (lowVoltTimer == 0) {
      lowVoltTimer = millis();
    }
    if (millis() - lowVoltTimer > BATT_DEBOUNCE) {
      showLowBatText = true;
    }
  } else { 
    lowVoltTimer = 0; 
    showLowBatText = false; 
  }

  int targetContrast = manualMode ? manualContrast : (warActive ? 255 : 20);
  
  if (abs(currentContrast - targetContrast) > 1) {
    float step = (targetContrast > currentContrast) ? 0.10 : 0.25;
    currentContrast += (targetContrast - currentContrast) * step; 
    u8g2_builtin.setContrast((int)currentContrast);
    
    if (currentContrast > 210 && !insaneModeActive) {
      applyHardwareBoost(true);
    }
    if (currentContrast < 90 && insaneModeActive) {
      applyHardwareBoost(false);
    }
  }

  // THE 3-CHAR TAGGED BROADCAST
  if (millis() - lastBroadcast > 50) {
    Serial0.print("ROL:"); 
    Serial0.print(roll); 
    Serial0.print(",");
    
    Serial0.print("PIT:"); 
    Serial0.print(pitch); 
    Serial0.print(",");
    
    Serial0.print("HED:"); 
    Serial0.print(heading); 
    Serial0.print(",");
    
    Serial0.print("ALT:"); 
    Serial0.print(altitude);
    Serial0.print(",");
    
    Serial0.print("BAT:"); 
    Serial0.print(vBat); 
    Serial0.print(",");
    
    Serial0.print("GFO:"); 
    Serial0.print(currentG);
    Serial0.print(",");
    
    Serial0.print("WAR:"); 
    Serial0.println(warActive ? 1 : 0);
    
    lastBroadcast = millis();
  }

  // HUD RENDERING
  float distSq = (filteredX * filteredX) + (filteredY * filteredY);
  bool blink = (millis() % 200 < 100);
  bool ledOn = (distSq < 1.5) || (showLowBatText && blink);
  digitalWrite(LED_BLUE, ledOn ? LOW : HIGH);

  u8g2_builtin.firstPage();
  do {
    u8g2_builtin.setFont(u8g2_font_04b_03_tr);
    
    const char* sym = "-";
    if (!isBenchMode) {
      if (currentlyReceiving) {
        sym = warActive ? "*" : "+";
      } else {
        sym = "X";
      }
    }
    
    u8g2_builtin.drawStr(2, 5, sym); 
    u8g2_builtin.drawStr(66, 5, sym);
    u8g2_builtin.drawStr(2, 39, sym); 
    u8g2_builtin.drawStr(66, 39, sym);

    u8g2_builtin.drawCircle(36, 20, 17); 
    u8g2_builtin.drawCircle(36, 20, 18);
    
    int curX = 36 + (int)filteredX; 
    int curY = 20 + (int)filteredY;
    
    u8g2_builtin.drawBox(curX - 1, curY - 1, 3, 3);
    
    for (int i = 0; i < 6; i++) {
      float a = i * 1.047;
      int bx = curX + (cos(a) * 12) - 1;
      int by = curY + (sin(a) * 12) - 1;
      u8g2_builtin.drawBox(bx, by, 2, 2);
    }
    
    u8g2_builtin.drawLine(0, 10, 0, 30);
    int gY = constrain(20 - (int)((currentG - 1.0) * 8.0), 10, 30);
    u8g2_builtin.drawHLine(0, gY, 3);

    if (showLowBatText) {
      char vBuf; 
      dtostrf(vBat, 4, 1, vBuf);
      u8g2_builtin.drawStr(25, 39, "LOW "); 
      u8g2_builtin.drawStr(45, 39, vBuf);
    }
    if (manualMode) {
      u8g2_builtin.drawStr(28, 5, "MAN");
    }
  } while (u8g2_builtin.nextPage());
}