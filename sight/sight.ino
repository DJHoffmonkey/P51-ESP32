#include <U8g2lib.h>
#include <Wire.h>

// --- HARDWARE INITIALIZATION ---
// This specific OLED is 72x40. We use "HW_I2C" for speed.
// Pins 5/6 are the SDA/SCL lines on your ESP32-C3.
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 6, /* data=*/ 5);

const int BOOT_BUTTON = 9;   // Built-in button on most C3 boards
const int LED_BLUE = 8;      // Physical "Master Caution" LED

// --- MISSION-SPECIFIC CONSTANTS ---
const int ARM_RC_CHANNEL = 13;       // We look at the 14th channel (index 13)
const float LEAD_SENSITIVITY = 6.0; // How much the reticle "leads" your turn
const float ACCEL_1G = 512.0;       // Raw IMU value that represents 1G of gravity
const float VOLT_THRESHOLD = 14.0;   // 3.5V per cell (4S pack)
const unsigned long BATT_DEBOUNCE = 2000; // Time in ms battery must stay low to trigger alarm

// --- SYSTEM VARIABLES ---
int manualContrast = 20; 
float currentContrast = 20.0; 
bool manualMode = false, insaneModeActive = false, txMax = false, lastBootState = HIGH, isLongPress = false;
unsigned long diagnosticTimer = 0, buttonDownTime = 0, lastRequest = 0, lastDataTime = 0;
unsigned long lowVoltTimer = 0; 

// Physics Variables (using Floats for smooth movement)
float filteredX = 0, filteredY = 0, roll = 0, pitch = 0, lastPitch = 0, lastRoll = 0, currentG = 1.0; 
float vBat = 16.0; 
uint16_t armSwitchValue = 1000; 
bool sessionHasMSP = false; 
bool showLowBatText = false;

void setup() {
  Serial.begin(115200); // USB connection for your computer
  
  // Serial1 is the dedicated Hardware UART for the Flight Controller (Pins 20/21)
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  
  Wire.begin(5, 6);
  Wire.setClock(400000); // Set I2C to "Fast Mode" for high-speed HUD refresh
  u8g2.begin();
  
  applyHardwareBoost(false); // Start in low-power "Bench Mode"
  u8g2.setContrast(20); 
  
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH); // LEDs on these boards are often "Active Low" (High = Off)
}

// THE "STARK" HACK: Directly sending HEX commands to the SSD1306 controller
// to overdrive the internal charge pump for maximum sunlight visibility.
void applyHardwareBoost(bool enable) {
  if (enable) {
    u8g2.sendF("c", 0xD9); u8g2.sendF("c", 0xF1); // Increase Pre-charge period
    u8g2.sendF("c", 0xDB); u8g2.sendF("c", 0x40); // Maximize VCOMH deselect level
    u8g2.sendF("c", 0x8D); u8g2.sendF("c", 0x14); // Enable Charge Pump
    insaneModeActive = true;
  } else {
    u8g2.sendF("c", 0xD9); u8g2.sendF("c", 0x22); // Reset to factory defaults
    u8g2.sendF("c", 0xDB); u8g2.sendF("c", 0x20); 
    u8g2.sendF("c", 0x8D); u8g2.sendF("c", 0x14); 
    insaneModeActive = false;
  }
}

// Draws the main circle. Layering circles creates a "bold" look for better optics.
void drawThickCircle(int x, int y, int r, int thickness) {
  for (int i = 0; i < thickness; i++) u8g2.drawCircle(x, y, r + i);
}

// This moves the aiming dot and hexagon ring based on your flight dynamics.
void drawReticle(float ox, float oy) {
  drawThickCircle(36, 20, 17, 2); 
  int curX = 36 + (int)ox; int curY = 20 + (int)oy;
  u8g2.drawBox(curX - 1, curY - 1, 3, 3); // Aiming pip
  
  // Draw the hexagonal outer pips for that fighter-jet aesthetic
  for (int i = 0; i < 6; i++) {
    float a = i * 1.047; // 60 degrees in radians
    u8g2.drawBox(curX + (cos(a) * 12) - 1, curY + (sin(a) * 12) - 1, 2, 2);
  }
}

// Formats a standard MultiWii Serial Protocol (MSP) request.
// Structure: $ (Header) | M (MultiWii) | < (To FC) | Size | Command | Checksum
void sendMSPRequest(uint8_t cmd) { 
  uint8_t req[] = {'$', 'M', '<', 0, cmd, cmd}; 
  Serial1.write(req, 6); 
}

// The Parser: Sifts through the Serial stream to find the data we want.
void readMSPResponse() {
  while (Serial1.available() >= 6) {
    if (Serial1.peek() != '$') { Serial1.read(); continue; } // Skip junk until we see '$'
    if (Serial1.available() >= 6) {
      if (Serial1.read() == '$' && Serial1.read() == 'M' && Serial1.read() == '>') {
        uint8_t size = Serial1.read(); 
        uint8_t cmd = Serial1.read();
        sessionHasMSP = true; 
        
        if (cmd == 108) { // ATTITUDE (Roll/Pitch)
          int16_t angX = Serial1.read() | (Serial1.read() << 8); // Combine 2 bytes into 16-bit int
          int16_t angY = Serial1.read() | (Serial1.read() << 8); 
          roll = angX / 10.0; pitch = angY / 10.0; lastDataTime = millis();
          for (int i = 0; i < size - 4; i++) Serial1.read(); // Clear remaining bytes
        } 
        else if (cmd == 102) { // RAW IMU (G-force)
          for (int i = 0; i < 4; i++) Serial1.read(); // Skip Gyro data
          int16_t az = Serial1.read() | (Serial1.read() << 8); 
          // Low-pass filter for G-force to keep the HUD stable
          currentG = (((float)az / ACCEL_1G) * 0.1) + (currentG * 0.9);
          if (abs(currentG - 1.0) < 0.08) currentG = 1.0; // Nullify minor vibrations
          for (int i = 0; i < size - 6; i++) Serial1.read();
        } 
        else if (cmd == 110) { // ANALOG (Battery Voltage)
          float rawV = Serial1.read() / 10.0; 
          vBat = (rawV * 0.2) + (vBat * 0.8); // Smooth out the 60A throttle sag noise
          for (int i = 0; i < size - 1; i++) Serial1.read();
        } 
        else if (cmd == 105) { // MSP_RC (Radio Channels)
          for (int i = 0; i < 8; i++) Serial1.read(); // Skip Sticks (AIL, ELE, THR, RUD)
          // Skip AUX channels until we hit your ARM switch
          for (int i = 4; i < ARM_RC_CHANNEL; i++) { Serial1.read(); Serial1.read(); }
          armSwitchValue = Serial1.read() | (Serial1.read() << 8); 
          int bytesConsumed = 8 + ((ARM_RC_CHANNEL - 4) * 2) + 2; 
          for (int i = 0; i < (size - bytesConsumed); i++) Serial1.read(); 
        } 
        else { for (int i = 0; i < size; i++) Serial1.read(); } // Discard unknown commands
      }
    }
  }
}

void loop() {
  // Request fresh data from FC every 50ms (20Hz)
  if (millis() - lastRequest > 50) { 
    sendMSPRequest(108); sendMSPRequest(105); 
    sendMSPRequest(102); sendMSPRequest(110); 
    lastRequest = millis(); 
  }
  readMSPResponse();

  // --- CONTRAST LOGIC ---
  // If Armed, we ramp contrast up to 255 (Sunlight Mode).
  // If Disarmed, we keep it dim (20) to save the OLED life.
  bool isArmed = (armSwitchValue > 1800);
  bool warActive = isArmed || txMax;

  // --- BATTERY SAG PROTECTION ---
  // This logic checks if voltage is below threshold. 
  // It must STAY below for 2 seconds (BATT_DEBOUNCE) to trigger.
  if (isArmed && vBat < VOLT_THRESHOLD) {
    if (lowVoltTimer == 0) lowVoltTimer = millis();
    if (millis() - lowVoltTimer > BATT_DEBOUNCE) showLowBatText = true;
  } else {
    lowVoltTimer = 0; showLowBatText = false;
  }

  // Smoothly transition contrast to avoid a jarring "flash" on the pilot's eyes
  int targetContrast = manualMode ? manualContrast : (warActive ? 255 : 20);
  if (abs(currentContrast - targetContrast) > 1) {
    float speed = (targetContrast > currentContrast) ? 0.10 : 0.25;
    currentContrast += (targetContrast - currentContrast) * speed; 
    u8g2.setContrast((int)currentContrast);
    if (currentContrast > 210 && !insaneModeActive) applyHardwareBoost(true);
    if (currentContrast < 90 && insaneModeActive) applyHardwareBoost(false);
  }

  // --- PHYSICS CALCULATIONS ---
  bool currentlyReceiving = (millis() - lastDataTime < 1000);
  if (!sessionHasMSP) {
    // Demo Mode: If no FC is connected, simulate movement so you can test optics
    float t = millis() / 1000.0;
    filteredX = sin(t * 0.7) * 12.0; filteredY = cos(t * 0.5) * 10.0;
    currentG = 1.0 + (sin(t * 1.2) * 0.1);
  } else if (currentlyReceiving) {
    // HUD Lead Logic: Reticle moves in opposition to roll/pitch rate
    float pDelta = (pitch - lastPitch); float rDelta = (roll - lastRoll);
    filteredX = ( (rDelta * -LEAD_SENSITIVITY) * 0.12) + (filteredX * 0.88);
    filteredY = ( (pDelta * LEAD_SENSITIVITY) * 0.12) + (filteredY * 0.88);
    lastPitch = pitch; lastRoll = roll;
  }

  // --- LED MASTER CAUTION LOGIC ---
  float distSq = (filteredX * filteredX) + (filteredY * filteredY);
  bool ledOn = false;
  if (distSq < 1.0) ledOn = true; // Target Centered
  else if (showLowBatText) ledOn = (millis() % 200 < 100); // Rapid Battery Flash
  else if (distSq < 64.0 && (millis() % ((int)(distSq * 10) + 80) < 40)) ledOn = true; // Near-center alert
  digitalWrite(LED_BLUE, ledOn ? LOW : HIGH);

  // --- RENDERING LOOP ---
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_04b_03_tr);
    const char* symbol = currentlyReceiving ? ( (warActive || manualMode) ? "*" : "+" ) : "-";
    
    // Corners and G-Meter (Static elements)
    u8g2.drawStr(2, 5, symbol); u8g2.drawStr(66, 5, symbol);
    u8g2.drawStr(2, 39, symbol); u8g2.drawStr(66, 39, symbol);
    u8g2.drawLine(0, 10, 0, 30);
    int gPos = 20 - (int)((currentG - 1.0) * 8.0); 
    u8g2.drawHLine(0, constrain(gPos, 10, 30), 3); 

    // Battery Warning (Overlays only when critical)
    if (showLowBatText) {
      char vBuf[6]; dtostrf(vBat, 4, 1, vBuf);
      u8g2.drawStr(25, 39, "LOW "); u8g2.drawStr(45, 39, vBuf);
    }
    
    drawReticle(filteredX, filteredY);
  } while (u8g2.nextPage());
}