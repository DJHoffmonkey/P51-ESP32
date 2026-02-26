#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

/* --- SLAVE AVIONICS (ESP32-C3 #2) ---
 * Role: 5-Screen Instrument Cluster
 * Displays: 
 * 1 & 2: 1.3" SPI SH1106 (Main Panels - CS Pins 0 and 7)
 * 3 & 4: 0.96" I2C SSD1306 (Aux Panels - Addresses 0x3C and 0x3D)
 * 5: 0.49" I2C SSD1306 (Ribbon Compass - Shared Bus)
 */

struct Gauge { 
  int x, y, r; 
  float minVal, maxVal; 
  const char* label; 
};

// 2. CONSTRUCTORS
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2_L(U8G2_R1, 0, 1, 2); // Left 1.3"
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2_R(U8G2_R1, 7, 1, 2); // Right 1.3"
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2_P1(U8G2_R0, U8X8_PIN_NONE, 6, 5); // 0.96" #1
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2_P2(U8G2_R0, U8X8_PIN_NONE, 6, 5); // 0.96" #2
U8G2_SSD1306_72X40_ER_1_HW_I2C      u8g2_S(U8G2_R0,  U8X8_PIN_NONE, 6, 5); // 0.49" Compass

// 3. GLOBAL VARIABLES
float vBat, currentG, pitch, roll, heading;
bool masterArmed = false;
int currentSlaveContrast = 20;

// Stencil offsets for the 1.3" screens
Gauge leftPanel[] = { {32, 32, 28, 0, 700, "MPH"}, {32, 96, 28, 0, 10000, "ALT"} };
Gauge rightPanel[] = { {32, 96, 28, -2000, 2000, "VSI"} };

// 4. CLEAN BRIGHTNESS SYNC (No Hardware Overdrive)
void syncContrast(bool armed) {
  int target = armed ? 255 : 20;
  if (currentSlaveContrast != target) {
    u8g2_L.setContrast(target);
    u8g2_R.setContrast(target);
    u8g2_P1.setContrast(target);
    u8g2_P2.setContrast(target);
    u8g2_S.setContrast(target);
    currentSlaveContrast = target;
  }
}

// 5. DRAWING FUNCTIONS
void drawNeedle(U8G2 &u8, Gauge g, float val) {
  u8.drawCircle(g.x, g.y, g.r);
  // 270-degree sweep: 135 to 405
  float angle = map(constrain(val, g.minVal, g.maxVal) * 10, g.minVal * 10, g.maxVal * 10, 135, 405) * (PI / 180.0);
  u8.drawLine(g.x, g.y, g.x + cos(angle) * (g.r - 4), g.y + sin(angle) * (g.r - 4));
  u8.setFont(u8g2_font_04b_03_tr);
  u8.drawStr(g.x - 10, g.y + g.r - 2, g.label);
}

void parseData(String data) {
  int c1 = data.indexOf(','); int c2 = data.indexOf(',', c1 + 1);
  int c3 = data.indexOf(',', c2 + 1); int c4 = data.indexOf(',', c3 + 1);
  int c5 = data.indexOf(',', c4 + 1);
  
  if (c5 != -1) {
    vBat = data.substring(0, c1).toFloat();
    currentG = data.substring(c1 + 1, c2).toFloat();
    pitch = data.substring(c2 + 1, c3).toFloat();
    roll = data.substring(c3 + 1, c4).toFloat();
    heading = data.substring(c4 + 1, c5).toFloat();
    masterArmed = (data.substring(c5 + 1).toInt() == 1);
  }
}

void setup() {
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  u8g2_L.begin(); u8g2_R.begin();
  u8g2_P1.setI2CAddress(0x3C * 2); u8g2_P1.begin();
  u8g2_P2.setI2CAddress(0x3D * 2); u8g2_P2.begin();
  u8g2_S.begin();
  
  syncContrast(false); // Start Dim
}

void loop() {
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    parseData(line);

    // Instrument brightness
    syncContrast(masterArmed);

    // --- SCREEN RENDER ---
    u8g2_L.clearBuffer();
    drawNeedle(u8g2_L, leftPanel[0], (vBat - 12.0) * 150);
    drawNeedle(u8g2_L, leftPanel[1], (currentG * 1000));
    u8g2_L.sendBuffer();

    u8g2_R.clearBuffer();
    u8g2_R.drawCircle(32, 32, 28);
    float rad = roll * (PI / 180.0);
    float pOff = constrain(pitch, -15, 15);
    u8g2_R.drawLine(32 - cos(rad)*26 + sin(rad)*pOff, 32 - sin(rad)*26 - cos(rad)*pOff, 
                    32 + cos(rad)*26 + sin(rad)*pOff, 32 + sin(rad)*26 - cos(rad)*pOff);
    drawNeedle(u8g2_R, rightPanel[0], pitch * 50);
    u8g2_R.sendBuffer();

    u8g2_P1.firstPage(); do { drawNeedle(u8g2_P1, {64, 32, 26, 12, 17, "VOLTS"}, vBat); } while (u8g2_P1.nextPage());
    u8g2_P2.firstPage(); do { drawNeedle(u8g2_P2, {64, 32, 26, 0, 2, "G-LOAD"}, currentG); } while (u8g2_P2.nextPage());

    u8g2_S.firstPage(); do {
      u8g2_S.setFont(u8g2_font_04b_03_tr);
      u8g2_S.drawTriangle(36, 5, 33, 0, 39, 0); // Lubber line
      for (int i = -4; i <= 4; i++) {
        int xP = 36 + (i * 20) - ((int)heading % 10 * 2);
        if (xP > 0 && xP < 72) u8g2_S.drawLine(xP, 8, xP, 12);
      }
      u8g2_S.setCursor(28, 36); u8g2_S.print((int)heading);
    } while (u8g2_S.nextPage());
  }
}