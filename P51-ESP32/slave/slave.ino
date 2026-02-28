#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- PIN CONFIGURATION ---
const int BUILT_IN_OLED_SCL = 6;
const int BUILT_IN_OLED_SDA = 5;     
const int EXT_OLED_SCL = 10;
const int EXT_OLED_SDA = 7;     

// --- DISPLAY CONSTRUCTORS ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2e1(U8G2_R0, U8X8_PIN_NONE, EXT_OLED_SCL, EXT_OLED_SDA);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2e2(U8G2_R0, U8X8_PIN_NONE, EXT_OLED_SCL, EXT_OLED_SDA);
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2bi(U8G2_R0, U8X8_PIN_NONE, BUILT_IN_OLED_SCL, BUILT_IN_OLED_SDA);

enum GaugeType { P51_HORIZON, P51_ALTIMETER };

struct Instrument {
  const char* label;
  GaugeType type;
  float val1;      
  float val2;      
  int x, y, r;     
  U8G2* screen;    
};

struct CockpitLayout {
  Instrument horizon;
  Instrument altimeter;
};

// Initialize with specific screen pointers
CockpitLayout cockpit = {
  {"HORIZON",   P51_HORIZON,   0.0, 0.0, 64, 32, 30, &u8g2e1},
  {"ALTIMETER", P51_ALTIMETER, 0.0, 0.0, 64, 32, 30, &u8g2e2}
};

// --- DRAWING FUNCTIONS ---

void drawHorizon(Instrument &inst) {
  U8G2* dev = inst.screen;
  float rollRad = inst.val1 * (PI / 180.0);
  
  // Pitch moves the bar up/down. 0 is center.
  int pitchOff = map(constrain(inst.val2, -30, 30), -30, 30, -18, 18);

  // 1. Internal Bank Markers (the white ticks at the top of the 'mushroom')
  // These are inside the perimeter.
  for (int a = -60; a <= 60; a += 10) {
    float rad = (a - 90) * (PI / 180.0);
    int innerR = inst.r - 2;
    int outerR = (a % 30 == 0) ? inst.r - 8 : inst.r - 5; // Longer ticks at 30/60
    dev->drawLine(
      inst.x + innerR * cos(rad), inst.y + innerR * sin(rad),
      inst.x + outerR * cos(rad), inst.y + outerR * sin(rad)
    );
  }

  // 2. The Moving Horizon Bar
  // Calculated to be wide enough to cover the window during rolls
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  int xLine = 35 * cosR;
  int yLine = 35 * sinR;
  
  // Draw the horizon line (moves with pitch and rotates with roll)
  dev->drawLine(inst.x - xLine, inst.y - yLine + pitchOff, 
                inst.x + xLine, inst.y + yLine + pitchOff);

  // 3. The Mushroom Window "Stem" / Lower Mask
  // Drawing the internal "shelf" seen in the AN 5736-1A
  dev->drawTriangle(inst.x - 12, inst.y + 30, inst.x + 12, inst.y + 30, inst.x, inst.y + 10);

  // 4. The Fixed "W" / Pointer (The Yellow Reference in the photo)
  // This is the "Aperture" pointer.
  int py = inst.y + 2; 
  dev->drawHLine(inst.x - 14, py, 8);  // Left "wing"
  dev->drawHLine(inst.x + 6, py, 8);   // Right "wing"
  dev->drawVLine(inst.x, py - 4, 4);   // The center "pointing" tip
  
  // 5. Sky/Ground Ticks (Optional: subtle center pitch dots)
  dev->drawPixel(inst.x, inst.y + pitchOff - 10);
  dev->drawPixel(inst.x, inst.y + pitchOff + 10);
}

void drawAltimeter(Instrument &inst) {
  U8G2* dev = inst.screen;
  dev->drawCircle(inst.x, inst.y, inst.r);
  
  // Hundreds Hand
  float angle1 = map((long)inst.val1 % 1000, 0, 1000, 0, 360) - 90;
  float rad1 = angle1 * (PI / 180.0);
  dev->drawLine(inst.x, inst.y, inst.x + (inst.r-5)*cos(rad1), inst.y + (inst.r-5)*sin(rad1));

  // Thousands Hand (Small Disc)
  float angle2 = map((long)inst.val1 % 10000, 0, 10000, 0, 360) - 90;
  float rad2 = angle2 * (PI / 180.0);
  dev->drawDisc(inst.x + (inst.r-15)*cos(rad2), inst.y + (inst.r-15)*sin(rad2), 3);
}

void setup() {
  Wire.begin(EXT_OLED_SDA, EXT_OLED_SCL);
  Wire.setClock(400000); // High speed I2C for smooth gauges
  Serial1.begin(115200, SERIAL_8N1, 20, 21);

  // Set addresses to match your hardware resistors
  u8g2e1.setI2CAddress(0x3C * 2); 
  u8g2e2.setI2CAddress(0x3D * 2);
  
  u8g2bi.begin();
  u8g2e1.begin();
  u8g2e2.begin();

  u8g2bi.setFont(u8g2_font_04b_03_tr);
  u8g2bi.clearBuffer();
  u8g2bi.drawStr(0, 15, "SYSTEMS DUAL-LINK");
  u8g2bi.sendBuffer();
}

void loop() {
  if (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n');
    data.trim();

    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);

    if (c1 != -1 && c2 != -1) {
      float altVal   = data.substring(0, c1).toFloat();
      float rollVal  = data.substring(c1 + 1, c2).toFloat();
      float pitchVal = data.substring(c2 + 1).toFloat();

      // Update data model
      cockpit.horizon.val1 = rollVal;
      cockpit.horizon.val2 = pitchVal;
      cockpit.altimeter.val1 = altVal;

      // Draw Buffer 1 (Horizon)
      u8g2e1.clearBuffer();
      drawHorizon(cockpit.horizon);
      u8g2e1.sendBuffer();

      // Draw Buffer 2 (Altimeter)
      u8g2e2.clearBuffer();
      drawAltimeter(cockpit.altimeter);
      u8g2e2.sendBuffer();
    }
  }
}