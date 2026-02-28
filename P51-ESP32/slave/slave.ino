#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

auto& console = Serial;     // USB Port to your PC (Serial Monitor)
auto& fromMaster = Serial1;   // Hardware pins 20/21 receiving from Master

// --- PIN CONFIGURATION ---
const int BUILT_IN_OLED_SCL = 6;
const int BUILT_IN_OLED_SDA = 5;     
const int EXT_OLED_SCL = 10;
const int EXT_OLED_SDA = 7;     

// --- DISPLAY CONSTRUCTORS ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2e1(U8G2_R0, U8X8_PIN_NONE, EXT_OLED_SCL, EXT_OLED_SDA);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2e2(U8G2_R0, U8X8_PIN_NONE, EXT_OLED_SCL, EXT_OLED_SDA);
U8G2_SSD1306_72X40_ER_F_SW_I2C u8g2bi(U8G2_R0, U8X8_PIN_NONE, BUILT_IN_OLED_SCL, BUILT_IN_OLED_SDA);

enum GaugeType { P51_HORIZON, P51_ALTIMETER };

struct Instrument {
  const char* label;
  GaugeType type;
  float val1;      // Roll (Horizon) or Feet (Altimeter)
  float val2;      // Pitch (Horizon)
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

float roll = 0.0, pitch = 0.0, heading = 0.0, altitude = 0.0, vBat = 0.0, currentG = 0.0;
bool warActive = false;

// --- DRAWING FUNCTIONS ---

void drawHorizon(Instrument &inst) {
  U8G2* dev = inst.screen;
  float rollRad = inst.val1 * (PI / 180.0);
  // Pitch mapping: adjust -15, 15 to change how "sensitive" the bar is
  int pitchOff = map(constrain(inst.val2, -30, 30), -30, 30, -15, 15);

  // 1. BANK SCALE (Internal Ticks)
  // Drawn from -60 to +60 degrees
  for (int a = -60; a <= 60; a += 10) {
    float rad = (a - 90) * (PI / 180.0);
    int rOut = inst.r - 1; // Flush with your stencil edge
    int rIn = (a % 30 == 0) ? inst.r - 7 : inst.r - 4; // Long ticks at 30/60
    dev->drawLine(inst.x + rIn * cos(rad), inst.y + rIn * sin(rad),
                  inst.x + rOut * cos(rad), inst.y + rOut * sin(rad));
  }

  // 2. MOVING HORIZON BAR
  // Long enough to stay visible behind the circular mask during rolls
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  int xL = 38 * cosR;
  int yL = 38 * sinR;
  dev->drawLine(inst.x - xL, inst.y - yL + pitchOff, 
                inst.x + xL, inst.y + yL + pitchOff);

  // 3. THE "MUSHROOM" PEDESTAL (Bottom Mask)
  // Instead of a solid triangle, we use a narrower base to look like the real gauge
  dev->drawBox(inst.x - 2, inst.y + 10, 5, 22); // The "stem"
  dev->drawDisc(inst.x, inst.y + 10, 4, U8G2_DRAW_ALL); // The rounded top of the pedestal

  // 4. FIXED AIRCRAFT REFERENCE (The "W")
  // Drawing the yellow-tipped wings and center pointer
  int ay = inst.y + 8; // Vertical position of the reference
  dev->drawHLine(inst.x - 16, ay, 10); // Left wing
  dev->drawHLine(inst.x + 6, ay, 10);  // Right wing
  dev->drawVLine(inst.x, ay - 6, 4);   // The top center "needle"
  
  // Fixed "W" Aircraft Reference
  dev->drawHLine(inst.x - 15, inst.y + 5, 10);
  dev->drawHLine(inst.x + 5, inst.y + 5, 10);
  dev->drawFrame(inst.x - 2, inst.y + 3, 5, 5);
}

void drawAltimeter(Instrument &inst) {
  U8G2* dev = inst.screen;
  float alt = inst.val1;
  int cx = inst.x;
  int cy = inst.y;
  int r = inst.r;

  dev->drawCircle(cx, cy, r);
  
  // 1. Long Hand (100s of feet) - Full circle every 1000ft
  float angle1 = (fmod(alt, 1000.0) * 0.36) - 90.0;
  float rad1 = angle1 * (PI / 180.0);
  dev->drawLine(cx, cy, cx + (r-2)*cos(rad1), cy + (r-2)*sin(rad1));

  // 2. Short Wide Hand (1000s of feet) - Full circle every 10,000ft
  float angle2 = (fmod(alt, 10000.0) * 0.036) - 90.0;
  float rad2 = angle2 * (PI / 180.0);
  dev->drawLine(cx, cy, cx + (r-12)*cos(rad2), cy + (r-12)*sin(rad2));
  
  // 3. 10k Hand (Thin with end-marker) - Full circle every 100,000ft
  float angle3 = (fmod(alt, 100000.0) * 0.0036) - 90.0;
  float rad3 = angle3 * (PI / 180.0);
  dev->drawLine(cx, cy, cx + (r-8)*cos(rad3), cy + (r-8)*sin(rad3));
  dev->drawPixel(cx + (r-6)*cos(rad3), cy + (r-6)*sin(rad3)); // Tiny tip
}

void setup() {
  Wire.begin(EXT_OLED_SDA, EXT_OLED_SCL);
  Wire.setClock(400000);
  console.begin(115200);  
  fromMaster.begin(115200, SERIAL_8N1, 20, 21);

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

void readMasterData() {
  if (fromMaster.available() > 0) {
    String tag = fromMaster.readStringUntil(':'); 
    float value = fromMaster.parseFloat();
    
    // --- CONSOLE PRINTING
    console.print("TAG: "); console.print(tag);
    console.print(" | VAL: "); console.println(value);
    
    if (tag == "ROL") roll = value;
    else if (tag == "PIT") pitch = value;
    else if (tag == "HED") heading = value;
    else if (tag == "ALT") altitude = value;
    else if (tag == "BAT") vBat = value;
    else if (tag == "GFO") currentG = value;
    else if (tag == "WAR") warActive = (value > 0.5);
    
    // Clear trailing separators
    while (fromMaster.available() > 0 && (fromMaster.peek() == ',' || fromMaster.peek() == '\n')) {
      fromMaster.read();
    }
  }
}

void loop() {
  // 1. Listen for the tagged stream from Master
  if (fromMaster.available() > 0) {
    String tag = fromMaster.readStringUntil(':'); 
    float value = fromMaster.parseFloat();
    
    // 2. Map tagged values to the correct instrument variables
    if (tag == "ROL") cockpit.horizon.val1 = value;
    else if (tag == "PIT") cockpit.horizon.val2 = value;
    else if (tag == "ALT") cockpit.altimeter.val1 = value;
    
    // 3. Clean up the separator (comma or newline)
    while (fromMaster.available() > 0 && (fromMaster.peek() == ',' || fromMaster.peek() == '\n' || fromMaster.peek() == '\r')) {
      fromMaster.read();
    }
  }

  // 4. Refresh Displays (Mechanical style: constant sweep)
  u8g2e1.clearBuffer();
  drawHorizon(cockpit.horizon);
  u8g2e1.sendBuffer();

  u8g2e2.clearBuffer();
  drawAltimeter(cockpit.altimeter);
  u8g2e2.sendBuffer();
}