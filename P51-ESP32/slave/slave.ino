#include <Arduino.h>
#include <U8g2lib.h>

// Internal Screen Pins
const int BUILT_IN_OLED_SCL = 6;
const int BUILT_IN_OLED_SDA = 5;     

// External Screen Pins (CONFIRMED WORKING)
const int EXT_OLED_SCL = 10;
const int EXT_OLED_SDA = 7;     

// Constructors
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2e(U8G2_R0, EXT_OLED_SCL, EXT_OLED_SDA, U8X8_PIN_NONE);
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2bi(U8G2_R0, U8X8_PIN_NONE, BUILT_IN_OLED_SCL, BUILT_IN_OLED_SDA);

void setup() {
  Serial.begin(115200);
  
  // Initialize Internal Screen
  u8g2bi.begin();
  u8g2bi.setFont(u8g2_font_04b_03_tr);
  
  // Initialize External Screen (using the working GraphicsTest logic)
  u8g2e.begin();
  u8g2e.setFont(u8g2_font_6x12_tf);

  // Serial1 for Master Communication (Pins 20/21)
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  
  Serial.println("Slave Setup Complete - Both Screens Ready.");
}

void loop() {
  // 1. Internal Screen Update
  u8g2bi.clearBuffer();
  u8g2bi.drawStr(0, 15, "SLAVE ACTIVE");
  u8g2bi.sendBuffer();

  // 2. Handle Data from Master
  if (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      // 3. External Screen Update
      u8g2e.clearBuffer();
      u8g2e.setDrawColor(1);
      u8g2e.setFont(u8g2_font_ncenB14_tr);
      
      // Print the data from Master
      u8g2e.setCursor(0, 40);
      u8g2e.print(data); 
      
      // Draw a border to keep it professional
      u8g2e.drawFrame(0, 0, 128, 64);
      u8g2e.sendBuffer();
      
      // Also mirror to Serial Monitor for debugging
      Serial.print("Received from Master: ");
      Serial.println(data);
    }
  }
}