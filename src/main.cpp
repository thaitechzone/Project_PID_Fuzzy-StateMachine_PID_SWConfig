/**
 * ESP32 PID Temperature Controller (Professional Edition)
 * ---------------------------------------------------
 * รายละเอียด Hardware:
 * - MCU: ESP32 DevKit V2 Board (THAITECHZONE)
 * - Sensor: DS18B20 (ต่อที่ GPIO 14)
 * - Heater: MOSFET หรือ SSR (ต่อที่ GPIO 13) -> ควบคุมด้วย PWM
 * - Display: OLED 0.96" (I2C: SDA=21, SCL=22)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <DevSW.h>
#include <DevRelay.h>
#include <Preferences.h>

// =========================================
// 1. การกำหนดขาพอร์ต (Pin Definitions)
// =========================================
#define HEATER_PIN 13    // ขาจ่ายสัญญาณ PWM ไปยัง Heater Driver
#define ONE_WIRE_BUS 14  // ขา Data ของ Sensor DS18B20

// Manual Control IO (สำรองไว้ใช้ในอนาคต)
#define SW1_PIN 34       // สวิตช์ 1 (Input Only - ต้องต่อ R Pull-up 10k)
#define SW2_PIN 35       // สวิตช์ 2 (Input Only - ต้องต่อ R Pull-up 10k)
#define SW3_PIN 32       // สวิตช์ 3 (Input Only - ต้องต่อ R Pull-up 10k)
#define RL1_PIN 17       // รีเลย์ 1
#define RL2_PIN 16       // รีเลย์ 2
#define RL3_PIN 4        // รีเลย์ 3

// Isolated Inputs (สำรองไว้ใช้ในอนาคต)
#define ISOIN1_PIN 33
#define ISOIN2_PIN 27

// =========================================
// 2. การตั้งค่าระบบ (System Settings)
// =========================================
// ขอบเขตอุณหภูมิและความปลอดภัย
#define TEMP_MIN 25.0    // ค่าต่ำสุดที่ยอมให้ตั้ง
#define TEMP_MAX 100.0    // ค่าสูงสุด และจุดตัด Safety Cutoff

// การตั้งค่า PWM (สำหรับ ESP32)
const int PWM_FREQ = 1000;     // ความถี่ 1kHz (เหมาะกับ MOSFET)
const int PWM_CHANNEL = 0;     // ช่องสัญญาณ PWM 0
const int PWM_RESOLUTION = 8;  // ความละเอียด 8-bit (ค่า 0-255)

// =========================================
// 3. ประกาศตัวแปรและ Object
// =========================================
// ตั้งค่าจอ OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ตั้งค่า Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Preferences สำหรับบันทึกการตั้งค่า
Preferences preferences;

// ตัวแปร PID
double Setpoint, Input, Output;
// ค่า Tuning (ปรับจูนตามความเหมาะสมของระบบจริง)
double Kp = 5.0, Ki = 0.5, Kd = 1.0; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// สร้าง Object สำหรับสวิตช์ทั้ง 3 ตัว
// SW1 = Mode/Enter, SW2 = Down, SW3 = Up
DevSW Mode(SW1_PIN);  // Mode/Enter
DevSW Down(SW2_PIN);  // Down
DevSW Up(SW3_PIN);    // Up

// สร้าง Object สำหรับ Relay ทั้ง 3 ตัว
// Active LOW (false) เนื่องจาก Relay Module ส่วนใหญ่ใช้ LOW เปิด
DevRelay RL1(RL1_PIN, false);  // รีเลย์ 1
DevRelay RL2(RL2_PIN, false);  // รีเลย์ 2
DevRelay RL3(RL3_PIN, false);  // รีเลย์ 3

// =========================================
// 4. State Machine Definitions
// =========================================
enum SystemState {
  STATE_PROCESSING,      // PID Control (Normal Operation)
  STATE_SETPOINT_CONFIG, // Configure Setpoint
  STATE_PID_CONFIG,      // Configure PID Parameters (Kp, Ki, Kd)
  STATE_MANUAL_PWM,      // Manual PWM Test Mode
  STATE_MANUAL_RELAY     // Manual Relay Control Mode
};

enum PIDConfigParam {
  CONF_KP,  // กำลังปรับ Kp
  CONF_KI,  // กำลังปรับ Ki
  CONF_KD   // กำลังปรับ Kd
};

enum RelaySelect {
  SEL_RL1,  // เลือก Relay 1
  SEL_RL2,  // เลือก Relay 2
  SEL_RL3   // เลือก Relay 3
};

// ตัวแปร State Machine
SystemState currentState = STATE_PROCESSING;
PIDConfigParam pidConfigParam = CONF_KP;

// ตัวแปรสำหรับ Manual PWM Mode
int manualPWM = 0;

// ตัวแปรสำหรับ Manual Relay Mode
RelaySelect selectedRelay = SEL_RL1;

// ตัวแปรจับเวลา
unsigned long lastDisplayTime = 0;

// ประกาศชื่อฟังก์ชันล่วงหน้า (Function Prototypes)
void updateDisplay();
void displayError(String title, String msg);
void debugSerial();

// Settings Management
void saveSettings();
void loadSettings();

// State Machine Functions
void handleButtonPress();
void processState();
void stateProcessing();
void stateSetpointConfig();
void statePIDConfig();
void stateManualPWM();
void stateManualRelay();

// Display Functions
void displayStateProcessing();
void displayStateSetpoint();
void displayStatePIDConfig();
void displayStateManualPWM();
void displayStateManualRelay();

void setup() {
  Serial.begin(115200);
  
  // --- A. ตั้งค่า PWM สำหรับ Heater ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // เริ่มต้นปิด Heater

  // --- B. เริ่มต้น Sensor ---
  sensors.begin();

  // --- C. เริ่มต้นจอ OLED ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // ถ้าจอเสีย ให้หยุดทำงานตรงนี้
  }
  
  // แสดง Logo เริ่มต้น
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.println(F("SYSTEM STARTING..."));
  display.setCursor(10, 40);
  display.println(F("PWM PID CONTROL"));
  display.display();
  delay(1500);

  // --- D. เริ่มต้นสวิตช์ ---
  Mode.begin();
  Down.begin();
  Up.begin();

  // --- E. เริ่มต้น Relay (ต้องทำก่อน loadSettings) ---
  RL1.begin();
  RL2.begin();
  RL3.begin();

  // --- F. โหลดการตั้งค่าจาก Memory ---
  loadSettings();
  
  // กำหนดขอบเขต Output ให้ตรงกับ PWM (0-255)
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);

  Serial.println(F("--- ESP32 PID Ready ---"));
  Serial.println(F("Settings loaded from memory"));
  Serial.print(F("Setpoint: ")); Serial.println(Setpoint);
  Serial.print(F("Kp: ")); Serial.print(Kp);
  Serial.print(F(", Ki: ")); Serial.print(Ki);
  Serial.print(F(", Kd: ")); Serial.println(Kd);
}

// =========================================
// Main Loop
// =========================================
void loop() {
  // อัปเดทสถานะสวิตช์ทั้งหมด
  Mode.update();
  Down.update();
  Up.update();

  // จัดการปุ่มกด
  handleButtonPress();

  // ประมวลผลตาม State ปัจจุบัน
  processState();

  // แสดงผล (ทุกๆ 200ms)
  if (millis() - lastDisplayTime > 200) {
    updateDisplay();
    debugSerial();
    lastDisplayTime = millis();
  }
}

// =========================================
// State Machine Functions
// =========================================

/**
 * จัดการปุ่มกดตาม State ปัจจุบัน
 */
void handleButtonPress() {
  // SW1 = Mode/Enter - จะจัดการใน state functions แต่ละตัว
  // (ใช้ long press 3s ทุก state)

  // SW3 = Up - เพิ่มค่า
  if (Up.onPressed()) {
    switch (currentState) {
      case STATE_SETPOINT_CONFIG:
        Setpoint += 1.0;
        if (Setpoint > TEMP_MAX) Setpoint = TEMP_MAX;
        Serial.print(F("Setpoint UP: ")); Serial.println(Setpoint);
        break;
        
      case STATE_PID_CONFIG:
        if (pidConfigParam == CONF_KP) {
          Kp += 0.5;
          if (Kp > 50.0) Kp = 50.0;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print(F("Kp UP: ")); Serial.println(Kp);
        } else if (pidConfigParam == CONF_KI) {
          Ki += 0.1;
          if (Ki > 10.0) Ki = 10.0;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print(F("Ki UP: ")); Serial.println(Ki);
        } else if (pidConfigParam == CONF_KD) {
          Kd += 0.1;
          if (Kd > 10.0) Kd = 10.0;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print(F("Kd UP: ")); Serial.println(Kd);
        }
        break;
        
      case STATE_MANUAL_PWM:
        manualPWM += 10;
        if (manualPWM > 255) manualPWM = 255;
        Serial.print(F("Manual PWM UP: ")); 
        Serial.print(manualPWM);
        Serial.print(F(" ("));
        Serial.print(map(manualPWM, 0, 255, 0, 100));
        Serial.println(F("%)"));
        break;
        
      case STATE_MANUAL_RELAY:
        // กด Up เพื่อเปิด Relay ที่เลือก
        if (selectedRelay == SEL_RL1) {
          RL1.on();
          Serial.println(F("RL1 ON"));
        } else if (selectedRelay == SEL_RL2) {
          RL2.on();
          Serial.println(F("RL2 ON"));
        } else if (selectedRelay == SEL_RL3) {
          RL3.on();
          Serial.println(F("RL3 ON"));
        }
        break;
        
      case STATE_PROCESSING:
        // ไม่ทำอะไรในโหมดปกติ
        break;
    }
  }

  // SW2 = Down - ลดค่า
  if (Down.onPressed()) {
    switch (currentState) {
      case STATE_SETPOINT_CONFIG:
        Setpoint -= 1.0;
        if (Setpoint < TEMP_MIN) Setpoint = TEMP_MIN;
        Serial.print(F("Setpoint DOWN: ")); Serial.println(Setpoint);
        break;
        
      case STATE_PID_CONFIG:
        if (pidConfigParam == CONF_KP) {
          Kp -= 0.5;
          if (Kp < 0.0) Kp = 0.0;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print(F("Kp DOWN: ")); Serial.println(Kp);
        } else if (pidConfigParam == CONF_KI) {
          Ki -= 0.1;
          if (Ki < 0.0) Ki = 0.0;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print(F("Ki DOWN: ")); Serial.println(Ki);
        } else if (pidConfigParam == CONF_KD) {
          Kd -= 0.1;
          if (Kd < 0.0) Kd = 0.0;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print(F("Kd DOWN: ")); Serial.println(Kd);
        }
        break;
        
      case STATE_MANUAL_PWM:
        manualPWM -= 10;
        if (manualPWM < 0) manualPWM = 0;
        Serial.print(F("Manual PWM DOWN: ")); 
        Serial.print(manualPWM);
        Serial.print(F(" ("));
        Serial.print(map(manualPWM, 0, 255, 0, 100));
        Serial.println(F("%)"));
        break;
        
      case STATE_MANUAL_RELAY:
        // กด Down เพื่อปิด Relay ที่เลือก
        if (selectedRelay == SEL_RL1) {
          RL1.off();
          Serial.println(F("RL1 OFF"));
        } else if (selectedRelay == SEL_RL2) {
          RL2.off();
          Serial.println(F("RL2 OFF"));
        } else if (selectedRelay == SEL_RL3) {
          RL3.off();
          Serial.println(F("RL3 OFF"));
        }
        break;
        
      case STATE_PROCESSING:
        // ไม่ทำอะไรในโหมดปกติ
        break;
    }
  }
}

/**
 * เรียกใช้ฟังก์ชันตาม State ปัจจุบัน
 */
void processState() {
  switch (currentState) {
    case STATE_PROCESSING:
      stateProcessing();
      break;
      
    case STATE_SETPOINT_CONFIG:
      stateSetpointConfig();
      break;
      
    case STATE_PID_CONFIG:
      statePIDConfig();
      break;
      
    case STATE_MANUAL_PWM:
      stateManualPWM();
      break;
      
    case STATE_MANUAL_RELAY:
      stateManualRelay();
      break;
  }
}

/**
 * STATE_PROCESSING - ทำงาน PID ควบคุมปกติ
 */
void stateProcessing() {
  static bool readyToChange = false; // พร้อมเปลี่ยน state หรือยัง
  
  // ตรวจสอบการกดค้าง 3 วินาที -> พร้อมเปลี่ยน
  if (Mode.pressedFor(3000)) {
    readyToChange = true;
  }
  
  // เมื่อปล่อยปุ่ม -> ตรวจสอบว่าพร้อมหรือไม่
  if (Mode.wasReleased()) {
    if (readyToChange) {
      // กดค้างครบ 3s แล้วปล่อย -> เปลี่ยน state
      currentState = STATE_SETPOINT_CONFIG;
      Serial.println(F("Mode held 3s + Released -> SETPOINT_CONFIG"));
      readyToChange = false;
      return;
    }
    // ปล่อยก่อน 3s -> รีเซ็ต
    readyToChange = false;
  }
  
  // อ่านค่าอุณหภูมิ
  sensors.requestTemperatures(); 
  double currentTemp = sensors.getTempCByIndex(0);

  // ตรวจสอบความปลอดภัย (Safety Checks)
  
  // กรณี 1: Sensor มีปัญหา (ค่า -127 หรือ 85)
  if (currentTemp == -127.00 || currentTemp == 85.00) {
    ledcWrite(PWM_CHANNEL, 0); // ตัด Heater ทันที
    displayError("SENSOR", "ERROR");
    return; 
  }
  
  Input = currentTemp;

  // กรณี 2: อุณหภูมิเกินกำหนด (Overheat)
  if (Input > TEMP_MAX) {
    ledcWrite(PWM_CHANNEL, 0); // ตัด Heater ทันที
    Output = 0;
    displayError("OVERHEAT", "> 100C");
    Serial.println(F("ALARM: Overheat detected!"));
    return;
  }

  // คำนวณ PID
  myPID.Compute();

  // ส่งค่าไปยัง Hardware (PWM Output)
  ledcWrite(PWM_CHANNEL, (int)Output);
}

/**
 * STATE_SETPOINT_CONFIG - ตั้งค่า Setpoint
 */
void stateSetpointConfig() {
  static bool readyToChange = false; // พร้อมเปลี่ยน state หรือยัง
  
  // ตรวจสอบการกดค้าง 3 วินาที -> พร้อมบันทึกและไปต่อ
  if (Mode.pressedFor(3000)) {
    readyToChange = true;
  }
  
  // เมื่อปล่อยปุ่ม -> ตรวจสอบว่าพร้อมหรือไม่
  if (Mode.wasReleased()) {
    if (readyToChange) {
      // กดค้างครบ 3s แล้วปล่อย -> บันทึกและเปลี่ยน state
      saveSettings();
      currentState = STATE_PID_CONFIG;
      pidConfigParam = CONF_KP; // เริ่มที่ Kp
      Serial.println(F("Mode held 3s + Released -> Setpoint Saved -> PID_CONFIG"));
      readyToChange = false;
    } else {
      // ปล่อยก่อน 3s -> รีเซ็ต
      readyToChange = false;
    }
  }
}

/**
 * STATE_PID_CONFIG - ตั้งค่า PID Parameters
 */
void statePIDConfig() {
  static bool readyToChange = false; // พร้อมเปลี่ยน state หรือยัง
  
  // ตรวจสอบการกดค้าง 3 วินาที -> พร้อมบันทึกและไปต่อ
  if (Mode.pressedFor(3000)) {
    readyToChange = true;
  }
  
  // เมื่อปล่อยปุ่ม
  if (Mode.wasReleased()) {
    unsigned long duration = Mode.pressedDuration();
    
    if (readyToChange && duration >= 3000) {
      // กดค้างครบ 3s แล้วปล่อย -> บันทึกและเปลี่ยน state
      saveSettings();
      currentState = STATE_MANUAL_PWM;
      Serial.println(F("Mode held 3s + Released -> PID Saved -> MANUAL_PWM"));
      readyToChange = false;
    } else if (duration < 3000) {
      // กดสั้นกว่า 3 วินาที -> สลับพารามิเตอร์
      if (pidConfigParam == CONF_KP) {
        pidConfigParam = CONF_KI;
        Serial.println(F("  -> Adjust Ki"));
      } else if (pidConfigParam == CONF_KI) {
        pidConfigParam = CONF_KD;
        Serial.println(F("  -> Adjust Kd"));
      } else {  // CONF_KD
        pidConfigParam = CONF_KP;
        Serial.println(F("  -> Adjust Kp"));
      }
      readyToChange = false; // reset flag
    } else {
      // กดค้างแต่ไม่ครบ 3s แน่นอน -> reset flag
      readyToChange = false;
    }
  }
}

/**
 * STATE_MANUAL_PWM - ทดสอบ PWM แบบ Manual
 */
void stateManualPWM() {
  static bool readyToChange = false; // พร้อมเปลี่ยน state หรือยัง
  
  // ส่งค่า PWM แบบ Manual
  ledcWrite(PWM_CHANNEL, manualPWM);
  
  // ไม่อ่าน Sensor เพื่อไม่ให้ผลกระทบการกดปุ่ม
  // (requestTemperatures ใช้เวลา ~750ms ทำให้ตอบสนองช้า)
  
  // ตรวจสอบการกดค้าง 3 วินาที -> พร้อมเปลี่ยน
  if (Mode.pressedFor(3000)) {
    readyToChange = true;
  }
  
  // เมื่อปล่อยปุ่ม -> ตรวจสอบว่าพร้อมหรือไม่
  if (Mode.wasReleased()) {
    if (readyToChange) {
      // กดค้างครบ 3s แล้วปล่อย -> เปลี่ยน state
      currentState = STATE_MANUAL_RELAY;
      selectedRelay = SEL_RL1; // เริ่มที่ Relay 1
      Serial.println(F("Mode held 3s + Released -> MANUAL_RELAY"));
      readyToChange = false;
    } else {
      // ปล่อยก่อน 3s -> รีเซ็ต
      readyToChange = false;
    }
  }
}

/**
 * STATE_MANUAL_RELAY - ควบคุม Relay แบบ Manual
 */
void stateManualRelay() {
  static bool readyToChange = false; // พร้อมเปลี่ยน state หรือยัง
  
  // ตรวจสอบการกดค้าง 3 วินาที -> พร้อมกลับไป PID
  if (Mode.pressedFor(3000)) {
    readyToChange = true;
  }
  
  // เมื่อปล่อยปุ่ม
  if (Mode.wasReleased()) {
    unsigned long duration = Mode.pressedDuration();
    
    if (readyToChange && duration >= 3000) {
      // กดค้างครบ 3s แล้วปล่อย -> กลับไป PID
      // บันทึก Relay State ก่อนออก
      saveSettings();
      
      // แสดงข้อความยืนยัน
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(10, 20);
      display.println(F("PID AUTO"));
      display.setCursor(20, 40);
      display.println(F("MODE"));
      display.display();
      delay(1000); // แสดง 1 วินาที
      
      currentState = STATE_PROCESSING;
      Serial.println(F("Mode held 3s + Released -> Relay Saved -> PROCESSING"));
      readyToChange = false;
    } else if (duration < 3000) {
      // กดสั้นกว่า 3 วินาที -> สลับ Relay
      if (selectedRelay == SEL_RL1) {
        selectedRelay = SEL_RL2;
        Serial.println(F("  -> Select RL2"));
      } else if (selectedRelay == SEL_RL2) {
        selectedRelay = SEL_RL3;
        Serial.println(F("  -> Select RL3"));
      } else {  // SEL_RL3
        selectedRelay = SEL_RL1;
        Serial.println(F("  -> Select RL1"));
      }
      readyToChange = false; // reset flag
    } else {
      // กรณีอื่นๆ -> reset flag
      readyToChange = false;
    }
  }
}

// =========================================
// Display Functions
// =========================================

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // แสดงผลตาม State
  switch (currentState) {
    case STATE_PROCESSING:
      displayStateProcessing();
      break;
      
    case STATE_SETPOINT_CONFIG:
      displayStateSetpoint();
      break;
      
    case STATE_PID_CONFIG:
      displayStatePIDConfig();
      break;
      
    case STATE_MANUAL_PWM:
      displayStateManualPWM();
      break;
      
    case STATE_MANUAL_RELAY:
      displayStateManualRelay();
      break;
  }

  display.display();
}

/**
 * แสดงผลโหมด STATE_PROCESSING
 */
void displayStateProcessing() {
  // ส่วนหัว
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  // แสดงข้อความเตือน + นับถอยหลัง
  if (Mode.pressedFor(3000)) {
     display.print(F(">> RELEASE NOW <<"));
  } else if (Mode.pressedFor(2000)) {
    display.print(F(">> Config [1] <<"));
  } else if (Mode.pressedFor(1000)) {
    display.print(F(">> Config [2] <<"));
  } else {
    display.print(F("PID AUTO MODE"));
  }

  // แสดง PV (ตัวใหญ่)
  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print(F("PV:"));
  display.print(Input, 1); 

  // แสดง Setpoint และ % กำลังไฟ
  display.setTextSize(1);
  display.setCursor(0, 36);
  display.print(F("Set:")); display.print(Setpoint, 0);
  display.print(F(" Pwr:")); 
  display.print(map(Output, 0, 255, 0, 100)); // แปลงเป็น %
  display.print(F("%"));

  // แสดงค่า PID Tuning
  display.setCursor(0, 48);
  display.print(F("P:")); display.print(Kp, 1);
  display.print(F(" I:")); display.print(Ki, 1);
  display.print(F(" D:")); display.print(Kd, 1);

  // กราฟแท่งแสดงกำลังไฟ Heater
  int barHeight = map(Output, 0, 255, 0, 64);
  display.drawRect(118, 0, 10, 64, WHITE);
  display.fillRect(118, 64 - barHeight, 10, barHeight, WHITE);
}

/**
 * แสดงผลโหมด STATE_SETPOINT_CONFIG
 */
void displayStateSetpoint() {
  display.setCursor(0, 0);
  display.print(F("CONFIG SETPOINT"));
  
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.print(F("UP/DOWN to adjust"));
  
  display.setCursor(0, 25);
  // แสดงข้อความเตือน + นับถอยหลัง
  if (Mode.pressedFor(3000)) {
    display.print(F(">> RELEASE NOW <<"));
  } else if (Mode.pressedFor(2000)) {
    display.print(F(">>>> Save [1] <<<<"));
  } else if (Mode.pressedFor(1000)) {
    display.print(F(">>>> Save [2] <<<<"));
  } else {
    display.print(F("Hold 3s to Save"));
  }

  // แสดงค่า Setpoint แบบใหญ่
  display.setTextSize(3);
  display.setCursor(10, 35);
  display.print(Setpoint, 1);
  display.print(F("C"));
}

/**
 * แสดงผลโหมด STATE_PID_CONFIG
 */
void displayStatePIDConfig() {
  display.setCursor(0, 0);
  display.print(F("CONFIG PID"));
  
  display.setTextSize(1);
  
  // แสดงข้อความเตือน + นับถอยหลัง
  if (Mode.pressedFor(3000)) {
    display.setCursor(0, 12);
    display.print(F(">> RELEASE NOW <<"));
  } else if (Mode.pressedFor(2000)) {
    display.setCursor(0, 12);
    display.print(F(">>>> Save [1] <<<<"));
  } else if (Mode.pressedFor(1000)) {
    display.setCursor(0, 12);
    display.print(F(">>>> Save [2] <<<<"));
  } else {
    display.setCursor(0, 12);
    display.print(F("Hold 3s Save&Exit"));
  }
  
  // แสดงว่ากำลังปรับพารามิเตอร์ไหน
  if (pidConfigParam == CONF_KP) {
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(F("> Kp (Proportional)"));
    display.setTextSize(2);
    display.setCursor(10, 42);
    display.print(Kp, 1);
  } else if (pidConfigParam == CONF_KI) {
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(F("> Ki (Integral)"));
    display.setTextSize(2);
    display.setCursor(10, 42);
    display.print(Ki, 2);
  } else if (pidConfigParam == CONF_KD) {
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(F("> Kd (Derivative)"));
    display.setTextSize(2);
    display.setCursor(10, 42);
    display.print(Kd, 2);
  }
}

/**
 * แสดงผลโหมด STATE_MANUAL_PWM
 */
void displayStateManualPWM() {
  // หัวข้อ
  display.setCursor(0, 0);
  display.print(F("MANUAL PWM MODE"));
  
  // คำแนะนำ
  display.setTextSize(1);
  
  // แสดงข้อความเตือน + นับถอยหลัง
  if (Mode.pressedFor(3000)) {
    display.setCursor(0, 12);
    display.print(F(">> RELEASE NOW <<"));
  } else if (Mode.pressedFor(2000)) {
    display.setCursor(0, 12);
    display.print(F(">>> Next [1] <<<"));
  } else if (Mode.pressedFor(1000)) {
    display.setCursor(0, 12);
    display.print(F(">>> Next [2] <<<"));
  } else {
    display.setCursor(0, 12);
    display.print(F("Hold 3s Next"));
  }

  // แสดงค่า PWM (0-255)
  display.setTextSize(1);
  display.setCursor(0, 26);
  display.print(F("PWM Value:"));
  display.setTextSize(2);
  display.setCursor(10, 38);
  display.print(manualPWM);

  // แสดงเปอร์เซ็นต์ PWM
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("Power: "));
  int pwmPercent = map(manualPWM, 0, 255, 0, 100);
  display.print(pwmPercent);
  display.print(F("%"));

  // กราฟแท่งแสดง PWM
  int barHeight = map(manualPWM, 0, 255, 0, 64);
  display.drawRect(118, 0, 10, 64, WHITE);
  display.fillRect(118, 64 - barHeight, 10, barHeight, WHITE);
}

/**
 * แสดงผลโหมด STATE_MANUAL_RELAY
 */
void displayStateManualRelay() {
  // หัวข้อ
  display.setCursor(0, 0);
  display.print(F("MANUAL RELAY MODE"));

  // คำแนะนำ
  display.setTextSize(1);
  
  // แสดงข้อความเตือน + นับถอยหลัง
  if (Mode.pressedFor(3000)) {
    display.setCursor(0, 12);
    display.print(F(">> RELEASE NOW <<"));
  } else if (Mode.pressedFor(2000)) {
    display.setCursor(0, 12);
    display.print(F(">>> Exit [1] <<<"));
  } else if (Mode.pressedFor(1000)) {
    display.setCursor(0, 12);
    display.print(F(">>> Exit [2] <<<"));
  } else {
    display.setCursor(0, 12);
    display.print(F("Hold 3s Exit"));
  }

  // แสดง Relay ที่เลือก
  display.setTextSize(2);
  display.setCursor(0, 26);  // จัดกลางจอ
  if (selectedRelay == SEL_RL1) {
    display.print(F(">>RELAY1<<"));
  } else if (selectedRelay == SEL_RL2) {
    display.print(F(">>RELAY2<<"));
  } else if (selectedRelay == SEL_RL3) {
    display.print(F(">>RELAY3<<"));
  }

  // แสดงสถานะ Relay ทั้ง 3 ตัว
  display.setTextSize(1);
  display.setCursor(0, 48);  // จัดกลาง
  display.print(F("R1:"));
  display.print(RL1.isOn() ? F("ON ") : F("OFF"));
  
  display.print(F(" R2:"));
  display.print(RL2.isOn() ? F("ON ") : F("OFF"));
  
  display.print(F(" R3:"));
  display.print(RL3.isOn() ? F("ON") : F("OFF"));
}

void displayError(String title, String msg) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(5, 10);
  display.println(title);
  
  display.setTextSize(2);
  display.setCursor(5, 35);
  display.print(F("!! "));
  display.print(msg);
  
  display.display();
}

void debugSerial() {
  // รูปแบบข้อมูลสำหรับ Serial Plotter และ Monitor
  Serial.print(F("State:"));
  
  switch (currentState) {
    case STATE_PROCESSING: Serial.print(F("PROC")); break;
    case STATE_SETPOINT_CONFIG: Serial.print(F("SP_CFG")); break;
    case STATE_PID_CONFIG: Serial.print(F("PID_CFG")); break;
    case STATE_MANUAL_PWM: Serial.print(F("MANUAL")); break;
    case STATE_MANUAL_RELAY: Serial.print(F("RELAY")); break;
  }
  
  Serial.print(F(", Set:")); Serial.print(Setpoint);
  Serial.print(F(", PV:")); Serial.print(Input);
  
  if (currentState == STATE_PROCESSING) {
    Serial.print(F(", Out:")); Serial.print(Output);
    Serial.print(F(", PWM%:")); Serial.print(map(Output, 0, 255, 0, 100));
  } else if (currentState == STATE_MANUAL_PWM) {
    Serial.print(F(", ManPWM:")); Serial.print(manualPWM);
  } else if (currentState == STATE_MANUAL_RELAY) {
    Serial.print(F(", RL1:")); Serial.print(RL1.isOn() ? F("ON") : F("OFF"));
    Serial.print(F(", RL2:")); Serial.print(RL2.isOn() ? F("ON") : F("OFF"));
    Serial.print(F(", RL3:")); Serial.print(RL3.isOn() ? F("ON") : F("OFF"));
  }
  
  Serial.println();
}

// =========================================
// Settings Management Functions
// =========================================

/**
 * บันทึกการตั้งค่าลง Flash Memory
 */
void saveSettings() {
  preferences.begin("pid-ctrl", false); // เปิด namespace "pid-ctrl" (Read/Write)
  
  preferences.putFloat("setpoint", (float)Setpoint);
  preferences.putFloat("kp", (float)Kp);
  preferences.putFloat("ki", (float)Ki);
  preferences.putFloat("kd", (float)Kd);
  
  // บันทึก Manual Relay State
  preferences.putUChar("selRelay", (uint8_t)selectedRelay);
  preferences.putBool("rl1State", RL1.isOn());
  preferences.putBool("rl2State", RL2.isOn());
  preferences.putBool("rl3State", RL3.isOn());
  
  preferences.end();
  
  Serial.println(F(">>> Settings saved to Flash Memory <<<"));
}

/**
 * โหลดการตั้งค่าจาก Flash Memory
 */
void loadSettings() {
  preferences.begin("pid-ctrl", true); // เปิด namespace "pid-ctrl" (Read-Only)
  
  // โหลดค่า หรือใช้ค่า default ถ้าไม่มีข้อมูล
  Setpoint = preferences.getFloat("setpoint", 40.0);  // default 40°C
  Kp = preferences.getFloat("kp", 5.0);               // default 5.0
  Ki = preferences.getFloat("ki", 0.5);               // default 0.5
  Kd = preferences.getFloat("kd", 1.0);               // default 1.0
  
  // โหลด Manual Relay State
  uint8_t savedRelay = preferences.getUChar("selRelay", 0); // default SEL_RL1
  selectedRelay = (RelaySelect)savedRelay;
  bool rl1State = preferences.getBool("rl1State", false);
  bool rl2State = preferences.getBool("rl2State", false);
  bool rl3State = preferences.getBool("rl3State", false);
  
  preferences.end();
  
  // จำกัดค่าให้อยู่ในช่วงที่ถูกต้อง
  if (Setpoint < TEMP_MIN) Setpoint = TEMP_MIN;
  if (Setpoint > TEMP_MAX) Setpoint = TEMP_MAX;
  if (Kp < 0.0) Kp = 0.0;
  if (Kp > 50.0) Kp = 50.0;
  if (Ki < 0.0) Ki = 0.0;
  if (Ki > 10.0) Ki = 10.0;
  if (Kd < 0.0) Kd = 0.0;
  if (Kd > 10.0) Kd = 10.0;
  
  // คืนค่าสถานะ Relay (ต้องทำหลัง begin())
  if (rl1State) RL1.on(); else RL1.off();
  if (rl2State) RL2.on(); else RL2.off();
  if (rl3State) RL3.on(); else RL3.off();
  
  Serial.println(F(">>> Settings loaded from Flash Memory <<<"));
  Serial.print(F("Selected Relay: ")); Serial.println(selectedRelay + 1);
  Serial.print(F("RL1: ")); Serial.print(RL1.isOn() ? F("ON") : F("OFF"));
  Serial.print(F(", RL2: ")); Serial.print(RL2.isOn() ? F("ON") : F("OFF"));
  Serial.print(F(", RL3: ")); Serial.println(RL3.isOn() ? F("ON") : F("OFF"));
}