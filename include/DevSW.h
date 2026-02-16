/**
 * DevSW.h - Device Switch Class
 * สำหรับจัดการสวิตช์พร้อม Debouncing
 * รองรับ ESP32 Input-Only Pins (34, 35, 32)
 * ใช้กับสวิตช์แบบ Pull-up (กด = LOW, ปล่อย = HIGH)
 */

#ifndef DEVSW_H
#define DEVSW_H

#include <Arduino.h>

class DevSW {
private:
    uint8_t pin;                    // Pin number
    bool currentState;              // สถานะปัจจุบัน (true = กด, false = ปล่อย)
    bool lastState;                 // สถานะครั้งก่อน
    bool lastReadState;             // สถานะที่อ่านได้ล่าสุด
    unsigned long lastDebounceTime; // เวลาที่เปลี่ยนสถานะล่าสุด
    unsigned long debounceDelay;    // ระยะเวลา debounce (ms)
    unsigned long pressedStartTime; // เวลาที่เริ่มกด (สำหรับกดค้าง)
    unsigned long lastPressedDuration; // ระยะเวลาที่กดค้างครั้งล่าสุด
    
    bool pressedFlag;               // Flag บอกว่าเพิ่งกดใหม่
    bool releasedFlag;              // Flag บอกว่าเพิ่งปล่อย

public:
    /**
     * Constructor
     * @param pin หมายเลข GPIO pin
     * @param debounce_ms ระยะเวลา debounce (default 30ms)
     */
    DevSW(uint8_t pin, unsigned long debounce_ms = 30) {
        this->pin = pin;
        this->debounceDelay = debounce_ms;
        this->currentState = false;
        this->lastState = false;
        this->lastReadState = true; // Pull-up = HIGH เมื่อไม่กด
        this->lastDebounceTime = 0;
        this->pressedStartTime = 0;
        this->lastPressedDuration = 0;
        this->pressedFlag = false;
        this->releasedFlag = false;
    }

    /**
     * เริ่มต้นการใช้งาน (เรียกใน setup)
     */
    void begin() {
        pinMode(pin, INPUT); // Input-only pins ไม่ต้องใส่ PULLUP
        lastReadState = digitalRead(pin);
        currentState = !lastReadState; // กลับด้าน เพราะ pull-up
        lastState = currentState;
    }

    /**
     * อัปเดทสถานะสวิตช์ (เรียกใน loop ทุกรอบ)
     */
    void update() {
        // อ่านสถานะปัจจุบัน
        bool reading = digitalRead(pin);

        // ถ้าสถานะเปลี่ยน รีเซ็ตตัวจับเวลา debounce
        if (reading != lastReadState) {
            lastDebounceTime = millis();
            lastReadState = reading;
        }

        // ถ้าผ่านเวลา debounce แล้ว และสถานะคงที่
        if ((millis() - lastDebounceTime) >= debounceDelay) {
            // แปลงสถานะ (pull-up: LOW = กด)
            bool newState = (reading == LOW);
            
            // ถ้าสถานะเปลี่ยนจริงๆ
            if (newState != currentState) {
                currentState = newState;
                
                // ตั้ง flag
                if (currentState) {
                    pressedFlag = true;      // เพิ่งกด
                    pressedStartTime = millis(); // เริ่มจับเวลา
                    lastPressedDuration = 0; // เคลียร์ค่าเก่า
                } else {
                    releasedFlag = true;     // เพิ่งปล่อย
                    // เก็บระยะเวลาที่กดค้างไว้ก่อนรีเซ็ต
                    if (pressedStartTime > 0) {
                        lastPressedDuration = millis() - pressedStartTime;
                    }
                    pressedStartTime = 0;    // รีเซ็ตเวลา
                }
            }
        }
    }

    /**
     * ตรวจสอบว่าสวิตช์ถูกกดอยู่หรือไม่
     * @return true = กดอยู่, false = ไม่ได้กด
     */
    bool isPressed() {
        return currentState;
    }

    /**
     * ตรวจสอบว่าเพิ่งกดใหม่หรือไม่ (Edge Detection)
     * @return true = เพิ่งกด (ครั้งเดียวต่อการกดหนึ่งครั้ง)
     */
    bool wasPressed() {
        if (pressedFlag) {
            pressedFlag = false; // Clear flag
            return true;
        }
        return false;
    }

    /**
     * ตรวจสอบว่าเพิ่งปล่อยหรือไม่
     * @return true = เพิ่งปล่อย
     */
    bool wasReleased() {
        if (releasedFlag) {
            releasedFlag = false; // Clear flag
            return true;
        }
        return false;
    }

    /**
     * ตรวจจับการกดสวิตช์ - เมื่อเพิ่งกดใหม่
     * เหมาะ wasPressed() สำหรับ Pull-up circuit
     * @return true = เพิ่งกดสวิตช์
     */
    bool onPressed() {
        return wasPressed();
    }

    /**
     * ตรวจจับขาขึ้น (Rising Edge) - เมื่อสัญญาณเปลี่ยนจาก LOW -> HIGH
     * เหมือน wasReleased() สำหรับ Pull-up circuit
     * @return true = ตรวจจับขาขึ้น (เพิ่งปล่อยสวิตช์)
     */
    bool onRisingEdge() {
        return wasReleased();
    }

    /**
     * คืนค่าระยะเวลาที่กดค้างไว้ (milliseconds)
     * @return เวลาที่กดค้าง (0 = ไม่ได้กด)
     */
    unsigned long pressedDuration() {
        // ถ้ากำลังกดอยู่
        if (currentState && pressedStartTime > 0) {
            return millis() - pressedStartTime;
        }
        // ถ้าเพิ่งปล่อย ให้คืนค่าที่เก็บไว้
        if (lastPressedDuration > 0) {
            return lastPressedDuration;
        }
        return 0;
    }

    /**
     * ตรวจสอบว่ากดค้างครบเวลาที่กำหนดหรือไม่
     * @param ms ระยะเวลาที่ต้องการ (milliseconds)
     * @return true = กดค้างครบเวลาแล้ว
     */
    bool pressedFor(unsigned long ms) {
        return (pressedDuration() >= ms);
    }

    /**
     * รีเซ็ต flags ทั้งหมด
     */
    void reset() {
        pressedFlag = false;
        releasedFlag = false;
    }

    /**
     * เปลี่ยนเวลา debounce
     * @param ms ระยะเวลาใหม่ (milliseconds)
     */
    void setDebounceDelay(unsigned long ms) {
        debounceDelay = ms;
    }

    /**
     * อ่านค่าดิบจาก GPIO (สำหรับ debug)
     * @return HIGH หรือ LOW
     */
    bool readRaw() {
        return digitalRead(pin);
    }

    /**
     * แสดงข้อมูล debug ผ่าน Serial
     */
    void debug(const char* name) {
        Serial.print(name);
        Serial.print(": Raw=");
        Serial.print(digitalRead(pin));
        Serial.print(" State=");
        Serial.print(currentState);
        Serial.print(" Pressed=");
        Serial.print(pressedFlag);
        Serial.print(" Released=");
        Serial.println(releasedFlag);
    }
};

#endif // DEVSW_H
