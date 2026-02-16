/**
 * DevRelay.h
 * Class สำหรับจัดการการทำงานของ Relay
 * รองรับทั้งแบบ Active HIGH และ Active LOW
 */

#ifndef DEV_RELAY_H
#define DEV_RELAY_H

#include <Arduino.h>

class DevRelay {
private:
  uint8_t pin;          // GPIO ที่ต่อ Relay
  bool activeHigh;      // true = Active HIGH, false = Active LOW
  bool currentState;    // สถานะปัจจุบัน (true = ON, false = OFF)

public:
  /**
   * Constructor
   * @param relayPin GPIO ที่ต่อกับ Relay
   * @param isActiveHigh true = Active HIGH (ส่ง HIGH เปิด), false = Active LOW (ส่ง LOW เปิด)
   */
  DevRelay(uint8_t relayPin, bool isActiveHigh = true) {
    pin = relayPin;
    activeHigh = isActiveHigh;
    currentState = false; // เริ่มต้นปิด
  }

  /**
   * เริ่มต้นการทำงาน (เรียกใน setup)
   */
  void begin() {
    pinMode(pin, OUTPUT);
    off(); // เริ่มต้นปิด Relay
  }

  /**
   * เปิด Relay
   */
  void on() {
    currentState = true;
    if (activeHigh) {
      digitalWrite(pin, HIGH);
    } else {
      digitalWrite(pin, LOW);
    }
  }

  /**
   * ปิด Relay
   */
  void off() {
    currentState = false;
    if (activeHigh) {
      digitalWrite(pin, LOW);
    } else {
      digitalWrite(pin, HIGH);
    }
  }

  /**
   * สลับสถานะ Relay (ON <-> OFF)
   */
  void toggle() {
    if (currentState) {
      off();
    } else {
      on();
    }
  }

  /**
   * ตั้งค่าสถานะโดยตรง
   * @param state true = เปิด, false = ปิด
   */
  void set(bool state) {
    if (state) {
      on();
    } else {
      off();
    }
  }

  /**
   * เช็คสถานะปัจจุบัน
   * @return true = กำลังเปิด, false = กำลังปิด
   */
  bool isOn() {
    return currentState;
  }

  /**
   * เช็คสถานะปัจจุบัน
   * @return true = กำลังปิด, false = กำลังเปิด
   */
  bool isOff() {
    return !currentState;
  }

  /**
   * อ่านค่า GPIO ตรงๆ (สำหรับ Debug)
   * @return HIGH หรือ LOW
   */
  int readPin() {
    return digitalRead(pin);
  }
};

#endif // DEV_RELAY_H
