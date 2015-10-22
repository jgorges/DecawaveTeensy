#include "DW1000.h"
#include <SPI.h>
#include <Wire.h>

void setup() {
  Serial.begin(38400);
  delay(5000);
  Serial.println("Starting setup:");
   
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  
  
  DW1000 dw(11, 12, 13, 10, 9);
  delay(1000);
  uint32_t id = dw.getDeviceID();
  float voltage = dw.getVoltage();
  
  delay(5000);
  Serial.print("ID Number: ");
  Serial.println(id);
  Serial.print("Voltage: ");
  Serial.println(voltage);
  
}

void loop() {
//  DW1000 dw(11, 12, 13, 10, 9);
//  float voltage = dw.getVoltage();
//  Serial.print("Voltage: ");
//  Serial.println(voltage);
//  delay(2000);
}
