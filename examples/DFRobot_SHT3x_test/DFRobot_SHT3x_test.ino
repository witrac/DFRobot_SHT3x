/*!
 * @file  DFRobot_SHT3x_test.ino
 * @brief DFRobot's SHT3x Humidity And Temperature Sensor Module
 * @n     This example demonstrates how to read the user registers to display
 * resolution and other settings. Uses the SHT3x library to display the current
 * humidity and temperature. Open serial monitor at 9600 baud to see readings.
 *        Error 999 if CRC is bad.
 * Hardware Connections:
 * -VCC = 3.3V
 * -GND = GND
 * -SDA = A4 (use inline 330 ohm resistor if your board is 5V)
 * -SCL = A5 (use inline 330 ohm resistor if your board is 5V)
 */

#include "DFRobot_SHT3x.h"
#include <Wire.h>

DFRobot_SHT3x sht3x;

void setup() {
  Serial.begin(9600);
  Serial.println("SHT3x Example!");
  sht3x.init(); // Init SHT3x Sensor
  delay(100);
}

void loop() {
  float humd = sht3x.readHumidity();    // Read Humidity
  float temp = sht3x.readTemperature(); // Read Temperature
  Serial.print("Time:");
  Serial.print(millis());
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");
  Serial.println();
  delay(1000);
}
