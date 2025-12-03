/*
  RS485_SHT20

  Tes baca sensor SHT20 melalui modul RS485.

  Rangkaian:
  - RX terhubung ke pin 18
  - TX terhubung ke pin 19
  - RTS terhubung ke pin 14

  https://www.khurslabs.com
*/

const byte rxPin = 16; // RX2
const byte txPin = 15; // TX2
const byte rtsPin = 14; // RTS pin
HardwareSerial modul(1);

#include <ModbusMaster.h>

ModbusMaster node;
uint8_t result1;
float temp, humi;
#define READING_DELAY 1000
long previousMillis;

void preTransmission() {
  digitalWrite(rtsPin, HIGH);
}

void postTransmission() {
  digitalWrite(rtsPin, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(rtsPin, OUTPUT);
  digitalWrite(rtsPin, LOW); // RTS ke LOW secara default untuk Mode RX

  modul.begin(9600, SERIAL_8N1, rxPin, txPin);
  node.begin(1, modul);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  if (millis() - previousMillis > READING_DELAY) {
    sendData();
    previousMillis = millis();
  }
}

void sendData() {
  result1 = node.readInputRegisters(0x001, 2);
  if (result1 == node.ku8MBSuccess) {
    temp = (node.getResponseBuffer(0x000) / 10.0f ); //temp
    humi = (node.getResponseBuffer(0x001) / 10.0f ); //humi
  }

  Serial.print("temp = ");
  Serial.print(temp);
  Serial.print(" , humi = ");
  Serial.println(humi);
}
