/*
  Relay

  Tes menyalakan dan mematikan Relay.

  Rangkaian:
  - Coil relay terhubung ke pin 48

  https://www.khurslabs.com
*/

const int relayPin1 = 48;    // pin untuk relay
const int relayPin2 = 47;    // pin untuk relay

void setup() {
  // inisialisasi pin relay sebagai output:
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
}

void loop() {
  digitalWrite(relayPin1, HIGH);  // nyalakan relay1
  delay(1000);                    // delay 1 detik
  digitalWrite(relayPin1, LOW);   // matikan relay1
  delay(5000);                    // delay 5 detik
  digitalWrite(relayPin2, HIGH);  // nyalakan relay1
  delay(1000);                    // delay 1 detik
  digitalWrite(relayPin2, LOW);   // matikan relay1
  delay(5000);                    // delay 5 detik
}
