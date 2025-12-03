/*
  ADS1115_Turbidity

  Tes baca nilai Turbidity air dengan sensor Turbidity melalui modul ADS1115.

  Rangkaian:
  - Sensor Turbidity terhubung ke pin A0 ADS1115
  - ADS1115 terhubung ke ESP32 dengan komunikasi I2C (SDA ke pin 8 dan SCL ke pin 9)

  https://www.khurslabs.com
*/

#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
int offset = 0;

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void loop(void)
{
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

  adc0 = ads.readADC_SingleEnded(0);
  // adc1 = ads.readADC_SingleEnded(1);
  // adc2 = ads.readADC_SingleEnded(2);
  // adc3 = ads.readADC_SingleEnded(3);

  volts0 = ads.computeVolts(adc0);
  // volts1 = ads.computeVolts(adc1);
  // volts2 = ads.computeVolts(adc2);
  // volts3 = ads.computeVolts(adc3);

  float ntu = ((-1120.4*(volts0*volts0))+(5742.3*volts0)-4352.9)-offset;
  // int ntu = map(volts0, 0.03, 4.3, 3000.0, 0.0);
  if(ntu<0){ntu=0;}
  if(volts0<2.5){ntu=3000.0;}
  Serial.print("analog=");
  Serial.print(adc0);
  Serial.print(" volt=");
  Serial.print(volts0);
  Serial.print(" ntu=");
  Serial.println(ntu); // print out the value you read:
  Serial.println();

  Serial.println("-----------------------------------------------------------");
  // Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  // Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
  // Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  // Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");

  delay(1000);
}
