#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ===== PIN LCD (sesuai program Anda) =====
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_SCK  12

#define RED 2
#define GREEN 3
#define BLUE 4

#define ldr 1
int nilai;

// Inisialisasi TFT
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ===== COLOR =====
#define TFT_BLACK   0x0000
#define TFT_WHITE   0xFFFF
#define TFT_RED     0xF800
#define TFT_GREEN   0x07E0
#define TFT_BLUE    0x001F
#define TFT_YELLOW  0xFFE0

void setup() {

  Serial.begin(115200);

  pinMode(ldr, INPUT);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // start SPI
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI);

  // init LCD
  tft.begin();

  // rotation landscape
  tft.setRotation(1);

  // clear screen
  tft.fillScreen(TFT_BLACK);

  // ===== TITLE =====
  tft.setTextSize(4);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(60,40);
  tft.println("Aerasea");

  // ===== HELLO =====
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(70,120);
  tft.println("Hello World");

  // ===== FOOTER =====
  tft.setTextSize(2);
  tft.setCursor(90,180);
  tft.println("ESP32-S3 TFT Test");

  delay(3000);
}

void loop() {

  nilai = analogRead(ldr);
  Serial.println(nilai);

  // tes warna layar
  tft.fillScreen(TFT_RED);
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  delay(1000);

  tft.fillScreen(TFT_GREEN);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
  delay(1000);

  tft.fillScreen(TFT_BLUE);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH);
  delay(1000);

  tft.fillScreen(TFT_BLACK);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  delay(1000);

}

// #include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ILI9341.h>

// // ===== PIN LCD =====
// #define TFT_CS   10
// #define TFT_DC   9
// #define TFT_RST  8
// #define TFT_MOSI 11
// #define TFT_MISO 13
// #define TFT_SCK  12

// Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// // ===== COLOR =====
// #define TFT_BLACK   0x0000
// #define TFT_WHITE   0xFFFF
// #define TFT_YELLOW  0xFFE0
// #define TFT_GREEN   0x07E0

// // fungsi untuk center text
// void drawCenterText(String text, int y, int size, uint16_t color) {

//   int16_t x1, y1;
//   uint16_t w, h;

//   tft.setTextSize(size);
//   tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

//   int x = (320 - w) / 2;   // 320 = lebar layar landscape

//   tft.setCursor(x, y);
//   tft.setTextColor(color);
//   tft.println(text);
// }

// void setup() {

//   Serial.begin(115200);

//   SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI);

//   tft.begin();
//   tft.setRotation(1); // landscape
//   tft.fillScreen(TFT_BLACK);

//   drawCenterText("mangat kau biji", 70, 3, TFT_YELLOW);
//   drawCenterText("dya malay eek kau", 120, 3, TFT_WHITE);
//   drawCenterText("tukang berak", 170, 3, TFT_GREEN);

// }

// void loop() {
// }