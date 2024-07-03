#include <Wire.h>

#define SLAVE_ADDR 0x42
String data;
int a;
bool verif = false;

void setup() {
  Wire.begin();  // Mulai komunikasi I2C
  Serial.begin(115200);
}

void loop() {
  // while (Serial.available() > 0) {
  //   char d = Serial.read();
  //   // data += d;
  //   if (d == 's') {
  //     a = Serial.parseInt();
  //   }
  // }
  int b = random(0, 10);
  int c = random(0, 10);
  // float d = random(0.0, 10.0);
  // float e = random(0.0, 10.0);
  // float f = random(0.0, 10.0);

  // String dat = String(a) + "#" + String(b) + "#" + String(c) + "#" + String(d) + "#" + String(e) + "#" + String(f);
  String dat = String(b) + "#" + String(c);
  // Serial.println(dat);
  // Perhatikan bahwa ukuran String yang ditransmisikan harus diambil menggunakan method `length()`
  int dataSize = dat.length();
  // Serial.println(dataSize);

  // Mengubah String menjadi array karakter (char array) untuk ditransmisikan melalui I2C
  char dataToSend[dataSize + 1];  // Ukuran array harus lebih besar dari jumlah karakter yang akan ditransmisikan, ditambah 1 untuk null terminator

  dat.toCharArray(dataToSend, dataSize + 1);  // Mengonversi String ke dalam char array
  Serial.println(dataToSend);
  // Mulai transmisi ke slave dengan alamat yang ditentukan
  Wire.beginTransmission(SLAVE_ADDR);

  // Mengirim data (array of characters) melalui I2C
  Wire.write((const uint8_t*)dataToSend, dataSize);

  // Akhiri transmisi
  Wire.endTransmission();

  delay(1000);
}
