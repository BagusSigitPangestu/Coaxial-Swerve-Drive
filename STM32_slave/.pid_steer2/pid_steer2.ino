#include <Arduino.h>
//#include <TimerOne.h>

// Definisikan pin-pin yang digunakan
#define pin1 PA1
#define pin2 PA3
#define enAM1 PB9
#define enBM1 PB8

#define pin3 PA4
#define pin4 PA5
#define pwmB PA6
#define enAM2 PB7
#define enBM2 PB6

// Definisikan konstanta-konstanta untuk pengaturan PID
const float encoderResolution = 330 * 2.2285714;  // Resolusi encoder dalam pulsa per rotasi
//const float degreePerPulse = 360.0 / encoderResolution;  // Jarak per pulsa encoder dalam derajat

// Variabel untuk PID
float Kp = 0.8; // kp 0.91 untuk sp 360, 180 | kp 1,25 untuk sp 90,45     last 0,95
float Kd = 0.6; // kd 0.021
float Ki = 0;
volatile long encoderCount1 = 0;
unsigned long prevTime = 0;
float ePrev = 0;
float eIntegral = 0;

// Variabel untuk setpoint
float setpointDegree = 0;  // Setpoint jarak yang diinginkan dalam derajat

int t = 0;

void setup() {
  // Inisialisasi Serial
  Serial.begin(115200);

  // Konfigurasi pin-pin
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enAM1, INPUT_PULLUP);
  pinMode(enBM1, INPUT_PULLUP);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(enAM2, INPUT);
  pinMode(enBM2, INPUT);

  // Attach interrupt untuk membaca encoder
  attachInterrupt(digitalPinToInterrupt(enAM1), readEncoder1, RISING);

  //  delay(10000);
  // analogWrite(pin1, 255);
  // analogWrite(pin2, 0);
}

void loop() {
  // Baca setpoint dari Serial
  cekSerial();
  //  float distance1 = encoderCount1 * degreePerPulse;
  
  //  Serial.println(encoderCount1);
  //  delay(500);

  // Kontrol motor ke setpoint menggunakan PID
  controlMotor();
  //  delay(20);
}

void cekSerial() {
  // Baca data dari Serial jika tersedia
  while (Serial.available() > 0) {
    char d = Serial.read();
    if (d == 's') {
      setpointDegree = Serial.parseInt();
    }
  }
}

void controlMotor() {
  // Hitung jarak yang telah ditempuh oleh motor
  float degreePerPulse = (setpointDegree / 360) * encoderResolution;

  // Hitung error
  float error = degreePerPulse - encoderCount1;

  // Hitung sinyal kontrol PID
  float pidOutput = calculatePID(error);

  // Kendalikan motor berdasarkan sinyal kontrol PID
  moveMotor(pidOutput);

  // Tampilkan setpoint dan jarak dalam Serial Monitor
  //  unsigned long prevMillis;
  //  if ((millis() - prevMillis) >= 1000) {
  //    prevMillis = millis();
  //    Serial.print(t);
  //    Serial.print("\t");
  //    Serial.print(degreePerPulse);
  //    Serial.print("\t");
  //    Serial.println(encoderCount1);
  //
  //    t++;
  //
  //  }
  // unsigned long currentTime = micros();

  // // Hitung selang waktu sejak loop sebelumnya
  // float deltaT = ((float)(currentTime - prevTime)) / 1000000.0;
  // Serial.print(deltaT);
  // Serial.print("\t");
  Serial.print(degreePerPulse);
  Serial.print("\t");
  Serial.println(encoderCount1);
  //  Serial.print("Setpoint: ");
  //
  //  Serial.print(" encoderCount1: ");

}

void readEncoder1() {
  // Baca pulsa encoder dan perbarui counter
  if (digitalRead(enBM1) == LOW) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

float calculatePID(float error) {
  //  // Waktu sekarang
  //    unsigned long currentTime = micros();
  //
  //  // Hitung selang waktu sejak loop sebelumnya
  //    float deltaT = ((float)(currentTime - prevTime)) / 1000000.0;

  // Hitung turunan error
  float eDerivative = (error - ePrev);
  //    float eDerivative = (error - ePrev) / deltaT;

  // Hitung sinyal kontrol PID
  float pidOutput = (Kp * error) + (Kd * eDerivative);

  // Simpan waktu dan error untuk loop berikutnya
  //  prevTime = currentTime;
  ePrev = error;

  // Kembalikan sinyal kontrol PID
  return pidOutput;
}

void moveMotor(float pidOutput) {
  // Batasi sinyal kontrol agar tidak melebihi 255 atau -255
  if (pidOutput > 255) {
    pidOutput = 255;
  } else if (pidOutput < -255) {
    pidOutput = -255;
  }

  //  if (pidOutput < 0) {
  //    analogWrite(pin1, abs(pidOutput));
  //    analogWrite(pin2, 0);
  //  } else {
  //    analogWrite(pin1, 0);
  //    analogWrite(pin2, abs(pidOutput));
  //  }

  // Tentukan arah putaran motor
  float directionA = (pidOutput >= 0) ? 0 : abs(pidOutput);
  float directionB = (pidOutput >= 0) ? abs(pidOutput) : 0;
  //
  //  // Aktifkan pin-pen yang sesuai dengan arah putaran motor
  analogWrite(pin1, directionA);
  analogWrite(pin2, directionB);
  //
  //  // Set kecepatan motor berdasarkan sinyal kontrol PID
  //  analogWrite(pwmA, abs(pidOutput));
}
