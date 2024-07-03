#include <Arduino.h>

// Definisikan pin-pin yang digunakan
#define pin1 PA1
#define pin2 PA3
#define pwmA PA0
#define enAM1 PB9
#define enBM1 PB8

#define pin3 PA4
#define pin4 PA5
#define pwmB PA6
#define enAM2 PB7
#define enBM2 PB6

// Definisikan konstanta-konstanta untuk pengaturan PID
const float encoderResolution = 330 * 2.2285714;  // Resolusi encoder dalam pulsa per rotasi
const float degreePerPulse = 360.0 / encoderResolution;  // Jarak per pulsa encoder dalam derajat

// Variabel untuk PID
float Kp = 2.1;
float Kd = 0.1;
float Ki = 0;
volatile long encoderCount1 = 0;
unsigned long prevTime = 0;
float ePrev = 0;
float eIntegral = 0;

// Variabel untuk setpoint
int setpointDegree = 90;  // Setpoint jarak yang diinginkan dalam derajat

void setup() {
  // Inisialisasi Serial
  Serial.begin(9600);

  // Konfigurasi pin-pin
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(enAM1, INPUT);
  pinMode(enBM1, INPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(enAM2, INPUT);
  pinMode(enBM2, INPUT);

  // Attach interrupt untuk membaca encoder
  attachInterrupt(digitalPinToInterrupt(enAM1), readEncoder1, RISING);

  delay(5000);
}

void loop() {
  // Baca setpoint dari Serial
//  cekSerial();
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
  float distance1 = encoderCount1 * degreePerPulse;

  // Hitung error
  float error = setpointDegree - distance1;

  // Hitung sinyal kontrol PID
  float pidOutput = calculatePID(error);

  // Kendalikan motor berdasarkan sinyal kontrol PID
  moveMotor(pidOutput);

  // Tampilkan setpoint dan jarak dalam Serial Monitor
  Serial.print("Setpoint: ");
  Serial.print(setpointDegree);
  Serial.print(" degrees, Distance1: ");
  Serial.println(distance1);
}

void readEncoder1() {
  // Baca pulsa encoder dan perbarui counter
  if (digitalRead(enAM1) > digitalRead(enBM1)) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

float calculatePID(float error) {
  // Waktu sekarang
  unsigned long currentTime = micros();

  // Hitung selang waktu sejak loop sebelumnya
  float deltaT = ((float)(currentTime - prevTime)) / 1000000.0;

  // Hitung turunan error
  float eDerivative = (error - ePrev) / deltaT;

  // Hitung integral error
  eIntegral += error * deltaT;

  // Hitung sinyal kontrol PID
  float pidOutput = (Kp * error) + (Kd * eDerivative) + (Ki * eIntegral);

  // Simpan waktu dan error untuk loop berikutnya
  prevTime = currentTime;
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

  // Tentukan arah putaran motor
  int directionA = (pidOutput >= 0) ? LOW : HIGH;
  int directionB = (pidOutput >= 0) ? HIGH : LOW;

  // Aktifkan pin-pen yang sesuai dengan arah putaran motor
  digitalWrite(pin1, directionA);
  digitalWrite(pin2, directionB);

  // Set kecepatan motor berdasarkan sinyal kontrol PID
  analogWrite(pwmA, abs(pidOutput));
}
