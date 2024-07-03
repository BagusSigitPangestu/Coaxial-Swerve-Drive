#include <Wire.h>

// Definisikan pin-pin yang digunakan
#define pin1 PA1
#define pin2 PA3  // motor steer
#define enAM1 PB9
#define enBM1 PB8

#define pin3 PA6
#define pin4 PA7  // motor drive
#define enAM2 PB7
#define enBM2 PB6

#define pinHall PB5

//-----------------------------------------------------------------------------------//
// Definisikan konstanta-konstanta untuk pengaturan PID motor steer
const float encoderResolution = 330 * 2.2285714;  // Resolusi encoder dalam pulsa per rotasi
// Variabel untuk PID
float Kp_steer = 0.8;  // kp 0.91 untuk sp 360, 180 | kp 1,25 untuk sp 90,45
float Kd_steer = 0.6;  // kd 0.021
volatile long encoderCount1 = 0;
float ePrev_steer = 0;
float eIntegral_steer = 0;
// Variabel untuk setpoint
float setpointDegree = 0;  // Setpoint jarak yang diinginkan dalam
//-----------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------//
// Variabel untuk setpoint RPM
int setpointRPM = 500;  // Setpoint RPM yang diinginkan
// Konstanta untuk pengaturan PID
const float gearRatio = 0.4725;                        // Rasio roda gigi dari motor ke roda
const float pulsesPerRevolution = 206.25 * gearRatio;  // Pulsa per rotasi roda
// Variabel untuk PID
float Kp_drive = 1.1;   // 1.1
float Ki_drive = 0.9;   // Turunkan nilai Ki untuk mengurangi osilasi 0.09
float Kd_drive = 0.01;  // Turunkan nilai Kd untuk mengurangi osilasi 0.01
volatile long encoderCount2 = 0;
float ePrev_drive = 0;
float eIntegral_drive = 0;
float motorRPM = 0;
//-----------------------------------------------------------------------------------//

String data;

int speed, degree;
bool posZero = false;


TwoWire Wire2(PB11, PB10);

#define SLAVE_ADDR 10
#define DATA_SIZE 20

char receivedData[DATA_SIZE + 1];  // Buffer untuk menyimpan data yang diterima, ditambah 1 untuk null terminator

void setup() {
  Wire2.begin(SLAVE_ADDR);        // Mulai sebagai perangkat slave dengan alamat 9
  Wire2.onReceive(receiveEvent);  // Menentukan fungsi yang akan dipanggil saat data diterima
  Serial.begin(9600);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enAM1, INPUT);
  pinMode(enBM1, INPUT);


  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(enAM2, INPUT);
  pinMode(enBM2, INPUT);
  attachInterrupt(digitalPinToInterrupt(enAM1), EN_steer, RISING);
  attachInterrupt(digitalPinToInterrupt(enAM2), EN_drive, RISING);


  pinMode(pinHall, INPUT);
  delay(5000);
  // while (digitalRead(pinHall) != LOW) {
  //   analogWrite(pin1, 50);
  //   analogWrite(pin2, 0);
  // }
  // analogWrite(pin1, 0);
  // analogWrite(pin2, 0);
  // delay(1000);

  // encoderCount1 = 0;
  // encoderCount2 = 0;
  // posZero = true;

  // delay(1000);
}

void loop() {
  // Tidak ada yang perlu dilakukan di loop() karena data akan diproses dalam receiveEvent()
  // digitalWrite(PC13, LOW);
  // delay(100);
  // digitalWrite(PC13, HIGH);
  // delay(100);

//  TaskControlSerial();
//  TaskControlAngle();
  //  TaskControlSpeed();

  // delay(100);  // Beri sedikit waktu bagi perangkat slave untuk menanggapi datari
  // Serial.print("s=");
  // Serial.print(encoderCount1);
  // Serial.print("d=");
  // Serial.println(encoderCount2);
  // delay(500);
}

void TaskControlSerial() {
  // Baca data dari Serial jika tersedia
  while (Serial.available() > 0) {
    char d = Serial.read();
    // data += d;
    if (d == 's') {
      // setpointRPM = Serial.parseInt();
      setpointDegree = Serial.parseInt();
      //      Kp_drive = Serial.parseInt();
      //      Kd_drive = Serial.parseInt();
      //      Ki_drive = Serial.parseInt();
      //       Serial.print(setpointRPM);
      //       Serial.print("\t");
      //       Serial.print(Kp_drive);
      //       Serial.print("\t");
      //       Serial.print(Kd_drive);
      //       Serial.print("\t");
      //       Serial.println(Ki_drive);
      // data = "";
    }
  }
  //      delay(500);
}

void TaskControlAngle() {

  float degreePerPulse = (setpointDegree / 360) * encoderResolution;
  // Hitung error
  float error = degreePerPulse - encoderCount1;
  // Hitung sinyal kontrol PD
  float pdOutput = calculatePD(error);
  // Batasi sinyal kontrol agar tidak melebihi 255 atau -255
  if (pdOutput > 255) {
    pdOutput = 255;
  } else if (pdOutput < -255) {
    pdOutput = -255;
  }
  // Tentukan arah putaran motor
  float directionA = (pdOutput >= 0) ? 0 : abs(pdOutput);
  float directionB = (pdOutput >= 0) ? abs(pdOutput) : 0;

  // Aktifkan pin-pen yang sesuai dengan arah putaran motor
  analogWrite(pin1, directionA);
  analogWrite(pin2, directionB);
  // Set kecepatan motor berdasarkan sinyal kontrol PID

  // Tampilkan setpoint dan jarak dalam Serial Monitor
  Serial.print("Setpoint: ");
  Serial.print(degreePerPulse);
  Serial.print(" encoderCount1: ");
  Serial.println(encoderCount1);
}

void TaskControlSpeed() {
  unsigned long prevMillis;
  if ((millis() - prevMillis) >= 100) {
    prevMillis = millis();
    motorRPM = (encoderCount2 * 60 * 10) / pulsesPerRevolution;
    encoderCount2 = 0;
  }
  // Hitung error
  float error = setpointRPM - motorRPM;
  float pidOutput = calculatePID(error);

  // Batasi sinyal kontrol agar tidak melebihi 255 atau -255
  if (pidOutput > 1082.0) {
    pidOutput = 1082.0;
  } else if (pidOutput < 10.0) {
    pidOutput = 0;
  }

  // Tentukan arah putaran motor
  float directionA = (pidOutput >= 0) ? 0 : abs(pidOutput);
  float directionB = (pidOutput >= 0) ? abs(pidOutput) : 0;
  // Aktifkan pin-pen yang sesuai dengan arah putaran motor
  analogWrite(pin3, directionA * (255.0 / 1082.0));
  analogWrite(pin4, directionB * (255.0 / 1082.0));
  // Set kecepatan motor berdasarkan sinyal kontrol PID

  // Tampilkan setpoint dan RPM dalam Serial Monitor
  Serial.print("Setpoint RPM: ");
  Serial.println(setpointRPM);
  Serial.print(", Motor RPM: ");
  Serial.println(motorRPM);
  delay(100);
}

void EN_steer() {
  // Baca pulsa encoder dan perbarui counter
  if (digitalRead(enBM1) == LOW) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

void EN_drive() {
  // Baca pulsa encoder dan perbarui counter
  encoderCount2++;
}

// Fungsi ini akan dipanggil saat data diterima
void receiveEvent(int dataSize) {
  int index = 0;
  while (Wire2.available()) {            // Selama ada data yang tersedia untuk dibaca
    char receivedChar = Wire2.read();    // Baca karakter yang diterima
    receivedData[index] = receivedChar;  // Simpan karakter ke dalam buffer
    index++;
    if (index >= DATA_SIZE) {
      break;  // Jika buffer penuh, hentikan pembacaan
    }
  }
  receivedData[index] = '\0';  // Tambahkan null terminator untuk menandai akhir string
  Serial.println("Received Data: " + String(receivedData));

  // Proses data yang diterima di sini sesuai kebutuhan
  // Contoh: Pemisahan data dan tindakan berdasarkan data yang diterima

  // Contoh: Memecah data yang diterima berdasarkan tanda "#" dan mencetaknya ke Serial
  char *token = strtok(receivedData, "#");
  if (token != NULL) {
    speed = atoi(token);
    Serial.print(speed);
    Serial.print(" ");
    token = strtok(NULL, "#");
    if (token != NULL) {
      degree = atoi(token);
      Serial.print(degree);
    }
  }
  Serial.println();
}

float calculatePD(float error) {
  // Hitung turunan error
  float eDerivative = (error - ePrev_steer);
  // Hitung sinyal kontrol PID
  float pidOutput = (Kp_steer * error) + (Kd_steer * eDerivative);
  ePrev_steer = error;
  // Kembalikan sinyal kontrol PD
  return pidOutput;
}

float calculatePID(float error) {
  float eDerivative = (error - ePrev_drive);
  eIntegral_drive += error;
  // Hitung sinyal kontrol PID
  float pidOutput = (Kp_drive * error) + (Kd_drive * eDerivative) + (Ki_drive * eIntegral_steer);
  ePrev_drive = error;

  return pidOutput;
}
