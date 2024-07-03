#include <Wire.h>

// Definisikan pin-pin yang digunakan
#define pin1 PA3
#define pin2 PA1  // motor steer
#define enAM1 PB8
#define enBM1 PB9
#define pin3 PA6
#define pin4 PA7  // motor drive
#define enAM2 PB7
#define enBM2 PB6

#define pinHall PB5
#define pinLed PC13

// Konstanta PID Steer
const double Kp_steer = 1.17; // default 0.8
const double Kd_steer = 0.462; // 0.6
const double Ki_Steer = 0.002;
// modul D(kp = 0.97, Kd = 0.6), modul B(Kp = 1.1, Kd = 0.7)

// Konstanta PID Drive
const double Kp_drive = 0.05; //0.05
const double Ki_drive = 0.2; // 0.2
const double Kd_drive = 0.0008; //0.005
//modul B(Kp = 0.08, Ki = 0.5, Kd = 0.005)

// Variabel Steer
volatile long encoderCount1 = 0;
double lastError_steer = 0;
double eIntegral_steer = 0;

// Variabel Drive
short int setpointRPM = 0;
double lastRPM = 0;
double eIntegral_drive = 0;
volatile long encoderCount2 = 0;

// Konstanta umum
const double PPR_steer = 330;  // Pulses per revolution
const double GEAR_RATIO_steer = 2.2285714;
const double PPR_drive = 206.25;  // Pulses per revolution
const double GEAR_RATIO_drive = 0.4725;
const double SAMPLING_TIME = 100;  // in milliseconds

short int speed = 0;
short int degree = 0;

unsigned long prevTime;

TwoWire Wire2(PB11, PB10);

#define SLAVE_ADDR 13
#define DATA_SIZE 20

char receivedData[DATA_SIZE + 1];  // Buffer untuk menyimpan data yang diterima, ditambah 1 untuk null terminator

// Waktu untuk pengaturan interval
unsigned long lastTimePID_steer = 0;
unsigned long lastTimePID_drive = 0;
unsigned long lastTimeI2C = 0;

void setup() {
  //  Wire2.begin(SLAVE_ADDR);        // Mulai sebagai perangkat slave dengan alamat 13
  //  Wire2.onReceive(receiveEvent);  // Menentukan fungsi yang akan dipanggil saat data diterima
  Serial.begin(115200);

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
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);

  while (digitalRead(pinHall) != LOW) {
    short int dirA = (degree >= 0) ? 30 : 0;
    short int dirB = (degree >= 0) ? 0 : 30;
    analogWrite(pin1, dirA);
    analogWrite(pin2, dirB);
  }
  degree = 0;
  digitalWrite(pinLed, LOW);
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
//  delay(5000);

  encoderCount1 = 0;
  encoderCount2 = 0;
}

void loop() {
  unsigned long currentTime = millis();

  // Pembacaan I2C
  if (currentTime - lastTimeI2C >= 100) {  // Interval 100ms
    // Data I2C sudah diterima di receiveEvent
    TaskControlSerial();
    lastTimeI2C = currentTime;
  }

//      Serial.println(encoderCount2);
//      delay(100);

  //  // Pengendalian PID Steer
//    if (currentTime - lastTimePID_steer >= 50) {  // Interval 50ms
//      TaskControlAngle((double)degree);
//      lastTimePID_steer = currentTime;
//    }

  // Pengendalian PID Drive
  if (currentTime - lastTimePID_drive >= SAMPLING_TIME) {
    TaskControlSpeed(speed);
    lastTimePID_drive = currentTime;
  }
}

void TaskControlSerial() {
  // Baca data dari Serial jika tersedia
  while (Serial.available() > 0) {
    char d = Serial.read();
    // data += d;
    if (d == 's') {
      speed = Serial.parseInt();
//            degree = Serial.parseInt();
    }
  }
}

void TaskControlAngle(float setPoint) {
  static double PWM_L = 0;
  static double PWM_R = 0;

  double degreePerPulse = (setPoint / 360.0) * (PPR_steer * GEAR_RATIO_steer);
  double error = degreePerPulse - encoderCount1;
  //  eIntegral_steer += error;
  double eDerivative_steer = error - lastError_steer;
  double pdOutput_steer = (Kp_steer * error) + (Kd_steer * eDerivative_steer);

  pdOutput_steer = constrain(pdOutput_steer, -255, 255);

  PWM_L = (pdOutput_steer >= 0) ? 0 : abs(pdOutput_steer);
  PWM_R = (pdOutput_steer >= 0) ? abs(pdOutput_steer) : 0;

  analogWrite(pin1, PWM_L);
  analogWrite(pin2, PWM_R);

  Serial.print("Setpoint: ");
  Serial.print(degreePerPulse);
  Serial.print(" encoderCount1: ");
  Serial.println(encoderCount1);

  lastError_steer = error;
}

void TaskControlSpeed(short int setpoint) {
  static double PWM_L = 0;
  static double PWM_R = 0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  if ((currentTime - lastTime) >= SAMPLING_TIME) {
    double rev = (double)encoderCount2 / (PPR_drive * GEAR_RATIO_drive);
    double currentRPM = (rev / (SAMPLING_TIME / 1000.0)) * 60.0;
    encoderCount2 = 0;

    double error = setpoint - currentRPM;
    eIntegral_drive += (error * (SAMPLING_TIME / 1000.0));
    double eDerivative_drive = (currentRPM - lastRPM) / (SAMPLING_TIME / 1000.0);
    double pidOutput_drive = (Kp_drive * error) + (Ki_drive * eIntegral_drive) + (Kd_drive * eDerivative_drive);

    pidOutput_drive = constrain(pidOutput_drive, -255, 255);

    //    if (setpoint != 0) {
    //      PWM_L = (pidOutput_drive >= 0) ? abs(pidOutput_drive) : 0;
    //      PWM_R = (pidOutput_drive >= 0) ? 0 : abs(pidOutput_drive);
    //    } else {
    //      PWM_L = 0;
    //      PWM_R = 0;
    //    }
    PWM_L = (pidOutput_drive >= 0) ? abs(pidOutput_drive) : 0;
    PWM_R = (pidOutput_drive >= 0) ? 0 : abs(pidOutput_drive);

    analogWrite(pin3, PWM_L);
    analogWrite(pin4, PWM_R);

//        float deltaT = (currentTime / 1000.0);
////        prevTime = currentTime;
//        Serial.print(deltaT);
//        Serial.print("\t");

    Serial.print("Setpoint RPM: ");
    Serial.print(setpoint);
    Serial.print("\tcurrentRPM: ");
    Serial.println(currentRPM);

    lastRPM = currentRPM;
    lastTime = currentTime;
  }
}

void EN_steer() {
  if (digitalRead(enBM1) == LOW) {
    encoderCount1++;
  } else {
    encoderCount1--;
  }
}

void EN_drive() {
  if (digitalRead(enBM2) == LOW) {
    encoderCount2++;
  } else {
    encoderCount2--;
  }
}

void receiveEvent(int dataSize) {
  int index = 0;
  while (Wire2.available()) {
    char receivedChar = Wire2.read();
    receivedData[index] = receivedChar;
    index++;
    if (index >= DATA_SIZE) {
      break;
    }
  }
  receivedData[index] = '\0';
  Serial.println("Received Data: " + String(receivedData));

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
