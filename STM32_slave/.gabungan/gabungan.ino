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
#define pinLed PC13

//-----------------------------------------------------------------------------------//
// Definisikan konstanta-konstanta untuk pengaturan PID motor steer
// Variabel untuk setpoint
float setpointDegree = -45;  // Setpoint jarak yang diinginkan dalam

// Variabel untuk PID
volatile long encoderCount1 = 0;
double eDerivative_steer = 0;
double pdOutput_steer = 0;
double lastError_steer = 0;

// Encoder and Motor constants
const double PPR_steer = 330;  // Pulses per revolution
const double GEAR_RATIO_steer = 2.2285714;
//-----------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------//
// Variabel untuk setpoint RPM
short int setpointRPM = 0;  // Setpoint RPM yang diinginkan

// Variabel untuk PID
double lastRPM = 0;
double eIntegral_drive = 0;
double eDerivative_drive = 0;
double pidOutput_drive = 0;
volatile long encoderCount2 = 0;
// Encoder and Motor constants
const double PPR_drive = 206.25;  // Pulses per revolution
const double GEAR_RATIO_drive = 0.4725;
const double SAMPLING_TIME = 100;  // in milliseconds

//-----------------------------------------------------------------------------------//

String data;

short int speed;
double degree = -90.0;
bool posZero = false;


TwoWire Wire2(PB11, PB10);

#define SLAVE_ADDR 13
#define DATA_SIZE 20

char receivedData[DATA_SIZE + 1];  // Buffer untuk menyimpan data yang diterima, ditambah 1 untuk null terminator

void setup() {
  // Wire2.begin(SLAVE_ADDR);        // Mulai sebagai perangkat slave dengan alamat 9
  // Wire2.onReceive(receiveEvent);  // Menentukan fungsi yang akan dipanggil saat data diterima
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
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
  delay(5000);
  // while (digitalRead(pinHall) != LOW) {
  //   analogWrite(pin1, 50);
  //   analogWrite(pin2, 0);
  // }
  // digitalWrite(pinLed, LOW);
  // analogWrite(pin1, 0);
  // analogWrite(pin2, 0);
  // delay(1000);

  encoderCount1 = 0;
  encoderCount2 = 0;
  // posZero = true;

  // delay(1000);
}

void loop() {
  // Tidak ada yang perlu dilakukan di loop() karena data akan diproses dalam receiveEvent()
  // digitalWrite(PC13, LOW);
  // delay(100);
  // digitalWrite(PC13, HIGH);
  // delay(100);

  // TaskControlSerial();
  // Serial.println(degree);
  // delay(100);
  TaskControlAngle((double)degree);
  // TaskControlSpeed(speed);

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
    data += d;
    if (d == 's') {
      // speed = Serial.parseInt();
      // degree = Serial.parseInt();
      //      Kp_drive = Serial.parseInt();
      //      Kd_drive = Serial.parseInt();
      //      Ki_drive = Serial.parseInt();
            Serial.print(data);
      //       Serial.print("\t");
      //       Serial.print(Kp_drive);
      //       Serial.print("\t");
      //       Serial.print(Kd_drive);
      //       Serial.print("\t");
      //       Serial.println(Ki_drive);
      data = "";
    }
  }
  //      delay(500);
}

void TaskControlAngle(float setPoint) {
  // Variabel untuk PID
  float Kp = 0.8;  // kp 0.91 untuk sp 360, 180 | kp 1,25 untuk sp 90,45
  float Kd = 0.6;  // kd 0.021
  static double PWM_L = 0;
  static double PWM_R = 0;

  double degreePerPulse = (setPoint / 360.0) * (PPR_steer * GEAR_RATIO_steer);
  // PD calculate
  double error = degreePerPulse - encoderCount1;
  eDerivative_steer = error - lastError_steer;
  pdOutput_steer = (Kp * error) + (Kd * eDerivative_steer);

  // Batasi ouptut PD sesuai range PWM
  pdOutput_steer = constrain(pdOutput_steer, -255, 255);

  // Tentukan arah putaran motor
  PWM_L = (pdOutput_steer >= 0) ? 0 : abs(pdOutput_steer);
  PWM_R = (pdOutput_steer >= 0) ? abs(pdOutput_steer) : 0;

  // Aktifkan pin-pen yang sesuai dengan arah putaran motor
  analogWrite(pin1, PWM_L);
  analogWrite(pin2, PWM_R);

  // Tampilkan setpoint dan jarak dalam Serial Monitor
  Serial.print("Setpoint: ");
  Serial.print(setPoint);
  Serial.print(" encoderCount1: ");
  Serial.println(encoderCount1);
}

void TaskControlSpeed(short int setpoint) {
  // Variabel untuk PID
  double Kp = 0.183;  //
  double Ki = 0.49;   // Turunkan nilai Ki untuk mengurangi osilasi 0.09
  double Kd = 0.005;  // Turunkan nilai Kd untuk mengurangi osilasi 0.01
  static double PWM_L = 0;
  static double PWM_R = 0;

  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  if ((currentTime - lastTime) >= SAMPLING_TIME) {
    //Konversi PPR_drive ke RPM
    double rev = (double)encoderCount2 / (PPR_drive * GEAR_RATIO_drive);
    double currentRPM = (rev / (SAMPLING_TIME / 1000.0)) * 60.0;
    encoderCount2 = 0;

    //PID calculate
    double error = setpoint - currentRPM;
    eIntegral_drive += (error * (SAMPLING_TIME / 1000.0));
    eDerivative_drive = (currentRPM - lastRPM) / (SAMPLING_TIME / 1000.0);
    pidOutput_drive = (Kp * error) + (Ki * eIntegral_drive) + (Kd * eDerivative_drive);

    // Batasi Output dari PID sesuai range PWM
    pidOutput_drive = constrain(pidOutput_drive, -255, 255);

    if (setpoint != 0) {
      // Tentukan arah putaran motor
      PWM_L = (pidOutput_drive >= 0) ? abs(pidOutput_drive) : 0;
      PWM_R = (pidOutput_drive >= 0) ? 0 : abs(pidOutput_drive);
    } else {
      PWM_L = 0;
      PWM_R = 0;
    }
    // Aktifkan pin-pin syang sesuai dengan arah putaran motor
    analogWrite(pin3, PWM_L);
    analogWrite(pin4, PWM_R);

    // Tampilkan setpoint dan RPM dalam Serial Monitor
    Serial.print("Setpoint RPM: ");
    Serial.print(setpoint);
    Serial.print("\tMotor RPM: ");
    Serial.println(currentRPM);

    lastRPM = currentRPM;
    lastTime = currentTime;
  }
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
  if (digitalRead(enBM2) == LOW) {
    encoderCount2--;
  } else {
    encoderCount2++;
  }
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
