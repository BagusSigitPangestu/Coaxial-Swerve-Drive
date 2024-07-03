// STM32F103C8T6, BTS7960 Motor Driver, and Rotary Encoder (PPR 206.25, Gear Ratio 1:0.4725)

#define PWM_L PA6      // PWM signal to motor driver BTS7960 (left side)
#define PWM_R PA7      // PWM signal to motor driver BTS7960 (right side)
#define ENCODER_A PB7  // Encoder channel A
#define ENCODER_B PB6  // Encoder channel B

// PID constants
double Kp = 0.13;  //0.51
double Ki = 0.49;   //0.2
double Kd = 0.005;  //0.1

// Encoder and Motor constants
const double PPR = 206.25;  // Pulses per revolution
const double GEAR_RATIO = 0.4725;
const double SAMPLING_TIME = 100;  // in milliseconds

// Variables for PID control
double setpoint = 0;
double input = 0;
double output = 0;
double lastInput = 0;
double integral = 0;

// Encoder variables
volatile long encoderCount = 0;
// // Encoder variables
// volatile long encoderCount = 0;
unsigned long lastTime = 0;

// ISR for Encoder
void encoderISR() {
  if (digitalRead(ENCODER_B) == LOW) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}
// 26120
// 18736
// 1616

void TaskControlSerial() {
  // Baca data dari Serial jika tersedia
  while (Serial.available() > 0) {
    char d = Serial.read();
    // data += d;
    if (d == 's') {
      setpoint = Serial.parseInt();
    }
  }
  //      delay(500);
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize pins
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Attach interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  // Initialize timer
  lastTime = millis();
}

void loop() {
  // Serial.println(encoderCount);
  // delay(100);
  TaskControlSerial();
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
   Serial.println(encoderCount);
   delay(100);
//  if ((currentTime - lastTime) >= SAMPLING_TIME) {
//    // Calculate RPM from encoder count
//    double revolutions = (double)encoderCount / (PPR * GEAR_RATIO);
//    double rpm = (revolutions / (SAMPLING_TIME / 1000.0)) * 60.0;
//    encoderCount = 0;
//
//    // PID calculations
//    input = rpm;
//    double error = setpoint - input;
//    integral += (Ki * error * (SAMPLING_TIME / 1000.0));
//    double derivative = (input - lastInput) / (SAMPLING_TIME / 1000.0);
//    output = (Kp * error) + integral - (Kd * derivative);
//
//    // Limit output to PWM range
//    output = constrain(output, -255, 255);
//
//    // Set motor speed and direction
//    if (setpoint != 0) {
//      if (output >= 0) {
//        analogWrite(PWM_L, output);
//        analogWrite(PWM_R, 0);
//      } else {
//        analogWrite(PWM_L, 0);
//        analogWrite(PWM_R, -output);
//      }
//    } else {
//      analogWrite(PWM_L, 0);
//      analogWrite(PWM_R, 0);
//    }
//
//
//    // Print debug information
//    Serial.print("Setpoint: ");
//    Serial.print(setpoint);
//    Serial.print(" RPM, Input: ");
//    Serial.println(rpm);
//    // Serial.print(" RPM, Output: ");
//    // Serial.println(output);
//    // Update last variables
//    lastInput = input;
//    lastTime = currentTime;
//  }
}
