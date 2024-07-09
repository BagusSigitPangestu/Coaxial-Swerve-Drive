/*
  Panggil file hedder main
*/
#include "main.h"

void readStick()
{

  lx = Ps3.data.analog.stick.lx;
  ly = Ps3.data.analog.stick.ly;
  rx = Ps3.data.analog.stick.rx;
}

void addValue(int degreeVal[4], int speedVal[4])
{
  for (int i = 0; i < 4; i++)
  {
    Steer_SP[i] = degreeVal[i];
    Drive_SP[i] = speedVal[i];
  }
}
void onConnect()
{
  for(int i = 0; i < 3; i++){
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
    delay(100);
  }
  Serial.println("Connected.");
}

void setup()
{
  Serial.begin(9600);
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  pinMode(pinBuzzer, OUTPUT);
  Ps3.attach(readStick);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:1a:80:94:ad:ff");
  Serial.println("Ready.");

  zeroDegree = EEPROM.read(addsDegree);
  for (int i = 0; i < 4; i++)
  {
    Steer_SP[i] = zeroDegree;
  }

  // Core 0 setup
  xTaskCreatePinnedToCore(
      I2CCommunicationSendTask,
      "I2CCommunicationSendTask",
      10000,
      NULL,
      1,
      &I2CTask,
      0);

  // Core 1 setup
  xTaskCreatePinnedToCore(
      mainApplicationTask,
      "mainApplicationTask",
      10000,
      NULL,
      1,
      &mainTask,
      1);
}

void loop()
{
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

void mainApplicationTask(void *parameter)
{

  pinMode(2, OUTPUT);

  for (;;)
  {
    // while (Serial.available() > 0)
    // {
    //   char d = Serial.read();
    //   data += d;
    //   if (d == '$')
    //   {
    //     parsing = true;
    //   }

    //   if (parsing)
    //   {
    //     // Serial.print("data masuk= " + data);
    //     int index = 0;
    //     for (int i = 0; i < data.length(); i++)
    //     {
    //       if (data[i] == '#')
    //       {
    //         index++;
    //         ArrData[index] = "";
    //       }
    //       else
    //       {
    //         ArrData[index] += data[i];
    //       }
    //     }

    //     for (int j = 0; j < 4; j++)
    //     {
    //       Drive_SP[j] = ArrData[1].toInt();
    //       Steer_SP[j] = ArrData[2].toInt();
    //     }
    //     Serial.print(Drive_SP[0]);
    //     Serial.print("\t");
    //     Serial.print(Drive_SP[1]);
    //     Serial.print("\t");
    //     Serial.print(Drive_SP[2]);
    //     Serial.print("\t");
    //     Serial.println(Drive_SP[3]);

    //     Serial.print(Steer_SP[0]);
    //     Serial.print("\t");
    //     Serial.print(Steer_SP[1]);
    //     Serial.print("\t");
    //     Serial.print(Steer_SP[2]);
    //     Serial.print("\t");
    //     Serial.println(Steer_SP[3]);
    //     Serial.println("");

    //     data = "";
    //     parsing = false;
    //   }
    // }
    // Menghitung sudut dalam radian
    // float angleRadians = atan2(joystick_LY, joystick_LX);

    // // Mengonversi radian ke derajat
    // float angleDegrees = angleRadians * (180.0 / PI);
    // Serial.println(angleDegrees);

    // Serial.print(joystick_LY);
    // Serial.print("\t");
    // Serial.println(joystick_LX);

      if (ly <= -30 && (lx >= -40 && lx <= 40) || Ps3.data.button.up)
      {
        Serial.println("Speed 600 dan degree 0");
        int degreeVal[4] = {0, 0, 0, 0};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 30 && (lx >= -50 && lx <= 50) || Ps3.data.button.down)
      {
        Serial.println("Speed 600 dan degree 180");
        int degreeVal[4] = {180, 180, 180, 180};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (lx >= 30 && (ly >= -50 && ly <= 50) || Ps3.data.button.right)
      {
        Serial.println("Speed 600 dan degree 90");
        int degreeVal[4] = {90, 90, 90, 90};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (ly <= -30 && (lx >= 31 && lx <= 127))
      {
        Serial.println("Speed 600 dan degree 45");
        int degreeVal[4] = {45, 45, 45, 45};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 30 && (lx >= 31 && lx <= 127))
      {
        Serial.println("Speed 600 dan degree 135");
        int degreeVal[4] = {135, 135, 135, 135};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (lx <= -30 && (ly >= -50 && ly <= 50) || Ps3.data.button.left)
      {
        Serial.println("Speed 600 dan degree -90");
        int degreeVal[4] = {-90, -90, -90, -90};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (ly <= -50 && (lx <= -36 && lx >= -128))
      {
        Serial.println("Speed 600 dan degree -45");
        int degreeVal[4] = {-45, -45, -45, -45};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (ly >= 35 && (lx <= -36 && lx >= -128))
      {
        Serial.println("Speed 600 dan degree -135");
        int degreeVal[4] = {-135, -135, -135, -135};
        int speedVal[4] = {300, 300, 300, 300};
        addValue(degreeVal, speedVal);
      }
      else if (rx <= -40){
        Serial.println("Speed 500");
        int degreeVal[4] = {-45, -135, 45, 135};
        int speedVal[4] = {200, 200, 200, 200};
        addValue(degreeVal, speedVal);
      }
      else if(rx >= 40){
        Serial.println("Speed 500");
        int degreeVal[4] = {135, 45, -135, -45};
        int speedVal[4] = {200, 200, 200, 200};
        addValue(degreeVal, speedVal);
      }
      else
      {
        Serial.println("Speed 0 dan degree 0");
        for (int i = 0; i < 4; i++)
        {
          Drive_SP[i] = 0;
        }
      }

    if (Steer_SP[0] > 0)
    {
      eepromVal = 1;
    }
    else
    {
      eepromVal = -1;
    }
    // EEPROM.write(addsDegree, eepromVal);
    // EEPROM.commit();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void I2CCommunicationSendTask(void *parameter)
{
  Wire.begin();

  for (;;)
  {
    // Drive_SP[0] = 600;
    // Steer_SP[0] = 45;

    // String dataModule1 = String(Drive_SP[0]) + "#" + String(Steer_SP[0]);
    // // String dataModule2 = String(Drive_SP[1]) + "#" + String(Steer_SP[1]);
    // // String dataModule3 = String(Drive_SP[2]) + "#" + String(Steer_SP[2]);
    // // String dataModule4 = String(Drive_SP[3]) + "#" + String(Steer_SP[3]);

    // int dataSize = dataModule1.length();
    // char dataToSend[dataSize + 1];
    // dataModule1.toCharArray(dataToSend, dataSize + 1); // Mengonversi String ke dalam char array
    // Serial.println(dataToSend);
    // // Mulai transmisi ke slave dengan alamat yang ditentukan
    // Wire.beginTransmission(SLAVE_ADDR_MODULE[0]);
    // // Mengirim data (array of characters) melalui I2C
    // Wire.write((const uint8_t *)dataToSend, dataSize);
    // // Akhiri transmisi
    // Wire.endTransmission();
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    String dataModule[4];
    for (int i = 0; i < 4; i++)
    {
      dataModule[i] = String(Drive_SP[i]) + "#" + String(Steer_SP[i]);
      int dataSize = dataModule[i].length();
      char dataToSend[dataSize + 1];
      dataModule[i].toCharArray(dataToSend, dataSize + 1); // Mengonversi String ke dalam char array
      // Mulai transmisi ke slave dengan alamat yang ditentukan
      Wire.beginTransmission(SLAVE_ADDR_MODULE[i]);
      // Mengirim data (array of characters) melalui I2C
      Wire.write((const uint8_t *)dataToSend, dataSize);
      // Akhiri transmisi
      Wire.endTransmission();
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}
