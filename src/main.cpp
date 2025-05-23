#include <Arduino.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define MOTOR_A1 41  // 右后 向后
#define MOTOR_A2 15  // 右后 向前
#define MOTOR_B1 16  // 右前 向后
#define MOTOR_B2 17  // 右前 向前
#define MOTOR_C1 11  // 左前 向前
#define MOTOR_C2 10  // 左前 向后
#define MOTOR_D1 9   // 左后 向后
#define MOTOR_D2 8   // 左后 向前

#define VL_SCL_FRONT 13
#define VL_SDA_FRONT 14

#define VL_SCL_BACK 6
#define VL_SDA_BACK 5

const char *ssid = "ChinaNet-GSLh";
const char *password = "uhcxwszh";
unsigned int localPort = 8888;  // 本地端口

// PWM 频率 & 通道
const int PWM_FREQ = 1000;       // PWM 频率 1kHz
const int PWM_RESOLUTION = 7;    // 7-bit，占空比 0-127
const int CH_LEFT = 0;           // 左电机 PWM 通道
const int CH_LEFT_REVERSE = 1;   // 左电机 PWM 反向通道
const int CH_RIGHT = 2;          // 右电机 PWM 通道
const int CH_RIGHT_REVERSE = 3;  // 右电机 PWM 反向通道

void setMotor(int leftSpeed, int rightSpeed);

WiFiUDP udp;

VL53L0X frontSensor;
VL53L0X backSensor;

bool VLinited = false;
bool OAEnable = false;

void setup() {
  // 此时 A 是 0x29，B 是 0x30
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(MOTOR_C1, OUTPUT);
  pinMode(MOTOR_C2, OUTPUT);
  pinMode(MOTOR_D1, OUTPUT);
  pinMode(MOTOR_D2, OUTPUT);

  // 设置 PWM
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_LEFT_REVERSE, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_RIGHT_REVERSE, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_C1, CH_LEFT);
  ledcAttachPin(MOTOR_D2, CH_LEFT);
  ledcAttachPin(MOTOR_C2, CH_LEFT_REVERSE);
  ledcAttachPin(MOTOR_D1, CH_LEFT_REVERSE);
  ledcAttachPin(MOTOR_A2, CH_RIGHT);
  ledcAttachPin(MOTOR_B2, CH_RIGHT);
  ledcAttachPin(MOTOR_A1, CH_RIGHT_REVERSE);
  ledcAttachPin(MOTOR_B1, CH_RIGHT_REVERSE);

  Serial.begin(9600);

  Wire.begin(VL_SDA_FRONT, VL_SCL_FRONT);
  frontSensor.setBus(&Wire);
  if (frontSensor.init()) {
    VLinited = true;
    OAEnable = true;
    frontSensor.setMeasurementTimingBudget(20000);  // 20ms
    frontSensor.startContinuous();                  // 开始连续测量
  } else {
    VLinited = false;
    OAEnable = false;
    Serial.println("Front VL53L0X initialization failed!");
  }

  Wire1.begin(VL_SDA_BACK, VL_SCL_BACK);
  backSensor.setBus(&Wire1);
  if (backSensor.init()) {
    VLinited = true;
    OAEnable = true;
    backSensor.setMeasurementTimingBudget(20000);  // 20ms
    backSensor.startContinuous();                  // 开始连续测量
  } else {
    VLinited = false;
    OAEnable = false;
    Serial.println("Back VL53L0X initialization failed!");
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi!");
  Serial.print("WiFi IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(localPort);  // 开始监听
}

void loop() {
  byte packetBuffer[2];
  int packetSize = udp.parsePacket();

  if (packetSize > 0) {  // 检查是否有数据包到达
    int len = udp.read(packetBuffer, sizeof(packetBuffer));

    if (len == 1) {  // 单字节指令，用于切换 OAEnable
      if (!VLinited) {
        Serial.println("VL53L0X not initialized!");
        return;
      }
      if (packetBuffer[0] == 0x00) {
        OAEnable = false;
        Serial.println("OA: OFF");
      } else if (packetBuffer[0] == 0x01) {
        OAEnable = true;
        Serial.println("OA: ON");
      }
    } else if (len == 2) {
      udp.read(packetBuffer, 2);

      int leftSpeed = (int8_t)packetBuffer[0];
      int rightSpeed = (int8_t)packetBuffer[1];

      Serial.print("Left Speed: ");
      Serial.print(leftSpeed);
      Serial.print(" | Right Speed: ");
      Serial.println(rightSpeed);

      if (OAEnable) {
        int frontDistance = frontSensor.readRangeContinuousMillimeters();
        Serial.print("Front distance: ");
        Serial.print(frontDistance);
        Serial.println(" mm");

        int backDistance = backSensor.readRangeContinuousMillimeters();
        Serial.print("Back distance: ");
        Serial.print(backDistance);
        Serial.println(" mm");

        if (frontDistance < 300) {
          if (leftSpeed > 0 && rightSpeed > 0) {
            leftSpeed = 0;
            rightSpeed = 0;
          }
          Serial.println("Forward blocked!");
        }

        if (backDistance < 300) {
          if (leftSpeed < 0 && rightSpeed < 0) {
            leftSpeed = 0;
            rightSpeed = 0;
          }
          Serial.println("Back blocked!");
        }
      }

      setMotor(leftSpeed, rightSpeed);
    }
  }
  delay(32);
  setMotor(0, 0);

  /* Serial.print("Distance1: ");
  Serial.print(getDistance(1));
  Serial.println("cm");
  Serial.print("Distance2: ");
  Serial.print(getDistance(2));
  Serial.println("cm"); */

  /* Serial.println("start");
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  delay(500);
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  delay(500);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_C1, HIGH);
  digitalWrite(MOTOR_C2, LOW);
  delay(500);
  digitalWrite(MOTOR_C1, LOW);
  digitalWrite(MOTOR_D1, HIGH);
  digitalWrite(MOTOR_D2, LOW);
  delay(500);
  digitalWrite(MOTOR_D1, LOW);
  delay(1000); */
}

void setMotor(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {  // 左电机前进
    ledcWrite(CH_LEFT, leftSpeed);
    ledcWrite(CH_LEFT_REVERSE, 0);
  } else {  // 左电机后退
    ledcWrite(CH_LEFT, 0);
    ledcWrite(CH_LEFT_REVERSE, -leftSpeed);
  }

  if (rightSpeed > 0) {  // 右电机前进
    ledcWrite(CH_RIGHT, rightSpeed);
    ledcWrite(CH_RIGHT_REVERSE, 0);
  } else {  // 右电机后退
    ledcWrite(CH_RIGHT, 0);
    ledcWrite(CH_RIGHT_REVERSE, -rightSpeed);
  }
}
