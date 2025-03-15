#include <Arduino.h>
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

#define HC_TRIG_1 13  // 后 出
#define HC_ECHO_1 14  // 后 入
#define HC_TRIG_2 40  // 前 出
#define HC_ECHO_2 21  // 前 入

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
float getDistance(int which);

// 定义全局变量
TaskHandle_t Task1;  // 核心 0 的任务句柄
TaskHandle_t Task2;  // 核心 1 的任务句柄

// 定义互斥锁
SemaphoreHandle_t xMutex;

// 其他全局变量
bool frontBrake = false;
bool backBrake = false;
float distance1 = -1, distance2 = -1;

// UDP 相关代码
WiFiUDP udp;

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

float getDistance(int which) {
  int TRIG_PIN = which == 1 ? HC_TRIG_1 : HC_TRIG_2;
  int ECHO_PIN = which == 1 ? HC_ECHO_1 : HC_ECHO_2;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 读取回波时间 (微秒)
  float distance = duration * 0.034 / 2;           // 计算距离 (cm)
  return distance;
}

// 核心 0 的任务：处理 UDP 和电机控制
void TaskCore0(void *pvParameters) {
  byte packetBuffer[2];
  while (1) {
    int packetSize = udp.parsePacket();
    if (packetSize == 2) {
      udp.read(packetBuffer, 2);

      int leftSpeed = (int8_t)packetBuffer[0];
      int rightSpeed = (int8_t)packetBuffer[1];

      Serial.print("Left Speed: ");
      Serial.print(leftSpeed);
      Serial.print(" | Right Speed: ");
      Serial.println(rightSpeed);

      // 线程安全地访问 brake
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        if (leftSpeed > 0 && rightSpeed > 0 && frontBrake) return;
        if (leftSpeed < 0 && rightSpeed < 0 && backBrake) return;
        setMotor(leftSpeed, rightSpeed);
        xSemaphoreGive(xMutex);  // 释放锁
      }
    }
    delay(32);
    setMotor(0, 0);
  }
}

// 核心 1 的任务：处理超声波测距和避障逻辑
void TaskCore1(void *pvParameters) {
  while (1) {
    float distance1 = getDistance(1);
    float distance2 = getDistance(2);

    if (distance1 >= 0) {
      Serial.print("Distance1: ");
      Serial.print(distance1);
      Serial.println("cm");
    }

    if (distance2 >= 0) {
      Serial.print("Distance2: ");
      Serial.print(distance2);
      Serial.println("cm");
    }

    // 线程安全地修改 brake
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      frontBrake = distance2 < 20;
      backBrake = distance1 < 20;
      xSemaphoreGive(xMutex);  // 释放锁
    }

    delay(16);  // 控制任务循环频率
  }
}

void setup() {
  // 初始化串口
  Serial.begin(9600);

  // 初始化 GPIO 和 PWM
  pinMode(HC_TRIG_1, OUTPUT);
  pinMode(HC_ECHO_1, INPUT);
  pinMode(HC_TRIG_2, OUTPUT);
  pinMode(HC_ECHO_2, INPUT);

  // 设置 PWM
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_LEFT_REVERSE, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_RIGHT_REVERSE, PWM_FREQ, PWM_RESOLUTION);

  // 绑定 PWM 到引脚
  ledcAttachPin(MOTOR_C1, CH_LEFT);
  ledcAttachPin(MOTOR_D2, CH_LEFT);
  ledcAttachPin(MOTOR_C2, CH_LEFT_REVERSE);
  ledcAttachPin(MOTOR_D1, CH_LEFT_REVERSE);
  ledcAttachPin(MOTOR_A2, CH_RIGHT);
  ledcAttachPin(MOTOR_B2, CH_RIGHT);
  ledcAttachPin(MOTOR_A1, CH_RIGHT_REVERSE);
  ledcAttachPin(MOTOR_B1, CH_RIGHT_REVERSE);

  // 连接 WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  udp.begin(localPort);

  // 创建任务并绑定到核心
  xTaskCreatePinnedToCore(TaskCore0,    // 任务函数
                          "TaskCore0",  // 任务名称
                          10000,        // 堆栈大小（字节）
                          NULL,         // 参数
                          1,            // 优先级
                          &Task1,       // 任务句柄
                          0             // 核心编号
  );

  xTaskCreatePinnedToCore(TaskCore1,    // 任务函数
                          "TaskCore1",  // 任务名称
                          10000,        // 堆栈大小（字节）
                          NULL,         // 参数
                          1,            // 优先级
                          &Task2,       // 任务句柄
                          1             // 核心编号
  );

  // 初始化互斥锁
  xMutex = xSemaphoreCreateMutex();
}

void loop() {
  // 主循环可以留空，所有任务都在 FreeRTOS 中运行
}