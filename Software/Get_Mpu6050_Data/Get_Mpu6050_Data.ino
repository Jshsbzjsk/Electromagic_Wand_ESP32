/*
 * ------------------------------------------------------------
 * This work is derived from the following project:
 * Source: https://github.com/Songyeyaosong/MagicWand
 * Original Author: Songyeyaosong
 *
 * Modified by: dimo333
 * ------------------------------------------------------------
 */

#include "Wire.h"
#include "MPU6050.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

MPU6050 mpu;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
volatile bool collectDataRequest = false;

// ====== UUID（随便定义即可）======
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

// 定义每秒采样次数
const int freq = 100;
const int second = 2;
// const int freq = 64;
// const int second = 2;
// 重力分量
float gravity_x;
float gravity_y;
float gravity_z;
// 换算到x,y轴上的角速度
float roll_v, pitch_v;
// 上次更新时间
unsigned long prevTime;
// 三个状态，先验状态，观测状态，最优估计状态
float gyro_roll, gyro_pitch;        //陀螺仪积分计算出的角度，先验状态
float acc_roll, acc_pitch;          //加速度计观测出的角度，观测状态
float k_roll, k_pitch;              //卡尔曼滤波后估计出最优角度，最优估计状态
// 误差协方差矩阵P
float e_P[2][2];         //误差协方差矩阵，这里的e_P既是先验估计的P，也是最后更新的P
// 卡尔曼增益K
float k_k[2][2];         //这里的卡尔曼增益矩阵K是一个2X2的方阵

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Phone Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Phone Disconnected");
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.empty()) {
      return;
    }

    String rxString = String(rxValue.c_str());
    rxString.trim();

    Serial.print("Received: ");
    Serial.println(rxString);

    if (rxString == "collect_data") {
      collectDataRequest = true;
      Serial.println("collect_data request received");
    }
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);/////////////////////////////////////////////////////////////////////////////////////这里修改成你esp对应型号的scl和sda的引脚，不清楚的查看另一个esp资料的文件夹
  mpu.initialize();

  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050连接失败");
  //   while (1);
  // }
  Serial.println("MPU6050连接成功");

  resetState();

  BLEDevice::init("MCue");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("BLE Waiting for connection...");
}

void loop() {
  if (collectDataRequest) {
    collectDataRequest = false;
    Serial.println("Start collecting data for 2 seconds...");
    resetState();
    for (int i = 0; i < freq * second; i++) {
      kalman_update(i);
    }
    Serial.println();
    Serial.println("collect_data finished");
  }
}

void kalman_update(int i) {
  // 计算微分时间
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // 时间间隔（秒）
  prevTime = currentTime;

  // 获取角速度和加速度
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 转换加速度计数据为g值
  float Ax = ax / 16384.0;
  float Ay = ay / 16384.0;
  float Az = az / 16384.0;
  float Ox, Oy, Oz;

  // 转换陀螺仪数据为弧度/秒
  float Gx = gx / 131.0 / 180 * PI;
  float Gy = gy / 131.0 / 180 * PI;
  float Gz = gz / 131.0 / 180 * PI;

  // step1:计算先验状态
  // 计算x,y轴上的角速度
  roll_v = Gx + ((sin(k_pitch) * sin(k_roll)) / cos(k_pitch)) * Gy + ((sin(k_pitch) * cos(k_roll)) / cos(k_pitch)) * Gz; //roll轴的角速度
  pitch_v = cos(k_roll) * Gy - sin(k_roll) * Gz; //pitch轴的角速度
  gyro_roll = k_roll + dt * roll_v; //先验roll角度
  gyro_pitch = k_pitch + dt * pitch_v; //先验pitch角度

  // step2:计算先验误差协方差矩阵
  e_P[0][0] = e_P[0][0] + 0.0025;//这里的Q矩阵是一个对角阵且对角元均为0.0025
  e_P[0][1] = e_P[0][1] + 0;
  e_P[1][0] = e_P[1][0] + 0;
  e_P[1][1] = e_P[1][1] + 0.0025;

  // step3:更新卡尔曼增益
  k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3);
  k_k[0][1] = 0;
  k_k[1][0] = 0;
  k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3);

  // step4:计算最优估计状态
  // 观测状态
  // roll角度
  acc_roll = atan2(Ay, Az);
  //pitch角度
  acc_pitch = -atan2(Ax, sqrt(Ay * Ay + Az * Az));
  /*最优估计状态*/
  k_roll = gyro_roll + k_k[0][0] * (acc_roll - gyro_roll);
  k_pitch = gyro_pitch + k_k[1][1] * (acc_pitch - gyro_pitch);

  // step5:更新协方差矩阵
  e_P[0][0] = (1 - k_k[0][0]) * e_P[0][0];
  e_P[0][1] = 0;
  e_P[1][0] = 0;
  e_P[1][1] = (1 - k_k[1][1]) * e_P[1][1];

  // 计算重力加速度方向
  gravity_x = -sin(k_pitch);
  gravity_y = sin(k_roll) * cos(k_pitch);
  gravity_z = cos(k_roll) * cos(k_pitch);

  // 重力消除
  Ax = Ax - gravity_x;
  Ay = Ay - gravity_y;
  Az = Az - gravity_z;

  // 得到全局空间坐标系中的相对加速度
  Ox = cos(k_pitch) * Ax + sin(k_pitch) * sin(k_roll) * Ay + sin(k_pitch) * cos(k_roll) * Az;
  Oy = cos(k_roll) * Ay - sin(k_roll) * Az;
  Oz = -sin(k_pitch) * Ax + cos(k_pitch) * sin(k_roll) * Ay + cos(k_pitch) * cos(k_roll) * Az;

  // 打印数据
  // 打印数据
  Serial.print(Ox*9.8);///////////////////////////////////////
  Serial.print(",");
  Serial.print(Oy*9.8);
  Serial.print(",");
  Serial.print(Oz*9.8);

  if (i != freq * second - 1) {
    Serial.println();  // 最后一帧换行
  }

  delay(1000 / freq);
}

void resetState() {
  // 读取加速度计数据
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // 转换加速度计数据为g值
  float Ax = ax / 16384.0;
  float Ay = ay / 16384.0;
  float Az = az / 16384.0;

  // 计算Pitch和Roll
  k_pitch = -atan2(Ax, sqrt(Ay * Ay + Az * Az));
  k_roll = atan2(Ay, Az);

  // 误差协方差矩阵P
  e_P[0][0] = 1;
  e_P[0][1] = 0;
  e_P[1][0] = 0;
  e_P[1][1] = 1;

  // 卡尔曼增益K
  k_k[0][0] = 0;
  k_k[0][1] = 0;
  k_k[1][0] = 0;
  k_k[1][1] = 0;

  prevTime = millis();
}

