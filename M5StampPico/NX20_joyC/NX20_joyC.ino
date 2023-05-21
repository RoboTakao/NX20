#include "M5Atom.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "1010"
#define CHRX_UUID "1012"
#define CHTX_UUID "1011"

byte joyLX=100, joyLY=100, joyRX=100, joyRY=100, joyLSW, joyRSW, joyLDistance, joyRDistance;

BLEServer* pServer = NULL;
BLECharacteristic* pCharTx = NULL;
BLECharacteristic* pCharRx = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const uint8_t Srv0 = 26; //GPIO Swing
const uint8_t Srv1 = 18; //GPIO Boom
const uint8_t Srv2 = 19; //GPIO Srm
const uint8_t Srv3 = 21; //GPIO Hand
const uint8_t Srv4 = 22; //GPIO Hand
const uint8_t Srv5 = 25; //GPIO Hand

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3, srv_CH4 = 4, srv_CH5 = 5; //チャンネル
const uint8_t Srv_RF = 1, Srv_RR = 32, Srv_LF = 3, Srv_LR = 33; //GPIO No.
const uint8_t srvCH_RF = 6, srvCH_RR = 7, srvCH_LF = 8, srvCH_LR = 9; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 16; //PWM 16bit(0～65535)

int mRF = 0, mRR = 0, mLF = 0, mLR = 0;

int pulseMIN = 1640;  //0deg 500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6
int pulseMAX = 8190;  //180deg 2500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6

int cont_min = 0;
int cont_max = 180;
int cont_thres = 40;
int cont_thres2 = 10;

int angZero[] = {90, 60, 90, 60, 90, 90};
int ang0[6];
int ang1[6];
int ang_b[6];
char ang_c[6];
float ts=160;  //150msごとに次のステップに移る
float td=20;   //10回で分割

int move_mode = 1;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value.length()>0) {
      joyLX=value[0];
      joyLY=value[1];
      joyRX=value[2];
      joyRY=value[3];
      joyLDistance=value[4];
      joyRDistance=value[5];
      joyLSW=value[6];
      joyRSW=value[7];
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void Initial_Value(){  //initial servo angle
  for (int j=0; j <=5 ; j++){
      ang0[j] = angZero[j];
  }
  for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j];
  }
  servo_set();
}

void Srv_drive(int srv_CH,int SrvAng){
  SrvAng = map(SrvAng, cont_min, cont_max, pulseMIN, pulseMAX);
  ledcWrite(srv_CH, SrvAng);
}

void servo_set(){     //線形補完してサーボに指令値を送る関数
  int a[6],b[6];
  
  for (int j=0; j <=5 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }

  for (int k=0; k <=td ; k++){

      Srv_drive(srv_CH0, a[0]*float(k)/td+b[0]);
      Srv_drive(srv_CH1, a[1]*float(k)/td+b[1]);
      Srv_drive(srv_CH2, a[2]*float(k)/td+b[2]);
      Srv_drive(srv_CH3, a[3]*float(k)/td+b[3]);
      Srv_drive(srv_CH4, a[4]*float(k)/td+b[4]);
      Srv_drive(srv_CH5, a[5]*float(k)/td+b[5]);

      delay(ts/td);
  }
}

void motor_drive(int motor_RF, int motor_RR, int motor_LF, int motor_LR){
    motor_RF = map(motor_RF, -cont_max, cont_max, pulseMIN, pulseMAX);
    motor_RR = map(motor_RR, -cont_max, cont_max, pulseMIN, pulseMAX);
    motor_LF = map(motor_LF, -cont_max, cont_max, pulseMIN, pulseMAX);
    motor_LR = map(motor_LR, -cont_max, cont_max, pulseMIN, pulseMAX);
  
    ledcWrite(srvCH_RF, motor_RF);
    ledcWrite(srvCH_RR, motor_RR);
    ledcWrite(srvCH_LF, motor_LF);
    ledcWrite(srvCH_LR, motor_LR);
}

void setup() {
  // void M5Atom::begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable )
  M5.begin(true, false, true);
  Serial.begin(151200);
  setupBLE();

  pinMode(Srv0, OUTPUT);
  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  pinMode(Srv3, OUTPUT);
  pinMode(Srv4, OUTPUT);
  pinMode(Srv5, OUTPUT);
  pinMode(Srv_RF, OUTPUT);
  pinMode(Srv_RR, OUTPUT);
  pinMode(Srv_LF, OUTPUT);
  pinMode(Srv_LR, OUTPUT);
  
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH0, PWM_Hz, PWM_level);
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);
  ledcSetup(srv_CH3, PWM_Hz, PWM_level);
  ledcSetup(srv_CH4, PWM_Hz, PWM_level);
  ledcSetup(srv_CH5, PWM_Hz, PWM_level);
  ledcSetup(srvCH_RF, PWM_Hz, PWM_level);
  ledcSetup(srvCH_RR, PWM_Hz, PWM_level);
  ledcSetup(srvCH_LF, PWM_Hz, PWM_level);
  ledcSetup(srvCH_LR, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv0, srv_CH0);
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);
  ledcAttachPin(Srv3, srv_CH3);
  ledcAttachPin(Srv4, srv_CH4);
  ledcAttachPin(Srv5, srv_CH5);
  ledcAttachPin(Srv_RF, srvCH_RF);
  ledcAttachPin(Srv_RR, srvCH_RR);
  ledcAttachPin(Srv_LF, srvCH_LF);
  ledcAttachPin(Srv_LR, srvCH_LR);

  Initial_Value();

  M5.dis.drawpix(0, 0x000000);  //Dummy
  M5.dis.drawpix(0, 0x0000ff);  //blue 0x0000ff
}

void loop() {
  M5.update();
  if ( M5.Btn.wasReleased() ) {
    Initial_Value();
  }
  
  checkBLE();

  int lx = (int)map(joyLX, 0, 200, -10, 10);
  int ly = (int)map(joyLY, 200, 0, -10, 10);
  int rx = (int)map(joyRX, 200, 0, -10, 10);
  int ry = (int)map(joyRY, 0, 200, -10, 10);
  int my = (int)map(joyLX, 0, 200, -100, 100);
  int mx = (int)map(joyLY, 0, 200, -100, 100);
  if(abs(lx) < 4) lx = 0;
  if(abs(ly) < 4) ly = 0;
  if(abs(rx) < 4) rx = 0;
  if(abs(ry) < 4) ry = 0;

  if(joyRSW == 1)
  {
    move_mode += 1;
    if(move_mode == 2) move_mode = -1;
    if(move_mode == 1){
        M5.dis.drawpix(0, 0x000000);  //Dummy
        M5.dis.drawpix(0, 0x0000ff);  //blue 0x0000ff
    }
    if(move_mode == 0){
        M5.dis.drawpix(0, 0x000000);  //Dummy
        M5.dis.drawpix(0, 0x00ff00);  //Green 0x00ff00
    }
    if(move_mode == -1){
        M5.dis.drawpix(0, 0x000000);  //Dummy
        M5.dis.drawpix(0, 0xff0000);  //Red 0xff0000
    }
  }

  if(move_mode == 1)
  {
    ang1[0] = ang1[0] + lx;
    if(ang1[0] > 180){
      ang1[0] = 180;
    }
    if(ang1[0] < 0){
      ang1[0] = 0;
    }

    ang1[2] = ang1[2] + ly;
    if(ang1[2] > 180){
      ang1[2] = 180;
    }
    if(ang1[2] < 0){
      ang1[2] = 0;
    }

    if(joyLSW == 0)
    {
      ang1[1] = ang1[1] + ry;
      if(ang1[1] > 180){
        ang1[1] = 180;
      }
      if(ang1[1] < 0){
        ang1[1] = 0;
      }
      
      ang1[3] = ang1[3] + rx;
      if(ang1[3] > 180){
        ang1[3] = 180;
      }
      if(ang1[3] < 0){
        ang1[3] = 0;
      }
    }else if(joyLSW == 1)
    {
      ang1[4] = ang1[4] + ry;
      if(ang1[4] > 180){
        ang1[4] = 180;
      }
      if(ang1[4] < 0){
        ang1[4] = 0;
      }
      
      ang1[5] = ang1[5] + rx;
      if(ang1[5] > 180){
        ang1[5] = 180;
      }
      if(ang1[5] < 0){
        ang1[5] = 0;
      }
    }
  }else if(move_mode == -1)
  {
    if(abs(my) > cont_thres){
      if(mx > cont_thres){  //前輪旋回
        mRF = my;
        mRR = 0;
        mLF = -my;
        mLR = 0;
      }else if(mx < -cont_thres){  //後輪旋回
        mRF = 0;
        mRR = my;
        mLF = 0;
        mLR = -my;
      }else{  //超信地旋回
        mRF = my;
        mRR = my;
        mLF = -my;
        mLR = -my;
      }
    }else if(cont_thres2 < my && my < cont_thres){
      if(mx > cont_thres){  //右前信地旋回
        mRF = mx;
        mRR = mx;
        mLF = 0;
        mLR = 0;
      }else if(mx < -cont_thres){  //右信地旋回
        mRF = 0;
        mRR = 0;
        mLF = mx;
        mLR = mx;
      }
    }else if(-cont_thres < my && my < -cont_thres2){
      if(mx > cont_thres){    //左前信地旋回
        mRF = 0;
        mRR = 0;
        mLF = mx;
        mLR = mx;
      }else if(mx < -cont_thres){  //左後信地旋回
        mRF = mx;
        mRR = mx;
        mLF = 0;
        mLR = 0;
      }
    }else if(abs(my) <= cont_thres2 && abs(mx)>cont_thres){  //前後進
      mRF = mx;
      mRR = mx;
      mLF = mx;
      mLR = mx;
    }else{
      mRF = 0;
      mRR = 0;
      mLF = 0;
      mLR = 0;
    }
  }else if(move_mode == 0){
    if(my < -cont_thres){
      if(mx > cont_thres){  //右斜め前
        mRF = 0;
        mRR = -my;
        mLF = -my;
        mLR = 0;
      }else if(mx < -cont_thres){  //右斜め後
        mRF = my;
        mRR = 0;
        mLF = 0;
        mLR = my;
      }else{  //右スライド
        mRF = my;
        mRR = -my;
        mLF = -my;
        mLR = my;
      }
    }else if(my > cont_thres){
      if(mx > cont_thres){  //左斜め前
        mRF = my;
        mRR = 0;
        mLF = 0;
        mLR = my;
      }else if(mx < -cont_thres){  //左斜め後
        mRF = 0;
        mRR = -my;
        mLF = -my;
        mLR = 0;
      }else{  //左スライド
        mRF = my;
        mRR = -my;
        mLF = -my;
        mLR = my;
      }
    }else if(abs(mx) > cont_thres){   //前後進
      mRF = mx;
      mRR = mx;
      mLF = mx;
      mLR = mx;
    }else{
      mRF = 0;
      mRR = 0;
      mLF = 0;
      mLR = 0;
    }
  }
  
  servo_set();
  motor_drive(-mRF, -mRR, mLF, mLR);

}

void setupBLE() {
  BLEDevice::init("NX20_M5Stamp");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharTx = pService->createCharacteristic(CHTX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharRx = pService->createCharacteristic(CHRX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharRx ->setCallbacks(new MyCallbacks());
  pCharTx->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void checkBLE() {
    // notify changed value
  if (deviceConnected) {
      pCharTx->setValue((uint8_t*)&value, 8);
      pCharTx->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}
