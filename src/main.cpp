#include <Arduino.h>
#include <CAN.h>//えすぺ内臓can
#include <PS4Controller.h>

//========================構造体宣言============================
typedef struct{//足回り構造体
  int16_t motor0=0;
  int16_t motor1=0;
  int16_t motor2=0;
  int16_t motor3=0;
  uint16_t m_id=3;
  uint16_t m_speed=0;
  int m_deg=0;
}ashimawari;

typedef struct{//センサー基盤構造体
  uint16_t s_id1=10;
  uint16_t s_id2=11;
  uint16_t rotaen0=0;
  uint16_t rotaen1=0;
  uint16_t rotaen2=0;
  uint16_t rotaen3=0;
  uint16_t rotaen4=0;
  uint8_t rimit0=0;
  uint8_t rimit1=0;
  uint8_t rimit2=0;
  uint8_t rimit3=0;
  uint8_t rimit4=0;
}sensorkiban;

typedef struct{//サーボ基板構造体
  uint16_t v_id=144;
  int servo0=0;
  int servo1=120;
  int  servo2=137;
  int servo3=0;
  int  servo4=0;
  int servo5=0;
  int servo6=0;
  int servo7=0;
}savokiban;
//=============================================================

ashimawari asma;
sensorkiban seki;
savokiban voba;

//===========================================関数宣言======================================
int voba_move(savokiban sk){//サーボデーター送信
  CAN.beginPacket(sk.v_id);
  CAN.write((unsigned char)sk.servo0);
  CAN.write((unsigned char)sk.servo1);
  CAN.write((unsigned char)sk.servo2);
  CAN.write((unsigned char)sk.servo3);
  CAN.write((unsigned char)sk.servo4);
  CAN.write((unsigned char)sk.servo5);
  CAN.write((unsigned char)sk.servo6);
  CAN.write((unsigned char)sk.servo7);
  return CAN.endPacket();
}

int motor_move(int16_t id,int16_t m1,int16_t m2,int16_t m3,int16_t m4){//モーターデーター送信
  CAN.beginPacket(id);
  CAN.write(m1&0xff);
  CAN.write(m1>>8);
  CAN.write(m2&0xff);
  CAN.write(m2>>8);
  CAN.write(m3&0xff);
  CAN.write(m3>>8);
  CAN.write(m4&0xff);
  CAN.write(m4>>8);
  return CAN.endPacket();
}

int asma_move(ashimawari *am){//足回り
  am->motor0=sin(am->m_deg*PI/180)*am->m_speed;
  am->motor1=sin((am->m_deg+90)*PI/180)*am->m_speed;
  am->motor2=sin((am->m_deg+180)*PI/180)*am->m_speed;
  am->motor3=sin((am->m_deg+270)*PI/180)*am->m_speed;
  return motor_move(am->m_id,am->motor0,am->motor1,am->motor2,am->motor3);
}

void Sensor(int packetSize){//センサーデーター受信
  if(CAN.packetId()==seki.s_id1){
    seki.rotaen0=CAN.read()<<8|CAN.read();
    seki.rotaen1=CAN.read()<<8|CAN.read();
    seki.rotaen2=CAN.read()<<8|CAN.read();
    seki.rotaen3=CAN.read()<<8|CAN.read();
  }
  if(CAN.packetId()==seki.s_id2){
    seki.rotaen4=CAN.read()<<8|CAN.read();
    seki.rimit0=CAN.read();
    seki.rimit1=CAN.read();
    seki.rimit2=CAN.read();
    seki.rimit3=CAN.read();
    seki.rimit4=CAN.read();
  }
}
//========================================================================================

void setup() {
  Serial.begin(115200);
  PS4.begin("ec:94:cb:6f:cc:8a");//ec:94:cb:6f:cc:8a
  
  CAN.setPins(4,5);//( rx , tx )
  CAN.begin(1E6);
  CAN.onReceive(Sensor);
}
//===========================変数宣言=============================
uint16_t k1_id=1;//機構モーター1
uint16_t k2_id=2;//機構モーター2
uint16_t k3_id=4;//機構モーター3

unsigned char hassya=1;
boolean sp=0;
boolean soutenL=0;
boolean soutenR=0;
boolean servo_ena=0;
signed char arm_updw=0;
signed char  hata_grab=0;
signed char arm_zengo=0;
signed char  gyouten=0;
signed char syoukou=0;

boolean triangleb=0;
boolean shareb=0;
boolean r3b=0;
boolean crossb=0;
//===============================================================

unsigned char LRanalog_bit(unsigned char lb){
  if(lb>100){
    return 1;
  }else{
    return 0;
  }
}

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
    if (PS4.isConnected()) {

      //====================================足回り===========================================
      if(PS4.Share()^shareb){//回転スピード調整
        shareb=PS4.Share();
        if(shareb)sp=!sp;
      }
      
      if(PS4.LStickX()>10 or PS4.LStickY()>10 or PS4.LStickX()<-10 or PS4.LStickY()<-10 ){//足回り移動
        asma.m_deg=135-atan2(PS4.LStickX(),PS4.LStickY())*180/PI;
        asma.m_speed=hypot(PS4.LStickX(),PS4.LStickY())*60;
        asma_move(&asma);
        
      }else{
        if(PS4.RStickX()>10 or PS4.RStickX()<-10){//足回り回転
              motor_move(asma.m_id,PS4.RStickX()*(-40+sp*20),PS4.RStickX()*(-40+sp*20),PS4.RStickX()*(-40+sp*20),PS4.RStickX()*(-40+sp*20));
          }else{
              motor_move(asma.m_id,0,0,0,0);
          }
      }
      //====================================================================================

      //==========================機構関係=================================
      if(PS4.Up()){//仰角を上にする
        gyouten=1;
      }else{
          if(PS4.Down()){//仰角を下にする
            gyouten=(signed char)-1;
          }else{
                gyouten=0;
          }
      }
    
      if(PS4.Cross()^crossb){//発射機構のモータを回す
        crossb=PS4.Cross();
        if(crossb)hassya=hassya<<1;
        if(hassya==B1000)hassya=1;
      }
      
      soutenL=PS4.L2();//発射機構の左を装填

      soutenR=PS4.R2();//発射機構の右を装填

      hata_grab=PS4.Square();

      syoukou=PS4.Circle();

      if(PS4.Right()){//仰角を上にする
        arm_zengo=(signed char)-1;
      }else{
          if(PS4.Left()){//仰角を下にする
            arm_zengo=1;
          }else{
                arm_zengo=0;
          }
      }
      
      if(PS4.Triangle()^triangleb){//回転スピード調整
        triangleb=PS4.Triangle();
        if(triangleb)arm_updw=!arm_updw;
      }


      if(PS4.Options()){
        voba.servo2=137;
        voba.servo1=120;
      }

      //===================================================================

      //==========カメラ首振り============
      if(PS4.L1()){
        voba.servo2+=2;
        if(voba.servo2>255)voba.servo2=255;
      }
      if(PS4.R1()){
        voba.servo2-=2;
        if(voba.servo2<0)voba.servo2=0;
      }
      //=================================

      //============カメラ立て振り=============
      if(PS4.R3()^r3b){
        r3b=PS4.R3();
        if(r3b)servo_ena=!servo_ena;
      }
      if(servo_ena){
        if(PS4.RStickY()>10 or PS4.RStickY()<-10){
          voba.servo1+=PS4.RStickY()/50;
          if(voba.servo1>120)voba.servo1=120;//正面
          if(voba.servo1<55)voba.servo1=55;//下向き
        }
      }
      //======================================
      
      //Serial.printf("s1=%d,s2=%d,%d,l2=%d,r2=%d,%d\n",voba.servo1,voba.servo2,servo_ena,PS4.L2(),PS4.R2(),gyouten);
      //Serial.printf("%d,%d,%d,%d,%d\n",rimit0,rimit1,rimit2,rimit3,rimit4);
      
      //=============================CANCANするところ=========================
      voba_move(voba);
      
      motor_move(k2_id,
                              8000*gyouten,
                              syoukou*-10000,
                              arm_zengo*10000,
                              hata_grab*-10000
      );
      motor_move(k1_id,
                             soutenR*-10000,
                             soutenL*10000,
                             -13000*((hassya>>1)&1)-15000*((hassya>>2)&1),
                             0
      );
      motor_move(k3_id,
                             0,
                             (signed char)((arm_updw*2)-1)*10000*PS4.Triangle(),
                             0,
                             0
      );
      //=======================================================================
    }
  delay(20);

}
