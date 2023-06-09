#include <Arduino.h>
#include <CAN.h>           //えすぺ内臓can
#include <PS4Controller.h> //えすぺ内臓bluetooth

#define CAN_Enable 1 // CANCANするかどうか
// boolean ROS_Enable=1;//ROSに託すかどうか

#define RB_Cross ((JM.buttons_1 >> 0) & 1)
#define RB_Circle ((JM.buttons_1 >> 1) & 1)
#define RB_Square ((JM.buttons_1 >> 2) & 1)
#define RB_Triangle ((JM.buttons_1 >> 3) & 1)
#define RB_Share ((JM.buttons_1 >> 4) & 1)
#define RB_PS ((JM.buttons_1 >> 5) & 1)
#define RB_Option ((JM.buttons_1 >> 6) & 1)

#define RB_L1 ((JM.buttons_2 >> 1) & 1)
#define RB_R1 ((JM.buttons_2 >> 2) & 1)
#define RB_Up ((JM.buttons_2 >> 3) & 1)
#define RB_Down ((JM.buttons_2 >> 4) & 1)
#define RB_Left ((JM.buttons_2 >> 5) & 1)
#define RB_Right ((JM.buttons_2 >> 6) & 1)

#define RIMIT0 12
#define RIMIT1 14
#define RIMIT2 27
#define RIMIT3 26
#define RIMIT4 25
#define RIMIT5 33
#define RIMIT6 32
#define RIMIT7 23
#define RIMIT8 22

#define AUTOREFULY 21

#define POTEN0 39
#define POTEN1 36

//========================構造体宣言============================
typedef struct
{ // 足回り構造体
  int16_t motor0 = 0;
  int16_t motor1 = 0;
  int16_t motor2 = 0;
  int16_t motor3 = 0;
  uint16_t m_id = 3;
  uint16_t m_speed = 0;
  int m_deg = 0;
} ashimawari;

typedef struct
{ // センサー基盤構造体
  uint16_t s_id1 = 10;
  uint16_t s_id2 = 11;
  uint64_t timestamp = 0;
  uint16_t rotaen0 = 0;
  uint16_t rotaen1 = 0;
  uint16_t rotaen2 = 0;
  uint16_t rotaen3 = 0;
  uint16_t rotaen4 = 0;
  uint8_t rimit0 = 0; // つかむ
  uint8_t rimit1 = 0; // アーム昇降↑
  uint8_t rimit2 = 0; // アーム昇降↓
  uint8_t rimit3 = 0; // アーム展開
  uint8_t rimit4 = 0; // 仰角↓
} sensorkiban;

typedef struct
{ // センサー基盤構造体
  uint16_t s_id1 = 12;
  uint16_t s_id2 = 13;
  uint64_t timestamp = 0;
  uint16_t rotaen0 = 0;
  uint16_t rotaen1 = 0;
  uint16_t rotaen2 = 0;
  uint16_t rotaen3 = 0;
  uint16_t rotaen4 = 0;
  uint8_t rimit0 = 0; // 右発射
  uint8_t rimit1 = 0; // 左発射
  uint8_t rimit2 = 0; //
  uint8_t rimit3 = 0; //
  uint8_t rimit4 = 0; // カメラ展開
} sensorkiban2;

typedef struct
{ // サーボ基板構造体
  uint16_t v_id = 144;
  int servo0 = 0;
  int servo1 = 120;
  int servo2 = 137;
  int servo3 = 0;
  int servo4 = 0;
  int servo5 = 0;
  int servo6 = 0;
  int servo7 = 0;
} savokiban;

struct JoyMsg // ROSからコントローラーの信号を得る
{
  int8_t left_stick_x;
  int8_t left_stick_y;
  int8_t right_stick_x;
  int8_t right_stick_y;
  uint8_t l2;
  uint8_t r2;
  uint8_t buttons_1;
  uint8_t buttons_2;
} __attribute__((packed));

/**
struct TargetVelocityMsg {//ID:20
  int16_t vx; // 前後方向の速度[m/s] * 1000 前が+　後ろが-
  int16_t vy; //　左右方向の速度[m/s] * 1000 左が+ 右が-
  int16_t ang_vel; // 回転速度[rad/s] * 1000 左旋回が+ 右旋回が-
} __attribute__((packed));

struct CameraAngleMsg {//ID:21
  int16_t pitch; // 上下方向の角度[rad] * 1000 上が-　下が+
  int16_t yaw; // 左右方向の角度[rad] * 1000 左が+ 右が-
} __attribute__((packed));

struct FireCommandMsg {//ID23
  bool enable; // 発射を行うかどうか trueなら発射する
} __attribute__((packed));

struct CameraLiftMsg {//ID24
  int16_t command; // カメラ展開機構の上下　上が+ 下が-
} __attribute__((packed));

struct ArmControlMsg {//ID25
  int16_t lift_command; // 旗回収機構の上下展開 上が+　下が-　リミットスイッチにあったたら止める
  int16_t grabber_command; // 旗回収の掴む機構　そのままモータの出力に渡す
} __attribute__((packed));
**/

//=============================================================

/**
TargetVelocityMsg TVM={0,0,0};
CameraAngleMsg CAM={0,0};
FireCommandMsg FCM={0};
CameraLiftMsg CLM={0};
ArmControlMsg ACM={0,0};
**/

JoyMsg JM = {0, 0, 0, 0, 0, 0, 0, 0};

ashimawari asma;
sensorkiban seki;
sensorkiban2 seki2;
savokiban voba;

//===========================変数宣言=============================
uint16_t k1_id = 1; // 機構モーター1
uint16_t k2_id = 2; // 機構モーター2
uint16_t k3_id = 4; // 機構モーター3

unsigned char soutenL = 0; // 装填左 trueで回る
unsigned char soutenR = 0; // 装填右 trueで回る

// ボタン判定用
boolean triangleb = 0;
boolean shareb = 0;
boolean crossb = 0;
boolean squareb = 0;
boolean L1b = 0;
boolean R1b = 0;
boolean psb = 0;
boolean sp = 1;

signed char arm_updw = 0;  // アーム昇降
signed char arm_zengo = 0; // アーム前後
signed char gyoukaku = 0;  // 仰角
signed char zandan = 0;
signed char zandan1 = 0;

unsigned char syoukou = 0;   // カメラ昇降
unsigned char hassya = 1;       // 発射
unsigned char arm_updw_rim = 0; // アームを1ボタンで上下させるためのやつ（語彙力）
unsigned char hata_grab = 0;    // つかむ

int16_t aps=0;
//===============================================================

//===========================================関数宣言======================================
int voba_move(savokiban sk) // サーボデーター送信
{
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

void zandan_send(unsigned char zandan)
{
  CAN.beginPacket(22);
  CAN.write(zandan);
  CAN.endPacket();
  return;
}

int motor_move(int16_t id, int16_t m1, int16_t m2, int16_t m3, int16_t m4) // モーターデーター送信
{
#ifdef CAN_Enable
  if (m1 > 0x3fff)
    m1 = 0x3fff;
  if (m1 < -0x3fff)
    m1 = -0x3fff;
  if (m2 > 0x3fff)
    m2 = 0x3fff;
  if (m2 < -0x3fff)
    m2 = -0x3fff;
  if (m3 > 0x3fff)
    m3 = 0x3fff;
  if (m3 < -0x3fff)
    m3 = -0x3fff;
  if (m4 > 0x3fff)
    m4 = 0x3fff;
  if (m4 < -0x3fff)
    m4 = -0x3fff;

  CAN.beginPacket(id);
  CAN.write(m1 & 0xff);
  CAN.write(m1 >> 8);
  CAN.write(m2 & 0xff);
  CAN.write(m2 >> 8);
  CAN.write(m3 & 0xff);
  CAN.write(m3 >> 8);
  CAN.write(m4 & 0xff);
  CAN.write(m4 >> 8);
  CAN.endPacket();
#endif

  return 0;
}

int asma_move(ashimawari *am) // 足回り
{
  int kaiten;
  kaiten = ((JM.r2 * -1) + JM.l2) * (8 + sp * 8);
  am->motor0 = sin(am->m_deg * PI / 180) * am->m_speed;
  am->motor1 = sin((am->m_deg + 90) * PI / 180) * am->m_speed;
  am->motor2 = sin((am->m_deg + 180) * PI / 180) * am->m_speed;
  am->motor3 = sin((am->m_deg + 270) * PI / 180) * am->m_speed;
  return motor_move(am->m_id, am->motor0 + kaiten, am->motor1 + kaiten, am->motor2 + kaiten, am->motor3 + kaiten);
}

void Sensor(int packetSize) // CANデーター受信
{
//#ifdef CAN_Enable
//   unsigned char _rim0, _rim1;
//   if (CAN.packetId() == seki.s_id1)
//   {
//     seki.timestamp = CAN.read() | (CAN.read() << 8) | (CAN.read() << 16) | (CAN.read() << 24) | (CAN.read() << 32);
//     _rim0 = CAN.read();
//     seki.rotaen0 = CAN.read() << 8 | CAN.read();

//     seki.rimit0 = _rim0 & 1;
//     seki.rimit1 = (_rim0 >> 1) & 1;
//     seki.rimit2 = (_rim0 >> 2) & 1;
//     seki.rimit3 = (_rim0 >> 3) & 1;
//     seki.rimit4 = (_rim0 >> 4) & 1;
//   }

//   if (CAN.packetId() == seki.s_id2)
//   {
//     seki.rotaen1 = CAN.read() << 8 | CAN.read();
//     seki.rotaen2 = CAN.read() << 8 | CAN.read();
//     seki.rotaen3 = CAN.read() << 8 | CAN.read();
//     seki.rotaen4 = CAN.read() << 8 | CAN.read();
//   }

//   if (CAN.packetId() == seki2.s_id1)
//   {
//     seki2.timestamp = CAN.read() | (CAN.read() << 8) | (CAN.read() << 16) | (CAN.read() << 24) | (CAN.read() << 32);
//     _rim1 = CAN.read();
//     seki2.rotaen0 = CAN.read() << 8 | CAN.read();

//     seki2.rimit0 = _rim1 & 1;
//     seki2.rimit1 = (_rim1 >> 1) & 1;
//     seki2.rimit2 = (_rim1 >> 2) & 1;
//     seki2.rimit3 = (_rim1 >> 3) & 1;
//     seki2.rimit4 = (_rim1 >> 4) & 1;
//   }
//   if (CAN.packetId() == seki2.s_id2)
//   {
//     seki2.rotaen1 = CAN.read() << 8 | CAN.read();
//     seki2.rotaen2 = CAN.read() << 8 | CAN.read();
//     seki2.rotaen3 = CAN.read() << 8 | CAN.read();
//     seki2.rotaen4 = CAN.read() << 8 | CAN.read();
//   }
// #endif

  if (CAN.packetId() == 20)
  {
    JM.left_stick_x = CAN.read() * -1;
    JM.left_stick_y = CAN.read();
    JM.right_stick_x = CAN.read() * -1;
    JM.right_stick_y = CAN.read();
    JM.l2 = CAN.read();
    JM.r2 = CAN.read();
    JM.buttons_1 = CAN.read();
    JM.buttons_2 = CAN.read();
  }
  if (CAN.packetId() == 26){
    aps=CAN.read()|(CAN.read()<<8);

    voba.servo0=CAN.read();
  }

  // if(CAN.packetId()==20){
  //   TVM.vx=CAN.read()|(CAN.read()<<8);
  //   TVM.vy=CAN.read()|(CAN.read()<<8);
  //   TVM.ang_vel=CAN.read()|(CAN.read()<<8);
  // }
  // if(CAN.packetId()==21){
  //   CAM.pitch=CAN.read()|(CAN.read()<<8);
  //   CAM.yaw=CAN.read()|(CAN.read()<<8);
  // }
  // if(CAN.packetId()==23){
  //   FCM.enable=CAN.read();
  // }
  // if(CAN.packetId()==24){
  //   CLM.command=CAN.read()|(CAN.read()<<8);
  // }
  // if(CAN.packetId()==25){
  //   ACM.grabber_command=CAN.read()|(CAN.read()<<8);
  //   ACM.lift_command=CAN.read()|(CAN.read()<<8);
  // }
}
//zandan_calculate(analogRead(POTEN0), 3680, 131) + zandan_calculate(analogRead(POTEN1), 2210, 110);

  //R 2210 POTEN1
  //L 3680 POTEN0

int8_t zandan_calculateR(uint16_t value)
{
  return (value - 2210-55) / 110;
}

int8_t zandan_calculateL(uint16_t value)
{
  return (3680 - value-65) / 131;
}
//========================================================================================

void setup()
{
  pinMode(RIMIT0, INPUT_PULLUP);
  pinMode(RIMIT1, INPUT_PULLUP);
  pinMode(RIMIT2, INPUT_PULLUP);
  pinMode(RIMIT3, INPUT_PULLUP);
  pinMode(RIMIT4, INPUT_PULLUP);
  pinMode(RIMIT5, INPUT_PULLUP);
  pinMode(RIMIT6, INPUT_PULLUP);
  pinMode(RIMIT7, INPUT_PULLUP);
  pinMode(RIMIT8, INPUT_PULLUP);
  pinMode(POTEN0, INPUT);
  pinMode(POTEN1, INPUT);

  Serial.begin(115200); // デバッグ用シリアル

  // PS4.begin("70:B8:F6:5B:51:56");//練習機
  PS4.begin("ec:94:cb:6f:cc:8a"); // 本番機

#ifdef CAN_Enable
  CAN.setPins(4, 5);     //( RX , TX )
  CAN.begin(1E6);        // 1MHzで通信
  CAN.onReceive(Sensor); // CAN受信割り込み
#endif
}

void loop()
{
  if (PS4.isConnected())
  {
    JM.left_stick_x = PS4.LStickX();
    JM.left_stick_y = PS4.LStickY();
    JM.right_stick_x = PS4.RStickX();
    JM.right_stick_y = PS4.RStickY();
    JM.l2 = PS4.L2Value();
    JM.r2 = PS4.R2Value();
    JM.buttons_1 = (PS4.Cross() << 0) | (PS4.Circle() << 1) | (PS4.Square() << 2) | (PS4.Triangle() << 3) | (PS4.Share() << 4) | (PS4.PSButton() << 5) | (PS4.Options() << 6);
    JM.buttons_2 = (PS4.L1() << 1) | (PS4.R1() << 2) | (PS4.Up() << 3) | (PS4.Down() << 4) | (PS4.Left() << 5) | (PS4.Right() << 6);
  }

  seki.rimit0 = 1 - digitalRead(RIMIT0);
  seki.rimit1 = 1 - digitalRead(RIMIT1);
  seki.rimit2 = 1 - digitalRead(RIMIT2);
  seki.rimit3 = 1 - digitalRead(RIMIT3);
  seki.rimit4 = 1 - digitalRead(RIMIT4);
  seki2.rimit0 = 1 - digitalRead(RIMIT5);
  seki2.rimit1 = 1 - digitalRead(RIMIT6);
  seki2.rimit2 = 1 - digitalRead(RIMIT7);
  seki2.rimit3 = 1 - digitalRead(RIMIT8);

  if (digitalRead(AUTOREFULY))
  {
    //====================================足回り===========================================
    if (RB_Share ^ shareb)
    { // 回転スピード調整
      shareb = RB_Share;
      if (shareb)
        sp = !sp;
    }

    if (JM.left_stick_x > 10 or JM.left_stick_y > 10 or JM.left_stick_x < -10 or JM.left_stick_y < -10 or JM.l2 > 10 or JM.r2 > 10)
    { // 足回り移動
      asma.m_deg = 135 - atan2(JM.left_stick_x, JM.left_stick_y) * 180 / PI;
      asma.m_speed = hypot(JM.left_stick_x, JM.left_stick_y) * 67;
      asma_move(&asma);
    }
    else
    {
      motor_move(asma.m_id, 0, 0, 0, 0);
    }
    //====================================================================================

    //==========================機構関係================================
    if (RB_Up and seki.rimit4 == 0)
    { // 仰角を上にする
      gyoukaku = 1;
    }
    else
    {
      if (RB_Down)
      { // 仰角を下にする
        gyoukaku = (signed char)-1;
      }
      else
      {
        gyoukaku = 0;
      }
    }

    if (RB_Cross ^ crossb)
    { // 発射機構のモータを回す
      crossb = RB_Cross;
      if (crossb)
        hassya = hassya << 1;
      if (hassya == 0b1000)
        hassya = 1;
    }

    if (RB_L1 ^ L1b)
    { // 発射機構の左を装填
      L1b = RB_L1;
      if (L1b and soutenL == 0)
        soutenL = 1;
    }
    if (seki2.rimit1)
    {
      if (soutenL == 1)
      {
        soutenL = 0b11;
      }
    }
    else
    {
      if (soutenL == 0b11)
        soutenL = 0b0;
    }

    if (RB_R1 ^ R1b)
    { // 発射機構の右を装填
      R1b = RB_R1;
      if (R1b and soutenR == 0)
        soutenR = 1;
    }
    if (seki2.rimit0)
    {
      if (soutenR == 1)
      {
        soutenR = 0b11;
      }
    }
    else
    {
      if (soutenR == 0b11)
        soutenR = 0b0;
    }

    //==================アーム関係=====================

    if (RB_Square ^ squareb)
    { // 発射機構のモータを回す
      squareb = RB_Square;
      if (squareb)
      {
        if (hata_grab == 0)
          hata_grab = 0b1;
      }
      else
      {
        if (hata_grab == 0b10)
          hata_grab = 0b101;
        if (hata_grab == 0b1000)
          hata_grab = 0;
      }
    }

    if (seki.rimit0)
    {
      if (hata_grab == 0b1)
        hata_grab = 0b10;
    }
    else
    {
      if (hata_grab == 0b101)
        hata_grab = 0b1000;
    }

    syoukou = RB_Circle*(1-seki2.rimit3); // カメラ昇降

    if(seki2.rimit3==1){
      syoukou|=0b10;
    }

    if (RB_Right and seki.rimit3 == 0)
    { // アームを前方
      arm_zengo = (signed char)-1;
    }
    else
    {
      if (RB_Left)
      { // アーム後方
        arm_zengo = 1;
      }
      else
      {
        arm_zengo = 0;
      }
    }

    if (seki.rimit1 or seki.rimit2)
    {
      if (arm_updw_rim == 0)
        arm_updw_rim = 0b1;
    }
    else
    {
      if (arm_updw_rim == 0b10)
        arm_updw_rim = 0;
    }

    if (RB_Triangle ^ triangleb)
    { // アーム昇降
      triangleb = RB_Triangle;
      if (triangleb)
      {
        arm_updw = !arm_updw;
      }
      else
      {
        if (arm_updw_rim == 0b1)
          arm_updw_rim = 0b10;
      }
    }
    //================================================

    //====================================================================

    //===========================カメラ振り振り=============================

    // 開く137   閉じる 30

    if (RB_Option)
    { // カメラ初期位置
      voba.servo2 = 137;
      voba.servo1 = 120;
    }

    if (JM.right_stick_y > 10 or JM.right_stick_y < -10)
    {
      voba.servo1 += JM.right_stick_y / 50;
      if (voba.servo1 > 120)
        voba.servo1 = 120; // 正面
      if (voba.servo1 < 55)
        voba.servo1 = 55; // 下向き
    }

    if (JM.right_stick_x > 10 or JM.right_stick_x < -10)
    {
      voba.servo2 -= JM.right_stick_x / 50;
      if (voba.servo2 > 255)
        voba.servo2 = 255; // 右
      if (voba.servo2 < 0)
        voba.servo2 = 0; // 左
    }
    //=====================================================================
    //     if (RB_Up)
    //   voba.servo0++;
    // if (RB_Down)
    //   voba.servo0--;
    //=============================CANCANするところ=========================

    voba_move(voba);
    motor_move(k2_id,
               8000 * gyoukaku,
               (syoukou&1) * -9000,
               arm_zengo * 15000,
               RB_Square * (hata_grab & 1) * -10000);
    motor_move(k1_id,
               RB_R1 * (soutenR & 1) * -9000,
               RB_L1 * (soutenL & 1) * 9000,
               -13000 * ((hassya >> 1) & 1) - 15000 * ((hassya >> 2) & 1),
               aps);
    motor_move(k3_id,
               0,
               (signed char)((arm_updw * 2) - 1) * 10000 * RB_Triangle * (1 - (arm_updw_rim & 1)),
               0,
               0);
    //=====================================================================

  }

  // 左右

    //R 2210 POTEN1
  //L 3680 POTEN0

  zandan=zandan_calculateR(analogRead(POTEN0)) + zandan_calculateL(analogRead(POTEN1));

  if (zandan ^ zandan1)//残弾数更新
  {
    zandan1 = zandan;
    zandan_send(zandan1);
  }
  /*
  voba.servo1=0x7f-((CAM.pitch*0x7f/PI)/1000)+0x7f;//上下
  voba.servo2=((CAM.yaw*0x7f/PI)/1000)+0x7f;//左右
  voba_move(voba);

  motor_move(k3_id,
                          0,
                          0,
                          0,
                          0
    );

  motor_move(k2_id,
                          0,
                          CLM.command,
                          -ACM.lift_command*(1-seki.rimit3),
                          ACM.grabber_command
  );

  motor_move(k1_id,
                          FCM.enable*-10000,//装填右
                          FCM.enable*10000,//装填左
                          FCM.enable*-13000,//発射
                          0
  );

  asma.m_deg=atan2(TVM.vy,TVM.vx)*180/PI+45;
  asma.m_speed=hypot(TVM.vx,TVM.vy)*5;
  asma_move(&asma);
  */

  //Serial.printf("POTENTION<%d,%d><%d,%d>\n",analogRead(POTEN0),analogRead(POTEN1),zandan_calculateR(analogRead(POTEN0)) , zandan_calculateL(analogRead(POTEN1)));
  // Serial.printf("%d,%d",analogRead(POTEN0),analogRead(POTEN1));
  // Serial.printf("deg=%d,speed=%d,%d,%d\n",asma.m_deg,asma.m_speed,JM.l2,JM.r2);
  // Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",JM.left_stick_x,JM.left_stick_y,JM.right_stick_x,JM.right_stick_y,JM.l2,JM.r2,JM.buttons_1,JM.buttons_2);
  // Serial.printf("SERVO<%d,%d,%d>\n", voba.servo0, voba.servo1, voba.servo2);
  Serial.printf("SWITCH<%d,%d,%d,%d,%d><%d,%d,%d,%d,%d>,%d,%d\n", seki.rimit0, seki.rimit1, seki.rimit2, seki.rimit3, seki.rimit4, seki2.rimit0, seki2.rimit1, seki2.rimit2, seki2.rimit3, seki2.rimit4);
  // Serial.printf("%d,%d,pitch=%d,yaw=%d\n",voba.servo1,voba.servo2,CAM.pitch,CAM.yaw);//ROSからのデーター（カメラ回転）

  delay(20);
}