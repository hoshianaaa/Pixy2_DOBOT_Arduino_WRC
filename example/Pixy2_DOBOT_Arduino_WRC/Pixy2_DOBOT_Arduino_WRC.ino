/****************************************Copyright(c)*****************************************************
**                                      TechShare Inc
**
**                                  https://techshare.co.jp/
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           DOBOT_wrc_pixy.2.2
** Latest modified Date:2016-06-01
** Latest Version:      V1.0.0
** Descriptions:        Demo main program
**
**--------------------------------------------------------------------------------------------------------
** Created by:          RyotaroHoshina
** Created date:        2019-03-13
** Version:             V1.0.0
** Descriptions:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
/*動作手順
   0.dobotをhomeで修正
   1.MODEを"5"にして、EEPROMをリセット
   2.MODEを"1"にしてdemoをスタート

*/

////////////////////////////////////////////////////////////////////////////////
//MODE (1 demo, 2 test用, 3 stop, 4 colormove, 5 EEPROM_CLEAR, 6 EEPROM_watch)//
////////////////////////////////////////////////////////////////////////////////

const int MODE = 1;

/////////////////////////////////////////////////
//pinなど
const int LIMIT_SWITCH_PIN = 4, TRIGGER_PIN = 5;
const int RED = 1, GREEN = 2, BLUE = 3, YELLO = 4; //pixyで登録した色のsigunature番号を入力


//postion
const int DEMO_HOME_POS[3] = {200, 0, 20};//pixyでブロックの座標を取得する位置 default:{200, 0, 20}
const int LIMIT_SWITCH_POS[3] = {215, 20, 35};
const int LIMIT_SWITCH_DOWN = 15;

#include <SPI.h>
#include <Pixy2_rm_zumo.h>
#include <EEPROM.h>

Pixy2 pixy;
#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

/*********************************************************************************************************
** Global parameters
*********************************************************************************************************/
PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

uint64_t gQueuedCmdIndex;
/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();

  //ledピン設定
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(6, INPUT);
  pixy.init();

  gPTPCmd.x = 200;
  gPTPCmd.y = 0;
  gPTPCmd.z = 0;
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
  ProtocolProcess(); Serial.println("/////   0    //////");
  delay(2000);
}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void Serialread()
{
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
      RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
    }
  }
}
/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void printf_begin(void)
{
  fdevopen( &Serial_putc, 0 );
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void InitRAM(void)
{


  //Set PTP Model
  gPTPCoordinateParams.xyzVelocity = 100;
  gPTPCoordinateParams.rVelocity = 100;
  gPTPCoordinateParams.xyzAcceleration = 80;
  gPTPCoordinateParams.rAcceleration = 80;

  gPTPCommonParams.velocityRatio = 50;
  gPTPCommonParams.accelerationRatio = 50;

  gPTPCmd.ptpMode = MOVJ_XYZ;
  gPTPCmd.x = 200;
  gPTPCmd.y = 0;
  gPTPCmd.z = 50;
  gPTPCmd.r = 0;

  gQueuedCmdIndex = 0;


}

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age > 30)
    return pixy.ccc.blocks[0].m_index;

  return -1;
}


// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block *trackBlock(uint8_t index)
{
  uint8_t i;

  for (i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (index == pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}






/*********************************************************************************************************
** Function name:       PTPMove
** Descriptions:        指定した座標に移動
** Input parameters:    x座標,y座標,z座標,on/off
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMove(int x, int y, int z, bool on) {

  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  if (on & 0x01)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
  else SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

  ProtocolProcess();
}


/*********************************************************************************************************
** Function name:       PTPMoveHome
** Descriptions:        homeポジションに移動
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMoveHome() {

  gPTPCmd.x = DEMO_HOME_POS[0];//5行目あたりで宣言
  gPTPCmd.y = DEMO_HOME_POS[1];
  gPTPCmd.z = DEMO_HOME_POS[2];
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  // if (on & 0x01)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
  // else SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

  ProtocolProcess();
}

/*********************************************************************************************************
** Function name:       PTPMovePos
** Descriptions:        3次元配列で指定した座標に移動
** Input parameters:    postion,suctioncup_on,off
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMovePos(int pos[3], bool on) {

  gPTPCmd.x = pos[0];
  gPTPCmd.y = pos[1];
  gPTPCmd.z = pos[2];
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  if (on & 0x01)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
  else SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

  ProtocolProcess();
}

/*********************************************************************************************************
** Function name:       PTPMoveLimitSwitch
** Descriptions:        3次元配列で指定した座標に移動
** Input parameters:    postion,suctioncup_on,off
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
bool PTPMoveLimitSwitch() {
  SuctionLoop(1);
  delay(500);
  int val = digitalRead(LIMIT_SWITCH_PIN);
  int result;
  PTPMovePos(LIMIT_SWITCH_POS, true);
  delay(500);
  PTPMoveDelta(0, 0, -LIMIT_SWITCH_DOWN, 1);
  delay(500);
  SuctionLoop(10);
  delay(1500);
  if (val == digitalRead(LIMIT_SWITCH_PIN))result = false;
  else result = true;
  SuctionLoop(10);
  delay(500);
  PTPMoveDelta(0, 0, LIMIT_SWITCH_DOWN, 1);
  delay(300);
  SuctionLoop(10);
  return result;
}

/*********************************************************************************************************
** Function name:       PTPMoveColor
** Descriptions:        フィールドのカラーゾーンにブロックをおく,counter[sig-1]を1つ増やす
** Input parameters:    signature,counter
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMoveColor(int signature) {
  int sig = signature;
  int home_r[3] = { 160, -220, 50};
  int home_g[3] = { 80, -220, 50};
  int home_b[3] = { 30, -220, 50};
  int home_y[3] = { 140, 180, 30};

  int max_count = 0;
  int counter[4];
  int r_pos[3] = { 160, -245, -33};
  int g_pos[3] = { 83, -245, -33};
  int b_pos[3] = { 27, -245, -33};
  int y_pos[3] = { 140, 180, -33};

  //EEPROMからカウンタ値の読み込み
  counter[0] = EEPROM.read(0);//赤のブロックのカウンタ値
  counter[1] = EEPROM.read(1);//緑
  counter[2] = EEPROM.read(2);//青
  counter[3] = EEPROM.read(3);//黄



  max_count = max(counter[0], counter[1]);
  max_count = max(max_count, counter[2]);
  if (max_count > 4) {
    home_r[2] += 25 * (max_count - 4) / 2;
    home_g[2] += 25 * (max_count - 4) / 2;
    home_b[2] += 25 * (max_count - 4) / 2;
  }

  //ブロックを置く位置の計算
  if (counter[RED - 1] % 2 == 0)r_pos[2] += int(counter[RED - 1] / 2) * 25;
  else if (counter[RED - 1] % 2 == 1) {
    //r_pos[0] += 10;
    r_pos[1] += 50;
    r_pos[2] += (int(counter[RED - 1] / 2)) * 25;
  }
  if (counter[GREEN - 1] % 2 == 0)g_pos[2] += int(counter[GREEN - 1] / 2) * 25;
  else if (counter[GREEN - 1] % 2 == 1) {
    //r_pos[0] += 10;
    g_pos[1] += 50;
    g_pos[2] += (int(counter[GREEN - 1] / 2)) * 25;
  }
  if (counter[BLUE - 1] % 2 == 0)b_pos[2] += int(counter[BLUE - 1] / 2) * 25;
  else if (counter[BLUE - 1] % 2 == 1) {
    //r_pos[0] += 10;
    b_pos[1] += 50;
    b_pos[2] += (int(counter[BLUE - 1] / 2)) * 25;
  }

  y_pos[2] += counter[YELLO - 1] * 25;



  delay(1000);
  if (sig == RED) {
    PTPMovePos(home_r, true);
    delay(500);
    PTPMovePos(r_pos, true);
    delay(1000);
    PTPMoveDelta(0, 0, -10, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 0);
    delay(500);
    PTPMoveDelta(0, 0, 10 , 0);
    delay(500);
    PTPMovePos(home_r, false);
    delay(500);
    counter[RED - 1]++;
    EEPROM.write(0, counter[0]);//カウンタ値をEEPROMに書き込む
  }
  if (sig == GREEN) {

    PTPMovePos(home_g, true);
    delay(500);
    PTPMovePos(g_pos, true);
    delay(1000);
    PTPMoveDelta(0, 0, -10, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 0);
    delay(500);
    PTPMoveDelta(0, 0, 10 , 0);
    delay(500);
    PTPMovePos(home_g, false);
    delay(500);
    counter[GREEN - 1]++;
    EEPROM.write(1, counter[1]);
  }
  if (sig == BLUE) {


    PTPMovePos(home_b, true);
    delay(500);
    PTPMovePos(b_pos, true);
    delay(1000);
    PTPMoveDelta(0, 0, -10, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 0);
    delay(500);
    PTPMoveDelta(0, 0, 10 , 0);
    delay(500);
    PTPMovePos( home_b, false);
    delay(500);
    counter[BLUE - 1]++;
    EEPROM.write(2, counter[2]);
  }


  if (sig == YELLO) {

    PTPMovePos(y_pos, true);
    delay(1000);
    PTPMoveDelta(0, 0, -10, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 1);
    delay(500);
    PTPMoveDelta(0, 0, 0, 0);
    delay(500);
    PTPMoveDelta(0, 0, 10 , 0);
    delay(500);
    PTPMovePos( y_pos, false);
    delay(500);
    counter[YELLO - 1]++;
    EEPROM.write(3, counter[3]);
  }






}
/*********************************************************************************************************
** Function name:       PTPMoveDelta
** Descriptions:        指定分だけ移動
** Input parameters:    Δx,Δy,Δz,on/off
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMoveDelta(int x, int y, int z, bool on) {

  gPTPCmd.x += x;
  gPTPCmd.y += y;
  gPTPCmd.z += z;
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  if (on & 0x01)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
  else SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

  ProtocolProcess();
}



/*********************************************************************************************************
** Function name:       Pixy_ytoDobot_x
** Descriptions:        物体までの距離をpixyのy値からDobotの必要なxの移動量に変換
** Input parameters:    pixy_y
** Output parameters:   none
** Returned value:      dobot_x
*********************************************************************************************************/
double Pixy_yToDobot_x(double pixy_y) {
  return (-pixy_y + 100) * 30 / 72 + 290;
}


/*********************************************************************************************************
** Function name:       Pixy_xtoDobot_y
** Descriptions:        物体までの距離をpixyのx値からDobotの必要なyの移動量に変換
** Input parameters:    pixy_x
** Output parameters:   none
** Returned value:      dobot_y
*********************************************************************************************************/
double Pixy_xToDobot_y(double pixy_x) {
  return (-pixy_x + 314 / 2) * 30 / 72;
}

/*********************************************************************************************************
** Function name:       TriangleError_x,TriangleError_y
** Descriptions:        三角測量により,ボックスの高さによって生じる誤差を計算
** Input parameters:    num
** Output parameters:   error_value
** Returned value:      none
*********************************************************************************************************/

double TriangleError_x(double dobot_x) {
  double delta_x;
  delta_x = dobot_x - 210;
  return delta_x * 5 / 27;
}

double TriangleError_y(double dobot_y) {
  double delta_y;
  delta_y = dobot_y;
  return delta_y * 5 / 27;
}

/*********************************************************************************************************
** Function name:       SuctionLoop
** Descriptions:        サクションカップonをnum回繰り返す
** Input parameters:    num
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void SuctionLoop(int num) {
  for (int i = 0; i < num; i++) {
    PTPMoveDelta(0, 0, 0, 1);
    delay(10);
  }
}

/*********************************************************************************************************
** Function name:       Speed
** Descriptions:        dobotのスピード変更
** Input parameters:    speed (0～100)
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void Speed(int sp) {
  gPTPCommonParams.velocityRatio = sp;
  gPTPCommonParams.accelerationRatio = sp;
  SetPTPCommonParams(&gPTPCommonParams, true, &gQueuedCmdIndex);
  
  ProtocolProcess();
}



/*********************************************************************************************************
** Function name:       MissCounter
** Descriptions:        ブロックを取りにいく動作で3回失敗したら、ベルトコンベアを動かす
** Input parameters:    none
** Output parameters:   none
** Returned value:      count
*********************************************************************************************************/
int MissCounter(bool reset) {
  static int count;
  if (reset == 0) {
    count++;
    if (count > 2) {
      count = 0;
      digitalWrite(5, HIGH);
      delay(100);
      digitalWrite(5, LOW);
    }
  }

  else {
    count = 0;
  }

  return count;
}




/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void loop()
{

  int x, y, z;
  static int sig;

  InitRAM();

  ProtocolInit();


  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  static uint32_t timer = millis();
  static uint32_t count = 0;

  PTPMoveHome();
  static int16_t index = -1;
  Block *block = NULL;
  delay(2000);

  if (MODE == 1) {
    while (1) {
      delay(1000);
      PTPMoveHome();
      PTPMoveDelta(0, 0, 0, 0);
      delay(1000);
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks < 1) {
        digitalWrite(5, HIGH);
        delay(100);
        digitalWrite(5, LOW);
        //Serial.println("///////////////////////5pinHIGH");
      }
      else digitalWrite(5, LOW);
      Serial.println(pixy.ccc.numBlocks);
      if (index == -1) // search....
      {
        Serial.println("Searching for block...");
        index = acquireBlock();
        if (index >= 0)Serial.println("Found block!");
      }
      if (index >= 0)block = trackBlock(index);
      if (block)
      {
      }
      else // no object detected, stop motors, go into search state
      {
        index = -1; // set search state
        //digitalWrite(5,HIGH);
      }
      sig = block->m_signature;

      if (index != -1 && digitalRead(6) != HIGH) {
        MissCounter(false);
        x = Pixy_yToDobot_x(block->m_y);//pixyのy座標をdobotのx座標に変換
        // x -= TriangleError_x(x);//ブロックの高さによる誤差を修正
        delay(1);
        y = Pixy_xToDobot_y(block->m_x);
        // y -= TriangleError_y(y);
        delay(1);
        x = Pixy_yToDobot_x(block->m_y);
        PTPMove(x, y, 20, 0);
        delay(500);
        PTPMove(x, y, 10, 0);
        delay(500);
        PTPMoveDelta(0, 0, 0, 1);
        delay(500);
        PTPMoveDelta(0, 0, 10, 1);
        delay(500);
        digitalWrite(5, LOW);
        if (PTPMoveLimitSwitch()) {
          MissCounter(true);
          digitalWrite(5, HIGH);
          delay(100);
          //Serial.println("hello");
          digitalWrite(5, LOW);
          PTPMoveColor(block->m_signature);
          PTPMoveHome();
          delay(2000);
        }


      }
    }
  }

  else if (MODE == 2) {
    while (1) {
      Serial.println(MissCounter(0));
      delay(3000);
    }
  }

  else if (MODE == 3) {
    while (1) {}
  }
  else if (MODE == 4) {
    int mode4_counter[4] = {0, 0, 0, 0};
    int mode4_color = 1;
    while (1) {



      SuctionLoop(5);
      delay(3000);
      PTPMoveColor(RED);
      delay(1000);

      SuctionLoop(5);
      delay(3000);
      PTPMoveColor(GREEN);
      delay(1000);

      SuctionLoop(5);
      delay(3000);
      PTPMoveColor(BLUE);
      delay(1000);

      PTPMovePos(DEMO_HOME_POS, true);
      delay(1000);

      SuctionLoop(5);
      delay(3000);
      PTPMoveColor(YELLO);
      delay(1000);
    }
  }
  //EEPROMreset
  else if (MODE == 5) {
    for (int i = 0; i < 4; i++) {
      EEPROM.write(i, 0);
    }
    while (1) {
      Serial.println("MODE5 EEPROMリセット完了");
      for (int i = 0; i < 4; i++) {
        Serial.print(i); Serial.print(":"); Serial.println(EEPROM.read(i));
      }
      delay(1000);
    }
  }
  else if (MODE == 6) {
    while (1) {
      for (int i = 0; i < 4; i++)Serial.println(EEPROM.read(i));
      delay(1000);
    }
  }
}
