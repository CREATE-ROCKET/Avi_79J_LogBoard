#include <Arduino.h>
#include <ICM20602.h>
#include <LPS25HB.h>
#include <SPIflash.h>
#include <SPICREATE.h>
#include <CAN/CAN.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
// put function declarations here:
#define led_pin 22 // 22
#define SPICLOCK 33
#define SPIMISO 25
#define SPIMOSI 26
#define FLASHCS 27
#define LPSCS 13
#define ICMCS 15
#define UART_RX 3
#define UART_TX 1
#define CAN_RX 17
#define CAN_TX 18
#define CAMERAPIN 4
#define counterPeriod 20
SPICREATE::SPICreate spi;
LPS lps;
ICM icm;
Flash flash;
uint8_t count = 0;
int pcount = 0;
uint8_t lcount = 0;
CAN_CREATE CAN;
Servo servo1;
Servo servo2;
bool flag = false;
bool risyou = false;
bool kaisan = false;
bool lflag = false;
bool lflagl = true;
bool lpsflag = true;         // ここを変える
bool icmflag = true;         // ここを変える
bool sensorgetflag = true;   // ここを変える
bool kaisanlpsflag = true;   // ここを変える
bool kaisantimerflag = true; // ここを変える
bool kemuriicmflag = true;   // ここを変える
bool kemuritenkatyuusi = false;
bool sequenceend = false;
bool islogging = false;
bool timerstartflag = true;
bool sequencestartflag = false;
bool risyoukentisousinflag = false;
bool standby = false;
int bx = 0;
int bh = 0;
int bc = 0;
int bkc = 0;
int tikatikasyuuki = 500;
int k;
int kasokudoc;
int kemurikasokudoc;
int logcount = 0;
unsigned long starttime;
unsigned long logtime;
unsigned long risyoutime;
u_int8_t LPS25_data[3] = {0, 0, 0};
int16_t ICM_data[6] = {0, 0, 0, 0, 0, 0};
uint8_t ICMraw[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u_int8_t last_modified_LPS25_data[3] = {0, 0, 0};
hw_timer_t *timer = NULL;
u_int64_t heikina = 0;
u_int32_t SpiFlashLatestAddress = 0;
u_int8_t data[256];
bool datalogflag = false;
int datacount = 0;
int frmax = 200;
char canreceive = 0;
bool kemurisend = false;
int sensorcount = 0;
int kakudo = 90;
void risyoudetect()
{
  risyoutime = millis() - 1000;
  risyou = true;
}
void cansend(char msg[100], int msglength)
{
  for (int i = 0; i < msglength; i++)
  {
    CAN.sendPacket(0x13, msg[i]);
  }
  CAN.sendPacket(0x13, '\n');
}
void logroutine()
{
  if (timerstartflag)
  {
    starttime = millis();
    timerstartflag = false;
  }
  logtime = millis() - starttime;
  for (int i = 0; i < 4; i++)
  {
    data[32 * logcount + i] = 0xFF & logtime >> (8 * i);
    // Serial.println(data[32 * logcount + i]);
  }
  for (int i = 4; i < 7; i++)
  {
    data[32 * logcount + i] = 0xFF & LPS25_data[i - 4];
  }
  for (int i = 7; i < 21; i++)
  {
    data[32 * logcount + i] = ICMraw[i - 7];
  }
  for (int i = 21; i < 32; i++)
  {
    data[32 * logcount + i] = 0xFF & i;
  }
  logcount++;
  if (logcount >= 8)
  {
    logcount = 0;
    flash.write(SpiFlashLatestAddress, data);
    SpiFlashLatestAddress += 0x100;
  }
}
IRAM_ATTR void counter() // 1msで呼ばれる
{
  if (sequenceend || !sequencestartflag)
  {
    digitalWrite(led_pin, LOW);
  }
  else
  {
    // 以下センサーの値を取得
    if (sensorgetflag)
    {
      sensorcount++;
      icm.Get(ICM_data, ICMraw);
      if (islogging)
      {
        datalogflag = true;
      }
      if (sensorcount >= 19)
      {
        lps.Get(LPS25_data);
        sensorcount = 0;
      }
    }
    // 以上センサーの値を取得
    if (risyou)
    {
      if (kaisantimerflag)
      {
        if (!kaisan)
        {
          if (millis() - risyoutime > 9500) // ここを変える
          {
            kaisan = true;
          }
        }
      }
      if (kaisanlpsflag)
      {
        // 以下lpsの条件
        if (pcount % 20 == 1 && millis() - risyoutime > 5000)
        {
          for (int i = 0; i < 3; i++)
          {
            LPS25_data[i] = LPS25_data[i] * (1 - (20 / (frmax * 1000)) + last_modified_LPS25_data[i] * (20 / (frmax * 1000)));
            last_modified_LPS25_data[i] = LPS25_data[i];
          }
          bx += ((LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200) / 4096; // ヘクトパスカル*200の値が格納されています
          if (lcount == 5)
          {
            if (bh < bx / 5 && bh != 0)
            {
              bkc++;
              if (bkc > 9) // ここを変える テストのときは4 本番は9
              {
                kaisan = true;
              }
            }
            else
            {
              bkc = 0;
            }
            bh = bx / 5;
            bx = 0;
            lcount = 0;
          }
          lcount++;
        }
      }
      if (kaisan && kemuriicmflag)
      {
        heikina += ((ICM_data[0]) * (ICM_data[0]) + (ICM_data[1]) * (ICM_data[1]) + (ICM_data[2]) * (ICM_data[2])) * 16 * 16 / 16384 / 16384;
        if (pcount % 20 == 19)
        {
          if (heikina / 20 < 9.8 * 9.8 / 100)
          {
            kemurikasokudoc++;
            if (kemurikasokudoc > 2)
            { // 本番は50
              kemuritenkatyuusi = true;
              tikatikasyuuki = 250;
            }
          }
          else
          {
            kemurikasokudoc = 0;
          }
          heikina = 0;
        }
      }
    }
    else
    {
      if (lpsflag)
      {
        // 以下lpsの条件
        if (pcount % 20 == 1)
        {
          // lps.Get(LPS25_data);
          for (int i = 0; i < 3; i++)
          {
            LPS25_data[i] = LPS25_data[i] * (1 - (20 / (frmax * 1000)) + last_modified_LPS25_data[i] * (20 / (frmax * 1000)));
            last_modified_LPS25_data[i] = LPS25_data[i];
          }
          bx += ((LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200) / 4096; // ヘクトパスカル*200の値が格納されています
          if (lcount == 5)
          {
            if ((bh - bx / 5) >= 20 && bh != 0) // ここを変える テストのときは1 本番は20
            {
              bc++;
              if (bc > 4)
              {
                lflag = true;
                risyoudetect();
              }
            }
            else
            {
              bc = 0;
            }
            bh = bx / 5;
            bx = 0;
            lcount = 0;
          }
          lcount++;
        }
      }
      if (icmflag)
      {
        // 以下icmの条件
        heikina += ((ICM_data[0]) * (ICM_data[0]) + (ICM_data[1]) * (ICM_data[1]) + (ICM_data[2]) * (ICM_data[2])) * 16 * 16 / 16384 / 16384;
        if (pcount % 20 == 19)
        {
          if (heikina / 20 > 4 * 9.8 * 9.8) // ここを変える
          {
            kasokudoc++;
            if (kasokudoc > 50) // ここを変える テストのときは2 本番は50
            {
              lflag = true;
              risyoudetect();
            }
          }
          else
          {
            kasokudoc = 0;
          }
          heikina = 0;
        }
      }
    }
    pcount++;
    // 以下LEDちかちか
    if (pcount >= tikatikasyuuki)
    {
      pcount = 0;
      if ((lflag && !kaisan) || kemuritenkatyuusi)
      {
        if (lflagl)
        {
          lflagl = false;
          digitalWrite(led_pin, HIGH);
        }
        else
        {
          lflagl = true;
          digitalWrite(led_pin, LOW);
        }
      }
      else
      {
        digitalWrite(led_pin, HIGH);
      }
      if (kemuritenkatyuusi)
      {
        if (!kemurisend)
        {
          kemurisend = true;
        }
      }
    }
  }
}
void setup()
{
  // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  pinMode(led_pin, OUTPUT);
  pinMode(CAMERAPIN, OUTPUT);
  digitalWrite(led_pin, HIGH);
  Serial.begin(115200);
  spi.begin(VSPI, SPICLOCK, SPIMISO, SPIMOSI);
  lps.begin(&spi, LPSCS, 6000000);
  icm.begin(&spi, ICMCS, 6000000);
  digitalWrite(LPSCS, HIGH);
  digitalWrite(ICMCS, HIGH);
  flash.begin(&spi, FLASHCS, 6000000);
  digitalWrite(FLASHCS, HIGH);
  timer = timerBegin(0, getApbFrequency() / 1000000, true);
  timerAttachInterrupt(timer, &counter, false);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
  CAN.setPins(CAN_RX, CAN_TX);
  CAN.begin(100E3);
  servo1.attach(14);
  servo1.write(90);
  servo2.attach(16);
  servo2.write(90);
  for (int i = 0; i < 256; i++)
  {
    data[i] = 0;
  }
  Serial.println("Start");
  cansend("LOGSTART", 8);
  sequencestartflag = true;
}
void erase()
{
  flash.erase();
}
void loop()
{
  if (CAN.available())
  {
    char cmd = (char)CAN.read();
    Serial.println(cmd);
    canreceive = cmd;
    if (standby)
    {
      if (cmd == 'a')
      { // クローズ
        servo1.write(kakudo);
        servo2.write(90);
      }
      else if (cmd == 'b')
      { // オープン
        servo1.write(0);
        servo2.write(0);
      }
      else if (cmd == 'C')
      {
        kakudo++;
      }
      else if (cmd == 'D')
      {
        kakudo--;
      }
      else if (cmd == 'f')
      {
        cansend("FLASHERASE", 10);
        erase();
        cansend("ERASED", 6);
      }
      else if (cmd == 'l')
      {
        islogging = true;
      }
      else if (cmd == 'm')
      {
        islogging = false;
      }
      else if (cmd == 'k')
      {
        digitalWrite(CAMERAPIN, HIGH);
      }
      else if (cmd == 'j')
      {
        digitalWrite(CAMERAPIN, LOW);
      }
      else if (cmd == 'P')
      {
        servo2.write(90);
      }
      else if (cmd == 'Q')
      {
        servo1.write(kakudo);
      }
      else if (cmd == 'x')
      {
        CAN.sendPacket(0x13, 'x');
      }
      else if (cmd == 'D')
      {
        servo1.write(0);
      }
      else if (cmd == 'R')
      {
        servo2.write(0);
      }else if(cmd=='Z'){
        kakudo=90;
      }
      standby = false;
    }
    else
    {
      if (cmd == 'e')
      {
        sequenceend = true;
      }
      else if (cmd == 's')
      {
        standby = true;
        cansend("STANDBY", 7);
      }
      else if (cmd == 'G')
      {
        String kk = String(kakudo);
        int len = kk.length() + 1;
        char data[len];
        kk.toCharArray(data, len);
        cansend(data, len);
      }
      else
      {
        cansend("ERROR", 5);
      }
    }
  }
  if (Serial.available())
  {
    char cmd = (char)Serial.read();
    Serial.println(cmd);
    if (cmd == 'a')
    {
      servo1.write(90);
      servo2.write(90);
    }
    else if (cmd == 'b')
    {
      servo1.write(0);
      servo2.write(0);
    }
    else if (cmd == 'c')
    {
      digitalWrite(CAMERAPIN, HIGH);
    }
    else if (cmd == 'd')
    {
      digitalWrite(CAMERAPIN, LOW);
    }
    else if (cmd == 'e')
    {
      sequenceend = true;
    }
    else if (cmd == 'f')
    {
      flash.erase();
    }
    else if (cmd == 'r')
    {
      Serial.print("data until memory");
      Serial.println(SpiFlashLatestAddress);
      if (sequenceend)
      {
        for (int k = 0; k <= SpiFlashLatestAddress; k += 0x100)
        {
          u_int8_t rx[256];
          flash.read(k, rx); // flash1.read(0, rx); ???
          // 読み込んだデータをシリアルで表示
          for (int i = 0; i < 8; i++)
          {
            for (int j = 0; j < 32; j++)
            {
              Serial.print(rx[i * 32 + j]);
              Serial.print(",");
            }
            Serial.println(" ");
          }
          Serial.println("page" + String(k / 0x100));
        }
      }
    }
    else if (cmd == 'l')
    {
      islogging = true;
    }
    else if (cmd == '@')
    {
      for (int i = 0; i < 256; i++)
      {
        Serial.print(data[i]);
        data[i] = i;
      }
      Serial.println("datalog");
      flash.write(SpiFlashLatestAddress, data);
    }
    else if (cmd == 'I')
    {
      Serial.println("ICMwhiami");
      Serial.println(icm.WhoAmI());
      Serial.println("icmvalue");
      icm.Get(ICM_data);
      for (int i = 0; i < 6; i++)
      {
        Serial.println(ICM_data[i]);
      }
    }
  }
  if (risyou && !risyoukentisousinflag)
  {
    cansend("RIFTOFF", 7);
    risyoukentisousinflag = true;
  }
  if (lflag)
  {
    if (kaisan)
    {
      servo1.write(90);
      servo2.write(90); // ここで開傘
    }
    if (kemurisend)
    {
      CAN.sendPacket(0x13, 'i');
      kemurisend = false;
    }
  }
  if (datalogflag)
  {
    // Serial.println("datalog");
    logroutine();
    datalogflag = false;
  }
}