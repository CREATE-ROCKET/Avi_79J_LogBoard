#include <Arduino.h>
#include <ICM20602.h>
#include <LPS25HB.h>
#include <SPIflash.h>
#include <SPICREATE.h>
#include <CAN/CAN.h>
#include <ESP32Servo.h>
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
u_int8_t tx[256];
u_int8_t rx[256];
int kiroku;
Servo servo1;
Servo servo2;
bool flag = false;
bool risyou = false;
bool kaisan = false;
bool lflag = false;
bool lflagl = true;
bool lpsflag = false;
bool icmflag = true;
bool kaisanlpsflag = true;
bool kaisanicmflag = false;
bool kaisantimerflag = false;
int rcount = 0;
int bx = 0;
int bh = 0;
int bc = 0;
int k;
int kasokudoc;
u_int8_t LPS25_data[3] = {0, 0, 0};
u_int8_t last_modified_LPS25_data[3] = {0, 0, 0};
hw_timer_t *timer = NULL;
u_int64_t heikina = 0;
int frmax = 200;
char canreceive = 0;
IRAM_ATTR void counter() // 1msで呼ばれる
{
  if (risyou)
  {
    if (kaisantimerflag)
    {
      if (!kaisan)
      {
        if (rcount > 8500)
        {
          kaisan = true;
        }
        rcount++;
      }
    }
    if (kaisanlpsflag)
    {
      // 以下lpsの条件
      if (pcount % 20 == 1)
      {
        lps.Get(LPS25_data);
        for (int i = 0; i < 3; i++)
        {
          LPS25_data[i] = LPS25_data[i] * (1 - (20 / (frmax * 1000)) + last_modified_LPS25_data[i] * (20 / (frmax * 1000)));
          last_modified_LPS25_data[i] = LPS25_data[i];
        }
        bx += ((LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200) / 4096; // ヘクトパスカル*200の値が格納されています
        if (lcount == 5)
        {
          kiroku = (bh - bx / 5);
          if (bh < bx / 5 && bh != 0)
          {
            bc++;
            if (bc > 4)
            {
              kaisan = true;
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
  }else
    {
      if (lpsflag)
      {
        // 以下lpsの条件
        if (pcount % 20 == 1)
        {
          lps.Get(LPS25_data);
          for (int i = 0; i < 3; i++)
          {
            LPS25_data[i] = LPS25_data[i] * (1 - (20 / (frmax * 1000)) + last_modified_LPS25_data[i] * (20 / (frmax * 1000)));
            last_modified_LPS25_data[i] = LPS25_data[i];
          }
          bx += ((LPS25_data[0] + LPS25_data[1] * 256 + LPS25_data[2] * 65536) * 200) / 4096; // ヘクトパスカル*200の値が格納されています
          if (lcount == 5)
          {
            kiroku = (bh - bx / 5);
            if ((bh - bx / 5) >= 1 && bh != 0)
            {
              bc++;
              if (bc > 4)
              {
                lflag = true;
                risyou=true;
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
        int16_t ICM_data[6];
        icm.Get(ICM_data);
        heikina += ((ICM_data[0]) * (ICM_data[0]) + (ICM_data[1]) * (ICM_data[1]) + (ICM_data[2]) * (ICM_data[2])) * 16 * 16 / 16384 / 16384;
        if (pcount % 20 == 19)
        {
          if (heikina / 20 > 4 * 9.8 * 9.8)
          {
            kasokudoc++;
            if (kasokudoc > 2)
            { // 本番は50
              lflag = true;
              risyou = true;
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
  if (pcount >= 500)
      {
        pcount = 0;
        if (lflag)
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
      }
}
void setup()
{
  // put your setup code here, to run once:
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
  servo1.write(0);
  servo2.attach(16);
  servo2.write(0);
}
void loop()
{
  if (!lflag)
  {
    // Serial.print(bh);
    // Serial.print(",");
    // Serial.print(bc);
    // Serial.print(",");
    // Serial.print(kiroku);
    // Serial.print(",");
    // Serial.println(pcount);
    Serial.print(heikina);
    Serial.print(",");
    Serial.print(kasokudoc);
    Serial.print(",");
    Serial.println(pcount);
    Serial.println(icm.WhoAmI());
    delay(10);
  }
  // Serial.print(canreceive);
  if (CAN.available())
  {
    char cmd = (char)CAN.read();
    canreceive = cmd;
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
    CAN.sendPacket(0x13, cmd);
  }
  if(kaisan){
    servo1.write(90);
    servo2.write(90); // ここで開傘
  }
}