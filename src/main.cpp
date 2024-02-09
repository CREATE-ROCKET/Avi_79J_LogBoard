#include <Arduino.h>
#include <ICM20602.h>
#include <LPS25HB.h>
#include <SPIflash.h>
#include <SPICREATE.h>
// put function declarations here:
#define led_pin 22//22
#define SPICLOCK 33
#define SPIMISO 25
#define SPIMOSI 26
#define FLASHCS 27
#define LPSCS 13
#define ICMCS 15
#define UART_RX 3
#define UART_TX 1
#define counterPeriod 20
SPICREATE::SPICreate spi;
LPS lps;
ICM icm;
Flash flash;
uint8_t count = 0;
u_int8_t tx[256];
u_int8_t rx[256];
bool flag = false;
bool lflag = false;
bool lpsflag=false;
bool icmflag=false;
int bx=0;
int bh=0;
int bc=0;
int k;
int kasokudoc;
u_int8_t LPS25_data[3];
hw_timer_t * timer=NULL;
u_int64_t heikina;
IRAM_ATTR void counter()
{
  if(lpsflag){
  //以下lpsの条件
  if(count%20==1){
      lps.Get(LPS25_data);
      bx+=((LPS25_data[0]+LPS25_data[1]*256+LPS25_data[2]*65536)*200)/4096;//ヘクトパスカル*200の値が格納されています
      if(count%5==4){
        if((bh-bx/5)>=1&&bh!=0){
          bc++;
          if(bc>4){
            lflag=true;
          }
        }else{
          bc=0;
        }
        bh=bx/5;
        bx=0;
      }
  }
  }
  if(icmflag){
      //以下icmの条件
      int16_t ICM_data[6];
      icm.Get(ICM_data);
      heikina+=((ICM_data[0])*(ICM_data[0])+(ICM_data[1])*(ICM_data[1])+(ICM_data[2])*(ICM_data[2]))*16*16/16384/16384;
      if(count%20==19){
        if(heikina/20>4*9.8*9.8){
        kasokudoc++;
        if(kasokudoc>2){//本番は50
          lflag=true;
        }
      }else{
        kasokudoc=0;
      }
      heikina=0;
      }
  }
      //以下LEDちかちか
      switch (count % 400)
      {
      case 0:
        digitalWrite(led_pin, HIGH);
        //Serial1.println("hello Serial1");
        break;
      case 200:
      if(lflag){
        digitalWrite(led_pin, LOW);
      }else{
        digitalWrite(led_pin, HIGH);
      }
        //Serial.println("hello Serial");
        break;
      }
      if(count>800){
        count=count%100;
      }
      count++;
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  Serial.begin(115200);
  Serial.println("hello");
  spi.begin(VSPI, SPICLOCK, SPIMISO, SPIMOSI);
  lps.begin(&spi,LPSCS,6000000);
  icm.begin(&spi,ICMCS,6000000);
  Serial.println("hello");
  timer=timerBegin(0,getApbFrequency()/1000000,true);
  timerAttachInterrupt(timer, &counter,false);
  timerAlarmWrite(timer,1000,true);
  timerAlarmEnable(timer);
}
void loop()
{
  Serial.print(heikina);
  Serial.print(",");
  Serial.println(kasokudoc);
  delay(10);
}