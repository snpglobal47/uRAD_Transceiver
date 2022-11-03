#include <SoftReset.h>
#include <CircularQueue.h>
#include <EEPROM.h>

// 외부 핀 정의
#define PIN_RELAY_1     4     // 검출 시 활성화 될 릴레이 핀
#define PIN_RELAY_2     9     // 검출 시 활성화 될 릴레이 핀

// 옵션
#define USE_BUZZER      0     // 내부 부저(시험용) 사용 여부 설정
#if USE_BUZZER
#define PIN_BUZZER      10    // 검출 시 활성화 될 부저 핀. PWM 가능한 핀으로 선택할 것
#define BUZZER_FREQ     2400  // 주파수 2.4kHz
#endif

// 매크로
#define MPS_TO_KMPH(x)    ((x) * 3.6)                   // m/s -> km/h
#define KMPH_TO_MPS(x)    ((x) * 0.2777777777)          // km/h -> m/s

///////////////////////////////////////////////
///////////// 전역변수
String sRecv = "";
unsigned long tsTimeout = 0;          // 검출신호 유지시간 만료 타임스탬프
unsigned long onTimeout = 0; 
u16 nHoldingTime = 3000;
u16 maxHoldingTime = 60000;
int nHighCut = 36.111111111111111;    // 36.1111... m/s = 130 km/h, // -38.8888888..... m/s == 140 km/h; 
// 너무 빠른 속도로 다가오는 것은 오류로 판단/제거
uint8_t offDetect = true;
int sp_alive = 0;
int err_alive = 0;

// Checksum 계산 함수
u08 CalcCheckSum(char* str)
{
  u08 uChk = 0x00;
  int len = strlen(str);
  for(int i = 0 ; i < len ; i++)
  {
    if((str[i] != '$') && (str[i] != '!') && (str[i] != '*'))
      uChk ^= str[i];
  }
  return uChk;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);

  //digitalWrite(PIN_RAD, HIGH);  // switch ON uRAD
  digitalWrite(PIN_RELAY_1, HIGH);
  digitalWrite(PIN_RELAY_2, HIGH);
  delay(10);
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(PIN_RELAY_2, LOW);
}

void loop() {
  char c;
  bool error;

  if(Serial.available())
  {
    c = Serial.read();
    if(c == '\n'){
      if(strcmp("$SPDOP", sRecv.substring(0, 6).c_str()) == 0) {
        int first = sRecv.indexOf(",");
        int two = sRecv.indexOf(",", first+1);
        int three = sRecv.indexOf(",", two+1);
        String command = sRecv.substring(three+1, three+2);

        if(command == "1")
        {
          tsTimeout = millis() + nHoldingTime;
          if(offDetect == true)
          {
            onTimeout = millis() + maxHoldingTime;
            offDetect = false;
          }
                   
          digitalWrite(PIN_RELAY_1, HIGH); 
          digitalWrite(PIN_RELAY_2, HIGH);
          //Serial.println("Realy ON!!!!");
          //Serial.println(tsTimeout);
        }

        if(millis() > tsTimeout)
        {
          //Serial.println("Time over");
          digitalWrite(PIN_RELAY_1, LOW); 
          digitalWrite(PIN_RELAY_2, LOW);
          offDetect = true;

#if USE_BUZZER
      noTone(PIN_BUZZER);
#endif
        }
        
        if(command == "0")
        {
          digitalWrite(PIN_RELAY_1, LOW); 
          digitalWrite(PIN_RELAY_2, LOW);
        }
      }
      sRecv = "";
    }
    else
    {
      sRecv += c;
      //Serial.println(sRecv);
    }
  }

  if(millis() > onTimeout && digitalRead(PIN_RELAY_1) == HIGH && digitalRead(PIN_RELAY_2 == HIGH))
        {
          Serial.println("Timeout_RESET");
          digitalWrite(PIN_RELAY_1, LOW);
          digitalWrite(PIN_RELAY_2, LOW);          
          delay(3000);
          offDetect = true;
          soft_restart();
        }
}
