#include <SoftReset.h>
#include <CircularQueue.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// 외부 핀 정의
#define PIN_RELAY_1     4     // 검출 시 활성화 될 릴레이 핀
#define PIN_RELAY_2     9     // 검출 시 활성화 될 릴레이 핀
#define PIN_RELAY_3     14    // 릴레이 추가 A0핀
#define PIN_RELAY_4     15    // 릴레이 추가 A1핀

// 부저 옵션
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
u16 nHoldingTime = 3000;              // 릴레이 동작 유지시간
u16 maxHoldingTime = 60000;
int nHighCut = 36.111111111111111;    // 36.1111... m/s = 130 km/h, // -38.8888888..... m/s == 140 km/h; 너무 빠른 속도로 다가오는 것은 오류로 판단/제거
uint8_t offDetect = true;
uint8_t rFlag = false;                // 릴레이 동작 플래그

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
  pinMode(PIN_RELAY_3, OUTPUT);
  pinMode(PIN_RELAY_4, OUTPUT);
  
  digitalWrite(PIN_RELAY_1, HIGH);
  digitalWrite(PIN_RELAY_2, HIGH);
  digitalWrite(PIN_RELAY_3, HIGH);
  digitalWrite(PIN_RELAY_4, HIGH);
  delay(10);
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(PIN_RELAY_2, LOW);
  digitalWrite(PIN_RELAY_3, LOW);
  digitalWrite(PIN_RELAY_4, LOW);

  wdt_enable(WDTO_4S);
}

void loop() {
  char c;

  if(Serial.available())
  {
    c = Serial.read();
    if(c == '\n'){
      if(strcmp("$SPDOP", sRecv.substring(0, 6).c_str()) == 0) {
        int first = sRecv.indexOf(",");
        int two = sRecv.indexOf(",", first+1);
        int three = sRecv.indexOf(",", two+1);
        String command = sRecv.substring(three+1, three+2);
        String charSpeed = sRecv.substring(first+1, two);
        float rSpeed = charSpeed.toFloat();
        //Serial.println(rSpeed);

        //속도 값에 따른 릴레이 동작 조건
        // RELAY_1 : 스피커, RELAY_2 : X(스페어), RELAY_3 : 전광판에 추돌주의 , RELAY_4 : 전광판에 안전운행
        // 조건1: 속도 20km 이하일때, RELAY_1 : OFF / RELAY_3 : ON / RELAY_4 : OFF
        // 조건2: 속도 21km 이상 60km이하일때, RELAY_1 : ON / RELAY_2 : OFF / RELAY_3 : ON / RELAY_4 : OFF
        // 조건3: 속도 60km 이상 일때, RELAY_1: OFF, RELAY_2: OFF / RELAY_3 : OFF / RELAY_4 : ON
        if(command == "1")      //레이더에서 차량 감지 했을경우에만 릴레이 동작
        {      
            if(rSpeed > 3 && rSpeed < 20)
            {
              rFlag = true;
              digitalWrite(PIN_RELAY_1, LOW);
              digitalWrite(PIN_RELAY_4, LOW);
              digitalWrite(PIN_RELAY_3, HIGH);
              tsTimeout = millis() + nHoldingTime;
              //Serial.println("추돌주의");
              //Serial.println(tsTimeout);
            }

            else if(rSpeed > 20 && rSpeed < 60)
            {
              rFlag = true;
              digitalWrite(PIN_RELAY_4, LOW);
              digitalWrite(PIN_RELAY_1, HIGH);
              digitalWrite(PIN_RELAY_3, HIGH);
              tsTimeout = millis() + nHoldingTime;
              //Serial.println("추돌주의+사이렌");
            }

            else if(rSpeed > 60)
            {
              rFlag = true;
              digitalWrite(PIN_RELAY_1,LOW);
              digitalWrite(PIN_RELAY_3,LOW);
              digitalWrite(PIN_RELAY_4,HIGH);
              tsTimeout = millis() + nHoldingTime;
              //Serial.println("안전운행");
            }
        }
        // 레이더 릴레이 신호가 0이면 릴레이 모두 OFF
        if(command == "0")
        {
          digitalWrite(PIN_RELAY_1, LOW); 
          digitalWrite(PIN_RELAY_2, LOW);
          digitalWrite(PIN_RELAY_3, LOW);
          digitalWrite(PIN_RELAY_4, LOW);
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
        if(millis() > tsTimeout && rFlag == true)
        {
          //Serial.println("Time over");
          digitalWrite(PIN_RELAY_1, LOW); 
          digitalWrite(PIN_RELAY_2, LOW);
          digitalWrite(PIN_RELAY_3, LOW);
          digitalWrite(PIN_RELAY_4, LOW);
          rFlag = false;

#if USE_BUZZER
      noTone(PIN_BUZZER);
#endif
        }
        //와치독 리셋
        wdt_reset();
}
