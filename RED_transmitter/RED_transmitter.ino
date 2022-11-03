#include <SoftReset.h>
#include <CircularQueue.h>
#include <uRAD.h>   // include the library
#include <uRAD_SDK11.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//////////////////////////////////////////////////////////
// 상수값 정의
#define SW_VERSION      "v.0.93ml"     //레이더 SDK 1.1버전 업데이트(라이브러리 일부 변경)
//#define SW_VERSION      "v0.92ml"     // 릴레이 Timeout 추가
//#define SW_VERSION      "v0.91m1"     // 소스 코드 정리. 통신 오류 수정.
//#define SW_VERSION      "v0.91ro"     // 소스 코드 정리. RF 시험용(Rx 전용)
//#define SW_VERSION      "v0.91"   // 시온테크 납품 버전

// 외부 핀 정의
#define PIN_RAD         6     // uRAD on/off
#define PIN_RAD_SEL     7     // SPI SEL for uRAD
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

//////////////////////////////////////////////////////////
// 전역 변수
//uRAD  my_uRAD;  // uRAD Object (1.0버전 SN1942 이하 버전 적용)
uRAD_SDK11 my_uRAD; // uRAD Object (1.1버전 SN 1942 이상 버전)
int return_code;    // (1.1버전부터 사용되는 변수)
const uint8_t QUEUE_SIZE = 31;        // 약 1초 분량의 데이터(1개당 약 33ms 소요): 검출 여부만 저장됨
CircularQueue q;                      // 검출여부 queue
String sRecv = "";                    // 데이터 통신용 버퍼
unsigned long tsTimeout = 0;          // 검출신호 유지시간 만료 타임스탬프
unsigned long onTimeout = 0;          // 
int nHighCut = 36.111111111111111;    // 36.1111... m/s = 130 km/h,       // -38.8888888..... m/s == 140 km/h;                   // 너무 빠른 속도로 다가오는 것은 오류로 판단/제거
uint8_t offDetect = true;
u16 maxHoldingTime = 60000;           // 검출 신호 최대 시간(30초 동안 검출 상태 시, 레이더 리셋)
float sp1 = 0;
int sp_alive = 0;
int err_alive = 0;

//Soft Serial
const int pin_rx = 2;
const int pin_tx = 3;

// 레이다 설정 파라미터
uint8_t mode;                         // Waveform: 1(CW), 2(Sawtooth), 3(Triangular), 4(Dual Rate)
uint8_t f0;                           // Operataion frequency
uint8_t BW;                           // Band Width(When Mode = 2,3,4)
uint8_t Ns;                           // The Number of samples
uint8_t Ntar;                         // The Maximum Number of Targets
uint8_t Vmax;                         // The Maximum velocity
uint8_t MTI;                          // Moving Target Indication
uint8_t Mth;                          // The Sensitivity(as a Movement Detector)
uint8_t alpha;
bool distance_true;
bool velocity_true;
bool SNR_true;
bool I_true;
bool Q_true;
bool movement_true;
uint8_t NtarDetected[1];
float velocity[5];                    // 타겟 속력 - 최대 5개
float SNR[5];                         // 타겟 SNR(신호 대 잡음비) - 최대 5개

// EEPROM에 저장되어 있는 값들 
u16 nSpeedUnder = 5;                  // 속도 하한값(km/h)
u16 nSpeedUpper = 100;                // 속도 상한값(km/h)
float dSNRUnder = 5.0;                // SNR 하한값(dB)
float dSNRUpper = 40.0;               // SNR 상한값(dB)
u16 nDetectionTime = 100;             // 감지 시간(ms)
float dDetectionProportion = 0.666;   // 최소 검출 비율(%)
u16 nHoldingTime = 3000;              // 검출신호 유지시간

SoftwareSerial my_serial(pin_rx, pin_tx);

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

// 초기설정 함수
void setup() {
  //Serial.begin(250000);   // serial port baud rate
  Serial.begin(115200);   // serial port baud rate
  my_serial.begin(115200);

  mode = 1;   // doppler mode with best velocity accuracy(CW)
  f0 = 125;   // working at 24.125 GHz
  BW = 0;     // X
  Ns = 200;   // 200 samples. update rate = 21
  Ntar = 5;   // 5 targets of interest
  Vmax = 75;  // searching along the full velocity range. 75m/s(270km/h)까지
  MTI = 0;    // X
  Mth = 0;    // X
  alpha = 20;         // signal has to be 10 dB higher than its surrounding
  distance_true = false;     // request distance information
  velocity_true = true;    // mode 2 does not provide velocity information
  SNR_true = true;      // Signal-to-Noise-Ratio information requested
  I_true = false;       // In-Phase Component (RAW data) not requested
  Q_true = false;       // Quadrature Component (RAW data) not requested
  movement_true = false;    // not interested in boolean movement detection

  // switch ON uRAD
  my_uRAD.turnON();
    
  // load the configuration
  my_uRAD.loadConfiguration(mode, f0, BW, Ns, Ntar, Vmax, MTI, Mth, alpha, distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true);

  int nIndex = 0;
  EEPROM.get(nIndex, nSpeedUnder);            nIndex += sizeof(nSpeedUnder);
  EEPROM.get(nIndex, nSpeedUpper);            nIndex += sizeof(nSpeedUpper);
  EEPROM.get(nIndex, dSNRUnder);              nIndex += sizeof(dSNRUnder);
  EEPROM.get(nIndex, dSNRUpper);              nIndex += sizeof(dSNRUpper);
  EEPROM.get(nIndex, nDetectionTime);         nIndex += sizeof(nDetectionTime);
  EEPROM.get(nIndex, dDetectionProportion);   nIndex += sizeof(dDetectionProportion);
  EEPROM.get(nIndex, nHoldingTime);           nIndex += sizeof(nHoldingTime);
  if((nSpeedUnder == 0xffff) && (nSpeedUpper == 0xffff))
  {
    nSpeedUnder = 5;
    nSpeedUpper = 100;
    dSNRUnder = 5.0;
    dSNRUpper = 40.0;
    nDetectionTime = 100;
    dDetectionProportion = 0.666;
    nHoldingTime = 3000;
  }
  sRecv = "";
  CreateQueue(&q, QUEUE_SIZE);  
  Serial.println("\r\n");
  Serial.print("* Reverse Entry Detector ");
  Serial.print(SW_VERSION);
  Serial.print("\r\n");

  //pinMode(PIN_RAD, OUTPUT);  
  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);
 
  //digitalWrite(PIN_RAD, HIGH);  // switch ON uRAD  
  digitalWrite(PIN_RELAY_1, HIGH);
  digitalWrite(PIN_RELAY_2, HIGH);
  delay(10);
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(PIN_RELAY_2, LOW);

#if USE_BUZZER
  tone(PIN_BUZZER, BUZZER_FREQ, 100);
#endif
}

// 메인루프
void loop() {
  char c;
  bool error;
  bool relayOn = false;    // 릴레이 신호 변수
  if(Serial.available())
  {
    c = Serial.read();
    if(c == '\n'){
      //Serial.print("ok,");
      //Serial.println(sRecv.substring(0, 6).c_str());
      if(strcmp("$SPDPA", sRecv.substring(0, 6).c_str()) == 0){
        int first = sRecv.indexOf(",");
        int two = sRecv.indexOf(",", first+1);
        int three = sRecv.indexOf(",", two+1);
        int four = sRecv.indexOf(",", three+1);
        int five = sRecv.indexOf(",", four+1);
        int six = sRecv.indexOf(",", five+1);
        int seven = sRecv.indexOf(",", six+1);          
        int eight = sRecv.indexOf(",", seven+1);          
        String command = sRecv.substring(eight+1, eight+2);
        //Serial.print("ok,");
        //Serial.println(command);
        if(command == "c")
        {
          nSpeedUnder = sRecv.substring(first+1, two).toInt();
          nSpeedUpper = sRecv.substring(two+1, three).toInt();
          dSNRUnder = sRecv.substring(three+1, four).toDouble();          
          dSNRUpper = sRecv.substring(four+1, five).toDouble();
          nDetectionTime = sRecv.substring(five+1, six).toInt();
          dDetectionProportion = sRecv.substring(six+1, seven).toDouble() / 100;
          nHoldingTime = sRecv.substring(seven+1, eight).toInt();
          int nIndex = 0;
          EEPROM.put(nIndex, nSpeedUnder);            nIndex += sizeof(nSpeedUnder);
          EEPROM.put(nIndex, nSpeedUpper);            nIndex += sizeof(nSpeedUpper);
          EEPROM.put(nIndex, dSNRUnder);              nIndex += sizeof(dSNRUnder);
          EEPROM.put(nIndex, dSNRUpper);              nIndex += sizeof(dSNRUpper);
          EEPROM.put(nIndex, nDetectionTime);         nIndex += sizeof(nDetectionTime);
          EEPROM.put(nIndex, dDetectionProportion);   nIndex += sizeof(dDetectionProportion);
          EEPROM.put(nIndex, nHoldingTime);           nIndex += sizeof(nHoldingTime);
          
          char sSend[82];
          sprintf(sSend, "$SPDPA,%d,%d,%d.%01d,%d.%01d,%d,%d.%01d,%d,r",
                  nSpeedUnder, nSpeedUpper, (int)dSNRUnder, (int)(dSNRUnder * 10) % 10, (int)dSNRUpper, (int)(dSNRUpper * 10) % 10, nDetectionTime, (int)(dDetectionProportion * 100), (int)(dDetectionProportion * 100 * 10) % 10, nHoldingTime);
          sprintf(sSend + strlen(sSend), "*%02x\r\n", CalcCheckSum(sSend));
          Serial.print(sSend);
        }
      }
      else if(strcmp("$SPPWR", sRecv.substring(0, 6).c_str()) == 0){
        
      }
      else if(strcmp("$SPSPQ", sRecv.substring(0, 6).c_str()) == 0){
        int first = sRecv.indexOf(",");        
        String sentence = sRecv.substring(first+1, first+4);
        if(sentence == "DPA")
        {
          char sSend[82];
          sprintf(sSend, "$SPDPA,%d,%d,%d.%01d,%d.%01d,%d,%d.%01d,%d,r",
                  nSpeedUnder, nSpeedUpper, (int)dSNRUnder, (int)(dSNRUnder * 10) % 10, (int)dSNRUpper, (int)(dSNRUpper * 10) % 10, nDetectionTime, (int)(dDetectionProportion * 100), (int)(dDetectionProportion * 100 * 10) % 10, nHoldingTime);
          sprintf(sSend + strlen(sSend), "*%02x\r\n", CalcCheckSum(sSend));
          Serial.print(sSend);
        }
      }
      sRecv = "";
    }
    else
    {
      sRecv += c;
    }
  }
  
  // 32.71 ms for 1 loop, 30.57 for 1 sec.
  // target detection request
  uint8_t tmp;
  uint8_t bDetectNow = false;
  uint8_t bDetect = false;
  memset(velocity, 0, sizeof(velocity));
  memset(SNR, 0, sizeof(SNR));
  //error = my_uRAD.detection(NtarDetected, 0, velocity, SNR, 0, 0, 0);
  return_code = my_uRAD.detection(NtarDetected, 0, velocity, SNR, 0, 0, 0);


    if (return_code != 0) {              // 레이더에서 0이 아닌 값이 계속 들어오면 레이더 오류로 판단후 리셋
      err_alive++;
      Serial.println(err_alive);
      if (err_alive > 400){
        Serial.print("err Alive overflow..");
        soft_restart();
      }
    }
  //if (!error) {
  
    if (!return_code) {
     
    err_alive = 0;
    //Serial.print("return_code: ");
    //Serial.println(return_code);

      //Serial.print(velocity[0] );
      //Serial.println(SNR[0]);
      //Serial.print(velocity[1] );
      //Serial.println(SNR[1]);
      //Serial.print(velocity[2] );
      //Serial.println(SNR[2]);
      //Serial.print(velocity[3] );
      //Serial.println(SNR[3]);
      //Serial.print(velocity[4] );
      //Serial.println(SNR[4]);
      
    int nMaxIndex, nMinIndex;
    float max_speed = -200;
    float min_speed = 200;
    for (uint8_t i = 0; i < Ntar; i++) 
    {  // iterate between targets    
      if(SNR[i] < 0)
        continue;
      if(velocity[i] < -nHighCut)    // 역방향이므로
      {
        velocity[i] = 0;
      }

      if(velocity[0] > 0)   /// 첫번째로 양수가 들어오면 멀어지는것으로 판단 모두 0으로 만든다.
      {
        velocity[0] = 0;
        velocity[1] = 0;
        velocity[2] = 0;
        velocity[3] = 0;
        velocity[4] = 0; 
      }
      
      if(velocity[i] > max_speed)
      {
        max_speed = velocity[i];
        nMaxIndex = i;
      }
      if(velocity[i] < min_speed)
      {
        min_speed = velocity[i];
        nMinIndex = i;
      }  
    }
    
    min_speed = -MPS_TO_KMPH(min_speed);     // 다가오는 속도, km/h 기준으로
    
    if((min_speed != 0) && (sp1 != min_speed))  // 동일한 속도로 계속 감지될 경우 레이더 오류로 감지해 시스템을 리셋한다.
    {
      sp1 = min_speed;
      sp_alive = 0;
    }  
    if((min_speed != 0) && (sp1 == min_speed))
    {
      sp_alive++;
    }
    if(sp_alive > 600)
    {
      Serial.print("overflow reset....");
      delay(1000);
      soft_restart();
    }
    
    if((min_speed > nSpeedUnder) && (min_speed < nSpeedUpper))
    {
      if((SNR[nMinIndex] > dSNRUnder) && (SNR[nMinIndex] < dSNRUpper))
      {
        bDetectNow = true;
      }
    }
    EnQueue(&q, bDetectNow);

    if(bDetectNow && (GetQueueSize(&q) == QUEUE_SIZE))      // 현재 상태가 Detect 되었을 때만 
    {      
      uint8_t b;
      int nDetectCount = 0;
      int SampleToGet = nDetectionTime / 1000. * QUEUE_SIZE + 0.99;
      for(int i = QUEUE_SIZE - 1 ; i >= QUEUE_SIZE - SampleToGet ; i--)
      {
        LookQueue(&q, i, &b);
        if(b)
          nDetectCount++;
      }
      //if(nDetectCount > QUEUE_SIZE * nThresholdDuration / 1000.)
      if(nDetectCount >= SampleToGet * dDetectionProportion)          // Duration 내에 감지횟수가 66% 이상이면
      {
        tsTimeout = millis() + nHoldingTime;
        if(offDetect == true)
        {
          onTimeout = millis() + maxHoldingTime;
          offDetect = false;
        }
        
        //Serial.println(onTimeout);
        //Serial.println(tsTimeout);
        //Serial.println(millis());
        digitalWrite(PIN_RELAY_1, HIGH); 
        digitalWrite(PIN_RELAY_2, HIGH); 
        /*
        char rSend[20];
        sprintf(rSend, "$SPREY,%d",
                relayOn);
        sprintf(rSend + strlen(rSend), "*%02x\r\n", CalcCheckSum(rSend));
        Serial.print(rSend);
        */
#if USE_BUZZER
        tone(PIN_BUZZER, BUZZER_FREQ);
#endif

        if(millis() > onTimeout && digitalRead(PIN_RELAY_1) == HIGH && digitalRead(PIN_RELAY_2 == HIGH))
        {
          Serial.println("Timeout uRAD RESET");
           
          //my_uRAD.turnOFF();
          //digitalWrite(PIN_RAD, LOW);          
          digitalWrite(PIN_RELAY_1, LOW);
          digitalWrite(PIN_RELAY_2, LOW);          
          delay(3000);
          //digitalWrite(PIN_RAD, HIGH);
          //my_uRAD.turnON();
          offDetect = true;
          soft_restart();
        }
      }
    }
        
    if(millis() > tsTimeout)
    {
      digitalWrite(PIN_RELAY_1, LOW);
      digitalWrite(PIN_RELAY_2, LOW);
      offDetect = true;

#if USE_BUZZER
      noTone(PIN_BUZZER);
#endif
    }

    
#if 1
    //if(!(/*(max_speed == 0) &&*/ (min_speed == 0)))
    {
      //if(GetQueueSize(&q) >= 31)
      {
        /*Serial.print("$SPDOP,");
        Serial.print(min_speed);    // to km/h
        Serial.print(",");        
        Serial.print(SNR[nMinIndex]);   
        Serial.print(",");        
        Serial.print(digitalRead(PIN_RELAY_1));
        Serial.print("\r\n");*/
        char sSend[82];
        sprintf(sSend, "$SPDOP,%d.%02d,%d.%02d,%d",
                (int)min_speed, (int)(min_speed * 100) % 100, (int)SNR[nMinIndex], (int)(SNR[nMinIndex] * 100) % 100, digitalRead(PIN_RELAY_1));
        sprintf(sSend + strlen(sSend), "*%02x\r\n", CalcCheckSum(sSend));
        Serial.print(sSend);
      }
    }
#endif
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
// 통신 Sentence
// $SPDOP: Doppler Value - 속도, SNR, 탐지 상태
// $SPDPA: Detection Parameter - 속도 상/하한값, SNR 상/하한값 등 검출을 위한 파라미터
// $SPSPQ: SP Query - 쿼리
