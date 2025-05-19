#include <Arduino.h>            // <<–– lagt til
#include <Adafruit_MAX31865.h>
#include <Nextion.h>
#include <SoftwareSerial.h>

/************ Global tweaks ****************************************/
const float   COOL_OVERSHOOT_FACTOR = 2.0;
const uint8_t COOL_MIN_PWM         = 1300;

/************ Nextion & serial *************************************/
SoftwareSerial mySerial(3, 4);

/************ System #1 ********************************************/
Adafruit_MAX31865 thermo2(A4, A3, A2, A1);
#define RREF1 4301.0
#define RNOMINAL1 1000.0
#define BTS_PWM_L1 9
#define BTS_PWM_R1 6
#define FAN_PIN1 13
NexText   tempDisplay1   = NexText(0, 9, "t1");
NexNumber targetDisplay1 = NexNumber(0, 5, "n1");
NexButton btnMinus1      = NexButton(0, 4, "b0");
NexButton btnPlus1       = NexButton(0, 6, "b1");
float targetTemp1 = 28.0;
float Kp1 = 15.0, Ki1 = 1.5;
float errorSum1 = 0;
int   pwmVal1   = 0;

/************ System #2 ********************************************/
Adafruit_MAX31865 thermo1(A5, A3, A2, A1);
#define RREF2 4301.0
#define RNOMINAL2 1000.0
#define BTS_PWM_L2 10
#define BTS_PWM_R2 11
#define FAN_PIN2 5
NexText   tempDisplay2   = NexText(1, 9, "t4");
NexNumber targetDisplay2 = NexNumber(1, 5, "n2");
NexButton btnMinus2      = NexButton(1, 5, "b3");
NexButton btnPlus2       = NexButton(1, 6, "b4");
float targetTemp2 = 14.0;
float Kp2 = 15.0, Ki2 = 1.5;
float errorSum2 = 0;
int   pwmVal2   = 0;

/************ Button callbacks *************************************/
void decreaseTemp1(void*) { targetTemp1 = max(targetTemp1 - 1, 0.0f);  targetDisplay1.setValue((int)targetTemp1); }
void increaseTemp1(void*) { targetTemp1 = min(targetTemp1 + 1, 100.0f); targetDisplay1.setValue((int)targetTemp1); }
void decreaseTemp2(void*) { targetTemp2 = max(targetTemp2 - 1, 0.0f);  targetDisplay2.setValue((int)targetTemp2); }
void increaseTemp2(void*) { targetTemp2 = min(targetTemp2 + 1, 100.0f); targetDisplay2.setValue((int)targetTemp2); }

NexTouch *nex_listen_list[] = { &btnMinus1,&btnPlus1,&btnMinus2,&btnPlus2,NULL };

/************ setup() **********************************************/
void setup()
{
  pinMode(BTS_PWM_L1, OUTPUT); pinMode(BTS_PWM_R1, OUTPUT); pinMode(FAN_PIN1, OUTPUT);
  pinMode(BTS_PWM_L2, OUTPUT); pinMode(BTS_PWM_R2, OUTPUT); pinMode(FAN_PIN2, OUTPUT);

  thermo1.begin(MAX31865_2WIRE);
  thermo2.begin(MAX31865_2WIRE);

  Serial.begin(9600);
  mySerial.begin(9600); nexInit();

  btnMinus1.attachPush(decreaseTemp1); btnPlus1.attachPush(increaseTemp1);
  btnMinus2.attachPush(decreaseTemp2); btnPlus2.attachPush(increaseTemp2);

  Serial.println(F("Merged System Initialized ✅"));
}

/************ loop() ***********************************************/
void loop()
{
  nexLoop(nex_listen_list);

  /* ---------- SYSTEM 1 ---------- */
  {
    float T = thermo1.temperature(RNOMINAL1, RREF1);
    if (T > -100 && T < 200)
    {
      float err  = targetTemp1 - T;
      errorSum1  = constrain(errorSum1 + err, -50, 50);
      float raw  = (Kp1 * err) + (Ki1 * errorSum1);
      if (targetTemp1 < 16.0 && raw < 0) raw *= COOL_OVERSHOOT_FACTOR;
      pwmVal1 = constrain(abs(raw), 0, 255);

      if (targetTemp1 < 16.0 && raw < 0 && (T - targetTemp1) > 0.3)
        pwmVal1 = max(pwmVal1, (int)COOL_MIN_PWM);          

      Serial.print(F("[S1] T="));Serial.print(T,2);
      Serial.print(F(" tgt="));Serial.print(targetTemp1,1);
      Serial.print(F(" raw="));Serial.print(raw);
      Serial.print(F(" PWM="));Serial.println(pwmVal1);

      char buf[10]; dtostrf(T,6,2,buf); tempDisplay1.setText(buf);
      targetDisplay1.setValue((int)targetTemp1);

      if (raw>0){ analogWrite(BTS_PWM_L1,pwmVal1); analogWrite(BTS_PWM_R1,0); digitalWrite(FAN_PIN1,LOW);}
      else if(raw<0){analogWrite(BTS_PWM_L1,0); analogWrite(BTS_PWM_R1,pwmVal1); digitalWrite(FAN_PIN1,HIGH);}
      else{analogWrite(BTS_PWM_L1,0); analogWrite(BTS_PWM_R1,0); digitalWrite(FAN_PIN1,LOW);}
    }
  }

  /* ---------- SYSTEM 2 ---------- */
  {
    float T = thermo2.temperature(RNOMINAL2, RREF2);
    if (T > -100 && T < 200)
    {
      float err  = targetTemp2 - T;
      errorSum2  = constrain(errorSum2 + err, -50, 50);
      float raw  = (Kp2 * err) + (Ki2 * errorSum2);
      if (targetTemp2 < 16.0 && raw < 0) raw *= COOL_OVERSHOOT_FACTOR;
      pwmVal2 = constrain(abs(raw), 0, 255);

      if (targetTemp2 < 16.0 && raw < 0 && (T - targetTemp2) > 0.3)
        pwmVal2 = max(pwmVal2, (int)COOL_MIN_PWM);          

      Serial.print(F("[S2] T="));Serial.print(T,2);
      Serial.print(F(" tgt="));Serial.print(targetTemp2,1);
      Serial.print(F(" raw="));Serial.print(raw);
      Serial.print(F(" PWM="));Serial.println(pwmVal2);

      char buf[10]; dtostrf(T,6,2,buf); tempDisplay2.setText(buf);
      targetDisplay2.setValue((int)targetTemp2);

      if (raw>0){ analogWrite(BTS_PWM_L2,pwmVal2); analogWrite(BTS_PWM_R2,0); digitalWrite(FAN_PIN2,LOW);}
      else if(raw<0){analogWrite(BTS_PWM_L2,0); analogWrite(BTS_PWM_R2,pwmVal2); digitalWrite(FAN_PIN2,HIGH);}
      else{analogWrite(BTS_PWM_L2,0); analogWrite(BTS_PWM_R2,0); digitalWrite(FAN_PIN2,LOW);}
    }
  }

  delay(500);
}
