/************************************************
 *  Includes
 ************************************************/
#include <Adafruit_MAX31865.h>
#include <Nextion.h>
#include <SoftwareSerial.h>

/************************************************
 * 1) Nextion & Shared Serial
 ************************************************/
SoftwareSerial mySerial(3, 4);

/************************************************
 * 2) FIRST SYSTEM – Page 0
 ************************************************/
Adafruit_MAX31865 thermo2(A4, A3, A2, A1);
#define RREF1     4301.0
#define RNOMINAL1 1000.0
#define BTS_PWM_L1  9
#define BTS_PWM_R1  6
#define FAN_PIN1   13

NexText   tempDisplay1   = NexText(0, 9, "t1");
NexNumber targetDisplay1 = NexNumber(0, 5, "n1");
NexButton btnMinus1      = NexButton(0, 4, "b0");
NexButton btnPlus1       = NexButton(0, 6, "b1");

float targetTemp1  = 28.0;
float Kp1 = 15.0, Ki1 = 1.5;
float errorSum1 = 0;
int   pwmVal1   = 0;

/************************************************
 * 3) SECOND SYSTEM – Page 1
 ************************************************/
Adafruit_MAX31865 thermo1(A5, A3, A2, A1);
#define RREF2     4301.0
#define RNOMINAL2 1000.0
#define BTS_PWM_L2 10
#define BTS_PWM_R2 11
#define FAN_PIN2   5

NexText   tempDisplay2   = NexText(1, 9, "t4");
NexNumber targetDisplay2 = NexNumber(1, 5, "n2");
NexButton btnMinus2      = NexButton(1, 5, "b3");
NexButton btnPlus2       = NexButton(1, 6, "b4");

float targetTemp2  = 14.0;
float Kp2 = 15.0, Ki2 = 1.5;
float errorSum2 = 0;
int   pwmVal2   = 0;

/************************************************
 * 4) Felles Nextion-liste
 ************************************************/
NexTouch *nex_listen_list[] = {
  &btnMinus1, &btnPlus1,
  &btnMinus2, &btnPlus2,
  NULL
};

/************************************************
 * 5)  Spesial­håndtering for kjøling < 20 °C
 ************************************************/
#define COOL_LIMIT_TEMP      20.0   // aktiver logikk kun under dette
#define FORCE_COOL_PWM       255    // konstant PWM mens vi ennå er over målet
#define HIGH_DIFF            2.0    // beholdt for “langt unna” (kan fjernes)
#define STUCK_DIFF           1.0
#define STUCK_TIMEOUT        30000UL
#define BOOST_STEP           80

unsigned long stuckStart1 = 0;
unsigned long stuckStart2 = 0;

/************************************************
 * 6)  Knapp-callbacks
 ************************************************/
void decreaseTemp1(void*) { targetTemp1 = max(targetTemp1 - 1, 0);  targetDisplay1.setValue((int)targetTemp1); }
void increaseTemp1(void*) { targetTemp1 = min(targetTemp1 + 1, 100); targetDisplay1.setValue((int)targetTemp1); }
void decreaseTemp2(void*) { targetTemp2 = max(targetTemp2 - 1, 0);  targetDisplay2.setValue((int)targetTemp2); }
void increaseTemp2(void*) { targetTemp2 = min(targetTemp2 + 1, 100); targetDisplay2.setValue((int)targetTemp2); }

/************************************************
 * setup()
 ************************************************/
void setup()
{
  pinMode(BTS_PWM_L1, OUTPUT); pinMode(BTS_PWM_R1, OUTPUT); pinMode(FAN_PIN1, OUTPUT);
  pinMode(BTS_PWM_L2, OUTPUT); pinMode(BTS_PWM_R2, OUTPUT); pinMode(FAN_PIN2, OUTPUT);

  thermo1.begin(MAX31865_2WIRE);
  thermo2.begin(MAX31865_2WIRE);

  Serial.begin(9600);
  mySerial.begin(9600);
  nexInit();

  btnMinus1.attachPush(decreaseTemp1); btnPlus1.attachPush(increaseTemp1);
  btnMinus2.attachPush(decreaseTemp2); btnPlus2.attachPush(increaseTemp2);

  Serial.println("System ready ✅");
}

/************************************************
 * coolingControl()
 *  – tvinger PWM = 255 helt til målet er nådd
 ************************************************/
int coolingControl(float currentTemp, float targetTemp,
                   float rawPower, int pidPWM,
                   unsigned long &stuckTimer)
{
  /*  Ingen spesiallogikk hvis målet er ≥ 20 °C eller vi IKKE er over målet */
  if (targetTemp >= COOL_LIMIT_TEMP || currentTemp <= targetTemp) return pidPWM;

  /*  1) Hold MAX kjøling mens vi fortsatt ligger over ønsket temperatur   */
  return FORCE_COOL_PWM;

  /*  — all logikk under blir aldri nådd før målet er passert —            */
  /*  (beholdt bare for referanse dersom du vil skru tilbake senere)       */

  /*  Langt unna (>2 °C)                                                      
  if (currentTemp - targetTemp > HIGH_DIFF) {
    stuckTimer = 0;
    return FORCE_COOL_PWM;
  }

  /*  “Stuck” (>1 °C over mål i 30 s)                                        
  if ((currentTemp - targetTemp) > STUCK_DIFF && rawPower < 0) {
    if (stuckTimer == 0) stuckTimer = millis();
    else if (millis() - stuckTimer > STUCK_TIMEOUT)
      return min(FORCE_COOL_PWM, pidPWM + BOOST_STEP);
  } else stuckTimer = 0;

  return pidPWM;
  */
}

/************************************************
 * loop()
 ************************************************/
void loop()
{
  nexLoop(nex_listen_list);

  /**************  System #1  *******************************************/
  {
    float t = thermo1.temperature(RNOMINAL1, RREF1);
    if (t > -100 && t < 200) {

      float error = targetTemp1 - t;
      errorSum1   = constrain(errorSum1 + error, -50, 50);
      float rawP  = (Kp1 * error) + (Ki1 * errorSum1);
      int   pidPWM= constrain(abs(rawP), 0, 255);

      pwmVal1 = coolingControl(t, targetTemp1, rawP, pidPWM, stuckStart1);

      if (rawP > 0) { // varme
        analogWrite(BTS_PWM_L1, pwmVal1); analogWrite(BTS_PWM_R1, 0);  digitalWrite(FAN_PIN1, LOW);
      } else if (rawP < 0) { // kjøling
        analogWrite(BTS_PWM_L1, 0);       analogWrite(BTS_PWM_R1, pwmVal1); digitalWrite(FAN_PIN1, HIGH);
      } else { // av
        analogWrite(BTS_PWM_L1, 0);       analogWrite(BTS_PWM_R1, 0);  digitalWrite(FAN_PIN1, LOW);
      }

      char buf[10]; dtostrf(t, 6, 2, buf);
      tempDisplay1.setText(buf); targetDisplay1.setValue((int)targetTemp1);

      Serial.print("[System #1] Temp=");  Serial.print(t, 2);
      Serial.print("°C  Target=");        Serial.print(targetTemp1, 1);
      Serial.print("°C  PWM=");           Serial.println(pwmVal1);
    }
  }

  /**************  System #2  *******************************************/
  {
    float t = thermo2.temperature(RNOMINAL2, RREF2);
    if (t > -100 && t < 200) {

      float error = targetTemp2 - t;
      errorSum2   = constrain(errorSum2 + error, -50, 50);
      float rawP  = (Kp2 * error) + (Ki2 * errorSum2);
      int   pidPWM= constrain(abs(rawP), 0, 255);

      pwmVal2 = coolingControl(t, targetTemp2, rawP, pidPWM, stuckStart2);

      if (rawP > 0) {
        analogWrite(BTS_PWM_L2, pwmVal2); analogWrite(BTS_PWM_R2, 0);  digitalWrite(FAN_PIN2, LOW);
      } else if (rawP < 0) {
        analogWrite(BTS_PWM_L2, 0);       analogWrite(BTS_PWM_R2, pwmVal2); digitalWrite(FAN_PIN2, HIGH);
      } else {
        analogWrite(BTS_PWM_L2, 0);       analogWrite(BTS_PWM_R2, 0);  digitalWrite(FAN_PIN2, LOW);
      }

      char buf[10]; dtostrf(t, 6, 2, buf);
      tempDisplay2.setText(buf); targetDisplay2.setValue((int)targetTemp2);

      Serial.print("[System #2] Temp=");  Serial.print(t, 2);
      Serial.print("°C  Target=");        Serial.print(targetTemp2, 1);
      Serial.print("°C  PWM=");           Serial.println(pwmVal2);
    }
  }

  delay(500);
}
