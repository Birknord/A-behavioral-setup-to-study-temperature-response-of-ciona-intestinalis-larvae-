/************************************************
 * Includes
 ************************************************/
#include <Adafruit_MAX31865.h>   // For PT1000 sensors
#include <Nextion.h>            // Official Nextion library
#include <SoftwareSerial.h>     // We use SoftwareSerial for Nextion

/************************************************
 * 1) Nextion & Shared Serial
 ************************************************/
// Nextion on D4 (TX) and D3 (RX):
//  => Arduino pin 4 = Nextion RX, pin 3 = Nextion TX
SoftwareSerial mySerial(3, 4);

/************************************************
 * 2) FIRST SYSTEM (H-bridge 1) - Page 0 - left
 ************************************************/
// PT1000 #1 via MAX31865 #1
//   CS=A4, DI=A3, DO=A2, CLK=A1
Adafruit_MAX31865 thermo1(A4, A3, A2, A1);
#define RREF1     4301.0
#define RNOMINAL1 1000.0

// BTS7960 pins
#define BTS_PWM_L1 9
#define BTS_PWM_R1 6

// Fan #1 pin
#define FAN_PIN1 13

// Nextion Page=0 Objects
//   t1 => (0,9), n1 => (0,5), b0 => (0,4), b1 => (0,6)
NexText   tempDisplay1   = NexText(0, 9, "t1");
NexNumber targetDisplay1 = NexNumber(0, 5, "n1");
NexButton btnMinus1      = NexButton(0, 4, "b0");
NexButton btnPlus1       = NexButton(0, 6, "b1");

// PI #1
float targetTemp1  = 28.0;  // default
float Kp1 = 10.0, Ki1 = 2.5;
float errorSum1    = 0;
int   pwmVal1      = 0;

// Button callbacks for system #1
void decreaseTemp1(void *ptr)
{
  Serial.println("System #1: Minus button pressed!");
  targetTemp1 = max(targetTemp1 - 1, 0);
  targetDisplay1.setValue((int)targetTemp1);
  Serial.print("Target1 Decreased => ");
  Serial.println(targetTemp1);
}

void increaseTemp1(void *ptr)
{
  Serial.println("System #1: Plus button pressed!");
  targetTemp1 = min(targetTemp1 + 1, 100);
  targetDisplay1.setValue((int)targetTemp1);
  Serial.print("Target1 Increased => ");
  Serial.println(targetTemp1);
}

/************************************************
 * 3) SECOND SYSTEM (H-bridge 2) - Page 1
 ************************************************/
// PT1000 #2 via MAX31865 #2
//   CS=A5, DI=A3, DO=A2, CLK=A1
Adafruit_MAX31865 thermo2(A5, A3, A2, A1);
#define RREF2     4301.0
#define RNOMINAL2 1000.0

// H-bridge 2 pins
#define BTS_PWM_L2 10
#define BTS_PWM_R2 11


// Fan #2 pin
#define FAN_PIN2 5

// Nextion Page=1 Objects
//   t4 => (1,9), n2 => (1,5), b3 => (1,5?), b4 => (1,6?)
//   *** Adjust if your actual pageIDs or compIDs differ ***
NexText   tempDisplay2   = NexText(1, 9, "t4");
NexNumber targetDisplay2 = NexNumber(1, 5, "n2");
NexButton btnMinus2      = NexButton(1, 5, "b3");
NexButton btnPlus2       = NexButton(1, 6, "b4");

// PI #2
float targetTemp2  = 14.0;  // default
float Kp2 = 10.0, Ki2 = 0.5;
float errorSum2    = 0;
int   pwmVal2      = 0;

// Button callbacks for system #2
void decreaseTemp2(void *ptr)
{
  Serial.println("System #2: Minus button pressed!");
  targetTemp2 = max(targetTemp2 - 1, 0);
  targetDisplay2.setValue((int)targetTemp2);
  Serial.print("Target2 Decreased => ");
  Serial.println(targetTemp2);
}

void increaseTemp2(void *ptr)
{
  Serial.println("System #2: Plus button pressed!");
  targetTemp2 = min(targetTemp2 + 1, 100);
  targetDisplay2.setValue((int)targetTemp2);
  Serial.print("Target2 Increased => ");
  Serial.println(targetTemp2);
}

/************************************************
 * 4) Combine Nextion Objects for All Buttons
 ************************************************/
NexTouch *nex_listen_list[] = {
  &btnMinus1,
  &btnPlus1,
  &btnMinus2,
  &btnPlus2,
  NULL
};

/************************************************
 * setup()
 ************************************************/
void setup()
{
  // --- Pins for system #1 (BTS7960 + Fan1) ---
  pinMode(BTS_PWM_L1, OUTPUT);
  pinMode(BTS_PWM_R1, OUTPUT);
  pinMode(FAN_PIN1, OUTPUT);

  // --- Pins for system #2 (L298N + Fan2) ---
  pinMode(BTS_PWM_L2, OUTPUT);
  pinMode(BTS_PWM_R2, OUTPUT);
  pinMode(FAN_PIN2, OUTPUT);

  // Start both MAX31865s
  thermo1.begin(MAX31865_2WIRE);
  thermo2.begin(MAX31865_2WIRE);

  // Debug Serial
  Serial.begin(9600);

  // Nextion
  mySerial.begin(9600);
  nexInit();  // no arguments

  // Attach button callbacks for both systems
  btnMinus1.attachPush(decreaseTemp1);
  btnPlus1.attachPush(increaseTemp1);
  btnMinus2.attachPush(decreaseTemp2);
  btnPlus2.attachPush(increaseTemp2);

  Serial.println("Merged System Initialized ✅");
}

/************************************************
 * loop()
 ************************************************/
void loop()
{
  // Listen for ANY Nextion events (all pages)
  nexLoop(nex_listen_list);

  // ---- [1] System #1: BTS7960 + PT1000 #1
  {
    float currentTemp1 = thermo1.temperature(RNOMINAL1, RREF1);
    if (currentTemp1 < -100 || currentTemp1 > 200) {
      Serial.println("System #1: Invalid reading!");
    } else {
      // PI for system #1
      float error1 = (targetTemp1 - currentTemp1);
      errorSum1    = constrain(errorSum1 + error1, -50, 50);
      float rawPow1 = (Kp1 * error1) + (Ki1 * errorSum1);
      pwmVal1       = constrain(abs(rawPow1), 0, 255);

      // Update Nextion page0 objects (temp, target)
      char buf1[10];
      dtostrf(currentTemp1, 6, 2, buf1);
      tempDisplay1.setText(buf1);
      targetDisplay1.setValue((int)targetTemp1);

      // Debug
      Serial.print("[System #1] Temp=");
      Serial.print(currentTemp1);
      Serial.print("°C  Target=");
      Serial.print(targetTemp1);
      Serial.print("°C  rawPower=");
      Serial.print(rawPow1);
      Serial.print(" => PWM=");
      Serial.print(pwmVal1);

      // Decide heating vs. cooling
      if (rawPow1 > 0) {
        // Heating
        Serial.println(" | Mode=Heating");
        analogWrite(BTS_PWM_L1, pwmVal1); // Heat
        analogWrite(BTS_PWM_R1, 0);
        digitalWrite(FAN_PIN1, LOW);
      }
      else if (rawPow1 < 0) {
        // Cooling
        Serial.println(" | Mode=Cooling");
        analogWrite(BTS_PWM_L1, 0);
        analogWrite(BTS_PWM_R1, pwmVal1); // Cool
        digitalWrite(FAN_PIN1, HIGH);
      }
      else {
        // rawPower=0 => off
        Serial.println(" | Mode=Off");
        analogWrite(BTS_PWM_L1, 0);
        analogWrite(BTS_PWM_R1, 0);
        digitalWrite(FAN_PIN1, LOW);
      }
    }
  }

  // ---- [2] System #2: BTS79602 + PT1000 #2
  {
    float currentTemp2 = thermo2.temperature(RNOMINAL2, RREF2);
    if (currentTemp2 < -100 || currentTemp2 > 200) {
      Serial.println("System #2: Invalid reading!");
    } else {
      // PI for system #2
      float error2 = (targetTemp2 - currentTemp2);
      errorSum2    = constrain(errorSum2 + error2, -50, 50);
      float rawPow2 = (Kp2 * error2) + (Ki2 * errorSum2);
      pwmVal2       = constrain(abs(rawPow2), 0, 255);

      // Update Nextion page1 objects (temp, target)
      char buf2[10];
      dtostrf(currentTemp2, 6, 2, buf2);
      tempDisplay2.setText(buf2);
      targetDisplay2.setValue((int)targetTemp2);

      // Debug
      Serial.print("[System #2] Temp=");
      Serial.print(currentTemp2);
      Serial.print("°C  Target=");
      Serial.print(targetTemp2);
      Serial.print("°C  rawPower=");
      Serial.print(rawPow2);
      Serial.print(" => PWM=");
      Serial.print(pwmVal2);

      // Decide heating vs. cooling
          if (rawPow2 > 0) {
        // Heating
        Serial.println(" | Mode=Heating");
        analogWrite(BTS_PWM_L2, pwmVal2); // Heat
        analogWrite(BTS_PWM_R2, 0);
        digitalWrite(FAN_PIN2, LOW);
      }
      else if (rawPow2 < 0) {
        // Cooling
        Serial.println(" | Mode=Cooling");
        analogWrite(BTS_PWM_L2, 0);
        analogWrite(BTS_PWM_R2, pwmVal2); // Cool
        digitalWrite(FAN_PIN2, HIGH);
      }
      else {
        // rawPower=0 => off
        Serial.println(" | Mode=Off");
        analogWrite(BTS_PWM_L2, 0);
        analogWrite(BTS_PWM_R2, 0);
        digitalWrite(FAN_PIN2, LOW);
      }
    }
  }

  delay(500); // short pause
}
