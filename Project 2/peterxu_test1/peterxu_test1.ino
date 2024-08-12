#define TIMER2_INTERVAL_MS 10     //Sets the polling rate of the sensors x TIMER_ITERATION_COUNT VALUE
#define TIMER_ITERATION_COUNT 10  // Number of times TIMER2 overflows before executing timer handler
#define BLUETOOTH_PULSE 500       // Number of times TIMER2 overflows before executing timer handler
#define USE_TIMER_2 true


// Serial Data input pin
#define BLUETOOTH_RX 19
// Serial Data output pin
#define BLUETOOTH_TX 18

#define STARTUP_DELAY 10  // Seconds
#define LOOP_DELAY 10     // miliseconds

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0
// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1



#include "TimerInterrupt.h"
#include "ISR_Timer.h"
#include <Servo.h>  //Need for Servo pulse output


#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
#define NO_HC -SR04   //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK  //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


byte serialRead = 0;  //for control serial communication
int signalADC = 0;    // the read out signal in 0-1023 corresponding to 0-5v
void readSensor();
int sonar();


//IR sensors
int irsensorL = A4;       //Input for Left IR sensor
int irsensorR = A5;       //Input for Right IR sensor
int irsensorLShort = A8;  //Input for Left IR sensor
int irsensorRShort = A9;  //Input for Right IR sensor
int signalADCL = 0;       // the read out signal in 0-1023 corresponding to 0-5v
int signalADCR = 0;       // the read out signal in 0-1023 corresponding to 0-5v
int signalADCLShort = 0;  // the read out signal in 0-1023 corresponding to 0-5v
int signalADCRShort = 0;  // the read out signal in 0-1023 corresponding to 0-5v

//Ultrasonic sensor
const int trigPin = 34;
const int echoPin = 35;
volatile float duration;
volatile int distance;


//Gyroscope
const int gyroPin = A3;     //define the pin that gyro is connected
volatile int loopTime = 0;  // T is the time of one loop
volatile unsigned long prevMillis = 0;
volatile int sensorValue = 0;   // read out value of sensor
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;      // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 3.0;  // because of gyro drifting, defining rotation angular velocity less
                                // than this value will be ignored
volatile float gyroRate = 0;    // read out value of sensor in voltage
volatile float currentAngle = 0;
volatile bool firstRun = true;
volatile bool cornerSet = false;
float gyro();


//Timer interrupt variables
volatile int measureCount = 0;  //Sets the starting point in the array for moving average
volatile int index = 1;         //Sets the starting point in the array replacing old values for moving average
const int avgSize = 10;         //This controls the size of the moving average filter. Reduce to speed up polling
int *refIRSens;
int *offIRSens;

//These are treated as circular arrays and store the values for the moving average calculation
volatile int arrIRL[avgSize] = { 0 };
volatile int arrIRR[avgSize] = { 0 };
volatile int arrIRLShort[avgSize] = { 0 };
volatile int arrIRRShort[avgSize] = { 0 };
volatile int arrUltra[avgSize] = { 0 };
volatile float arrGyro[avgSize] = { 0 };

//sumXXX is the sum of the values stored in arrXXX
//distXXX is the averaged distance calculated from sumXXX
volatile int distIRL;
volatile int sumIRL;
volatile int distIRR;
volatile int sumIRR;
volatile int distIRLShort;
volatile int sumIRLShort;
volatile int distIRRShort;
volatile int sumIRRShort;
volatile int distUltra;
volatile int sumUltra;
volatile float angGyro = 180;
volatile float sumGyro;
volatile float absAngle = 0;

volatile float IROffset = 46.0;
volatile float SonarOffset = 63.0;
volatile int timerCounter = 1;
volatile int bluetoothCounter = 0;

volatile float referenceAngle;

volatile int xValue = 0;
volatile int prevXValue = 0;
volatile int yValue = 0;
volatile int handOffOffSet = 0;
volatile int prevYValue = 0;
volatile int debugValue1 = 0;
volatile int debugValue2 = 0;
volatile int debugValue3 = 0;

volatile bool positionSet = false;
volatile bool switched = false;
volatile bool ShouldISwitch = false;

volatile int switchPoint;
volatile int difference;
volatile int IRRStart;

//Debug variables for calculating timing of a function
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;


// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo BackRightMotor;   // create servo object to control Vex Motor Controller 29
Servo FrontRightMotor;  // create servo object to control Vex Motor Controller 29
Servo FrontLeftMotor;   // create servo object to control Vex Motor Controller 29
Servo BackLeftMotor;    // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int speed_val = 100;
int speed_change;


//Timer interrupt function
void TimerHandler1() {
  if (timerCounter == TIMER_ITERATION_COUNT) {
    int temp;  //used to temperarily store variables

    /* 
    Each block of code essentially works the same,
    1. Store the calculed distance/value in temp
    2. Remove the old value from the calculated sum
    3. Store the new value in the appropriate position in the circular array
    4. Add the new value to the sum
    5. Calculate the distance from the average
  */
    signalADCL = analogRead(irsensorL);
    temp = 25905 * pow(signalADCL, -0.901);
    sumIRL -= arrIRL[measureCount];
    arrIRL[measureCount] = temp;
    sumIRL += temp;
    distIRL = getAverage(arrIRL) + IROffset;

    signalADCR = analogRead(irsensorR);
    temp = 114462 * pow(signalADCR, -1.146);
    sumIRR -= arrIRR[measureCount];
    arrIRR[measureCount] = temp;
    sumIRR += temp;
    distIRR = getAverage(arrIRR) + IROffset;

    signalADCLShort = analogRead(irsensorLShort);
    temp = 13595 * pow(signalADCLShort, -0.916);
    sumIRLShort -= arrIRLShort[measureCount];
    arrIRLShort[measureCount] = temp;
    sumIRLShort += temp;
    distIRLShort = getAverage(arrIRLShort);

    signalADCRShort = analogRead(irsensorRShort);
    temp = 23519 * pow(signalADCRShort, -1.014);
    sumIRRShort -= arrIRRShort[measureCount];
    arrIRRShort[measureCount] = temp;
    sumIRRShort += temp;
    distIRRShort = getAverage(arrIRRShort);

    temp = sonar();
    sumUltra -= arrUltra[measureCount];
    arrUltra[measureCount] = temp;
    sumUltra += temp;
    distUltra = getAverage(arrUltra);


    angGyro = gyro();

    //Coordernate position - peter wrote this
    if (positionSet) {
      xValue = 2000 - distUltra - SonarOffset;
      if (xValue < 0) { xValue = 0; }  //Prevents the X value from going below 0
      if (yValue > 600) {
        switched = true;
      }
      if (switched) {
        // yValue = switchPoint + (difference) * float(IRRStart - switchPoint) / (1200.0 - float(IRRStart)) * (float(IRRStart) - float(distIRR)) + (IRRStart - distIRR);
        yValue = 1200 - distIRR;
      } else {
        yValue = distIRL;
      }
      if (yValue < 0) { yValue = 0; }
    } else {
      xValue = 0;
      yValue = 0;
    }
    //The counter variable used to index the circular arrays
    if (measureCount == avgSize - 1) {
      measureCount = 0;
    } else {
      measureCount++;
    }
  } else {
    timerCounter += 1;
  }
}

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;

void setup(void) {

  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(19, INPUT);
  pinMode(18, OUTPUT);

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  digitalWrite(trigPin, LOW);

  ITimer2.init();
  if (ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, TimerHandler1)) {
    Serial.print(F("Starting  ITimer4 OK, millis() = "));
    Serial.println(millis());
  }

  Serial1.begin(115200);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  // delay(1000);
  SerialCom->println("Setup....");

  float sum = 0;

  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (int i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    sensorValue = analogRead(gyroPin);
    sum += sensorValue;
    delay(20);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
}

void loop(void)  //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:  //Lipo Battery Volage OK
      machine_state = running();
      break;
    case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000);  //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {




  static unsigned long previous_millis;

  // --==+== Code Starts Here ==+==--


  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC - SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


    turret_motor.write(pos);

    if (pos == 0) {
      pos = 45;
    } else {
      pos = 0;
    }
  }

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {  //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}
void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}
#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif
#ifndef NO_HC - SR04
#endif
#ifndef NO_READ_GYRO
#endif
//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  BackRightMotor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  FrontRightMotor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  FrontLeftMotor.detach();   // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  BackLeftMotor.detach();    // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  BackRightMotor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  FrontRightMotor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  FrontLeftMotor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  BackLeftMotor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

int sonar() {
  //noInterrupts();
  startTime = micros();
  delayMicroseconds(1000);
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = (duration * 0.34 / 2) * 1.0259 - 15.375;
  // Prints the distance on the Serial Monitor
  endTime = micros();
  //interrupts();
  return (distance);
}

float gyro() {
  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023;
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * gyroSupplyVoltage);
  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = -1 * gyroRate / gyroSensitivity;  // from Data Sheet, gyroSensitivity is 0.007 V/dps
  loopTime = millis() - prevMillis;
  prevMillis = millis();
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    //Because the execution time varies depending on the other functions being executed, this
    //keeps track of how much time has elapsed
    // we are running a loop of loopTime/1000 second).
    float fluff = (75.0 * float(loopTime)) / 50000.0;
    fluff = 0;
    float angleChange = angularVelocity / (1000 / loopTime) + fluff;
    currentAngle += angleChange;
    absAngle += angleChange;
    if (currentAngle < 0) {
      currentAngle += 360;
    }
  }
  if (currentAngle > 360) {
    float tempAnglePoint = currentAngle - int(currentAngle);
    currentAngle = float(int(currentAngle) % 360);
    currentAngle += tempAnglePoint;
  }
  if (firstRun) {  //Because of the variable timing, the first result must be disgarded
    firstRun = false;
    return 0.0;
  } else {
    return currentAngle;
  }
}

int sort_desc(const void *cmp1, const void *cmp2) {
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

int getAverage(int arrayInput[avgSize]) {
  int sortedArray[avgSize];
  for (int i = 0; i < avgSize; i++) {
    sortedArray[i] = arrayInput[i];
  }

  qsort(sortedArray, avgSize, sizeof(sortedArray[0]), sort_desc);
  int sum = 0;
  for (int i = 1; i < avgSize - 2; i++) {
    sum += sortedArray[i];
  }
  return sum / 7;
}

// front_left = lineariseMotors(front_left, 1);
// front_right = lineariseMotors(front_right, 2);
// back_left = lineariseMotors(back_left, 3);
// back_right = lineariseMotors(back_right, 4);

int lineariseMotors(int pwr, int motorNum) {
  if (abs(pwr) < 10) { return 0; }

  if (pwr >= 3) {
    switch (motorNum) {
      case 1: return pwr + 40;
      case 2: return pwr + 60;
      case 3: return pwr + 40;
      case 4: return pwr + 60;
    }
  } else if (pwr <= -3) {
    switch (motorNum) {
      case 1: return pwr - 60;
      case 2: return pwr - 40;
      case 3: return pwr - 60;
      case 4: return pwr - 40;
    }
  }
}
<<<<<<< Updated upstream

void drive(float v_x, float v_y, float omega_z) {
  
}
=======
>>>>>>> Stashed changes
