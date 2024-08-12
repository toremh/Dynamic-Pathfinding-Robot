/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 10/03/2023
  Author: Bhumik Patel
*/

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
    //temp = 167490 * pow(signalADCL, -1.204);
    //temp = 37662.0 * pow(signalADCL, -0.963);
    //temp = signalADCL;
    temp = 25905 * pow(signalADCL, -0.901);
    sumIRL -= arrIRL[measureCount];
    arrIRL[measureCount] = temp;
    sumIRL += temp;
    // distIRL = (sumIRL / ((float)avgSize)) + IROffset;
    distIRL = getAverage(arrIRL) + IROffset;
    //distIRL = (sumIRL / ((float)avgSize));

    signalADCR = analogRead(irsensorR);
    //temp = 168600 * pow(signalADCR, -1.204);
    //temp = 19708.0 * pow(signalADCR, -0.857);
    //temp = signalADCR;
    temp = 114462 * pow(signalADCR, -1.146);
    sumIRR -= arrIRR[measureCount];
    arrIRR[measureCount] = temp;
    sumIRR += temp;
    distIRR = getAverage(arrIRR) + IROffset;
    // distIRR = (sumIRR / ((float)avgSize)) + IROffset;

    signalADCLShort = analogRead(irsensorLShort);
    //temp = 167490 * pow(signalADCL, -1.204);
    //temp = 37662.0 * pow(signalADCL, -0.963);
    //temp = signalADCL;
    temp = 13595 * pow(signalADCLShort, -0.916);
    sumIRLShort -= arrIRLShort[measureCount];
    arrIRLShort[measureCount] = temp;
    sumIRLShort += temp;
    distIRLShort = getAverage(arrIRLShort);
    // distIRLShort = (sumIRLShort / ((float)avgSize));  // * (96.0 / 90.0);

    signalADCRShort = analogRead(irsensorRShort);
    //temp = 168600 * pow(signalADCR, -1.204);
    //temp = 19708.0 * pow(signalADCR, -0.857);
    //temp = signalADCR;
    temp = 23519 * pow(signalADCRShort, -1.014);
    sumIRRShort -= arrIRRShort[measureCount];
    arrIRRShort[measureCount] = temp;
    sumIRRShort += temp;
    distIRRShort = getAverage(arrIRRShort);
    // distIRRShort = (sumIRRShort / ((float)avgSize));

    temp = sonar();
    sumUltra -= arrUltra[measureCount];
    arrUltra[measureCount] = temp;
    sumUltra += temp;
    distUltra = getAverage(arrUltra);
    // distUltra = (sumUltra / ((float)avgSize));


    angGyro = gyro();



    //Coordernate position
    if (positionSet) {
      xValue = 2000 - distUltra - SonarOffset;
      if (xValue < 0) { xValue = 0; }  //Prevents the X value from going below 0

      // abs(distIRL - distIRR) < 10)
      // if ((abs(distIRR - distIRL) < 100) && (!switched) && distIRL > 0) {
      //   switched = true;
      //   //// Serial1.println("SWITCHED SWITCHED SWITCHED SWITCHED SWITCHED");
      //   //// Serial1.println(abs(distIRR - distIRL));
      //   //// Serial1.println(distIRL);
      //   //// Serial1.println(distIRR);
      //   handOffOffSet = distIRR - distIRL;  //Ensures a smooth transition between left and right IR sensors
      // }
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

    if (bluetoothCounter == (BLUETOOTH_PULSE / (TIMER_ITERATION_COUNT * TIMER2_INTERVAL_MS))) {
      serialOutput(xValue, yValue, debugValue1, debugValue2, debugValue3);
      if (!ShouldISwitch && bluetoothCounter == 5) {
        ShouldISwitch = !ShouldISwitch;
      }
      bluetoothCounter = 1;
    }
    bluetoothCounter += 1;
    timerCounter = 1;
  } else {
    timerCounter += 1;
  }
}


void bluetoothSerialOutputMonitor(int Value1, int Value2, int Value3, int Value4, int Value5) {
  // disable_motors();
  String Delimiter = " ";

  Serial1.print(Value1);
  Serial1.print(Delimiter);
  Serial1.print(Value2);
  Serial1.print(Delimiter);
  Serial1.print(Value3);
  Serial1.print(Delimiter);
  Serial1.print(Value4);
  Serial1.print(Delimiter);
  Serial1.println(Value5);
  // enable_motors();
}

void serialOutput(int Value1, int Value2, int Value3, int Value4, int Value5) {
  if (OUTPUTBLUETOOTHMONITOR) {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3, Value4, Value5);
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

  // Serial.print("Ready, waiting for ");
  // Serial.print(STARTUP_DELAY, DEC);
  // Serial.println(" seconds");

  // delaySeconds(STARTUP_DELAY);

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


  // delay(1000);  //settling time but no really needed
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


  // read_serial_command();

  // Serial.println(angGyro);
  // delay(500);


  // motorCheck();

  Serial1.print("Distance Right: ");
  Serial1.println(distIRRShort);
  Serial1.print("Distance Left: ");
  Serial1.println(distIRLShort);

  delay(500);
  // // Serial1.print("xValue: ");
  // //// Serial1.println(xValue);
  // // Serial1.print("xValue: ");
  // //// Serial1.println(xValue);
  // // Serial1.print("Left Rear: ");
  // //// Serial1.println(distIRLShort);
  // // Serial1.print("Right Rear: ");
  // //// Serial1.println(distIRRShort);
  // delay(500);
  // delay(500);
  // positionSet = true;  // enables the x and y coordinates
  // alignBack(100, 150);

  // move(200,0,0,400);
  // move(-200,0,0,400);

  // move(0, 0, 45, 200);
  // delay(500);
  // move(0, 0, 90, 200);
  // delay(500);
  // move(0, 0, 180, 200);
  // delay(500);
  // move(0, 0, 270, 200);
  // delay(500);
  // delay(100000);
  // move(0, 0, 180, 200);
  // delay(2000);
  // // Serial1.print(204500 * pow(signalADCL, -1.249));
  // // Serial1.print(" ; ");
  // //// Serial1.println(distIRL);
  // Serial.print("Ultra: ");
  // Serial.println(distUltra);
  // // Serial1.print(" LeftIR: ");
  // // Serial1.print(distIRL);
  // // Serial1.print(" RightIR: ");
  // // Serial1.print(distIRR);
  // // Serial1.print(" x: ");
  // // Serial1.print(xValue);
  // // Serial1.print(" y: ");
  // // Serial1.print(yValue);
  // // Serial1.print(" Offset: ");
  // //// Serial1.println(handOffOffSet);
  // Serial.print("debug: ");
  // Serial.println(debugValue1);
  // delay(1000);

  // delay(500);
  // //// Serial1.println(distUltra);

  // delay(3000);
  // findCorner();
  // delay(100000);

  //goCorner();

  // currentAngle = 180.0;

  //delay(1000);
  // motorCheck();


  // delay(2000);
  // goCorner();
  // delay(100000);


  // positionSet = true;  // enables the x and y coordinates

  // delay(1000);
  // alignBack(180, 250);
  //currentAngle = 180;






  // works well
  // goStraight(endDistance, 0.55, 0.16, 0.75, 10, 150.0, 1);
  // delay(1000);
  // goStraight(endDistance, 0.55, 0.15, 0.75, 10, 250.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.15, 0.75, 15, 250.0, 1);
  // delay(1000);
  // goStraight(startDistance, 0.55, 0.15, 0.75, 15, 350.0, 1);
  // delay(100);
  // positionSet = true;
  // strafe_left();
  // delay(500);


  float endDistance = 160.0;
  float startDistance = 1790.0;
  float parallelT = 150;


  /// self contained test
  //positionSet = true;
  // alignBack(endDistance, 150);
  // delay(1000);
  // alignBack(endDistance, 250, 1, 0);
  // delay(1000);
  // alignBack(endDistance, 350, 1, 0);
  // delay(1000);
  // alignBack(endDistance, 450, 1, 0);
  // delay(1000);
  // alignBack(endDistance, 550, 1, 0);
  // delay(1000);
  // alignBack(endDistance, 650, 1, 0);
  // delay(1000);
  // alignBack(endDistance, 750, 1, 0);
  // delay(1000);
  // alignBack(endDistance, 850, 1, 0);
  // delay(1000000);



  delay(1000);
  goCorner();  // MUST HAVE DELAY BEFORE
  positionSet = true;
  // //// Serial1.println("EXITED");
  delay(1000);
  alignBack(200, 200, 0, 0); //  alignBack(endDistance, 150, 0, 0);

  delay(5000);
  digitalWrite(LED_BUILTIN, 1);


  // delay(100000);
  float KpForward = 0.45;
  float KiForward = 0.04;
  float KpReturn = 0.62;
  int increment = 110;

  referenceAngle = currentAngle;
  goStraight(endDistance, KpForward, KiForward, 0.75, 15, parallelT, 1);
  //// Serial1.println("Leg 1 complete");
  parallelT += increment;
  goStrafe(endDistance, 0.55, 0.09, 0.75, 20, parallelT, 1);
  //// Serial1.println("Shift 1 complete");
  goStraight(startDistance, KpReturn, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 2 complete");
  parallelT += increment;
  alignBack(endDistance, parallelT, 1, 0);
  referenceAngle = currentAngle;
  delay(200);
  //// Serial1.println("Shift 2 complete");
  // goStraight(startDistance, 0.55, 0.09, 0.75, 15, 350.0, 1);
  goStraight(endDistance, KpForward, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 3 complete");
  parallelT += increment;
  goStrafe(endDistance, 0.55, 0.09, 0.75, 20, parallelT, 1);
  //// Serial1.println("Shift 3 complete");
  goStraight(startDistance, KpReturn, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 4 complete");
  parallelT += increment;
  alignBack(endDistance, parallelT, 1, 0);
  referenceAngle = currentAngle;
  delay(200);
  //// Serial1.println("Shift 4 complete");
  // goStraight(startDistance, 0.55, 0.09, 0.75, 20, parallelT, 1);
  goStraight(endDistance, KpForward, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 5 complete");
  parallelT += increment;
  // switched = !switched;
  // switchPoint = yValue;
  // IRRStart = 1200 - distIRR;
  goStrafe(endDistance, 0.55, 0.09, 0.75, 25, parallelT, 1);
  //// Serial1.println("Shift 5 complete");
  goStraight(startDistance, KpReturn, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 6 complete");
  // parallelT += increment;
  // switched = !switched;
  // switchPoint = yValue;
  IRRStart = 1200 - distIRR;
  // Serial1.println(switchPoint);
  // Serial1.println(IRRStart);
  alignBack(endDistance, parallelT, 1, 0);
  referenceAngle = currentAngle;
  delay(200);
  //// Serial1.println("Shift 6 complete");
  // goStraight(startDistance, 0.55, 0.09, 0.75, 20, 750.0, 1);
  goStraight(endDistance, KpForward, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 7 complete");
  parallelT += increment;
  goStrafe(endDistance, 0.55, 0.09, 0.75, 20, parallelT, 1);
  //// Serial1.println("Shift 7 complete");
  goStraight(startDistance, KpReturn, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 8 complete");
  parallelT += increment;
  alignBack(endDistance, parallelT, 1, 0);
  referenceAngle = currentAngle;
  delay(200);
  //// Serial1.println("Shift 8 complete");
  // goStraight(startDistance, 0.55, 0.09, 0.75, 15, 950.0, 1);
  goStraight(endDistance, KpForward, KiForward, 0.75, 25, parallelT, 1);
  //// Serial1.println("Leg 9 complete");
  parallelT += increment;
  goStrafe(endDistance, 0.55, 0.09, 0.75, 20, parallelT, 1);
  //// Serial1.println("Shift 9 complete");
  goStraight(startDistance, KpReturn, KiForward, 0.75, 15, parallelT, 1);
  //// Serial1.println("Leg 10 complete");
  positionSet = false;




  // goStraight(endDistance, 0.55, 0.15, 0.8, 10, 350.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.15, 0.8, 10, 450.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.15, 0.8, 10, 450.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.2, 0.8, 10, 550.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.15, 0.8, 10, 550.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.2, 0.8, 10, 650.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.15, 0.75, 10, 650.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.2, 0.8, 10, 750.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.15, 0.8, 10, 750.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.2, 0.8, 10, 850.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.15, 0.75, 10, 850.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.2, 0.8, 10, 950.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.15, 0.8, 10, 950.0, 1);
  // delay(100);
  // goStraight(endDistance, 0.55, 0.2, 0.8, 10, 1050.0, 1);
  // delay(100);
  // goStraight(startDistance, 0.55, 0.15, 0.75, 10, 1050.0, 1);
  // delay(100000);



  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

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
void HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(echoPin) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(echoPin) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif
void Analog_Range_A4() {
  // SerialCom->print("Analog Range A4:");
  // SerialCom->println(analogRead(A4));
}
#ifndef NO_READ_GYRO
void GYRO_reading() {
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif
//Serial command pasing
void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w':  //Move Forward
      case 'W':
        forward();
        SerialCom->println("Forward");
        break;
      case 's':  //Move Backwards
      case 'S':
        reverse();
        SerialCom->println("Backwards");
        break;
      case 'q':  //Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e':  //Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a':  //Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd':  //Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-':  //Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }
  }
}

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
void stop()  //Stop
{
  BackRightMotor.writeMicroseconds(1500);
  FrontRightMotor.writeMicroseconds(1500);
  FrontLeftMotor.writeMicroseconds(1500);
  BackLeftMotor.writeMicroseconds(1500);
}

void forward() {
  BackRightMotor.writeMicroseconds(1500 + speed_val);
  FrontRightMotor.writeMicroseconds(1500 + speed_val);
  FrontLeftMotor.writeMicroseconds(1500 - speed_val);
  BackLeftMotor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  BackRightMotor.writeMicroseconds(1500 - speed_val);
  FrontRightMotor.writeMicroseconds(1500 - speed_val);
  FrontLeftMotor.writeMicroseconds(1500 + speed_val);
  BackLeftMotor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  BackRightMotor.writeMicroseconds(1500 - speed_val);
  FrontRightMotor.writeMicroseconds(1500 - speed_val);
  FrontLeftMotor.writeMicroseconds(1500 - speed_val);
  BackLeftMotor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  BackRightMotor.writeMicroseconds(1500 + speed_val);
  FrontRightMotor.writeMicroseconds(1500 + speed_val);
  FrontLeftMotor.writeMicroseconds(1500 + speed_val);
  BackLeftMotor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  BackRightMotor.writeMicroseconds(1500 - speed_val);
  FrontRightMotor.writeMicroseconds(1500 + speed_val);
  FrontLeftMotor.writeMicroseconds(1500 + speed_val);
  BackLeftMotor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  BackRightMotor.writeMicroseconds(1500 + speed_val);
  FrontRightMotor.writeMicroseconds(1500 - speed_val);
  FrontLeftMotor.writeMicroseconds(1500 - speed_val);
  BackLeftMotor.writeMicroseconds(1500 + speed_val);
}



void goStraight(float target, float Kp, float Ki, float Kd, float errorMax, float parallelTarget, bool leftSensor) {

  // If leftSensor = 0 (False) -> going forward i.e. using left sensor to track driftError
  // else leftSensor = 1 (True) -> going backwards i.e. using right sensor to track driftError

  float currentPos, error = 0, u = 0, error_Prv = target, saturation, error_Prv_d = target, driftErrorPrvD = 0, yawVal, driftError_Prv;
  int Pwr = 0, driftPwr = 0, yawPwr = 0, Pwr_temp;
  float Pwr_Prv = 0, driftPwr_Prv = 0;

  float integralK = 0;
  //float integralK = integralK + deltaKi
  int driftError = 0, PwrLimit, referenceYaw;
  float yawError;

  // float driftKp = 0.5;
  // float driftKi = 0.00005;
  // driftKi = 0.0005;
  // float driftKd = 0.5;
  float driftKp = 2.0;
  float driftKi = 0.00005;
  driftKi = 0.0010;
  float driftKd = 0.5;

  if (abs(parallelTarget - yValue) > 40) {
    driftKp = 0.6;
  }

  float driftIntegral = 0;
  // float driftKp = 0.007;
  float yawKp = 8.0;  //7.0;//8.0;

  int deltaYaw, deltadrift;

  int iteration_count = 1;
  int iteration_check = 3;
  bool continueLoop = 1;


  float front_left = 0;
  float front_right = 0;
  float back_left = 0;
  float back_right = 0;
  referenceYaw = referenceAngle;

  while (continueLoop) {
    currentPos = distUltra;
    Serial.println("----------------------------------------");
    Serial.print("Pwr: ");
    Serial.println(yawPwr);
    Serial.print("error: ");
    Serial.println(error);
    // Serial.print("Drift Pwr: ");
    // Serial.println(driftPwr);
    // Serial.print("Drift error: ");
    // Serial.println(driftError);
    // Serial.print("Yaw Pwr: ");
    // Serial.println(yawPwr);
    // Serial.print("Yaw Error: ");
    // Serial.println(yawError);
    Serial.println("--------------------");
    debugValue1 = int(error);
    debugValue2 = int(driftError);
    debugValue3 = int(yawError);

    error = 2000 - (target + xValue);
    // error = currentPos - target;
    //Thought for if everything fails: we could simply change this to be <300 and hopefully integral kicks in for the more common 200 or so away scenario.
    if (abs(u) < 180) {
      //if (abs(u) < 300) {
      // Calculating the integral value so far by adding the current error, we could change
      integralK = integralK + 0.01 * error;
    }

    u = (Kp * error) + (Ki * integralK) + (Kd * (error - error_Prv_d));
    //Pwr = constrain(u, -600, 600);
    //if want to zoom zoom:
    //Pwr = constrain(u, -250, 250);
    Pwr = constrain(u, -150, 150);
    //Pwr = constrain((Pwr + Pwr_Prv) / 2, -200, 200);
    Pwr_Prv = Pwr;
    // if (Pwr < -10 && Pwr > -75) {
    //   Pwr -= 75;
    // } else if (Pwr > 10 && Pwr < 75) {
    //   Pwr += 75;
    // }

    driftError = parallelTarget - yValue;
    // if (leftSensor) {
    //   // if +ve strafe right if -ve strafe left
    //   driftError = parallelTarget - distIRL;
    // } else {
    //   // if +ve strafe left if -ve strafe right
    //   driftError = (1200 - parallelTarget) - (1200 - distIRR);
    // }

    deltadrift = driftKp * driftError + driftKi * driftIntegral + driftKd * (driftError - driftErrorPrvD);  // * driftError * driftError;
    driftPwr = deltadrift;
    if (abs(driftPwr) < 150) {
      // Calculating the integral value so far by adding the current error
      driftIntegral = driftIntegral + driftPwr;
    }
    //deltadrift = constrain(deltadrift, -200, 200);
    // driftPwr = constrain((deltadrift + driftPwr_Prv) / 2, -200, 200);
    // driftPwr_Prv = Pwr;

    // if (driftPwr < -10 && driftPwr > -75) {
    //   driftPwr -= 100;
    // } else if (driftPwr > 10 && driftPwr < 75) {
    //   driftPwr += 100;
    // }

    yawVal = angGyro;
    // +ve = clockwise ; -ve = anticlockwise

    yawError = (yawVal - referenceYaw);
    if (yawError > 180) {
      yawError -= 360;
    }
    deltaYaw = yawKp * yawError;
    //yawPwr = constrain(deltaYaw, -200, 200);
    yawPwr = deltaYaw;


    // front_left = adjustPower(Pwr + driftPwr - yawPwr);
    // front_right = adjustPower(-Pwr + driftPwr - yawPwr);
    // back_left = adjustPower(Pwr - driftPwr - yawPwr);
    // back_right = adjustPower(-Pwr - driftPwr - yawPwr);

    front_left = (Pwr + driftPwr - yawPwr);
    front_right = (-Pwr + driftPwr - yawPwr);
    back_left = (Pwr - driftPwr - yawPwr);
    back_right = (-Pwr - driftPwr - yawPwr);

    if (error > 200) {
      front_left = lineariseMotors(front_left, 1);
      front_right = lineariseMotors(front_right, 2);
      back_left = lineariseMotors(back_left, 3);
      back_right = lineariseMotors(back_right, 4);
    }


    //Saturate the outputs
    front_left = constrain(front_left, -700, 700);
    front_right = constrain(front_right, -700, 700);
    back_left = constrain(back_left, -700, 700);
    back_right = constrain(back_right, -700, 700);

    BackRightMotor.writeMicroseconds(1500 + back_right);
    FrontRightMotor.writeMicroseconds(1500 + front_right);
    FrontLeftMotor.writeMicroseconds(1500 + front_left);
    BackLeftMotor.writeMicroseconds(1500 + back_left);

    /*
    if (leftSensor) {
      BackRightMotor.writeMicroseconds(1500 - Pwr - yawPwr - 1.0 * driftPwr - 0.0 * driftPwr);
      FrontRightMotor.writeMicroseconds(1500 - Pwr - yawPwr + 1.0 * driftPwr - 0.0 * driftPwr);
      FrontLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr + 1.0 * driftPwr - 0.0 * driftPwr);
      BackLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr - 1.0 * driftPwr - 0.0 * driftPwr);
    } else {
      BackRightMotor.writeMicroseconds(1500 - Pwr - yawPwr + 1.0 * driftPwr + 0.0 * driftPwr);
      FrontRightMotor.writeMicroseconds(1500 - Pwr - yawPwr - 1.0 * driftPwr + 0.0 * driftPwr);
      FrontLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr - 1.0 * driftPwr + 0.0 * driftPwr);
      BackLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr + 1.0 * driftPwr + 0.0 * driftPwr);
    }
*/


    // // Serial1.print("Left Front: ");
    // //// Serial1.println(front_left);
    // // Serial1.print("Left Rear: ");
    // //// Serial1.println(back_left);
    // // Serial1.print("Right Rear: ");
    // //// Serial1.println(back_right);
    // // Serial1.print("Right Front: ");
    // //// Serial1.println(front_right);



    // After a certain number of iterations of the loop (Specified by iteration_check) this statement does its checks
    if ((iteration_count % iteration_check) == (iteration_check - 1)) {
      // If the error in the current iteration is equal to the error from the previous iteration check
      // and the error is also less than the maximum error speciied
      if ((error == error_Prv) && (abs(error) <= errorMax) && (driftError == driftError_Prv) && (abs(driftError) <= errorMax)) {
        // We then set the continue loop variable to 0 to ensure the loop does not run again
        continueLoop = 0;
      }
      // We then set the previous error to the current error which can be used later if we need to
      error_Prv = error;
      driftError_Prv = driftError;
    }

    // Saving the error from this iteration to be used to calculate the derivative part of the controller
    error_Prv_d = error;
    // To keep track of which iteration of the loop we are currently on we add one the the iteration count at the end of the loop
    iteration_count += 1;
    delay(20);
  }
  BackRightMotor.writeMicroseconds(1500);
  FrontRightMotor.writeMicroseconds(1500);
  FrontLeftMotor.writeMicroseconds(1500);
  BackLeftMotor.writeMicroseconds(1500);
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

void motorCheck() {
  int speed = 45;
  /*
  while (speed < 1500) {
    BackLeftMotor.writeMicroseconds(1500 - speed);
    // FrontRightMotor.writeMicroseconds(1500);
    // FrontLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr + driftPwr);
    // BackLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr - driftPwr);
    Serial.println(speed);
    speed++;
    delay(10);
  }
  */
  float front_left = 0;
  float front_right = 0;
  float back_left = 0;
  float back_right = 0;

  front_left = adjustPower(speed);
  front_right = adjustPower(-speed);
  back_left = adjustPower(speed);
  back_right = adjustPower(-speed);

  BackRightMotor.writeMicroseconds(1500 + back_right);
  FrontRightMotor.writeMicroseconds(1500 + front_right);
  FrontLeftMotor.writeMicroseconds(1500 + front_left);
  BackLeftMotor.writeMicroseconds(1500 + back_left);
}

int adjustPower(int power) {
  int newPower = power;
  int increasedPower;
  int offset = 10;
  if (power > 0) {
    newPower += offset;
  } else {
    newPower -= offset;
  }
  if (abs(power) < 60) {
    if (newPower > 0) {
      increasedPower = 60;
      return increasedPower;
    } else {
      increasedPower = -70;
      return increasedPower;
    }
  } else {
    return newPower;
  }
}

void findCorner() {
  int sonarReadings[12] = { 0 };
  float angReadings[12] = { 0.0 };
  float front_left = 150;
  float front_right = 150;
  float back_left = 150;
  float back_right = 150;
  float startAng = absAngle;
  float beginAngle = angGyro;
  float currentAng = 0.0, angChange = 0.0;
  //// Serial1.println(angGyro);
  for (int i = 0; i < 12; i++) {

    while (abs(angChange) < (float(i) * 22.5 + 22.5)) {
      currentAng = absAngle;
      angChange = currentAng - startAng;
      BackRightMotor.writeMicroseconds(1500 + back_right);
      FrontRightMotor.writeMicroseconds(1500 + front_right);
      FrontLeftMotor.writeMicroseconds(1500 + front_left);
      BackLeftMotor.writeMicroseconds(1500 + back_left);
      ////// Serial1.println(angChange);
    }
    // BackRightMotor.writeMicroseconds(1500);
    // FrontRightMotor.writeMicroseconds(1500);
    // FrontLeftMotor.writeMicroseconds(1500);
    // BackLeftMotor.writeMicroseconds(1500);
    sonarReadings[i] = sonar();
    angReadings[i] = angGyro;
    // Serial1.print("Value ");
    // Serial1.print(i);
    // Serial1.print(": ");
    //// Serial1.println(sonarReadings[i]);
    //// Serial1.println(angReadings[i]);
    //// Serial1.println(angChange);
    //delay(100);
  }
  BackRightMotor.writeMicroseconds(1500);
  FrontRightMotor.writeMicroseconds(1500);
  FrontLeftMotor.writeMicroseconds(1500);
  BackLeftMotor.writeMicroseconds(1500);

  int minDistanceIndex = 0;
  for (int j = 0; j < 12; j++) {
    if (sonarReadings[j] <= sonarReadings[minDistanceIndex]) {
      minDistanceIndex = j;
    }
  }
  // Serial1.print("Min Distance: ");
  //// Serial1.println(sonarReadings[minDistanceIndex]);
  // Serial1.print("Min Angle: ");
  //// Serial1.println(angReadings[minDistanceIndex]);
  delay(2000);
  float turnAngle = angReadings[minDistanceIndex] - angGyro;
  float currentAngle = angGyro;
  if (angReadings[minDistanceIndex] > angGyro) {
    turnAngle = -turnAngle;
  } else if (angReadings[minDistanceIndex] < angGyro) {
    turnAngle = turnAngle;
  }
  //// Serial1.println(angGyro);
  move(0, 0, turnAngle, 300);
}

//Function that moves to specified relative position.
void move(int distForward, int distL, float ang, int speed) {
  int errorForward = 0;
  int prevErrorForward = 0;
  int sensorDistForward = 0;
  int errorIntForward = 0;
  int errorDerivForward = 0;

  int errorL = 0;
  int prevErrorL = 0;
  int sensorDistL = 0;
  int errorIntL = 0;
  int errorDerivL = 0;

  volatile float errorAng = 0;
  volatile float prevErrorAng = 0;
  volatile float sensorAng = 0;
  volatile float errorIntAng = 0;
  volatile float errorDerivAng = 0;
  volatile float angChange = 0;

  int initialForward = xValue;
  int initialL = yValue;
  volatile float initialAng = absAngle;

  float velForward = 0;
  float velL = 0;
  volatile float velAng = 0;

  float Kp, Ki, Kd, KpL, KiL, KdL;

  if (ang == 0) {
    Kp = 13;
    Ki = 0.1;  // 0.15 overshoots slightly 0.1 undershoots slightly
    Kd = 0.0;
  } else {
    Kp = 0;
    Ki = 0;
    Kd = 0;
  }

  if (ang == 0) {
    KpL = 0;
    KiL = 00;
    KdL = 0;
  } else {
    KpL = 0;
    KiL = 0;
    KdL = 0;
  }


  // 1.65, 0.01 and 0.5 work well together
  float KpAng = 0.30;
  float KiAng = 0.009;
  float KdAng = 0.8;

  float front_left = 0;
  float front_right = 0;
  float back_left = 0;
  float back_right = 0;

  while (1) {
    prevErrorForward = errorForward;
    errorForward = distForward - sensorDistForward;
    errorDerivForward = errorForward - prevErrorForward;

    prevErrorL = errorL;
    errorL = distL - sensorDistL;
    errorDerivL = errorL - prevErrorL;

    errorAng = ang - sensorAng;
    // angDiff = errorAng - prevErrorAng;

    // if (abs(angDiff) < 180) {
    //   angChange += angDiff;
    //   Serial.println("1");
    // } else {
    //   angDiff = (-1) * int(constrain(angDiff, -1, 1)) * (360 - abs(angDiff));
    //   angChange += angDiff;
    // }
    // debugValue1 = int(angChange);
    // prevErrorAng = errorAng;
    // errorAng = angChange;
    errorDerivAng = errorAng - prevErrorAng;

    // Adding integral value
    // if ((abs(front_left) < 600) || (abs(front_right) < 600) || (abs(back_left) < 600) || (abs(back_right) < 600)) {
    //   errorIntForward += errorForward;
    //   errorIntL += errorL;
    //   errorIntAng += errorAng;
    // }

    if (errorForward < 600) {
      errorIntForward += errorForward;
    }
    if (errorL < 600) {
      errorIntL += errorL;
    }
    if (errorAng < 90) {
      errorIntAng += errorAng;
    }


    //VelocitL Calculations
    velForward = Kp * errorForward + Ki * errorIntForward + Kd * errorDerivForward;
    Serial.print("velForward: ");
    Serial.println(velForward);
    velL = KpL * errorL + KiL * errorIntL + KdL * errorDerivL;
    velAng = KpAng * errorAng + KiAng * errorIntAng + Kd * errorDerivAng;

    debugValue1 = int(errorForward);
    debugValue2 = int(errorL);
    debugValue3 = int(errorAng);

    //Note Values havn't been updated to account for change of orrientation;
    front_left = (velForward - velL - velAng * (74 + 76)) / 55.0;
    front_right = (-velForward - velL - velAng * (74 + 76)) / 55.0;
    back_left = (velForward + velL - velAng * (74 + 76)) / 55.0;
    back_right = (-velForward + velL - velAng * (74 + 76)) / 55.0;

    // front_left = lineariseMotors(front_left);
    // front_right = lineariseMotors(front_right);
    // back_left = lineariseMotors(back_left);
    // back_right = lineariseMotors(back_right);

    //Saturate the outputs
    front_left = constrain(front_left, -speed, speed);
    front_right = constrain(front_right, -speed, speed);
    back_left = constrain(back_left, -speed, speed);
    back_right = constrain(back_right, -speed, speed);


    BackRightMotor.writeMicroseconds(1500 + back_right);
    FrontRightMotor.writeMicroseconds(1500 + front_right);
    FrontLeftMotor.writeMicroseconds(1500 + front_left);
    BackLeftMotor.writeMicroseconds(1500 + back_left);
    // //// Serial1.println("-------");
    // // Serial1.print("Left Front: ");
    // //// Serial1.println(front_left);
    // // Serial1.print("Left Rear: ");
    // //// Serial1.println(back_left);
    // // Serial1.print("Right Rear: ");
    // //// Serial1.println(back_right);
    // // Serial1.print("Right Front: ");
    // //// Serial1.println(front_right);
    // // Serial1.print("ErrorL: ");
    // //// Serial1.println(errorL);
    // // Serial1.print("ErrorForward: ");
    // //// Serial1.println(errorForward);
    // // Serial1.print("errorAng: ");
    // //// Serial1.println(errorAng);
    // // Serial1.print("Y: ");
    // //// Serial1.println(abs(yValue - prevYValue));
    // // Serial1.print("X: ");
    // //// Serial1.println(abs(xValue - prevXValue));
    // // Serial1.print("Absolute Angle: ");
    // //// Serial1.println(absAngle);
    // // Serial1.print("Gyroscope Value: ");
    // //// Serial1.println(angGyro);
    // // Serial1.print("velForward: ");
    // //// Serial1.println(velForward);
    // // Serial1.print("velL: ");
    // //// Serial1.println(velL);


    //Sensor feedback loop
    sensorDistForward = xValue - initialForward;
    sensorDistL = yValue - initialL;
    sensorAng = -absAngle + initialAng;

    if (abs(((constrain(abs(distForward), 0, 1)) * errorForward + (constrain(abs(distL), 0, 1)) * errorL + (constrain(abs(ang), 0, 1)) * errorAng)) < 1 && (abs((constrain(abs(distForward), 0, 1)) * (abs(yValue - prevYValue)) + ((constrain(abs(distL), 0, 1)) * abs(xValue - prevXValue))) < 2)) {
      //stop motors
      BackRightMotor.writeMicroseconds(1500);
      FrontRightMotor.writeMicroseconds(1500);
      FrontLeftMotor.writeMicroseconds(1500);
      BackLeftMotor.writeMicroseconds(1500);
      break;
    }

    delay(50);
  }
}




void goCorner() {
  float minAngle = 0;
  firstRun = 1;
  float minDist = 10000;
  float front_left = 100;
  float front_right = 100;
  float back_left = 100;
  float back_right = 100;
  // Serial1.print("START ANGLE: ");
  //// Serial1.println(angGyro);
  // Serial1.print("CURRENT ANGLE: ");
  //// Serial1.println(currentAngle);
  int distance1, distance2;
  int error, target = 300, Pwr;

  float startAngle = absAngle;
  BackRightMotor.writeMicroseconds(1500 + back_right);
  FrontRightMotor.writeMicroseconds(1500 + front_right);
  FrontLeftMotor.writeMicroseconds(1500 + front_left);
  BackLeftMotor.writeMicroseconds(1500 + back_left);
  //  cw();
  while (abs(absAngle - startAngle) < 360) {
    if (distUltra < minDist && distUltra > 0) {
      minAngle = angGyro;
      minDist = distUltra;
      // Serial1.print("NEW MIN ANGLE = ");
      //// Serial1.println(minAngle);
      //// Serial1.println(distUltra);
    }
    // // Serial1.print("Gyro Angle = ");
    // //// Serial1.println(angGyro);
  }
  stop();
  delay(1000);
  minAngle -= 15;
  if (minAngle < 0) {
    minAngle += 360;
  }
  // Serial1.print("angGyro: ");
  //// Serial1.println(angGyro);
  // // Serial1.print("Now Pointing, minAngle = ");
  // //// Serial1.println(minAngle);
  // // Serial1.print("current Angle = ");
  // //// Serial1.println(angGyro);
  //turn towards closest wall
  //  ccw();
  front_left = -200;
  front_right = -200;
  back_left = -200;
  back_right = -200;
  // BackRightMotor.writeMicroseconds(1500 + back_right);
  // FrontRightMotor.writeMicroseconds(1500 + front_right);
  // FrontLeftMotor.writeMicroseconds(1500 + front_left);
  // BackLeftMotor.writeMicroseconds(1500 + back_left);
  // Serial1.print("Min Angle: ");
  //// Serial1.println(minAngle);
  // There seems to be a constant offset of some value
  // move(0, 0, 120, 200);
  float angleOffset = 0.0;  // 52
  float angToBeTravelled;
  if (angGyro < minAngle) {
    //// Serial1.println("NO1");
    //// Serial1.println(minAngle - angGyro);
    angToBeTravelled = -(minAngle - angGyro) + angleOffset;
  } else {
    //// Serial1.println("NO2");
    //// Serial1.println(abs(-angGyro + minAngle));
    angToBeTravelled = (angGyro - minAngle) - angleOffset;
  }
  if (angToBeTravelled > 180) {
    angToBeTravelled -= 360;
  }
  move(0, 0, angToBeTravelled, 120);
  //// Serial1.println("going towards wall");
  // go towards closest wall
  // BackLeftMotor.writeMicroseconds(1600);
  // FrontLeftMotor.writeMicroseconds(1600);
  // BackRightMotor.writeMicroseconds(1380);
  // FrontRightMotor.writeMicroseconds(1400);
  delay(100);

  while (1) {
    error = -(target - distUltra);
    Pwr = error * 1.0;
    Pwr = constrain(Pwr, -1, 1) * constrain(abs(Pwr), 75, 150);
    BackLeftMotor.writeMicroseconds(1500 + Pwr);
    FrontLeftMotor.writeMicroseconds(1500 + Pwr);
    BackRightMotor.writeMicroseconds(1500 - Pwr);
    FrontRightMotor.writeMicroseconds(1500 - Pwr);
    // // Serial1.print("error: ");
    // //// Serial1.println(error);

    if (abs(error) < 50) {
      stop();
      break;
    }
    delay(40);
  }

  delay(100);
  // turn right 90
  // Serial1.print("turning right second time, starting angle = ");
  float initialAng = angGyro;
  //// Serial1.println(initialAng);
  //  cw();
  move(0, 0, -90, 100);
  //// Serial1.println("stopping");
  //// Serial1.println(angGyro);
  stop();
  delay(100);
  distance1 = distUltra;
  move(0, 0, 180, 100);
  delay(100);
  distance2 = distUltra;

  if ((distance1 + distance2) > 1400) {
    move(0, 0, 190, 100);
    //// Serial1.println("DONE 1");
  } else {
    move(0, 0, 90, 100);
    //// Serial1.println("DONE 2");
  }

  while (1) {
    error = (target - distIRL);
    Pwr = error * 1.0;
    Pwr = constrain(Pwr, -1, 1) * constrain(abs(Pwr), 75, 150);

    BackLeftMotor.writeMicroseconds(1500 - Pwr);
    FrontLeftMotor.writeMicroseconds(1500 + Pwr);
    BackRightMotor.writeMicroseconds(1500 - Pwr);
    FrontRightMotor.writeMicroseconds(1500 + Pwr);
    // // Serial1.print("error: ");
    // //// Serial1.println(error);

    if (abs(error) < 50) {
      stop();
      break;
    }
    delay(40);
  }
  //// Serial1.println("HELLO");
  return;
}

void checkCorner() {

  float distance1 = distUltra;
  move(0, 0, 90, 200);
  delay(100);
  float distance2 = distUltra;
  if (distance1 > 1500 && distance2 < 1400) {
    move(0, 0, -180, 150);
    //alignBack();
    move(0, 0, 90, 120);
  } else move(0, 0, -90, 120);
}

void alignBack(int target, int parallelTarget, bool Kp_met, bool KDrift_met) {
  //// Serial1.println("ALIGNING BACK");

  float front_left = -100;
  float front_right = 100;
  float back_left = 100;
  float back_right = -100;

  float velForward = 80;
  float velL, velAng;


  bool continueLoop = 1;

  int error, driftError, errorStraight, prevErrorStraight, errorPrv = 0, driftErrorPrv = 0, errorStraightPrv = 0, past_derivative = 0;
  // float Kp = 0.02;
  // float Ki = 0.0001;
  // float Kd = 0.0;

  float Kp = 0.7;
  float Ki = 0.00001;
  float Kd = 0.1;

  // float driftKp = 0.7;
  // float driftKi = 0.001;
  // float driftKd = 0;
  // float driftKp = 0.8;
  // float driftKi = 0.00002;
  // float driftKd = 0.3;



  // POSSIBLY DECREASE Decrease what???
  // from go straight
  float driftKp = 0.5;
  float driftKi = 0.00005;
  driftKi = 0.001;
  float driftKd = 0.5;
  driftKi = 0.0001;
  driftKd = 0;

  if (abs(parallelTarget - yValue) > 60) {
    driftKp = 0.175;
  }


  float Kpstraight = 3.0;
  float Kistraight = 0.1;
  float Kdstraight = 50.0;
  Kdstraight = 20;
  // Kpstraight = 1.0;
  // Kistraight = 0.01;
  //Kpstraight = 0;

  float integralK = 0.0;
  float driftIntegral = 0.0;
  float integralStraight = 0.0;

  int speed = 400;

  int iteration_count = 0;


  // BackRightMotor.writeMicroseconds(1500 + back_right);
  // FrontRightMotor.writeMicroseconds(1500 + front_right);
  // FrontLeftMotor.writeMicroseconds(1500 + front_left);
  // BackLeftMotor.writeMicroseconds(1500 + back_left);

  // while (abs(distIRLShort) > 250) {
  //   delay(10);
  // }
  while (continueLoop) {
    error = target - xValue;
    if (error < 100) {
      integralK += error;
    }
    velForward = Kp * error + integralK * Ki + Kd * (error - errorPrv);
    velForward = constrain(velForward, -150, 150);
    // velForward = constrain(velForward, -1, 1) * 65 + constrain(velForward, -1, 1) * constrain(abs(velForward), 0, 30);
    errorPrv = error;
    //velForward = constrain(velForward, -1, 1) * constrain(abs(velForward), 70, 200);

    driftError = (parallelTarget - yValue);
    if (abs(driftError) > 110) {
      driftKp = 1.0;
    } else {
      driftKp = 0.5;
    }
    velL = driftKp * driftError + driftKi * driftIntegral + driftKd * (driftError - driftErrorPrv);  // * driftError * driftError;
    if (driftError < 150) {
      driftIntegral += velL;
    }
    // velL = constrain(velL, -150, 150);
    //velL = constrain(velL, -1, 1) * constrain(abs(velL), 100, 200);
    velL = constrain(velL, -200, 200);
    // velL = constrain(velL, -1, 1) * 70 + constrain(velL, -1, 1) * constrain(abs(velL), 0, 30);
    driftErrorPrv = driftError;
    // errorStraight = atan((distIRLShort - distIRRShort) / 179.0) * (180.0 / 3.14);
    errorStraight = (distIRLShort - distIRRShort);
    // Kpstraight = 5.0;
    if (abs(errorStraight) < 10) {
      integralStraight += errorStraight;
    } else {
      // Kpstraight = 5.0;
    }

    // ANGLE = TAN((LEFT SENSOR - RIGHT ) / 179)

    // if (abs(errorStraight) > 10) {
    //   errorStraight = -prevErrorStraight;
    // }
    prevErrorStraight = errorStraight;
    velAng = Kpstraight * errorStraight + integralStraight * Kistraight + Kdstraight * (0.8 * (errorStraight - errorStraightPrv) + 0.2 * (past_derivative));
    past_derivative = (errorStraight - errorStraightPrv);
    velAng = constrain(velAng, -100, 100);
    // velAng = constrain(ceil(velAng), -1, 1) * 50 + constrain(ceil(velAng), -1, 1) * constrain(abs(velAng), 0, 30);
    if (abs(error) > 50 && !Kp_met) {
      velAng = 0;
    }
    errorStraightPrv = errorStraight;
    //velAng = constrain(velAng, -1, 1) * constrain(velAng, 0, 30);

    debugValue1 = int(error);
    debugValue2 = int(driftError);
    debugValue3 = int(errorStraight);
    // debugValue1 = int(velForward);
    // debugValue2 = int(velL);
    // debugValue3 = int(velAng);
    // Note Values havn't been updated to account for change of orrientation;

    // // Serial1.print("Right: ");
    // //// Serial1.println(distIRRShort);
    // // Serial1.print("Left: ");
    // //// Serial1.println(distIRLShort);

    // // Serial1.print("error drift: ");
    //// Serial1.println(driftError);
    // Serial1.print("velAng: ");
    //// Serial1.println(velL);

    if (Kp_met) {
      velForward = 0;
    } else {
      velL = 0;
    }
    if (KDrift_met) {
      velL = 0;
      // //// Serial1.println("Starting Alignment");
    } else {
      // velAng = 0;
    }

    front_left = (velForward + velL - velAng);
    front_right = (-velForward + velL - velAng);
    back_left = (velForward - velL - velAng);
    back_right = (-velForward - velL - velAng);

    if (driftError > 20) {
      front_left = lineariseMotors(front_left, 1);
      front_right = lineariseMotors(front_right, 2);
      back_left = lineariseMotors(back_left, 3);
      back_right = lineariseMotors(back_right, 4);
    }

    front_left = constrain(front_left, -speed, speed);
    front_right = constrain(front_right, -speed, speed);
    back_left = constrain(back_left, -speed, speed);
    back_right = constrain(back_right, -speed, speed);

    BackRightMotor.writeMicroseconds(1500 + back_right);
    FrontRightMotor.writeMicroseconds(1500 + front_right);
    FrontLeftMotor.writeMicroseconds(1500 + front_left);
    BackLeftMotor.writeMicroseconds(1500 + back_left);
    // // Serial1.print("integralAng: ");
    // //// Serial1.println(integralStraight);
    // // Serial1.print("Error Straight: ");
    // //// Serial1.println(errorStraight);

    // // Serial1.print("Left Front: ");
    // //// Serial1.println(front_left);
    // // Serial1.print("Left Rear: ");
    // //// Serial1.println(back_left);
    // // Serial1.print("Right Rear: ");
    // //// Serial1.println(back_right);
    // // Serial1.print("Right Front: ");
    // //// Serial1.println(front_right);
    iteration_count++;
    delay(50);
    if (abs(error) < 10) {
      Kp_met = 1;
    }
    if (abs(driftError) < 10) {
      KDrift_met = 1;
    }

    // After a certain number of iterations of the loop (Specified by iteration_check) this statement does its checks
    if ((iteration_count % 6) == (5)) {
      // If the error in the current iteration is equal to the error from the previous iteration check
      // and the error is also less than the maximum error speciied
      if (Kp_met && abs(errorStraight) <= 1.0 && KDrift_met && (errorStraight == errorStraightPrv)) {
        continueLoop = 0;
        BackRightMotor.writeMicroseconds(1500);
        FrontRightMotor.writeMicroseconds(1500);
        FrontLeftMotor.writeMicroseconds(1500);
        BackLeftMotor.writeMicroseconds(1500);
        //// Serial1.println("DONE");
      }

      // We then set the previous error to the current error which can be used later if we need to
      prevErrorStraight = errorStraight;
    }
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

void goStrafe(float target, float Kp, float Ki, float Kd, float errorMax, float parallelTarget, bool leftSensor) {

  // If leftSensor = 0 (False) -> going forward i.e. using left sensor to track driftError
  // else leftSensor = 1 (True) -> going backwards i.e. using right sensor to track driftError

  float currentPos, error = 0, u = 0, error_Prv = target, saturation, error_Prv_d = target, driftErrorPrvD = 0, yawVal, driftError_Prv;
  int Pwr = 0, driftPwr = 0, yawPwr = 0, Pwr_temp;
  float Pwr_Prv = 0, driftPwr_Prv = 0;

  float integralK = 0;
  int driftError = 0, PwrLimit, referenceYaw;
  float yawError;

  float driftKp = 0.45;
  float driftKi = 0.00040;
  float driftKd = 0.15;



  float driftIntegral = 0;
  // float driftKp = 0.007;
  float yawKp = 8.0;  //7.0;//8.0;

  int deltaYaw, deltadrift;

  int iteration_count = 1;
  int iteration_check = 3;
  bool continueLoop = 1;


  float front_left = 0;
  float front_right = 0;
  float back_left = 0;
  float back_right = 0;
  referenceYaw = referenceAngle;

  while (continueLoop) {
    currentPos = distUltra;
    Serial.println("----------------------------------------");
    Serial.print("Pwr: ");
    Serial.println(yawPwr);
    Serial.print("error: ");
    Serial.println(error);
    // Serial.print("Drift Pwr: ");
    // Serial.println(driftPwr);
    // Serial.print("Drift error: ");
    // Serial.println(driftError);
    // Serial.print("Yaw Pwr: ");
    // Serial.println(yawPwr);
    // Serial.print("Yaw Error: ");
    // Serial.println(yawError);
    Serial.println("--------------------");
    debugValue1 = int(error);
    debugValue2 = int(driftError);
    debugValue3 = int(yawError);

    error = 2000 - (target + xValue);
    // error = currentPos - target;
    //if (abs(u)< 300) {
    if (abs(u) < 180) {
      // Calculating the integral value so far by adding the current error
      integralK = integralK + 0.01 * error;
    }

    u = (Kp * error) + (Ki * integralK) + (Kd * (error - error_Prv_d));
    //Pwr = constrain(u, -600, 600);
    Pwr = constrain(u, -150, 150);
    //Pwr = constrain((Pwr + Pwr_Prv) / 2, -200, 200);
    Pwr_Prv = Pwr;
    // if (Pwr < -10 && Pwr > -75) {
    //   Pwr -= 75;
    // } else if (Pwr > 10 && Pwr < 75) {
    //   Pwr += 75;
    // }

    driftError = parallelTarget - yValue;
    // if (leftSensor) {
    //   // if +ve strafe right if -ve strafe left
    //   driftError = parallelTarget - distIRL;
    // } else {
    //   // if +ve strafe left if -ve strafe right
    //   driftError = (1200 - parallelTarget) - (1200 - distIRR);
    // }

    deltadrift = driftKp * driftError + driftKi * driftIntegral + driftKd * (driftError - driftErrorPrvD);  // * driftError * driftError;
    driftPwr = deltadrift;
    //if (abs(driftPwr) <250) {
    if (abs(driftPwr) < 150) {
      // Calculating the integral value so far by adding the current error
      driftIntegral = driftIntegral + driftPwr;
    }
    //deltadrift = constrain(deltadrift, -200, 200);
    // driftPwr = constrain((deltadrift + driftPwr_Prv) / 2, -200, 200);
    // driftPwr_Prv = Pwr;

    // if (driftPwr < -10 && driftPwr > -75) {
    //   driftPwr -= 100;
    // } else if (driftPwr > 10 && driftPwr < 75) {
    //   driftPwr += 100;
    // }

    yawVal = angGyro;
    // +ve = clockwise ; -ve = anticlockwise

    yawError = (yawVal - referenceYaw);
    if (yawError > 180) {
      yawError -= 360;
    }
    deltaYaw = yawKp * yawError;
    //yawPwr = constrain(deltaYaw, -200, 200);
    yawPwr = deltaYaw;


    front_left = adjustPower(Pwr + driftPwr - yawPwr);
    front_right = adjustPower(-Pwr + driftPwr - yawPwr);
    back_left = adjustPower(Pwr - driftPwr - yawPwr);
    back_right = adjustPower(-Pwr - driftPwr - yawPwr);

    front_left = (Pwr + driftPwr - yawPwr);
    front_right = (-Pwr + driftPwr - yawPwr);
    back_left = (Pwr - driftPwr - yawPwr);
    back_right = (-Pwr - driftPwr - yawPwr);

    front_left = lineariseMotors(front_left, 1);
    front_right = lineariseMotors(front_right, 2);
    back_left = lineariseMotors(back_left, 3);
    back_right = lineariseMotors(back_right, 4);

    //Saturate the outputs
    front_left = constrain(front_left, -700, 700);
    front_right = constrain(front_right, -700, 700);
    back_left = constrain(back_left, -700, 700);
    back_right = constrain(back_right, -700, 700);

    BackRightMotor.writeMicroseconds(1500 + back_right);
    FrontRightMotor.writeMicroseconds(1500 + front_right);
    FrontLeftMotor.writeMicroseconds(1500 + front_left);
    BackLeftMotor.writeMicroseconds(1500 + back_left);

    /*
    if (leftSensor) {
      BackRightMotor.writeMicroseconds(1500 - Pwr - yawPwr - 1.0 * driftPwr - 0.0 * driftPwr);
      FrontRightMotor.writeMicroseconds(1500 - Pwr - yawPwr + 1.0 * driftPwr - 0.0 * driftPwr);
      FrontLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr + 1.0 * driftPwr - 0.0 * driftPwr);
      BackLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr - 1.0 * driftPwr - 0.0 * driftPwr);
    } else {
      BackRightMotor.writeMicroseconds(1500 - Pwr - yawPwr + 1.0 * driftPwr + 0.0 * driftPwr);
      FrontRightMotor.writeMicroseconds(1500 - Pwr - yawPwr - 1.0 * driftPwr + 0.0 * driftPwr);
      FrontLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr - 1.0 * driftPwr + 0.0 * driftPwr);
      BackLeftMotor.writeMicroseconds(1500 + Pwr - yawPwr + 1.0 * driftPwr + 0.0 * driftPwr);
    }
*/


    Serial.print("Left Front: ");
    Serial.println(front_left);
    Serial.print("Left Rear: ");
    Serial.println(back_left);
    Serial.print("Right Rear: ");
    Serial.println(back_right);
    Serial.print("Right Front: ");
    Serial.println(front_right);



    // After a certain number of iterations of the loop (Specified by iteration_check) this statement does its checks
    if ((iteration_count % iteration_check) == (iteration_check - 1)) {
      // If the error in the current iteration is equal to the error from the previous iteration check
      // and the error is also less than the maximum error speciied
      if ((error == error_Prv) && (abs(error) <= errorMax) && (driftError == driftError_Prv) && (abs(driftError) <= errorMax)) {
        // We then set the continue loop variable to 0 to ensure the loop does not run again
        continueLoop = 0;
      }
      // We then set the previous error to the current error which can be used later if we need to
      error_Prv = error;
      driftError_Prv = driftError;
    }

    // Saving the error from this iteration to be used to calculate the derivative part of the controller
    error_Prv_d = error;
    // To keep track of which iteration of the loop we are currently on we add one the the iteration count at the end of the loop
    iteration_count += 1;
    delay(20);
  }
  BackRightMotor.writeMicroseconds(1500);
  FrontRightMotor.writeMicroseconds(1500);
  FrontLeftMotor.writeMicroseconds(1500);
  BackLeftMotor.writeMicroseconds(1500);
}

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