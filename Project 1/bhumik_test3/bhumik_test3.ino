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

#define TIMER2_INTERVAL_MS 10     //Sets the polling rate of the sensors velX TIMER_ITERATION_COUNT VALUE
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
int irsensorL = A4;  //Input for Left IR sensor
int irsensorR = A5;  //Input for Right IR sensor
int signalADCL = 0;  // the read out signal in 0-1023 corresponding to 0-5v
int signalADCR = 0;  // the read out signal in 0-1023 corresponding to 0-5v

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
float rotationThreshold = 2.0;  // because of gyro drifting, defining rotation angular velocity less
                                // than this value will be ignored
volatile float gyroRate = 0;    // read out value of sensor in voltage
volatile float currentAngle = 0;
volatile bool firstRun = true;
volatile bool cornerSet = false;


//Timer interrupt variables
volatile int measureCount = 0;  //Sets the starting point in the array for moving average
volatile int index = 1;         //Sets the starting point in the array replacing old values for moving average
const int avgSize = 10;         //This controls the size of the moving average filter. Reduce to speed up polling
int *refIRSens;
int *offIRSens;

//These are treated as circular arrays and store the values for the moving average calculation
volatile int arrIRL[avgSize] = { 0 };
volatile int arrIRR[avgSize] = { 0 };
volatile int arrUltra[avgSize] = { 0 };
volatile float arrGyro[avgSize] = { 0 };

//sumXXX is the sum of the values stored in arrXXX
//distXXX is the averaged distance calculated from sumXXX
volatile int distIRL;
volatile int sumIRL;
volatile int distIRR;
volatile int sumIRR;
volatile int distUltra;
volatile int prevDistUltra;
volatile int sumUltra;
volatile float angGyro = 180;
volatile float sumGyro;

volatile float IROffset = 0;
volatile int timerCounter = 1;
volatile int bluetoothCounter = 0;

volatile int debugValue1 = 0;
volatile int debugValue2 = 0;
volatile int debugValue3 = 0;

volatile int xValue = 0;
volatile int prevXValue = 0;
volatile int yValue = 0;
volatile int prevYValue = 0;
volatile int debugValue = 0;

volatile int LFMotorPwr, LRMotorPwr, RFMotorPwr, RRMotorPwr;

volatile bool positionSet = false;

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
    sumIRL -= arrIRL[measureCount];
    arrIRL[measureCount] = temp;
    sumIRL += temp;
    distIRL = (sumIRL / ((float)avgSize)) + IROffset;

    signalADCR = analogRead(irsensorR);
    //temp = 168600 * pow(signalADCR, -1.204);
    //temp = 19708.0 * pow(signalADCR, -0.857);
    sumIRR -= arrIRR[measureCount];
    arrIRR[measureCount] = temp;
    sumIRR += temp;
    distIRR = (sumIRR / ((float)avgSize)) + IROffset;

    temp = sonar();
    sumUltra -= arrUltra[measureCount];
    arrUltra[measureCount] = temp;
    sumUltra += temp;
    distUltra = sumUltra / ((float)avgSize);
    if (abs(distUltra - prevDistUltra) > 10000000) { 
      distUltra = prevDistUltra; 
      } else {
        prevDistUltra = distUltra;
      }

    angGyro = gyro();

    //Coordernate position
    // if (positionSet) {
    //   xValue = 2000 - distUltra;
    //   if (xValue < 0) { xValue = 0; }
    //   if (distIRR >= 600) {
    //     yValue = 1200 - distIRL;
    //     if (yValue < 0) { yValue = 0; }
    //   } else {
    //     yValue = distIRR;
    //   }
    // } else {
    //   xValue = 0;
    //   yValue = 0;
    // }

    if (positionSet) {
      if (cornerSet) {
        prevYValue = yValue;
        prevXValue = xValue;
        xValue = 2000 - distUltra;
        if (xValue < 0) { xValue = 0; } //Prevents the X value from going below 0
        if (*refIRSens >= 600) {
          yValue = 1200 - *offIRSens;
        if (yValue < 0) { yValue = 0; }
      } else {
        yValue = *refIRSens;
      }
    } else {
      xValue = 0;
      yValue = 0;
    }

      }
      if (distIRR <= 300) { 
        refIRSens = &distIRR;
        offIRSens = &distIRL;
      } else {
        refIRSens = &distIRL;
        offIRSens = &distIRR;
      }
      cornerSet = true;

    //The counter variable used to index the circular arrays
    if (measureCount == avgSize - 1) {
      measureCount = 0;
    } else {
      measureCount++;
    }

    if (bluetoothCounter == (BLUETOOTH_PULSE / (TIMER_ITERATION_COUNT * TIMER2_INTERVAL_MS))) {
      serialOutput(xValue, yValue, debugValue1, debugValue2, debugValue3);
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
  delay(1000);
  SerialCom->println("Setup....");

  float sum = 0;

  noInterrupts();
  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (int i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    sensorValue = analogRead(gyroPin);
    sum += sensorValue;
    delay(60);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
  interrupts();

  delay(1000);  //settling time but no really needed
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




  // motorCheck();


  // Serial.println(distIRL);
  // Serial.print("Ultra: ");
  // Serial.println(distUltra);
  // Serial.print("LeftIR: ");
  // Serial.println(distIRL);
  // Serial.print("RightIR: ");
  // Serial.println(distIRR);
  // Serial.print("velX: ");
  // Serial.println(xValue);
  // Serial.print("Vely: ");
  // Serial.println(yValue);
  // Serial.print("debug: ");
  // Serial.println(debugValue);
  // delay(1000);

  positionSet = true;  // enables the velX and Vely coordinates
                       // delay(1000);
                       // goStraight(150.0, 0.55, 0.15, 0.8, 10, 150.0, 1);
                       // delay(1000);
                       // goStraight(150.0, 0.55, 0.2, 0.8, 10, 450.0, 1);
                       // delay(1000);
                       // goStraight(1700.0, 0.55, 0.2, 0.8, 10, 450.0, 1);

  delay(2000);
  // motorCheck();
  // move(500,0,0,300);
  // move(-500,0,0,300);

  int front_left, front_right, back_left, back_right;
  int speed = 400;

  for (int i = 0; i > -1000; i--) {
  delay (1000);
  //int velForward = i;
  int velL = 0;
  int velAng = 0;

    // front_left = (velForward - velL - velAng * (74 + 76)) / 55.0;
    // front_right = (-velForward - velL - velAng * (74 + 76)) / 55.0;
    // back_left = (velForward + velL - velAng * (74 + 76)) / 55.0;
    // back_right = (-velForward + velL - velAng * (74 + 76)) / 55.0;

    front_left = i;
    front_right = -i;
    back_left = i;
    back_right = -i;

    front_left = lineariseMotors(front_left,1);
    front_right = lineariseMotors(front_right,2);
    back_left = lineariseMotors(back_left,3);
    back_right = lineariseMotors(back_right,4);

    //Saturate the outputs
    front_left = constrain(front_left, -speed, speed);
    front_right = constrain(front_right, -speed, speed);
    back_left = constrain(back_left, -speed, speed);
    back_right = constrain(back_right, -speed, speed);


    BackRightMotor.writeMicroseconds(1500 + back_right);
    FrontRightMotor.writeMicroseconds(1500 + front_right);
    FrontLeftMotor.writeMicroseconds(1500 + front_left);
    BackLeftMotor.writeMicroseconds(1500 + back_left);

    Serial1.print("Forward = ");
    Serial1.println(front_left);

  }


  // BackRightMotor.writeMicroseconds(1500 - 300);
  // FrontRightMotor.writeMicroseconds(1500 - 300);
  // FrontLeftMotor.writeMicroseconds(1500 + 300);
  // BackLeftMotor.writeMicroseconds(1500 + 300);

  //Note Values havn't been updated to account for change of orrientation;
  // int velForward = 0;
  // int velL = 0;
  // int velAng = 10000;
  // float front_left = (velForward - velL + velAng * (74 + 76)) / 55.0;
  // float front_right = (-velForward - velL + velAng * (74 + 76)) / 55.0;
  // float back_left = (velForward + velL + velAng * (74 + 76)) / 55.0;
  // float back_right = (-velForward + velL + velAng * (74 + 76)) / 55.0;

  // //Saturate the outputs
  // // front_left = constrain(front_left, -600, 600);
  // // front_right = constrain(front_right, -600, 600);
  // // back_left = constrain(front_left, -600, 600);
  // // back_right = constrain(front_right, -600, 600);


  // BackRightMotor.writeMicroseconds(1500 + back_right);
  // FrontRightMotor.writeMicroseconds(1500 + front_right);
  // FrontLeftMotor.writeMicroseconds(1500 + front_left);
  // BackLeftMotor.writeMicroseconds(1500 + back_left);

  // Serial.print("front left: ");
  // Serial.println(front_left);
  // Serial.print("front Right: ");
  // Serial.println(front_right);
  // Serial.print("back left: ");
  // Serial.println(back_left);
  // Serial.print("back Right: ");
  // Serial.println(back_right);

  //delay(100000);



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

  float currentPos, error, u = 0, error_Prv = target, saturation, error_Prv_d = target, yawVal, driftError_Prv;
  int Pwr = 0, driftPwr = 0, yawPwr = 0, Pwr_temp;
  float Pwr_Prv = 0, driftPwr_Prv = 0;

  float integralK = 0;
  int driftError, yawError, PwrLimit, referenceYaw;


  float driftKp = 10.0;
  // float driftKp = 0.007;
  float yawKp = 8.0;  //8.0;

  int deltaYaw, deltadrift;

  int iteration_count = 1;
  int iteration_check = 3;
  bool continueLoop = 1;

  referenceYaw = angGyro;

  while (continueLoop) {
    currentPos = distUltra;
    Serial.println("----------------------------------------");
    Serial.print("Pwr: ");
    Serial.println(Pwr);
    Serial.print("error: ");
    Serial.println(error);
    Serial.print("Drift Pwr: ");
    Serial.println(driftPwr);
    Serial.print("Drift error: ");
    Serial.println(driftError);
    Serial.print("Yaw Pwr: ");
    Serial.println(yawPwr);
    Serial.print("Yaw Error: ");
    Serial.println(yawError);
    Serial.println("--------------------");
    debugValue = int(error);
    error = currentPos - target;
    if (abs(u) < 195) {
      // Calculating the integral value so far by adding the current error
      integralK = integralK + 0.01 * error;
    }

    u = (Kp * error) + (Ki * integralK) + (Kd * (error - error_Prv_d));
    Pwr = constrain(u, -600, 600);
    Pwr = constrain((Pwr + Pwr_Prv) / 2, -200, 200);
    Pwr_Prv = Pwr;

    if (leftSensor) {
      // if +ve strafe right if -ve strafe left
      driftError = parallelTarget - distIRL;
    } else {
      // if +ve strafe left if -ve strafe right
      driftError = parallelTarget - distIRR;
    }

    deltadrift = driftKp * driftError;  // * driftError * driftError;
    deltadrift = constrain(deltadrift, -200, 200);

    yawVal = angGyro;
    // +ve = clockwise ; -ve = anticlockwise

    yawError = (yawVal - referenceYaw);
    if (yawError > 180) {
      yawError -= 360;
    }
    deltaYaw = yawKp * yawError;
    yawPwr = constrain(deltaYaw, -200, 200);

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

    // Serial.print("Left Front: ");
    // Serial.println(1500 + Pwr - yawPwr + driftPwr);
    // Serial.print("Left Rear: ");
    // Serial.println(1500 + Pwr - yawPwr - driftPwr);
    // Serial.print("Right Rear: ");
    // Serial.println(1500 - Pwr - yawPwr - driftPwr);
    // Serial.print("Right Front: ");
    // Serial.println(1500 - Pwr - yawPwr + driftPwr);



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
  distance = duration * 0.34 / 2;
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
    float angleChange = angularVelocity / (1000 / loopTime);
    currentAngle += angleChange;
  }

  float tempAnglePoint = currentAngle - int(currentAngle);
  currentAngle = float(int(currentAngle) % 360);
  currentAngle += tempAnglePoint;


  if (firstRun) {  //Because of the variable timing, the first result must be disgarded
    firstRun = false;
    return 0;
  } else {
    return currentAngle;
  }
}

void motorCheck() {
  int speed = 55;
  //int offset = 13; forward
  int offset = -6;  // backwards
  while (speed < 150) {
    BackLeftMotor.writeMicroseconds(1500 - speed);
    FrontRightMotor.writeMicroseconds(1500 + (speed + offset));
    FrontLeftMotor.writeMicroseconds(1500 - speed);
    BackRightMotor.writeMicroseconds(1500 + (speed + offset));
    debugValue = speed;
    speed++;
    delay(2000);
  }
}

//-----NOT FULLY IMPLEMENTED-----
//TODO
// Test X/Y coordernates/make more robust
// Connect function to motors
// Tune

//Function that moves to specified relative position.
void move(int distForward, int distL, int ang, int speed) {
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

  float errorAng = 0;
  float prevErrorAng = 0;
  float sensorAng = 0;
  float errorIntAng = 0;
  float errorDerivAng = 0;
  float angDiff;
  float angChange = 0;

  int initialForward = xValue;
  int initialL = yValue;
  int initialAng = angGyro;

  float velForward = 0;
  float velL = 0;
  float velAng = 0;

  float Kp, Ki, Kd, KpL, KiL, KdL;

if (ang == 0) {
  Kp = 13;
  Ki = 0.1; // 0.15 overshoots slightly 0.1 undershoots slightly
  Kd = 0.0;
} else {
  Kp = 0;
  Ki = 0;
  Kd = 0;
}

if (ang == 0) {
  KpL = 13;
  KiL = 0.1;
  KdL = 0;
  } else {
  KpL = 0;
  KiL = 0;
  KdL = 0;
}

  float KpAng = 0.6;
  float KiAng = 0.008;
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
    angDiff = errorAng - prevErrorAng;
    
    if (abs(angDiff) < 180) {
      angChange += angDiff;
      Serial.println("1");
    } else {
      angDiff = (-1) *int(constrain(angDiff, -1, 1)) * (360 - abs(angDiff));
      angChange += angDiff;
    }
    debugValue = int(angChange);
    prevErrorAng = errorAng;
    errorAng = angChange;
    errorDerivAng = errorAng - prevErrorAng;

    // Adding integral value
    if ((abs(front_left) < 600) || (abs(front_right) < 600) || (abs(back_left) < 600) || (abs(back_right) < 600)) {
        errorIntForward += errorForward;
        errorIntL += errorL;
        errorIntAng += errorAng;
      }


    //VelocitL Calculations
    velForward = Kp * errorForward + Ki * errorIntForward + Kd * errorDerivForward;
    Serial.print("velForward: ");
    Serial.println(velForward);
    velL = KpL * errorL + KiL * errorIntL + KdL * errorDerivL;
    velAng = KpAng * errorAng + KiAng * errorIntAng + Kd * errorDerivAng;

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
    Serial1.println("-------");
    Serial1.print("Left Front: ");
    Serial1.println(front_left);
    Serial1.print("Left Rear: ");
    Serial1.println(back_left);
    Serial1.print("Right Rear: ");
    Serial1.println(back_right);
    Serial1.print("Right Front: ");
    Serial1.println(front_right);
    Serial1.print("ErrorL: ");
    Serial1.println(errorL);
    Serial1.print("ErrorForward: ");
    Serial1.println(errorForward);
    Serial1.print("errorAng");
    Serial1.println(errorAng);
    Serial1.print("velForward: ");
    Serial1.println(velForward);
    Serial1.print("velL: ");
    Serial1.println(velL);

    
    //Sensor feedback loop
    sensorDistForward = xValue - initialForward;
    sensorDistL = yValue - initialL;
    sensorAng = initialAng - angGyro;

    if(abs(((constrain(abs(distForward), 0, 1))*errorForward + (constrain(abs(distL), 0, 1))*errorL + (constrain(abs(ang), 0, 1))*errorAng)) < 10 && (abs(yValue - prevYValue) + abs(xValue - prevXValue)) < 2) {
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

// int lineariseMotors(int pwr) {
//   if (abs(pwr) < 10) { return 0; }
//   if (pwr >= 11) { return pwr + 45; }
//   if (pwr <= -11) { return pwr - 60; }
// }

int lineariseMotors(int pwr, int motorNum) {
  if (abs(pwr) < 10) { return 0; }

  if (pwr >= 11) { 
    switch (motorNum) {
      case 1: return pwr+40;
      case 2: return pwr+60;
      case 3: return pwr+40;
      case 4: return pwr+60;
    }
  } else if (pwr <= -11) {
    switch (motorNum) {
      case 1: return pwr-60;
      case 2: return pwr-40;
      case 3: return pwr-60;
      case 4: return pwr-40;
    }

  }
}
