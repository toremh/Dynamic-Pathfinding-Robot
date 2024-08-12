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
#define USE_TIMER_2 true


#include "TimerInterrupt.h"
#include "ISR_Timer.h"
#include <Servo.h>  //Need for Servo pulse output



//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

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


//Timer interrupt variables
volatile int measureCount = 0;  //Sets the starting point in the array for moving average
volatile int index = 1;         //Sets the starting point in the array replacing old values for moving average
const int avgSize = 10;         //This controls the size of the moving average filter. Reduce to speed up polling

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
volatile int sumUltra;
volatile float angGyro = 180;
volatile float sumGyro;

volatile int timerCounter;

//Debug variables for calculating timing of a function
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;


// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
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
    temp = 167490 * pow(signalADCL, -1.204);
    sumIRL -= arrIRL[measureCount];
    arrIRL[measureCount] = temp;
    sumIRL += temp;
    distIRL = (sumIRL / ((float)avgSize)) + 85.0;

    signalADCR = analogRead(irsensorR);
    temp = 168600 * pow(signalADCR, -1.204);
    sumIRR -= arrIRR[measureCount];
    arrIRR[measureCount] = temp;
    sumIRR += temp;
    distIRR = (sumIRR / ((float)avgSize)) + 85.0;

    temp = sonar();
    sumUltra -= arrUltra[measureCount];
    arrUltra[measureCount] = temp;
    sumUltra += temp;
    distUltra = sumUltra / ((float)avgSize);

    //temp = gyro();
    //sumGyro -= arrGyro[measureCount];
    //arrGyro[measureCount] = temp;
    //sumGyro += temp;
    //angGyro = sumGyro/((float)avgSize);
    angGyro = gyro();

    //The counter variable used to index the circular arrays
    if (measureCount == avgSize - 1) {
      measureCount = 0;
    } else {
      measureCount++;
    }
    timerCounter = 1;
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

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  digitalWrite(trigPin, LOW);

  ITimer2.init();
  if (ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, TimerHandler1)) {
    Serial.print(F("Starting  ITimer4 OK, millis() = "));
    Serial.println(millis());
  }


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


  //while(1){
  //  Serial.println(distUltra);
  //}

  // motorCheck();

  

  //    Serial.println("turning right");
  //  angGyro = 0;
  ////  cw();
  //  right_font_motor.writeMicroseconds(1600);
  //  right_rear_motor.writeMicroseconds(1600);
  //  left_font_motor.writeMicroseconds(1615);
  //  left_rear_motor.writeMicroseconds(1600);
  //  while (angGyro < 90) {
  //    delay(10);
  //  }
  //  stop();

  delay(2000);
  goCorner();
  // Serial.println(distIRL);
  // goStraight(150.0, 0.75, 0.2, 0.8, 10, 150.0, 1);
  delay(1000000);






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
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
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
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}



void goStraight(float target, float Kp, float Ki, float Kd, float errorMax, float parallelTarget, bool leftSensor) {

  // If leftSensor = 0 (False) -> going forward i.e. using left sensor to track driftError
  // else leftSensor = 1 (True) -> going backwards i.e. using right sensor to track driftError

  float currentPos, error, u = 0, error_Prv = target, saturation, error_Prv_d = target, yawVal, driftError_Prv;
  int Pwr = 0, driftPwr = 0, yawPwr = 0, Pwr_temp;
  float Pwr_Prv = 0;

  float integralK = 0;
  int driftError, yawError, PwrLimit, referenceYaw;

  float driftKp = 3.0;
  float yawKp = 5.0;

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
    error = currentPos - target;
    if (abs(u) < 195) {
      // Calculating the integral value so far by adding the current error
      integralK = integralK + 0.01 * error;
    }

    u = (Kp * error) + (Ki * integralK) + (Kd * (error - error_Prv_d));
    Pwr = constrain(u, -600, 600);
    Pwr = constrain((Pwr + Pwr_Prv) / 2, -200, 200);
    Pwr_Prv = Pwr;
    // if (Pwr < -10 && Pwr > -75) {
    //   Pwr -= 75;
    // } else if (Pwr > 10 && Pwr < 75) {
    //   Pwr += 75;
    // }

    if (leftSensor) {
      // if +ve strafe right if -ve strafe left
      driftError = parallelTarget - distIRL;
    } else {
      // if +ve strafe left if -ve strafe right
      driftError = parallelTarget - distIRR;
    }

    deltadrift = driftKp * driftError;
    driftPwr = constrain(deltadrift, -200, 200);

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
    yawPwr = constrain(deltaYaw, -200, 200);

    if (leftSensor) {
      left_font_motor.writeMicroseconds(1500 - Pwr - yawPwr - driftPwr);
      left_rear_motor.writeMicroseconds(1500 - Pwr - yawPwr + driftPwr);
      right_rear_motor.writeMicroseconds(1500 + Pwr - yawPwr + driftPwr);
      right_font_motor.writeMicroseconds(1500 + Pwr - yawPwr - driftPwr);
    } else {
      left_font_motor.writeMicroseconds(1500 - Pwr - yawPwr + driftPwr);
      left_rear_motor.writeMicroseconds(1500 - Pwr - yawPwr - driftPwr);
      right_rear_motor.writeMicroseconds(1500 + Pwr - yawPwr - driftPwr);
      right_font_motor.writeMicroseconds(1500 + Pwr - yawPwr + driftPwr);
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
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
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

  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }

  if (firstRun) {  //Because of the variable timing, the first result must be disgarded
    firstRun = false;
    return 0;
  } else {
    return currentAngle;
  }
}

void motorCheck() {
  int speed = 0;
  while (speed < 1500) {
    right_font_motor.writeMicroseconds(1500 - speed);
    // left_rear_motor.writeMicroseconds(1500);
    // right_rear_motor.writeMicroseconds(1500 + Pwr - yawPwr + driftPwr);
    // right_font_motor.writeMicroseconds(1500 + Pwr - yawPwr - driftPwr);
    Serial.println(speed);
    speed++;
    delay(10);
  }
}

void goCorner() {
  float minAngle = 0;
  angGyro = 0;
  float minDist = 10000;
  right_font_motor.writeMicroseconds(1600);
  right_rear_motor.writeMicroseconds(1600);
  left_font_motor.writeMicroseconds(1615);
  left_rear_motor.writeMicroseconds(1600);
  //  cw();
  while (angGyro < 350) {
    if (distUltra < minDist && distUltra > 0) {
      minAngle = angGyro;
      minDist = distUltra;
      Serial.print("NEW MIN ANGLE = ");
      Serial.println(minAngle);
      Serial.println(distUltra);
    }
    Serial.print("Gyro Angle = ");
    Serial.println(angGyro);
  }
  stop();
  delay(1000); 
  minAngle -= 10;
  if (minAngle < 0) {
    minAngle += 360;
  }
  Serial.print("Now Pointing, minAngle = ");
  Serial.println(minAngle);
  Serial.print("current Angle = ");
  Serial.println(angGyro);
  //turn towards closest wall
  //  ccw();
  right_font_motor.writeMicroseconds(1400);
  right_rear_motor.writeMicroseconds(1400);
  left_font_motor.writeMicroseconds(1380);
  left_rear_motor.writeMicroseconds(1400);
  while ( angGyro > minAngle) {
    Serial.print("pointing, ");
    Serial.println(angGyro);
    delay(10);
  }
  stop();

  Serial.println("going towards wall");
  // go towards closest wall
  reverse();
  delay(100);
  while (distUltra > 200 || distUltra < 0) {
    delay(10);
  }
  stop();
  delay(100);
  // turn right 90
  Serial.print("turning right, starting angle = ");
  float initialAng = angGyro;
  Serial.println(initialAng);
  //  cw();
  right_font_motor.writeMicroseconds(1600);
  right_rear_motor.writeMicroseconds(1600);
  left_font_motor.writeMicroseconds(1615);
  left_rear_motor.writeMicroseconds(1600);
  Serial.println(angGyro);
  while ((angGyro - initialAng) < 90 || (initialAng - angGyro) > 300) {
    Serial.println(angGyro);
    delay(10);
  }
  Serial.println("stopping");
  Serial.println(angGyro);
  stop();

  delay(100);
  // go forward until equidistant Replace with goStraight function
  reverse();
  while (distUltra > 200 || distUltra < 0) {
    delay(10);
  }
  stop();

  //  // turn right 180, point towards furthest wall
  //  angGyro = 0;
  //  cw;
  //  while (angGyro < 90) {
  //    delay(10);
  //  }
  //  stop();
  //  float Lcorner = 0;
  //  Lcorner = distUltra;
  //
  //  angGyro = 0;
  //  while (angGyro < 180) {
  //    cw();
  //  }
  //  stop();
  //  float Rcorner = 0;
  //  Rcorner = distUltra;
  //
  //  if (Lcorner > Rcorner) {
  //    angGyro = 0;
  //    while (angGyro > 90) {
  //      ccw();
  //    }
  //    stop();
  //  }
}
