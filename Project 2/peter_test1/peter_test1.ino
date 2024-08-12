#define TIMER2_INTERVAL_MS 10    //Sets the polling rate of the sensors x TIMER_ITERATION_COUNT VALUE
#define TIMER_ITERATION_COUNT 6  // Number of times TIMER2 overflows before executing timer handler
#define BLUETOOTH_PULSE 500
#define USE_TIMER_2 true


// Serial Data input pin
#define BLUETOOTH_RX 19
// Serial Data output pin
#define BLUETOOTH_TX 18
//Fan Pin
#define FAN_PIN 22

#define STARTUP_DELAY 10  // Seconds
#define LOOP_DELAY 10     // miliseconds

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0
// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

#define L_X 0.095
#define L_Y 0.110
#define WHEEL_R 0.027

#define UPPER_PHOTO_THRESH 3250



#include "TimerInterrupt.h"
#include "ISR_Timer.h"
#include <Servo.h>  //Need for Servo pulse output


#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
#define NO_HC -SR04   //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
// #define NO_BATTERY_V_OK  //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte servo = 44;
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


byte serialRead = 0;  //for control serial communication
int signalADC = 0;    // the read out signal in 0-1023 corresponding to 0-5v
void readSensor();
int sonar();


//IR sensors
const int irsensorL = A4;       //Input for Left IR sensor
const int irsensorR = A5;       //Input for Right IR sensor
const int irsensorLShort = A8;  //Input for Left IR sensor
const int irsensorRShort = A9;  //Input for Right IR sensor
const int photoPinLL = A12;
const int photoPinL = A13;
const int photoPinR = A15;
const int photoPinRR = A14;
volatile int signalADCL = 0;       // the read out signal in 0-1023 corresponding to 0-5v
volatile int signalADCR = 0;       // the read out signal in 0-1023 corresponding to 0-5v
volatile int signalADCLShort = 0;  // the read out signal in 0-1023 corresponding to 0-5v
volatile int signalADCRShort = 0;  // the read out signal in 0-1023 corresponding to 0-5v
volatile int signalADCPhotoLL = 0;
volatile int signalADCPhotoL = 0;
volatile int signalADCPhotoR = 0;
volatile int signalADCPhotoRR = 0;
volatile int stillExtinguishingCount = 0;

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
volatile float gyroRate = 0;  // read out value of sensor in voltage
volatile float currentAngle = 0;
volatile bool firstRun = true;
volatile bool cornerSet = false;
float gyro();
int fLCounter = 0;
int fRCounter = 0;
int fCounter = 0;
int unstuckCounter = 0;

volatile int front_left, front_right, back_left, back_right;


//Timer interrupt variables
volatile int measureCount = 0;  //Sets the starting point in the array for moving average
volatile int measurecountdist = 0;
volatile int index = 1;  //Sets the starting point in the array replacing old values for moving average
const int avgSize = 10;  //This controls the size of the moving average filter. Reduce to speed up polling
const int avgSizedist = 3;
int *refIRSens;
int *offIRSens;

//These are treated as circular arrays and store the values for the moving average calculation
volatile int arrIRFL[avgSizedist] = { 0 };
volatile int arrIRFR[avgSizedist] = { 0 };
volatile int arrIRL[avgSizedist] = { 0 };
volatile int arrIRR[avgSizedist] = { 0 };
volatile int arrUltra[avgSizedist] = { 0 };
volatile float arrGyro[avgSizedist] = { 0 };
volatile long arrPhotoLL[avgSize] = { 0 };
volatile long arrPhotoL[avgSize] = { 0 };
volatile long arrPhotoR[avgSize] = { 0 };
volatile long arrPhotoRR[avgSize] = { 0 };

//sumXXX is the sum of the values stored in arrXXX
//distXXX is the averaged distance calculated from sumXXX
volatile int distIRFL;
volatile int sumIRFL;
volatile int distIRFR;
volatile int sumIRFR;
volatile int distIRL;
volatile int sumIRL;
volatile int distIRR;
volatile int sumIRR;
volatile int distUltra;
volatile int sumUltra;
volatile float angGyro = 180;
volatile float sumGyro;
volatile float absAngle = 0;
volatile unsigned long distPhotoLL;
volatile unsigned long sumPhotoLL;
volatile unsigned long distPhotoL;
volatile unsigned long sumPhotoL;
volatile unsigned long distPhotoR;
volatile unsigned long sumPhotoR;
volatile unsigned long distPhotoRR;
volatile unsigned long sumPhotoRR;

volatile float IROffset = 46.0;
volatile float SonarOffset = 63.0;
volatile float PhotoOffset = 50;
volatile int timerCounter = 1;
volatile int bluetoothCounter = 0;

volatile int servoPosition = 0;

volatile float referenceAngle;

volatile int xValue = 0;
volatile int prevXValue = 0;
volatile int yValue = 0;
volatile int handOffOffSet = 0;
volatile int prevYValue = 0;
volatile int debugValue1 = 0;
volatile int debugValue2 = 0;
volatile int debugValue3 = 0;
volatile int extinguishCount = 0;

volatile bool positionSet = false;
volatile bool switched = false;
volatile bool ShouldISwitch = false;
volatile bool isExtinguishing = false;

volatile int switchPoint;
volatile int difference;
volatile int IRRStart;

//I want these ints to range from 0 to 2 so 3 states total, dont think i need the flags for the inside sensors?
volatile int photoRRFlag = 0;
volatile int photoLLFlag = 0;
volatile int photoRRHFlag = 0;
volatile int photoLLHFlag = 0;

//Debug v ariables for calculating timing of a function
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;

volatile unsigned long RawRR, RawR, RawL, RawLL, UnfilteredRawRR;


//commands for fuzzy logic/behavioural control
enum actions { FORWARD,
               STRAFELEFT,
               STRAFERIGHT,
               DIAGLEFT,
               DIAGRIGHT,
               CW,
               CCW,
               BACKWARD,
               ENABLEFAN,
               DISABLEFAN,
               STOP };
enum obstaclePos { FRONT,
                   FRONTLEFT,
                   FRONTRIGHT,
                   LEFT,
                   RIGHT,
                   CLEAR,
                   TIGHTSPACE,
                   WALL };
char *obstaclePosDebug[] = { "FRONT", "FRONTLEFT", "FRONTRIGHT", "LEFT", "RIGHT", "CLEAR", "TIGHTSPACE", "WALL" };
char *action[] = { "FORWARD", "STRAFELEFT", "CW", "CCW", "BACKWARD", "ENABLEFAN", "DISABLEFAN", "STOP" };
enum obstacleType { REGULAR,
                    FIRE };

//avoid function
volatile int ULTRA_THRESHOLD = 150;  //having this so low may be fucking with our object avoidance, creating a big dead spot where our ultra nor infras can see on the front. Raised it a bit
volatile int IRFL_THRESHOLD = 150;
volatile int IRFR_THRESHOLD = 150;
volatile int IRR_THRESHOLD = 70;  //was 100,
volatile int IRL_THRESHOLD = 70;  // was 100
volatile int IRL_HIGHTHRESHOLD = 350;
volatile int IRR_HIGHTHRESHOLD = 350;
volatile int PHOTOLL_THRESHOLD = 200 - PhotoOffset;
volatile int PHOTOL_THRESHOLD = 200 - PhotoOffset;
volatile int PHOTOR_THRESHOLD = 200 - PhotoOffset;
volatile int PHOTORR_THRESHOLD = 200 - PhotoOffset;
volatile int DEADZONE = 20;
obstaclePos position;


//phototransitors
int NOISE_THRESHOLD = 10;



bool spinComplete = false;
int driveForward = 0;
int startRotate = false;
int dAngle;

volatile actions last_direction = CW;


// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo BackRightMotor;   // create servo object to control Vex Motor Controller 29
Servo FrontRightMotor;  // create servo object to control Vex Motor Controller 29
Servo FrontLeftMotor;   // create servo object to control Vex Motor Controller 29
Servo BackLeftMotor;    // create servo object to control Vex Motor Controller 29
Servo MainServo;

int speed_val = 100;
int turningSpeed = 50;
int speed_change;


//Timer interrupt function
void TimerHandler1() {
  unsigned long diff;
  int temp;  //used to temperarily store variables
  if (timerCounter == TIMER_ITERATION_COUNT) {


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
    sumIRFL -= arrIRFL[measurecountdist];
    arrIRFL[measurecountdist] = temp;
    sumIRFL += temp;
    distIRFL = sumIRFL / avgSizedist;
    if (distIRFL <= 0) { distIRFL = 9999; }

    signalADCR = analogRead(irsensorR);
    temp = 114462 * pow(signalADCR, -1.146);
    sumIRFR -= arrIRFR[measurecountdist];
    arrIRFR[measurecountdist] = temp;
    sumIRFR += temp;
    distIRFR = sumIRFR / avgSizedist;
    if (distIRFR <= 0) { distIRFR = 9999; }

    signalADCLShort = analogRead(irsensorLShort);
    temp = 13595 * pow(signalADCLShort, -0.916);
    sumIRL -= arrIRL[measurecountdist];
    arrIRL[measurecountdist] = temp;
    sumIRL += temp;
    distIRL = sumIRL / avgSizedist;
    if (distIRL <= 0) { distIRL = 9999; }

    signalADCRShort = analogRead(irsensorRShort);
    temp = 23519 * pow(signalADCRShort, -1.014);
    sumIRR -= arrIRR[measurecountdist];
    arrIRR[measurecountdist] = temp;
    sumIRR += temp;
    distIRR = sumIRR / avgSizedist;
    if (distIRR <= 0) { distIRR = 9999; }

    temp = sonar();
    sumUltra -= arrUltra[measurecountdist];
    arrUltra[measurecountdist] = temp;
    sumUltra += temp;
    distUltra = sumUltra / avgSizedist;
    if (distUltra <= 0) { distUltra = 9999; }



    angGyro = gyro();

    //Coordinate position - peter wrote this
    if (positionSet) {
      xValue = 2000 - distUltra - SonarOffset;
      if (xValue < 0) {
        xValue = 0;  //Prevents the X value from going below 0
      }
      if (yValue > 600) {
        switched = true;
      }
      if (switched) {
        // yValue = switchPoint + (difference) * float(IRRStart - switchPoint) / (1200.0 - float(IRRStart)) * (float(IRRStart) - float(distIRR)) + (IRRStart - distIRR);
        yValue = 1200 - distIRFR;
      } else {
        yValue = distIRFL;
      }
      if (yValue < 0) {
        yValue = 0;
      }
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

    if (measurecountdist == avgSizedist - 1) {
      measurecountdist = 0;
    } else {
      measurecountdist++;
    }


    //     ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣶⣿⣿⣷⣶⣄⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀
    // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣾⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⣿⣷⣦⡀⠀⠀⠀⠀⠀
    // ⠀⠀⠀⠀⠀⠀⠀⢀⣾⣿⣿⡟⠁⣰⣿⣿⣿⡿⠿⠻⠿⣿⣿⣿⣿⣧⠀⠀⠀⠀
    // ⠀⠀⠀⠀⠀⠀⠀⣾⣿⣿⠏⠀⣴⣿⣿⣿⠉⠀⠀⠀⠀⠀⠈⢻⣿⣿⣇⠀⠀⠀
    // ⠀⠀⠀⠀⢀⣠⣼⣿⣿⡏⠀⢠⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀⠈⣿⣿⣿⡀⠀⠀
    // ⠀⠀⠀⣰⣿⣿⣿⣿⣿⡇⠀⢸⣿⣿⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡇⠀⠀
    // ⠀⠀⢰⣿⣿⡿⣿⣿⣿⡇⠀⠘⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⢀⣸⣿⣿⣿⠁⠀⠀
    // ⠀⠀⣿⣿⣿⠁⣿⣿⣿⡇⠀⠀⠻⣿⣿⣿⣷⣶⣶⣶⣶⣶⣿⣿⣿⣿⠃⠀⠀⠀
    // ⠀⢰⣿⣿⡇⠀⣿⣿⣿⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀
    // ⠀⢸⣿⣿⡇⠀⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠛⠉⢉⣿⣿⠀⠀⠀⠀⠀⠀
    // ⠀⢸⣿⣿⣇⠀⣿⣿⣿⠀⠀⠀⠀⠀⢀⣤⣤⣤⡀⠀⠀⢸⣿⣿⣿⣷⣦⠀⠀⠀
    // ⠀⠀⢻⣿⣿⣶⣿⣿⣿⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣦⡀⠀⠉⠉⠻⣿⣿⡇⠀⠀
    // ⠀⠀⠀⠛⠿⣿⣿⣿⣿⣷⣤⡀⠀⠀⠀⠀⠈⠹⣿⣿⣇⣀⠀⣠⣾⣿⣿⡇⠀⠀
    // ⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⣿⣦⣤⣤⣤⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀
    // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠻⢿⣿⣿⣿⣿⣿⣿⠿⠋⠉⠛⠋⠉⠉⠁⠀⠀⠀⠀
    // ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⠁

    // Execute Control Code Here:

    // control();

    // Serial1.print("Combined Left: ");
    // Serial1.print(LeftPhotoTranDist());
    // Serial1.print(", Combined Right: ");
    // Serial1.println(RightPhotoTranDist());

    // Serial1.print("distPhotoLL: ");
    // Serial1.print(distPhotoLL);
    // Serial1.print(", distPhotoL: ");
    // Serial1.print(distPhotoL);
    // Serial1.print(", distPhotoR: ");
    // Serial1.print(distPhotoR);
    // Serial1.print(", distPhotoRR: ");
    // Serial1.println(distPhotoRR);



    timerCounter = 1;
  } else {
    timerCounter += 1;
  }

  signalADCPhotoLL = analogRead(photoPinLL);  //2 stripe Top left
  if (signalADCPhotoLL != 0) {
    temp = 52817 * pow(signalADCPhotoLL, -0.693);  //Replace with lineariased equation
    sumPhotoLL -= arrPhotoLL[measureCount];
    arrPhotoLL[measureCount] = temp;
    sumPhotoLL += temp;
    distPhotoLL = getAverage(arrPhotoLL);
    // distPhotoLL = temp;
    RawLL = distPhotoLL;
    if (distPhotoLL < 400) {
      distPhotoLL = 9998;
    } else if (distPhotoLL > UPPER_PHOTO_THRESH) {
      distPhotoLL = 9999;
    }
  } else {
    distPhotoLL = 9998;
  }

  signalADCPhotoL = analogRead(photoPinL);  //1 stripes bottom left
  if (signalADCPhotoL != 0) {
    temp = 2944.4 * pow(signalADCPhotoL, -0.508);  //Replace with lineariased equation
    sumPhotoL -= arrPhotoLL[measureCount];
    arrPhotoL[measureCount] = temp;
    sumPhotoL += temp;
    distPhotoL = getAverage(arrPhotoL);
    // distPhotoL = temp;
    RawL = distPhotoL;
    if (distPhotoL < 85) {
      distPhotoL = 9998;
    } else if (distPhotoL > 700) {
      distPhotoL = 9999;
    }
  } else {
    distPhotoL = 9998;
  }


  signalADCPhotoR = analogRead(photoPinR);  //3 stripes bottom right
  if (signalADCPhotoR != 0) {
    temp = 3085.3 * pow(signalADCPhotoR, -0.513);  //Replace with lineariased equation
    sumPhotoR -= arrPhotoR[measureCount];
    arrPhotoR[measureCount] = temp;
    sumPhotoR += temp;
    distPhotoR = getAverage(arrPhotoR);
    // distPhotoR = temp;
    RawR = distPhotoR;
    if (distPhotoR < 85) {
      distPhotoR = 9998;
    } else if (distPhotoR > 700) {
      distPhotoR = 9999;
    }
  } else {
    distPhotoR = 9998;
  }

  signalADCPhotoRR = analogRead(photoPinRR);  //4 stripes top right
  if (signalADCPhotoRR != 0) {
    temp = 44542 * pow(signalADCPhotoRR, -0.659);  //Replace with lineariased equation
    UnfilteredRawRR = temp;
    sumPhotoRR -= arrPhotoRR[measureCount];
    arrPhotoRR[measureCount] = temp;
    sumPhotoRR += temp;
    distPhotoRR = getAverage(arrPhotoRR);
    // distPhotoRR = temp;
    RawRR = distPhotoRR;
    if (distPhotoRR < 400) {
      distPhotoRR = 9998;
    } else if (distPhotoRR > UPPER_PHOTO_THRESH) {  //was 2000
      distPhotoRR = 9999;
    }
  } else {
    distPhotoRR = 9998;
  }


  //Serial.println(diff);
}

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;

void setup(void) {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(19, INPUT);
  pinMode(18, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);  //set fan pin as output

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  digitalWrite(trigPin, LOW);

  ITimer2.init();
  if (ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, TimerHandler1)) {
    Serial.print(F("Starting  ITimer2 OK, millis() = "));
    Serial.println(millis());
  }

  Serial1.begin(115200);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  // delay(1000);
  SerialCom->println("Setup....");
  MainServo.write(180);
  MainServo.attach(servo);

  Serial.println(MainServo.read());

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
      Serial.println("STOPPED");
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
  //motors(avoid(distUltra, irsensorL, irsensorR, irsensorRShort, irsensorLShort));
  delay(33);
  //motors(avoid(distUltra, distIRFL, distIRFR, distIRR, distIRL));
  // actions action = STRAFELEFT;
  // motors(action);
  //Serial1.println("Working");
  // Serial.println(distPhotoRR);
  // Serial1.print("Main Loop: ");
  // Serial1.println(millis());

  //control();   //<------This executes all functions together
  // motors(avoid()); //works 28/05
  // motors(pointSimple());

  control();
  // motors(pointSimple());

  Serial1.print(" | Combined (L , R): ");
  Serial1.print(LeftPhotoTranDist());
  Serial1.print(" | ");
  Serial1.print(RightPhotoTranDist());
  Serial1.print(" | ");

  Serial1.print("distPhoto (LL , L, R, RR): ");
  Serial1.print(distPhotoLL);
  Serial1.print(" | ");
  Serial1.print(distPhotoL);
  Serial1.print(" | ");
  Serial1.print(distPhotoR);
  Serial1.print(" | ");
  Serial1.print(distPhotoRR);
  Serial1.print(" |");

  Serial1.print("Raw (LL , L, R, RR): ");
  Serial1.print(RawLL);
  Serial1.print(" | ");
  Serial1.print(RawL);
  Serial1.print(" | ");
  Serial1.print(RawR);
  Serial1.print(" | ");
  Serial1.print(RawRR);
  Serial1.print(" | ");
  Serial1.print(extinguishCount);
    // Serial1.print("Range (U, L, R): ");
    // Serial1.print(distUltra);
    // Serial1.print(" | ");
    // Serial1.print(distIRL);
    // Serial1.print(" | ");
    // Serial1.print(distIRR);
    // Serial1.print(" | ");
    // Serial1.print(turningSpeed);







    // Serial1.print("distIRFL: ");
    // Serial1.print(distIRFL);
    // Serial1.print(", distUltra: ");
    // Serial1.print(distUltra);
    // Serial1.print(", distIRFR: ");
    // Serial1.print(distIRFR);
    // Serial1.print(", distIRL: ");
    // Serial1.print(distIRL);
    // Serial1.print(", distIRR: ");
    // Serial1.print(distIRR);

    // control();








    // firePosSimple(distPhotoLL, distPhotoL, distPhotoR, distPhotoRR);

    //drive(6.0, 0.0, 20.0);

    // testMotors();



    Serial1.println("");





  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    // SerialCom->println("RUNNING---------");
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
    // SerialCom->print("Lipo level:");
    // SerialCom->print(Lipo_level_cal);
    // SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    // SerialCom->println("");
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
  pinMode(servo, INPUT);
}

void enable_motors() {
  BackRightMotor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  FrontRightMotor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  FrontLeftMotor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  BackLeftMotor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

//This function is slowing our output down. Chat GPT says that serial communication may also be slowing the programme down. This doesn't really make sense though cause the serial communication is fast without sonar.
//delay microseconds should only delay it by about a millisecond.
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
  duration = pulseIn(echoPin, HIGH, 17000);
  // Calculating the distance
  distance = (duration * 0.34 / 2) * 1.0259 - 15.375;
  // Prints the distance on the Serial Monitor
  endTime = micros();
  //interrupts();
  //Serial.println(endTime - startTime);
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

long sort_desc(const void *cmp1, const void *cmp2) {
  // Need to cast the void * to int *
  long a = *((long *)cmp1);
  long b = *((long *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

long getAverage(long arrayInput[avgSize]) {
  long sortedArray[avgSize];
  for (int i = 0; i < avgSize; i++) {
    sortedArray[i] = arrayInput[i];
  }

  qsort(sortedArray, avgSize, sizeof(sortedArray[0]), sort_desc);
  long sum = 0;
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
  if (abs(pwr) < 10) {
    return 0;
  }

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





// Function to test motors
void testMotors() {

  // Define power level increments
  int powerIncrement = 1;

  // Define maximum power level to test
  int maxPower = 600;

  // Define delay between power level changes (in milliseconds)
  int delayTime = 100;

  // Define gyroscope reading threshold to detect motor movement
  float gyroThreshold = 0.1;

  // Define array to hold motor names
  String motorNames[4] = { "Back Right", "Front Right", "Front Left", "Back Left" };

  // Loop through each motor
  for (int i = 0; i < 4; i++) {
    String motorName = motorNames[i];
    Serial1.print("Testing motor: ");
    Serial1.println(motorName);

    // Set initial power level
    int powerLevel = powerIncrement;

    // Loop until motor starts moving in positive direction
    bool positiveSpinDetected = false;
    while (!positiveSpinDetected && powerLevel <= maxPower) {
      setMotorPower(i, powerLevel);
      delay(delayTime);
      if (distUltra <= 200) {
        Serial1.print(" Positive spin detected! ");
        positiveSpinDetected = true;
        powerLevel = 0;
      }
      powerLevel += powerIncrement;
    }

    // Loop until motor starts moving in negative direction
    bool negativeSpinDetected = false;
    powerLevel = -powerIncrement;
    while (!negativeSpinDetected && powerLevel >= -maxPower) {
      setMotorPower(i, powerLevel);
      delay(delayTime);
      if (distUltra <= 200) {
        Serial1.print(" Negative spin detected! ");
        negativeSpinDetected = true;
        powerLevel = 0;
      }
      powerLevel -= powerIncrement;
    }

    // Stop motor and wait before moving on to the next one
    setMotorPower(i, 0);
    delay(1000);
  }
}

// Helper function to set motor power for a given index
void setMotorPower(int index, int powerLevel) {
  switch (index) {
    case 0:
      BackRightMotor.writeMicroseconds(1500 + powerLevel);
      break;
    case 1:
      FrontRightMotor.writeMicroseconds(1500 + powerLevel);
      break;
    case 2:
      FrontLeftMotor.writeMicroseconds(1500 + powerLevel);
      break;
    case 3:
      BackLeftMotor.writeMicroseconds(1500 + powerLevel);
      break;
  }
}

void drive(float v_x, float v_y, float w_z) {
  // Drive in any direction
  // v_x: +ve value = forward (vice versa)
  // v_y: +ve value = goes left (vice versa)
  // w_z: +ve value = CCW (vice versa)

  if ((v_x == 0 && v_y == 0) || (v_x == 0 && w_z == 0) || (v_y == 0 && w_z == 0)) {
    v_x = constrain(v_x, -1, 1) * constrain(abs(v_x), 2.0, 18.0);
    v_y = constrain(v_y, -1, 1) * constrain(abs(v_y), 2.0, 18.0);
    w_z = constrain(w_z, -1, 1) * constrain(abs(w_z), 11.0, 90.0);
  }

  front_left = (v_x - v_y - (L_X + L_Y) * w_z) / WHEEL_R;
  front_right = -(v_x + v_y + (L_X + L_Y) * w_z) / WHEEL_R;
  back_left = (v_x + v_y - (L_X + L_Y) * w_z) / WHEEL_R;
  back_right = -(v_x - v_y + (L_X + L_Y) * w_z) / WHEEL_R;

  Serial.print("Left Front: ");
  Serial.println(front_left);
  Serial.print("Left Rear: ");
  Serial.println(back_left);
  Serial.print("Right Rear: ");
  Serial.println(back_right);
  Serial.print("Right Front: ");
  Serial.println(front_right);

  FrontLeftMotor.writeMicroseconds(1500 + front_left);
  FrontRightMotor.writeMicroseconds(1500 + front_right);
  BackLeftMotor.writeMicroseconds(1500 + back_left);
  BackRightMotor.writeMicroseconds(1500 + back_right);
}

actions getUnstuck() {
  Serial1.print("getUnstuck()");
  actions behaviour = STOP;
  if (unstuckCounter < 10) {
    behaviour = BACKWARD;
  } else {  //not really necessary given how our function works i believe, cause we should go into fireseeking once we are far enough from the wall or issue,
    behaviour = CW;
  }
  unstuckCounter++;
  Serial1.print(behaviour);
  Serial1.print(", UnstuckCounter = ");  //keeps printing 1 but does also keep printing stuck in front of wall.
  Serial1.print(unstuckCounter);
  return behaviour;
}

bool objectsOnThreeSides() {
  if (distIRL < IRL_THRESHOLD && distIRR < IRR_THRESHOLD && distUltra < ULTRA_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectsOnBothSides() {
  if (distIRL < IRL_THRESHOLD && distIRR < IRR_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectInFront() {
  if (distUltra < ULTRA_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectFrontRightAndLeft() {
  if (distIRFL < IRFL_THRESHOLD && distIRFR < IRFR_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectFrontLeft() {
  if (distIRFL < IRFL_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectFrontRight() {
  if (distIRFR < IRFR_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectRight() {
  if (distIRR < IRR_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectLeft() {
  if (distIRL < IRL_THRESHOLD) {
    return true;
  }
  return false;
}

bool objectFrontAndLeftSide() {
  if (distIRL < IRL_HIGHTHRESHOLD && distUltra < ULTRA_THRESHOLD) {
    return true;
  } else {
    return false;
  }
}

bool objectFrontAndRightSide() {
  if (distIRR < IRR_THRESHOLD && distUltra < ULTRA_THRESHOLD) {
    return true;
  } else {
    return false;
  }
}

bool objectFarLeftSide() {
  if(distIRL < IRL_HIGHTHRESHOLD) {
    return true;
  }
  return false;
}

bool objectFarRightSide() {
    if(distIRR < IRR_HIGHTHRESHOLD) {
    return true;
  }
  return false;
}

int LeftPhotoTranDist() {
  if (distPhotoL < 9000 && distPhotoLL < 9000) {
    if (distPhotoL >= 85 && distPhotoL < 600) {
      return distPhotoL;
    } else if (distPhotoLL > 600 && distPhotoLL <= UPPER_PHOTO_THRESH) {
      return distPhotoLL;
    }
  } else if (distPhotoL < 9000) {
    return distPhotoL;
  } else if (distPhotoLL < 9000) {
    return distPhotoLL;
  }

  else if (distPhotoL == 9998) {
    return 9998;
  } else if (distPhotoLL == 9999) {
    return 9999;
  }
}


int RightPhotoTranDist() {
  if (distPhotoR < 9000 && distPhotoRR < 9000) {
    if (distPhotoR >= 85 && distPhotoR < 600) {
      return distPhotoR;
    } else if (distPhotoRR > 600 && distPhotoRR <= UPPER_PHOTO_THRESH) {  //was 2000
      return distPhotoRR;
    }
  } else if (distPhotoR < 9000) {
    return distPhotoR;
  } else if (distPhotoRR < 9000) {
    return distPhotoRR;
  }

  else if (distPhotoR == 9998) {
    return 9998;
  } else if (distPhotoRR == 9999) {
    return 9999;
  }
}

actions avoid() {
  actions behaviour = STOP;




  if (objectsOnThreeSides()) {
    Serial1.print("stuck between objects and wall");
    unstuckCounter = 0;
    return getUnstuck();
  }

  else if (fCounter > 30) {
    unstuckCounter = 0;
    Serial1.print("stuck in front of wall");
    fCounter = 0;
    return getUnstuck();
  }

  else if (unstuckCounter > 0 and unstuckCounter < 15) {  //in theory this should solve the issue of the unstuck function being unable to complete its full cycle.
    Serial1.print("Unstuck continues");
    return getUnstuck();
  }

  else if (objectsOnBothSides()) {
    if (distIRL - distIRR >= DEADZONE) {
      Serial1.print("Avoid Right");
      position = RIGHT;
    } else if (distIRL - distIRR <= DEADZONE) {
      Serial1.print("Avoid Left");
      position = LEFT;
    } else {
      position = TIGHTSPACE;
    }
  } else if (objectInFront()) {  // adding case for staring at obstacle in front and wall to left or right

    if (objectFrontRightAndLeft()) {
      if (objectFarLeftSide()) {
        Serial1.print("objects both in front and to the left");
        position = LEFT;
        fCounter++;
      } else if (objectFarRightSide()) {
        Serial1.print("objects both in front and to the right");
        position = RIGHT;
        fCounter++;
      } else
        Serial1.print("object in front, all front sensors detect");
      position = FRONT;
      fCounter++;
    }


    else if (objectFrontLeft()) {
       if (objectLeft()) {
        Serial1.print("objects both in front and to the left");
        position = LEFT;
        fCounter++;
      } else if (objectFarRightSide()) {
        Serial1.print("objects both in front and to the right");
        position = RIGHT;
        fCounter++;
      } else {
        Serial1.print("object in front left");
        position = FRONTLEFT;
        fLCounter = 4;
        fCounter++;
      }
    }

    else if (objectFrontRight()) {

      if (objectFarLeftSide()) {
        position = LEFT;
        fCounter++;
      }
      else if (objectLeft()) {
        Serial1.print("objects both in front and to the left");
        position = LEFT;
        fCounter++;
      } else if (objectRight()) {
        Serial1.print("objects both in front and to the right");
        position = RIGHT;
        fCounter++;
      } else {
        Serial1.print("object in front right");
        position = FRONTRIGHT;
        fRCounter = 4;
        fCounter++;
      }
    }

    else if (objectFarLeftSide()) {
        position = LEFT;
        fCounter++;
      }
    else if (objectLeft()) {
      Serial1.print("objects both in front and to the left");
      position = LEFT;
      fCounter++;

    } else if (objectRight()) {
      Serial1.print("objects both in front and to the right");
      position = RIGHT;
      fCounter++;
    }

    else {
      Serial1.print("object in front");
      position = FRONT;
      fCounter++;
    }
  }

  else if (objectFrontLeft()) {
    Serial1.print(" object in front left ");
    position = FRONTLEFT;
    fCounter++;
    fLCounter = 4;
    if (objectFarRightSide()) {
        position = RIGHT;
      }
  }

  else if (objectFrontRight()) {
    Serial1.print("object in front right");
    position = FRONTRIGHT;
    fRCounter = 4;
    fCounter++;
    if (objectFarLeftSide()) {
        position = LEFT;
      }
  }

  else if (objectRight()) {
    Serial1.print("object on right");
    position = RIGHT;
  }

  else if (objectLeft()) {
    Serial1.print("object on left");
    position = LEFT;
  }

  else if (fLCounter > 0) {
    position = FRONTLEFT;
  }

  else if (fRCounter > 0) {
    Serial1.print("object in front right, waiting on timer");
    position = FRONTRIGHT;
  }

  else {
    // Serial1.print("no objects seen");
    position = CLEAR;
  }

  fLCounter--;
  fRCounter--;

  // Serial1.print("Obstacle Possition: ");
  // Serial1.print(obstaclePosDebug[position]);
  // Serial1.print(", distIRFL: ");
  // Serial1.print(distIRFL);
  // Serial1.print(", distUltra: ");
  // Serial1.print(distUltra);
  // Serial1.print(", distIRFR: ");
  // Serial1.print(distIRFR);
  // Serial1.print(", distIRL: ");
  // Serial1.print(distIRL);
  // Serial1.print(", distIRR: ");
  // Serial1.println(distIRR);

  switch (position) {
    case FRONT:
      behaviour = STRAFELEFT;
      break;
    case FRONTRIGHT:
      behaviour = STRAFELEFT;
      break;
    case FRONTLEFT:
      behaviour = STRAFERIGHT;
      break;
    case RIGHT:
      behaviour = STRAFELEFT;
      break;
    case LEFT:
      behaviour = STRAFERIGHT;
      break;
    case CLEAR:
      behaviour = FORWARD;
      break;
    case TIGHTSPACE:
      behaviour = FORWARD;
      break;
  }

  return behaviour;
}

int fireDirection(int distPhotoLL, int distPhotoL, int distPhotoR, int distPhotoRR, int pt_space) {
  // Reject noise by checking if each input is below a certain level
  if (distPhotoLL < NOISE_THRESHOLD && distPhotoL < NOISE_THRESHOLD && distPhotoR < NOISE_THRESHOLD && distPhotoRR < NOISE_THRESHOLD) {
    return 999;
  }

  // Normalize the sensor readings and alternative code for a 2 and 2 design with close sensing and far sensing phototransistors
  float normLL = (float)distPhotoLL / (distPhotoLL + distPhotoRR);  //(float)distPhotoLL / (distPhotoLL + distPhotoRR)
  float normL = (float)distPhotoL / (distPhotoL + distPhotoR);      // (float)distPhotoL / (distPhotoL + distPhotoR)
  float normR = (float)distPhotoR / (distPhotoL + distPhotoR);      // float)distPhotoR / (distPhotoL + distPhotoR)
  float normRR = (float)distPhotoRR / (distPhotoLL + distPhotoRR);  // (float)distPhotoRR / (distPhotoLL + distPhotoRR)

  // Calculate the weighted average of the sensor positions
  // float avg_pos = (-1.5 * normLL + -0.5 * normL + 0.5 * normR + 1.5 * normRR) * pt_space;
  //altered code
  float avg_pos_far = tan((distPhotoLL - distPhotoRR));
  float avg_pos_close = (-0.5 * normL + 0.5 * normR) * pt_space;

  // Convert the average position to degrees
  //2 cases for when the cart is close enough
  //if (distPhotoL <300 or distPhotoR < 300) { //problem with this one is big changes in the swap over. Maybe fuzzy logic for whenever the phototransistor can detect anything? This should in theory work.
  // int direction = (int)(avg_pos / pt_space * 45);
  //int direction = (int)(avg_pos_far / pt_space * 45);
  //int direction = (int) (avg_pos_close / pt_space * 45);
  // }

  int direction = 1;
  return direction;
  //to make the code work i think this function also needs to change behaviour to firelocated or something.
}


int photoRRLowFlag() {
  if (distPhotoRR < 600 && photoRRFlag != 2 && photoRRFlag != 3) {  //number goes down below measurable eg distPhotoRR = 550
    photoRRFlag = 1;
  } else if (distPhotoRR > 600 && photoRRFlag == 1) {  //number comes back up eg. distPhotoRR = 650
    photoRRFlag = 2;
  } else if (distPhotoRR < 600 && photoRRFlag != 1 && photoRRFlag != 0) {  //number goes down again: wiggling distPhotoRR = 550, remains in here as distPhotoRR gets smaller
    photoRRFlag = 3;
  } else {  //reset to 0 once we have located our particular light source or the photosensors no longer detect any light.
    photoRRFlag = 0;
  }
  return photoRRFlag;
}



int photoLLLowFlag() {  //see above comments for the right flag
  if (distPhotoLL < 600 && photoLLFlag != 2 && photoLLFlag != 3) {
    photoLLFlag = 1;
  } else if (distPhotoLL > 600 && photoLLFlag == 1) {
    photoLLFlag = 2;
  } else if (distPhotoLL < 600 && photoLLFlag != 1 && photoLLFlag != 0) {
    photoLLFlag = 3;
  } else {
    photoLLFlag = 0;
  }
  return photoLLFlag;
}

int photoRRHighFlag() {
  if (distPhotoRR > UPPER_PHOTO_THRESH && photoRRHFlag != 2 && photoRRHFlag != 3) {  //number goes down below measurable eg distPhotoRR = 550
    photoRRHFlag = 1;
  } else if (distPhotoRR < UPPER_PHOTO_THRESH && photoRRHFlag == 1) {  //number comes back up eg. distPhotoRR = 650
    photoRRHFlag = 2;
  } else if (distPhotoRR > UPPER_PHOTO_THRESH && photoRRHFlag != 1 && photoRRHFlag != 0) {  //number goes down again: wiggling distPhotoRR = 550, remains in here as distPhotoRR gets smaller
    photoRRHFlag = 3;
  } else {  //reset to 0 once we have located our particular light source or the photosensors no longer detect any light.
    photoRRHFlag = 0;
  }
  return photoRRHFlag;
}

int photoLLHighFlag() {
  if (distPhotoLL > UPPER_PHOTO_THRESH && photoLLHFlag != 2 && photoLLHFlag != 3) {  //number goes down below measurable eg distPhotoRR = 550
    photoLLHFlag = 1;
  } else if (distPhotoLL < UPPER_PHOTO_THRESH && photoLLFlag == 1) {  //number comes back up eg. distPhotoRR = 650
    photoLLHFlag = 2;
  } else if (distPhotoLL > UPPER_PHOTO_THRESH && photoLLHFlag != 1 && photoLLHFlag != 0) {  //number goes down again: wiggling distPhotoRR = 550, remains in here as distPhotoRR gets smaller
    photoLLHFlag = 3;
  } else {  //reset to 0 once we have located our particular light source or the photosensors no longer detect any light.
    photoLLHFlag = 0;
  }
  return photoLLHFlag;
}

actions firePosSimple() {
  // Serial1.print(", distPhotoLL: ");
  // Serial1.print(distPhotoLL);
  // Serial1.print(", distPhotoRR: ");
  // Serial1.println(distPhotoRR);
  // current issue: long range sensing results in oscillations for unknown reasons.
  //If can't fix through sensor code possible workaround: check the gyro, if gyro shows oscillation back and forth for a certain number of times say 5 times then just turn for a certain time period.
  //can also implement similar control to what we did for close range.

  if (RightPhotoTranDist() < 9000 || LeftPhotoTranDist() < 9000) {

    if (RightPhotoTranDist() < 9000 && LeftPhotoTranDist() > 9000) {
      //turn right
      if (photoRRLowFlag() != 3) {
        turningSpeed = 30;
        Serial1.print("Turn Left");
        return CW;
      } else if (photoRRLowFlag == 3) {
        Serial1.print("Turn Right");
        return CCW;
      } else if (photoRRHighFlag() != 3) {
        turningSpeed = 30;
        Serial1.print("Turn Left");
        return CW;
      } else if (photoRRHighFlag() == 3) {
        Serial1.print("Turn Right");
        return CCW;
      }
    }

    else if (LeftPhotoTranDist() < 9000 && RightPhotoTranDist() > 9000) {
      //turn left
      if (photoLLLowFlag() != 3) {
        turningSpeed = 30;
        Serial1.print("Turn Right");
        return CCW;
      } else if (photoLLLowFlag == 3) {
        Serial1.print("Turn Left");
        return CW;
      } else if (photoLLHighFlag() != 3) {
        turningSpeed = 30;
        Serial1.print("Turn Right");
        return CCW;
      } else if (photoLLHighFlag() == 3) {
        Serial1.print("Turn Left");
        return CW;
      }

    }

    else if (LeftPhotoTranDist() - RightPhotoTranDist() >= 200) {
      //turn right
      turningSpeed = 30;
      Serial1.print("Turn right");
      return CW;
    } else if (RightPhotoTranDist() - LeftPhotoTranDist() >= 200) {
      //turn Left
      Serial1.print("Turn left");
      turningSpeed = 30;
      return CCW;
    } else if (LeftPhotoTranDist() - RightPhotoTranDist() >= 100) {
      //diagonal right
      Serial1.print("Diagonal right");
      return DIAGRIGHT;
    } else if (RightPhotoTranDist() - LeftPhotoTranDist() >= 100) {
      //diagonal Left
      Serial1.print("Diagonal left");
      return DIAGLEFT;
    } else
      // go striaght
      turningSpeed = 40;
    Serial1.print("Go Straight");
    return FORWARD;
  }

  else {
    return CW;
    // return last_direction;
  }
}


//drive in a straight line, stop, rotate and look for a fire, continue
actions seek(int fireDirection) {
  //Issue to fix: robot gets stuck looking at stuff from far away (has this been fixed since I saw it last?)
  if (fireDirection != 999) {
    if (fireDirection >= 10) {
      return CCW;
    } else if (fireDirection <= -10) {
      return CW;
    } else {
      return FORWARD;
    }
  } else if (fireDirection == 999 && driveForward < 30) {
    spinComplete = false;
    startRotate = true;  //was false earlier in case this breaks it
    driveForward++;
    return FORWARD;
  } else if (driveForward == 30 && fireDirection == 999 && !spinComplete) {
    if (startRotate) {
      dAngle = 0;
      !startRotate;
    }
    if (dAngle > 360) {
      spinComplete = true;
      driveForward = 0;
    }
    return CW;
  }
}

actions pointSimple() {
  // if ((RightPhotoTranDist() > 2000 && RightPhotoTranDist() < 3000) || (LeftPhotoTranDist() > 2000 && LeftPhotoTranDist() < 3000)) {
  //   return FORWARD;
  // }
  if (RightPhotoTranDist() < 9000 || LeftPhotoTranDist() < 9000) {
    return firePosSimple();
  } else Serial1.print("Searching");
  // return last_direction;
  turningSpeed = 40;
  return CW;
}

// actions point() {  // spin around and point towards the nearest fire
// if(turn &&){

// }
//   if (pointStart) {
//     currentAngle = 0;
//     fireAngle = 0;
//     currentDist = 9999;
//     fireDist = 9999;
//     pointStart = 0;
//     turn = 0
//   }


//   if (distPhotoL + distPhotoR < distPhotoLL + distPhotoRR) {
//     currentdist = (distPhotoL + distPhotoR) / 2;
//   } else currentdist = (distPhotoLL + distPhotoRR) / 2;

//   if (currentDist < fireDist) {
//     fireDist = currentDist;
//     fireAngle = currentAngle;
//   }

//   if (currentAngle >355){
//     pointStart = 1;
//     turn = 1;
//   }
//   return CCW;
// }


void motors(actions command) {


  switch (command) {
    case STOP:
      stop();
      break;
    case FORWARD:
      forward();
      break;
    case STRAFELEFT:
      strafe_left();
      break;
    case STRAFERIGHT:
      strafe_right();
      break;
    case CW:
      cw();
      break;
    case CCW:
      ccw();
      break;
    case BACKWARD:
      reverse();
      break;
    case ENABLEFAN:
      enableFan();
      break;
    case DISABLEFAN:
      disableFan();
      break;
    case DIAGLEFT:
      diagL();
      break;
    case DIAGRIGHT:
      diagR();
      break;
  }
}




void stop()  //Stop
{
  BackRightMotor.writeMicroseconds(1500);
  FrontRightMotor.writeMicroseconds(1500);
  FrontLeftMotor.writeMicroseconds(1500);
  BackLeftMotor.writeMicroseconds(1500);
}

void forward() {
  BackRightMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 4));
  FrontRightMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 2));
  FrontLeftMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 1));
  BackLeftMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 3));
}

void reverse() {
  BackRightMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 4));
  FrontRightMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 2));
  FrontLeftMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 1));
  BackLeftMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 3));
}

void ccw() {
  BackRightMotor.writeMicroseconds(1500 - lineariseMotors(turningSpeed, 4));
  FrontRightMotor.writeMicroseconds(1500 - lineariseMotors(turningSpeed, 2));
  FrontLeftMotor.writeMicroseconds(1500 - lineariseMotors(turningSpeed, 1));
  BackLeftMotor.writeMicroseconds(1500 - lineariseMotors(turningSpeed, 3));
}

void cw() {
  BackRightMotor.writeMicroseconds(1500 + lineariseMotors(turningSpeed, 4));
  FrontRightMotor.writeMicroseconds(1500 + lineariseMotors(turningSpeed, 2));
  FrontLeftMotor.writeMicroseconds(1500 + lineariseMotors(turningSpeed, 1));
  BackLeftMotor.writeMicroseconds(1500 + lineariseMotors(turningSpeed, 3));
}

void strafe_left() {
  BackRightMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 4));
  FrontRightMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 2));
  FrontLeftMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 1));
  BackLeftMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 3));
}

void strafe_right() {
  BackRightMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 4));
  FrontRightMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 2));
  FrontLeftMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 1));
  BackLeftMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 3));
}

void enableFan() {
  stop();
  digitalWrite(FAN_PIN, HIGH);
}

void disableFan() {
  digitalWrite(FAN_PIN, LOW);
}

void diagL() {
  BackRightMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 4));
  FrontRightMotor.writeMicroseconds(1500 - lineariseMotors(speed_val, 2));
  FrontLeftMotor.writeMicroseconds(1500 + .5 * lineariseMotors(speed_val, 1));
  BackLeftMotor.writeMicroseconds(1500 + .5 * lineariseMotors(speed_val, 3));
}

void diagR() {
  BackRightMotor.writeMicroseconds(1500 - .5 * lineariseMotors(speed_val, 4));
  FrontRightMotor.writeMicroseconds(1500 - .5 * lineariseMotors(speed_val, 2));
  FrontLeftMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 1));
  BackLeftMotor.writeMicroseconds(1500 + lineariseMotors(speed_val, 3));
}

actions extinguish() {
  turningSpeed = 80;
  return ENABLEFAN;
}

//if I've understood how you want to do things, we use avoid, extinguish and seek to return a command we want to do, then at the end of our control logic to choose which one we want to call we call motors with the command
void control() {
  //Start off with the weights you want to give to each sensor: (how much you rely on the sensor for a measurement eg in front of you. Weights should add up to 1 for a direction
  //front facing sensors
  //float ultrasoundWeight = 0.5;
  //float infraredFrontRightWeight = 0.25;
  //float infraredFrontLeftWeight = 0.25;
  //the left and right not weighted as only one sensor used for sensing these directions
  //int infraredRightWeight = 1;
  //int infraredLeftWeight = 1;
  //Phototransistor Weights
  //float photoCheckLLWeight = 0.25;
  //float photoCheckLWeight = 0.25;
  //float photoCheckRWeight = 0.25;
  //float photoCheckRRWeight = 0.25;
  int distanceCheck = 0;  //changed this to int
  //these checks tell us if the each individual sensor thinks there is something they can see close to us
  int distIRFRCheck = 0;
  int distIRFLCheck = 0;
  int distIRRCheck = 0;
  int distIRLCheck = 0;
  int photoCheckLL = 0;
  int photoCheckL = 0;
  int photoCheckR = 0;
  int photoCheckRR = 0;
  int fireDistanceCheck = 0;
  int avgPhotoCheckLR = 0;
  //dummy variables to check if the control system works. These values should trigger the avoid function
  //  int distance = 100;
  //  int distIRFR = 100;
  //  int distIRFL = 100;
  //  int distIRR = 100;
  //  int distIRL = 100;
  //  int distPhoto1 = 100;
  //  int distPhoto2 = 100;
  //  int distPhoto3 = 100;
  //  int distPhoto4 = 400;
  //check if there is an obstacle a certain distance ahead : 15 cm for us
  //checks if ultrasound is measuring an obstacle within 15 cm, if it is we set distance check to 1 indicating that the ultrasound thinks there is an obstacle there
  distanceCheck = (distUltra <= ULTRA_THRESHOLD) ? 1 : 0;  //  if (distance<=150) { distanceCheck = 1; }, these two codes are equivalent
  distIRFRCheck = (distIRFR <= IRFR_THRESHOLD) ? 1 : 0;
  distIRFLCheck = (distIRFL <= IRFL_THRESHOLD) ? 1 : 0;
  distIRRCheck = (distIRR <= IRR_THRESHOLD) ? 1 : 0;
  distIRLCheck = (distIRL <= IRL_THRESHOLD) ? 1 : 0;
  photoCheckLL = (distPhotoLL <= PHOTOLL_THRESHOLD) ? 1 : 0;
  photoCheckL = (distPhotoL <= PHOTOL_THRESHOLD) ? 1 : 0;
  photoCheckR = (distPhotoR <= PHOTOR_THRESHOLD) ? 1 : 0;
  photoCheckRR = (distPhotoRR <= PHOTORR_THRESHOLD) ? 1 : 0;
  fireDistanceCheck = (distUltra <= 120) ? 1 : 0;

  avgPhotoCheckLR = ((distPhotoL + distPhotoR) / 2.0 <= PHOTOL_THRESHOLD) ? 1 : 0;
  //multiple sensors of ours tell us if there is something in front of us, probably relying primarily on ultrasound.
  //front check is the final check to tell if there is something in front of us
  //float frontCheck = distanceCheck * ultrasoundWeight + distIRFRCheck * infraredFrontRightWeight + distIRFLCheck * infraredFrontLeftWeight;
  //check if phototransistors see an LED
  // if (photoCheckL == 1 || photoCheckR == 1) {
  if (extinguishCount == 2) {
    disable_motors();
    stopped();
    while (1) {
      Serial1.print("AHhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh");
      delay(1000);
    }
  } else if (avgPhotoCheckLR) {
    stillExtinguishingCount++;
    Serial1.print("Extinguish");
    motors(extinguish());
    PHOTOL_THRESHOLD = 300 - PhotoOffset;
    if (stillExtinguishingCount == 30) {
      isExtinguishing = true;
    }

    // if (avgPhotoCheckLR) {
    //   if (!fireDistanceCheck) {
    //     forward();
    //   } else {
    //     Serial1.print("Extinguish");
    //     motors(extinguish());
    //   }
    // }

  } else if (distIRFRCheck == 1 or distIRFLCheck == 1 or distanceCheck == 1 or distIRLCheck == 1 or distIRRCheck == 1) {
    //check if there is an object in front too close or to either side to activate avoid to move past it
    //Issue to fix: sometimes our robot does not avoid the obstacles. Need to check if its not entering the avoid state or if our avoid is just not doing well enough at avoiding the obstacles.
    //may need a more sensitive avoid function possibly, else may be able to fix by slowing down the speed of the robot a little bit: but we would be looking to get our robot to complete the course within a certain time period.
    if (isExtinguishing == true) {
      extinguishCount++;
      isExtinguishing = false;
    }
    stillExtinguishingCount = 0;

    PHOTOL_THRESHOLD = 200 - PhotoOffset;
    Serial1.print("Avoid");
    //Serial1.print(", fCounter: ");
    //Serial1.print(fCounter);
    disableFan();



    actions action = avoid();
    if (action == 5) {
      last_direction = 5;
    } else if (action == 6) {
      last_direction = 6;
    }


    if (photoCheckL) {
      action = CCW;  // 6
      fCounter = 0;
    }
    if (photoCheckR) {
      action = CW;  // 5
      fCounter = 0;
    }

    motors(action);
    //avoid(frontCheck, distIRLCheck, distIRRCheck) could do that way to pass what state it should enter.
  } else {
    stillExtinguishingCount = 0;
    PHOTOL_THRESHOLD = 200 - PhotoOffset;
    //seek(fireDirection);
    // Serial1.println(", control loop calls seek ");
    disableFan();
    motors(pointSimple());
    //motor(seek)());
  }

  if (RightPhotoTranDist() < 250 || LeftPhotoTranDist() < 250) {
    speed_val = 40;
    ULTRA_THRESHOLD = 80;
    IRFL_THRESHOLD = 50;
    IRFR_THRESHOLD = 50;
  } else {
    speed_val = 100;
    ULTRA_THRESHOLD = 150;  //changed from 100 to 150 to try and help the obstacle avoidance
    IRFL_THRESHOLD = 150;
    IRFR_THRESHOLD = 150;
  }
}
