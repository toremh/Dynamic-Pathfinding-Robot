#define TIMER4_INTERVAL_MS    100 //Sets the polling rate of the sensors
#define USE_TIMER_4     true
 
#include "TimerInterrupt.h"


byte serialRead = 0;  //for control serial communication  

int readSensor(); 
int sonar();

//IR sensors
int irsensorL = A4; //Input for Left IR sensor 
int irsensorR = A5; //Input for Right IR sensor 
int signalADCL = 0; // the read out signal in 0-1023 corresponding to 0-5v
int signalADCR = 0; // the read out signal in 0-1023 corresponding to 0-5v

//Ultrasonic sensor
const int trigPin = 34;
const int echoPin = 35;
volatile long duration;
volatile int distance;

//Gyroscope
const int gyroPin = A3;           //define the pin that gyro is connected  
volatile int loopTime = 0;   // T is the time of one loop
volatile unsigned long prevMillis = 0;                       
volatile int sensorValue = 0;           // read out value of sensor  
float gyroSupplyVoltage = 5;      // supply voltage for gyro 
float gyroZeroVoltage = 0;         // the value of voltage when gyro is zero  
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet  
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity less  
                                                       // than this value will be ignored 
volatile float gyroRate = 0;                      // read out value of sensor in voltage   
volatile float currentAngle = 0; 
volatile bool firstRun = true;

//Timer interrupt variables
volatile int measureCount = 0; //Sets the starting point in the array for moving average
volatile int index = 1; //Sets the starting point in the array replacing old values for moving average
const int avgSize = 10; //This controls the size of the moving average filter. Reduce to speed up polling

//These are treated as circular arrays and store the values for the moving average calculation
volatile int arrIRL[avgSize] = {0};
volatile int arrIRR[avgSize] = {0};
volatile int arrUltra[avgSize] = {0};
volatile float arrGyro[avgSize] = {0};

//sumXXX is the sum of the values stored in arrXXX
//distXXX is the averaged distance calculated from sumXXX
volatile int distIRL;
volatile int sumIRL;
volatile int distIRR;
volatile int sumIRR;
volatile int distUltra;
volatile int sumUltra;
volatile float angGyro;
volatile float sumGyro;

//Debug variables for calculating timing of a function
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
 
 //Timer interrupt function
void TimerHandler1() {

  int temp; //used to temperarily store variables
  Serial.println(millis());
  /* 
    Each block of code essentially works the same,
    1. Store the calculed distance/value in temp
    2. Remove the old value from the calculated sum
    3. Store the new value in the appropriate position in the circular array
    4. Add the new value to the sum
    5. Calculate the distance from the average
  */
  signalADCL = analogRead(irsensorL);
  temp = 167490*pow(signalADCL,-1.204);
  sumIRL -= arrIRL[measureCount];
  arrIRL[measureCount] = temp;
  sumIRL += temp;
  distIRL = sumIRL/((float)avgSize);

  signalADCR = analogRead(irsensorR);
  temp = 168600*pow(signalADCR,-1.204);
  sumIRR -= arrIRR[measureCount];
  arrIRR[measureCount] = temp;
  sumIRR += temp;
  distIRR = sumIRR/((float)avgSize);

  temp = sonar();
  sumUltra -= arrUltra[measureCount];
  arrUltra[measureCount] = temp;
  sumUltra += temp;
  distUltra = sumUltra/((float)avgSize);

  //temp = gyro();
  //sumGyro -= arrGyro[measureCount];
  //arrGyro[measureCount] = temp;
  //sumGyro += temp;
  //angGyro = sumGyro/((float)avgSize);
  angGyro = gyro();


//The counter variable used to index the circular arrays
  if (measureCount == avgSize-1) {
    measureCount = 0;
  }
  else {
    measureCount++;
  }
}
  
 
void setup() {
 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  ITimer4.init();
 
  if (ITimer4.attachInterruptInterval(TIMER4_INTERVAL_MS, TimerHandler1))
  {
    Serial.print(F("Starting  ITimer4 OK, millis() = ")); Serial.println(millis());
  }
  Serial.begin(9600); 

  //Calibrate the gyroscope 
  float sum = 0;     
  pinMode(gyroPin,INPUT);   
  
  noInterrupts();
  Serial.println("please keep the sensor still for calibration"); 
  Serial.println("get the gyro zero voltage"); 
  for (int i=0;i<100;i++)    // read 100 values of voltage when gyro is at still, to calculate the zero-drift.  
  {  
    sensorValue = analogRead(gyroPin); 
    sum += sensorValue; 
    delay(5); 
    } 
  gyroZeroVoltage = sum/100;    // average the sum as the zero drifting
  interrupts();
}
 
void loop() {




//------------------DEBUG---------------------
//  Serial.print("Left IR = ");
//  Serial.print(distIRL);
//  Serial.println("mm");

//  Serial.print("Right IR = ");
//  Serial.print(distIRR);
//  Serial.println("cm");

//  Serial.print("Ultrasonic = ");
//  Serial.print(distUltra);
//  Serial.println("cm");
//  Serial.print("Sonar function reading: ");
//  Serial.println(sonar());
Serial.println(angGyro);

//  for (int i = 0; i<=9; i++) {
//    Serial.print(arrUltra[i]);
//    Serial.print(" ");
//  }
 delay(1000);
//------------------DEBUG---------------------

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
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  endTime = micros();
  //interrupts();
  return(distance);
}

float gyro() {
  // convert the 0-1023 signal to 0-5v 
gyroRate = (analogRead(gyroPin)*gyroSupplyVoltage)/1023;    
 
// find the voltage offset the value of voltage when gyro is zero (still) 
gyroRate -= (gyroZeroVoltage/1023*gyroSupplyVoltage);    
 
// read out voltage divided the gyro sensitivity to calculate the angular velocity  
float angularVelocity = -1*gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
  loopTime = millis() - prevMillis;
  prevMillis = millis();
// if the angular velocity is less than the threshold, ignore it 
if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
{ 

  //Because the execution time varies depending on the other functions being executed, this
  //keeps track of how much time has elapsed


// we are running a loop of loopTime/1000 second).  
 float angleChange = angularVelocity/(1000/loopTime); 
 currentAngle += angleChange;  
  } 
 
  // keep the angle between 0-360 
  if (currentAngle < 0) 
    {currentAngle += 360;} 
  else if (currentAngle > 359) 
{currentAngle -= 360;} 

if (firstRun) { //Because of the variable timing, the first result must be disgarded
  firstRun = false;
  return 0;
} else {
return currentAngle;
}
}
