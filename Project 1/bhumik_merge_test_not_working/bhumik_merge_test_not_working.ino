


#define TIMER3_INTERVAL_MS    10
#define USE_TIMER_3     true
#define PI 3.1415926535897932384626433832795

#include <Servo.h>
#include "TimerInterrupt.h"


const int LeftMotorDir = 4;
const int MotorLeft = 5;
const int MotorRight = 6;
const int RightMotorDir = 7;

unsigned long startTime, endTime;


bool homingMode = 0;
bool homing = 0;
int homingCount = 101;

bool LimitHit = 0;

#define LimSwLeft A8
#define LimSwRight A9
#define LimSwBottom A10
#define LimSwTop A11

const int LeftEncoderA = 18;
const int LeftEncoderB = 19;
const int RightEncoderA = 20;
const int RightEncoderB = 21;


const int radius = 7;

long LeftEncoderCount = 0;
long RightEncoderCount = 0;


unsigned int outputPin1 = LED_BUILTIN;

void TimerHandler1(unsigned int outputPin = LED_BUILTIN)
{
  static bool toggle1 = false;
  static bool started = false;

  if (!started)
  {
    started = true;
    pinMode(outputPin, OUTPUT);
  }

#if (TIMER_INTERRUPT_DEBUG > 1)
  //timer interrupt toggles pin outputPin, default LED_BUILTIN
  Serial.print("pin1 = "); Serial.print(outputPin);
  Serial.print(" address: "); Serial.println((uint32_t) &outputPin );
#endif

  //Serial.println(LeftEncoderCount);
  //Serial.println(RightEncoderCount);

  //Serial.println(digitalRead(RightEncoderA));
  //  Serial.println(digitalRead(RightEncoderA));

  if ((digitalRead(LimSwLeft) || digitalRead(LimSwRight) || digitalRead(LimSwBottom) || digitalRead(LimSwTop))) {
    if (homingCount == 101) {
      homingCount = 0;
    }
    if (homingMode) {
      if (digitalRead(LimSwLeft)) {
        // go right
        digitalWrite(LeftMotorDir, LOW);
        digitalWrite(RightMotorDir, LOW);
      }
      if (digitalRead(LimSwRight)) {
        // go left
        digitalWrite(LeftMotorDir, HIGH);
        digitalWrite(RightMotorDir, HIGH);
        homing = 1;
      }
      if (digitalRead(LimSwBottom)) {
        // go up
        digitalWrite(LeftMotorDir, LOW);
        digitalWrite(RightMotorDir, HIGH);
        homing = 1;
      }
      if (digitalRead(LimSwTop)) {
        // go down
        digitalWrite(LeftMotorDir, HIGH);
        digitalWrite(RightMotorDir, LOW);
        homing = 1;
      }
    } else {
      digitalWrite(MotorLeft, LOW);
      digitalWrite(MotorRight, LOW);
      LimitHit = 1;
    }
  } else {

    //LimitHit = 0;
  }
  if (homingCount < 100 && homingMode) {
    digitalWrite(MotorLeft, 1);
    digitalWrite(MotorRight, 1);
    homing = 1;
    homingCount += 1;
  } else if (homingCount >= 100 && (homingCount != 101)) {
    if (homing) {

      digitalWrite(MotorLeft, LOW);
      digitalWrite(MotorRight, LOW);
    }
    homing = 0;
    homingCount = 101;
  }
}


void TimerHandler(unsigned int outputPin = LED_BUILTIN)
{
  static bool toggle = false;
  static bool started = false;

  if (!started)
  {
    started = true;
    pinMode(outputPin, OUTPUT);
  }

  //timer interrupt toggles pin outputPin, default LED_BUILTIN
}


void setup() {

  ITimer3.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,
  // For 16-bit timer 1, 3, 4 and 5, set frequency from 0.2385 to some KHz
  // For 8-bit timer 2 (prescaler up to 1024, set frequency from 61.5Hz to some KHz

  if (ITimer3.attachInterruptInterval(TIMER3_INTERVAL_MS, TimerHandler1, outputPin1))
  {
    Serial.print(F("Starting  ITimer3 OK, millis() = ")); Serial.println(millis());
  }


  // put your setup code here, to run once:
  pinMode(MotorLeft, OUTPUT);
  pinMode(MotorRight, OUTPUT);
  pinMode(LeftMotorDir, OUTPUT);
  pinMode(RightMotorDir, OUTPUT);

  pinMode(LeftEncoderA, INPUT);
  pinMode(RightEncoderA, INPUT);
  pinMode(LeftEncoderB, INPUT);
  pinMode(RightEncoderB, INPUT);

  //analogWrite(MotorLeft, 100);

  attachInterrupt(digitalPinToInterrupt(LeftEncoderA), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoderB), isrRight, CHANGE);

  Serial.begin(9600);





}

void loop() {


  startTime = millis();

  // Performing Homing Routine

  homingMode = 1;
  while (!homing) {
    digitalWrite(LeftMotorDir, HIGH);
    digitalWrite(RightMotorDir, HIGH);
    analogWrite(MotorLeft, 200);
    analogWrite(MotorRight, 200);
  }
  Serial.println("Left Home'd");
  while (homing) {
    delay(1);
    continue;
  }

  Serial.println("part 1");

  while (!homing) {
    digitalWrite(LeftMotorDir, HIGH);
    digitalWrite(RightMotorDir, LOW);
    analogWrite(MotorLeft, 200);
    analogWrite(MotorRight, 200);
  }
  Serial.println("Bottom Home'd");
  while (homing) {
    delay(1);
    continue;
  }

  homingMode = 0;


  delay(1000);

  // Insert Commands Past This Point


  // Rectangle
  delay(2000);
  startTime = millis();
  moveXY(88, false, 1.2, 0.000005, 1.8, 1);
  moveXY(60, true, 2, 0.00015, 1.2, 1);
  moveXY(-90, false, 1.9, 0.00003, 1.8, 1);
  moveXY(-60, true, 2, 0.000005, 1.2, 1);
  endTime = millis();
  Serial.print("Time Elapsed: ");
  Serial.println(endTime - startTime);
//  while(1) delay(10000);


  moveXY(90, true, 2, 0.0001, 1.2, 1);
//  delay(2500);
  moveXY(40, false, 1.4, 0.00001, 1.8, 1);
//  delay(5000);
  
  startTime = millis();
  drawCircle(40, 2, 0.001, 0.9, 2, 0.001, 0.9, 0.3);
  endTime = millis();
  Serial.print("Time Elapsed: ");
  Serial.println(endTime - startTime);
  while(1) delay(10000);







  // put your main code here, to run repeatedly:
  //  digitalWrite(LeftMotorDir, HIGH);
  //  delay(1000);
  //  digitalWrite(LeftMotorDir, LOW);
  //  delay(1000);
  //  bool LeftEncoderA_Val = digitalRead(LeftEncoderA);
  //  bool LeftEncoderB_Val = digitalRead(LeftEncoderB);
  //  Serial.print("Clockwise Encoder Count: ");
  //  Serial.println(LeftEncoderCount);
  //  Serial.print("Anti-Clockwise Encoder Count: ");
  //  Serial.println(LeftACWEncoderCount);


  //moveXY(30,false, 1.4, 0.00001, 1.8, 1);
  //delay(3000);
  //moveXY(60,true, 2, 0.0001, 1.2, 1);
  //delay(3000);
  //moveXY(-30,false, 1.9, 0.000018, 1.6, 1);
  //delay(3000);
  //moveXY(-60,true, 2, 0.00001, 1.2, 1);
  //delay(3000);

  // Working Values So Far:
  // 30mm
  // moveXY(-30,true, 2, 0.00001, 1.2, 1);
  // moveXY(30,true, 2, 0.0006, 0.8, 1);
  // moveXY(-30,false, 1.9, 0.000018, 1.6, 1);
  // moveXY(30,false, 1.4, 0.00001, 1.8, 1);
  //
  // 50mm
  // moveXY(-50,true, 2, 0.00001, 1.2, 1);
  // moveXY(50,true, 2, 0.0001, 1.2, 1);
  // moveXY(-50,false, 1.9, 0.000018, 1.6, 1);
  // moveXY(50,false, 1.4, 0.00001, 1.8, 1);


  //  moveXY(-50,true, 2, 0.00001, 1.2, 1);
  //  drawCircle(40, 2, 0.001, 0.9, 2, 0.001, 0.9, 0.3);
  //  delay(10000);


  //moveXY(30,true, 1.6, 0.01, 0.1, 2);
  //moveXY(-30,true, 3 , 0.0, 0, 2);


  //moveXY(30,false, 0.5, 0.01, 0, 3);
  //moveXY(30,true, 0.5, 0.01, 0, 3);
  //moveXY(-30,false, 0.5, 0.01, 0, 3);





  /*
    if (LeftEncoderCount > (8256/2)) {
      digitalWrite(MotorLeft, LOW);
      delay(1000);
      LeftEncoderCount = 0;
      analogWrite(MotorLeft, 255);
    }
  */
}


void isrLeft () {
  if (digitalRead (LeftEncoderA)) {
    if (!digitalRead(LeftEncoderB)) {
      LeftEncoderCount--;
    } else {
      LeftEncoderCount++;
    }
  } else {
    if (!digitalRead(LeftEncoderB)) {
      LeftEncoderCount++;
    } else {
      LeftEncoderCount--;
    }
  }
  //  Serial.print("Left: ");
  //  Serial.println(LeftEncoderCount);
}  // end of isr




void isrRight () {
  if (digitalRead (RightEncoderA)) {
    if (!digitalRead(RightEncoderB)) {
      RightEncoderCount++;
    } else {
      RightEncoderCount--;
    }
  } else {
    if (!digitalRead(RightEncoderB)) {
      RightEncoderCount--;
    } else {
      RightEncoderCount++;
    }
  }
  //    Serial.print("Right: ");
  //  Serial.println(RightEncoderCount);
}

// end of isr



float getPos(bool xPlane) {
  // 4028 Counts per revolution
  float Counts;
  if (!xPlane) {
    Counts = (RightEncoderCount - LeftEncoderCount) / (2.0);
  } else {
    Counts = (RightEncoderCount + LeftEncoderCount) / (2.0);
  }
  //Serial.println(Counts);
  float revolutions = Counts / 4028.0;
  return (revolutions * 44.0);
}



void moveXY(float target, bool xPlane, float Kp, float Ki, float Kd,  float errorMax) {
  float currentPos, error, u = 0, error_Prv = target, saturation, error_Prv_d = target;
  int Pwr = 0, Pwr_temp;
  float Pwr_Prv = 0;
  
  float integralK = -target;
  int encError, deltaUX, deltaUY, PwrLimit;
  float Kp_X = 0.10; // original = 0.18 straight in Right
  float Kp_Y = 0.12;
  int iteration_count = 1;
  int iteration_check = 2 ;
  bool continueLoop = 1;

  LeftEncoderCount = 0;
  RightEncoderCount = 0;


  while (continueLoop == 1) {
    currentPos = getPos(xPlane);

    // currentPos needs to check direction
    if (!xPlane) {
      error = target - currentPos;
    } else {
      error = -currentPos - target;
    }
    if ((abs(u) < 100) || error_Prv != error) {
      integralK = integralK + error;
      //      Serial.println(integralK);
    }
    u = (Kp * error) + (Ki * integralK) + (Kd * (error - error_Prv_d));
    if (!xPlane) {
      encError = LeftEncoderCount + RightEncoderCount;
    } else {
      encError = LeftEncoderCount - RightEncoderCount;
    }
    deltaUX = Kp_X * encError;
    deltaUY = Kp_Y * encError;
    //    deltaUX = constrain(deltaUX, -70, 70);
    //    deltaUY = constrain(deltaUY, -70, 70);
    //    Pwr = (int) constrain(u, -200, 200);

    // FOR SPEED AND POWER
//    deltaUX = constrain(deltaUX, -40, 30);
//    deltaUY = constrain(deltaUY, -40, 30);
//    Pwr = (int) constrain(u, -200, 200);

    if (abs(deltaUX) > abs(deltaUY)) {
      PwrLimit = 255 - abs(deltaUX);
    } else {
      PwrLimit = 255 - abs(deltaUY);
    }
    Pwr = (int) constrain(u, -PwrLimit, PwrLimit);
    
    saturation = abs((Pwr + Pwr_Prv) / 2);
    saturation = (int) ceil(saturation);
    Pwr = (int) constrain(Pwr, -saturation, saturation);

    Pwr_Prv = Pwr;
    if (!xPlane) {
      if (Pwr < 0) {
        digitalWrite(LeftMotorDir, HIGH);
        digitalWrite(RightMotorDir, LOW);

      } else {
        digitalWrite(LeftMotorDir, LOW);
        digitalWrite(RightMotorDir, HIGH);
      }
    } else {
      if (Pwr < 0) {
        digitalWrite(LeftMotorDir, LOW);
        digitalWrite(RightMotorDir, LOW);
      } else {
        digitalWrite(LeftMotorDir, HIGH);
        digitalWrite(RightMotorDir, HIGH);
      }
    }
    if (!LimitHit) {
      Pwr_temp = abs(Pwr);
      Pwr_temp = constrain(Pwr_temp, PwrLimit, PwrLimit);
      //      Serial.println(Pwr_temp);
      //      Serial.println(deltaUY);
      if (Pwr < 0) {

      }
      if (!xPlane) {
        if (Pwr > 0) {
          analogWrite(MotorLeft, Pwr_temp + deltaUY);
          analogWrite(MotorRight, Pwr_temp - deltaUY);
        }
        if (Pwr < 0) {
          analogWrite(MotorLeft, Pwr_temp - deltaUY);
          analogWrite(MotorRight, Pwr_temp + deltaUY);
        }
      } else {
        if (Pwr > 0) {
          analogWrite(MotorLeft, Pwr_temp - deltaUX);
          analogWrite(MotorRight, Pwr_temp + deltaUX);
        }
        if (Pwr < 0) {
          analogWrite(MotorLeft, Pwr_temp + deltaUY);
          analogWrite(MotorRight, Pwr_temp - deltaUY);
        }
      }

    } else {
      continueLoop = 0;
    }

    // After a certain number of iterations of the loop (Specified by iteration_check) this statement does its checks
    if ((iteration_count % iteration_check) == (iteration_check - 1)) {
      // If the error in the current iteration is equal to the error from the previous iteration check
      // and the error is also less than the maximum error speciied
      if (abs(error) <= errorMax) {
        if ((Pwr < 0) && (abs(error) < (abs(error_Prv) - 0.3 * errorMax)) && (abs(error) > abs(error_Prv) + 0.3 * errorMax)) {
          // We then set the continue loop variable to 0 to ensure the loop does not run again
          continueLoop = 0;
        } else if ((Pwr < 0) && (abs(error) > (abs(error_Prv) - 0.3 * errorMax)) && (abs(error) < abs(error_Prv) + 0.3 * errorMax)) {
          continueLoop = 0;
        }
      }
      // We then set the previous error to the current error which can be used later if we need to
      error_Prv = error;
    }

    // Saving the error from this iteration to be used to calculate the derivative part of the controller
    error_Prv_d = error;
    // To keep track of which iteration of the loop we are currently on we add one the the iteration count at the end of the loop
    iteration_count += 1;
    //    Serial.print("Power: ");
    //    Serial.println(Pwr_temp);
    Serial.print("Error: ");
    Serial.println(PwrLimit);
    //    Serial.print("Right: ");
    //    Serial.println(RightEncoderCount);
    //    Serial.print("Left: ");
    //    Serial.println(LeftEncoderCount);
    // Serial.println(abs(LeftEncoderCount - RightEncoderCount));
  }
  digitalWrite(MotorLeft, LOW);
  digitalWrite(MotorRight, LOW);
}


void drawCircle(float radius, float KpX, float KiX, float KdX, float KpY, float KiY, float KdY, float errorMax) {

  float degInc = 0.1;
  if (radius <= 10) {
    degInc = 0.8;
  } else if (radius > 10 && radius <= 25) {
    degInc = 0.4;
  }
  float currentDeg = 0;


  float targetX = 0;
  float targetY = 0;
  float targetXCor = 0;
  float targetYCor = 0;

  float errorX, errorY, errorPrvX, errorPrvY;
  float errorPrv_dX, errorPrv_dY;
  float absError;

  float posX, posY;

  float uX = 0;
  float uY = 0;

  float integralX = 0;
  float integralY = 0;

  int PwrX = 0;
  int PwrY = 0;

  LeftEncoderCount = 0;
  RightEncoderCount = 0;

  int iteration_count = 1;
  int iteration_check = 3;

  bool continueLoop = 1;
  while (continueLoop) {
//        Serial.print("X: ");
//        Serial.print(uX);
//        Serial.print("   ");
//        Serial.print("Y: ");
//        Serial.println(uY);
//    Serial.println(uX);

    targetX = radius * -cos(PI * (currentDeg / 180.0)) + radius + targetXCor;
    targetY = radius * sin(PI * (currentDeg / 180.0)) + targetYCor;
    posX = getPos(1);
    posY = getPos(0);
    errorX = -posX - targetX;
    errorY = -(targetY - posY);
    absError = sq(errorX) + sq(errorY);
    absError = sqrt(absError);

    if ((abs(uX) < 200) || errorPrvX != errorX) {
      integralX +=  errorX;
    }
    uX = (KpX * errorX) + (KiX * integralX) + (KdX * (errorX - errorPrv_dX));
    uX = 150 * uX;
    if ((abs(uY) < 200) || errorPrvY != errorY) {
      integralY +=  errorY;
    }
    uY = (KpY * errorY) + (KiY * integralY) + (KdY * (errorY - errorPrv_dY));
    uY = 150 * uY;

    // After a certain number of iterations of the loop (Specified by iteration_check) this statement does its checks
    if ((iteration_count % iteration_check) == (iteration_check - 1)) {
      // If the error in the current iteration is equal to the error from the previous iteration check
      // and the error is also less than the maximum error speciied
      if ((abs(errorX) <= errorMax) && (abs(errorY) <= errorMax)) {
        if ((currentDeg > 360)) {
          continueLoop = 0;
        }
      }
      // We then set the previous error to the current error which can be used later if we need to
      errorPrvX = errorX;
      errorPrvY = errorY;
    }
    errorPrv_dX = errorX;
    errorPrv_dY = errorY;

//    if (currentDeg > 330 && currentDeg < 360) {
//      targetXCor -= 0.0015;
//      targetYCor -= 0.00075;
//    }

    //r += 0.01;
    if (currentDeg < 315 && (absError < 0.6)) { // 150 power with 0.8 works
      //      currentDeg += ((degInc/(20.0)) * radius);
      currentDeg += ((degInc / (20.0)) * radius);
    } else if (currentDeg < 360 && (absError < 0.2)) {
      currentDeg += ((degInc / (20.0)) * radius);
      if (iteration_count % 100 == 0 && abs(targetXCor) < 1.0 && abs(targetYCor) < 1.0) {
        Serial.println(targetXCor);
//        targetXCor -= 0.22;
//        targetYCor -= 0.05;
        targetXCor -= 0.35;
        targetYCor -= 0.06;
      }
    }
    if (!LimitHit) {
      // Figure out what to do based on uX and uY

      // FIND DIRECTION NEED TO GO
      PwrX = constrain(uX, 240, 255);
      PwrY = constrain(uY, 240, 255);
      if (errorX > 0 && errorY > 0) {
        // bottom left
        if (abs(errorX) > abs(errorY)) {
          digitalWrite(LeftMotorDir, HIGH);
          digitalWrite(RightMotorDir, HIGH);
        } else {
          digitalWrite(LeftMotorDir, HIGH);
          digitalWrite(RightMotorDir, LOW);
        }
      } else if (errorX < 0 && errorY < 0) {
        // top right
        if (abs(errorX) > abs(errorY)) {
          digitalWrite(LeftMotorDir, LOW);
          digitalWrite(RightMotorDir, LOW);
        } else {
          digitalWrite(LeftMotorDir, LOW);
          digitalWrite(RightMotorDir, HIGH);
        }
      } else if (errorX > 0 && errorY < 0) {
        // top left
        if (abs(errorX) > abs(errorY)) {
          digitalWrite(LeftMotorDir, HIGH);
          digitalWrite(RightMotorDir, HIGH);
        } else {
          digitalWrite(LeftMotorDir, LOW);
          digitalWrite(RightMotorDir, HIGH);
        }
      } else if (errorX < 0 && errorY > 0) {
        // bottom right
        if (abs(errorX) > abs(errorY)) {
          digitalWrite(LeftMotorDir, LOW);
          digitalWrite(RightMotorDir, LOW);
        } else {
          digitalWrite(LeftMotorDir, HIGH);
          digitalWrite(RightMotorDir, LOW);
        }
      }
      analogWrite(MotorLeft, PwrX);
      analogWrite(MotorRight, PwrY);
    } else {
      continueLoop = 0;
    }
    // Saving the error from this iteration to be used to calculate the derivative part of the controller
    errorPrv_dX = errorX;
    errorPrv_dY = errorY;
    // To keep track of which iteration of the loop we are currently on we add one the the iteration count at the end of the loop
    iteration_count += 1;
  }
  Serial.println(iteration_count);
  digitalWrite(MotorLeft, LOW);
  digitalWrite(MotorRight, LOW);
  delay(50);
}

// http://andrewjkramer.net/motor-encoders-arduino/
// https://www.arduino.cc/reference/en/libraries/timerinterrupt/
// https://www.desmos.com/calculator/6hjayhfvng
//24.621
//31.090
// 14mm pulley diameter, 20 teeth

// If (abs error < (0.2) then we add to degrees).
