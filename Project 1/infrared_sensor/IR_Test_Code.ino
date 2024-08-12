int irsensorL = A4;     //sensor is attached on pinA0 
int irsensorR = A5;
byte serialRead = 0;  //for control serial communication  
int signalADC = 0;  // the read out signal in 0-1023 corresponding to 0-5v
void readSensor(); 
int sonar();

// defines pins numbers
const int trigPin = 34;
const int echoPin = 35;
// defines variables
long duration;
int distance;
 
void setup() { 
  // put your setup code here, to run once: 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600);    // start serial communication  
} 
 
void loop() { 
  // put your main code here, to run repeatedly: 

   
  signalADC = analogRead(irsensor);   // the read out is a signal from 0-1023 corresponding to 0-5v 
  readSensor(rightIR);
} 

int readSensor(String sensor);
{
 int sensorPin;
 int minDelay = 0; //the minimum period between polling the sensor
  if(sensor == leftIR) {
    sensorPin = irsensorL;
    minDelay = 55;
  }
  else if (sensor == rightIR) {
    sensorPin = irsensorR;
    minDelay = 55;
  }
//  else if (sensor == ultraSonic) {
//    sensorPin 
//  }
  
    int arr[10];
    int sum = 0;

    for (int i = 0; i <= 9; i++) {
    arr[i] = analogRead(sensorPin);
    delay(minDelay); //Minimum time between measurements
    }

    // Sorting the array
    for(int i=0; i<10; i++) {
        for(int j=i+1; j<10; j++) {
            if(arr[i] > arr[j]) {
                int temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }
    }

    // Removing the highest 2 and lowest 2 elements
    for(int i=2; i<8; i++) {
        sum += arr[i];
    }

    // Calculating the average of the remaining 6 numbers
    double avg = sum / 6.0;

     if(sensor == leftIR) {
      distIRL = 16749*pow(avg,-1.204);
    }
    else if (sensor == rightIR) {
      distIRR = 16860*pow(avg,-1.204);
    }
  //  else if (sensor == ultraSonic) {
  //    sensorPin 
  //  }
}

int sonar() {
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
  Serial.println(distance);
  return(distance);
}
