void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial communication
  while (!Serial) {
    // Wait for the serial port to connect
  }
  Serial.println("Serial communication started!");
}

void loop() {
  // put your main code here, to run repeatedly:
  control();
}

void control() {
  //Start off with the weights you want to give to each sensor: (how much you rely on the sensor for a measurement eg in front of you. Weights should add up to 1 for a direction
  //front facing sensors
  float ultrasoundWeight = 0.5;
  float infraredFrontRightWeight = 0.25;
  float infraredFrontLeftWeight = 0.25;
  //the left and right not weighted as only one sensor used for sensing these directions
  //int infraredRightWeight = 1;
  //int infraredLeftWeight = 1;
  //Phototransistor Weights
  float photoCheck1Weight = 0.25;
  float photoCheck2Weight = 0.25;
  float photoCheck3Weight = 0.25;
  float photoCheck4Weight = 0.25;
  float distanceCheck = 0;
  //these checks tell us if the each individual sensor thinks there is something they can see close to us
  int distIRFRCheck = 0;
  int distIRFLCheck = 0;
  int distIRRCheck = 0;
  int distIRLCheck = 0;
  int photoCheck1 = 0;
  int photoCheck2 = 0;
  int photoCheck3 = 0;
  int photoCheck4 = 0;
  //dummy variables to check if the control system works. These values should trigger the avoid function
    int distance = 100;
    int distIRFR = 100;
    int distIRFL = 100;
    int distIRR = 100;
    int distIRL = 100;
    int distPhoto1 = 100;
    int distPhoto2 = 100;
    int distPhoto3 = 100;
    int distPhoto4 = 400;
  //check if there is an obstacle a certain distance ahead : 15 cm for us
  //checks if ultrasound is measuring an obstacle within 15 cm, if it is we set distance check to 1 indicating that the ultrasound thinks there is an obstacle there
  distanceCheck = (distance <= 150) ? 1 : 0; //  if (distance<=150) { distanceCheck = 1; }, these two codes are equivalent
  distIRFRCheck = (distIRFR <= 150) ? 1 : 0;
  distIRFLCheck = (distIRFL <= 150) ? 1 : 0;
  distIRRCheck = (distIRR <= 150) ? 1 : 0;
  distIRLCheck = (distIRL <= 150) ? 1 : 0;
  photoCheck1 = (distPhoto1 <= 250) ? 1 : 0;
  photoCheck2 = (distPhoto2 <= 250) ? 1 : 0;
  photoCheck3 = (distPhoto3 <= 250) ? 1 : 0;
  photoCheck4 = (distPhoto4 <= 250) ? 1 : 0;
  //multiple sensors of ours tell us if there is something in front of us, probably relying primarily on ultrasound.
  //front check is the final check to tell if there is something in front of us
  float frontCheck = distanceCheck * ultrasoundWeight + distIRFRCheck * infraredFrontRightWeight + distIRFLCheck * infraredFrontLeftWeight;
  //check if phototransistors see an LED
  float fireCheck = photoCheck1Weight * photoCheck1 + photoCheck2Weight * photoCheck2 + photoCheck3Weight * photoCheck3 + photoCheck3Weight * photoCheck4;
  if (fireCheck >= 0.6) {
    extinguish();
  }
  else if (frontCheck >= 0.6 or distIRLCheck == 1 or distIRRCheck == 1) {
    //check if there is an object in front too close or to either side to activate avoid to move past it
    avoid();
    //avoid(frontCheck, distIRLCheck, distIRRCheck) could do that way to pass what state it should enter.
  }
  else {
    locate();
  }
}


void avoid() {
  Serial.println("avoid");
  return;
}
void locate() {
  Serial.println("locate");
  return;
}
void extinguish() {
  Serial.println("extinguish");
}
