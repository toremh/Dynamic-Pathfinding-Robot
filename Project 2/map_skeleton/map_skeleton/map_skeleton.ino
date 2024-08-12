#include <Kalman.h>
using namespace BLA;

#define MAP_SIZE_X 1000 // Size of the occupancy grid map in the X direction (2000/cell size)
#define MAP_SIZE_Y 600 // Size of the occupancy grid map in the Y direction  (1200/cell size)
#define CELL_SIZE 2   // Size of each cell in mm

// Dimensions of the matrices
#define Nstate 3 // length of the state vector : x ,y , angle
#define Nobs 3   // length of the measurement vector , should be able to detect if in front, to the left or right and what angle we are at.

// measurement std (to be characterized from your sensors)
#define n1 0.2 // noise on the 1st measurement component
#define n2 0.1 // noise on the 2nd measurement component 

// model std (1/inertia)
#define m1 0.01
#define m2 0.05

BLA::Matrix<Nobs> obs; // observation vector
KALMAN<Nstate,Nobs> K; // Kalman filter

bool grid[MAP_SIZE][MAP_SIZE]; // Occupancy grid map
int sensorReadings[8];         // Ultrasonic sensor readings, change this to be the correct pin.

//void updateOccupancyGrid()
//{
//  // Loop through each cell in the map
//  for (int x = 0; x < MAP_SIZE_X; x++)
//  {
//    for (int y = 0; y < MAP_SIZE_Y; y++)
//    {
//      // Calculate the distance from the cell to the robot using sensor readings, Kalman filter used to do this
//      int dist = calculateDistance(x, y);
//
//      // If the distance is less than a threshold, mark the cell as occupied
//      if (dist < THRESHOLD)
//      {
//        grid[x][y] = true;
//      }
//    }
//  }
//}

int objectlocation(int x, int y) //Kalman filter stuff: find where the object is on the map
{
  int location[0][0];         //object location in x and y
  
  return location;
}

void printOccupancyGrid()
{
  // Print the occupancy grid map
  for (int y = 0; y < MAP_SIZE_Y; y++)
  {
    for (int x = 0; x < MAP_SIZE_X; x++)
    {
      if (grid[x][y])
      {
        Serial.print("#");
      }
      else
      {
        Serial.print(".");
      }
    }
    Serial.println();
  }
}

void sensor_measure(){
  //rip this from the sensor measuring code return as observation
  /*We got multiple sensors. One is straight ahead. one at 45 degrees front left, one at 45 degrees front right
   * One on right, one on left. The filter needs to take in information from all of these sources, do I want to use them all to map though
   * ultrasound function should just return the front distance.
   * How does the grid work with our angles. Hmm diagonal squares occupied? Our thing is going to need to turn.
   * Left and right are more simple: just update left and right on the grid
   * take in front: see an obstacle it is in front of wherever the robot is facing.
   * Left front sees obstacle: go diagonally.
   * Left sees obstacle: fill in to the left, right goes to the right
   * I also need to implement object permanence: if there is already an object determined
   * to be there don't make it a new object. Hmm maybe I don't need to do anything? maybe just let
   * the map fill itself out and it will make no difference if square is already occupied.
   */
   //Need to measure x, y and angle. then how they change when the motor is activated. Our model input should be what the motion target is.
   //So our sensors can only measure what is in front of us + what is beside us.
   //Gyroscope is the only measurement for angle measurement
   //Use ultrasound for X
   //Use infrared for Y, not sure how to incorporate those angled infrareds?
   
}

void calculate_distance(int x, int y) {
  //calculate distance from object taken from kalman
}

void setup()
{
  Serial.begin(57600);
  //set up Kalman Filter matrices
  // time evolution matrix: this is the prediction of how the variables will change given their current states.
  K.F = {1.0, 0.0,
         0.0, 1.0};
  
  //measurement matrix: Our measurements we are measuring goes here. State of the system based on what we measure     
  K.H = {1.0, 0.0,
         0.0, 1.0};

  // measurement covariance matrix: Uncertainty related to measurements measured
  K.R = {n1*n1,   0.0,
           0.0, n2*n2};

  // model covariance matrix: uncertainty related to our predictive model
  K.Q = {m1*m1,   0.0,
           0.0, m2*m2};
   
//Initialise Occupancy grid
  for (int x = 0; x < MAP_SIZE; x++)
  {
    for (int y = 0; y < MAP_SIZE; y++)
    {
      grid[x][y] = false;
    }
  }
}

void loop()
{
  //update sensor data
    sensor_measure();

  //Apply Kalman Filter
  K.update(obs);


  // Update the occupancy grid map based on the sensor data
  updateOccupancyGrid();

  // Print the occupancy grid map
  printOccupancyGrid();
}

void sensor_update(){
  //read sensor data and calculate distance to be put into kalman filter
}
}


void main (){
  int ultrasoundWeight = 0.5;
  int infraredFrontRightWeight = 0.25;
  int infraredFrontLeftWeight = 0.25;
  //int infraredRightWeight = 1;
  //int infraredLeftWeight = 1;
  int photoCheck1Weight = 0.25;
  int photoCheck2Weight = 0.25;
  int photoCheck3Weight = 0.25;
  int photoCheck4Weight = 0.25;
  int distanceCheck = 0;
  //check if there is an obstacle a certain distance ahead : 15 cm for us
  while (distance<=150) { //checks if ultrasound is measuring an obstacle within 15 cm
    distanceCheck = 1;
  }
  else {
    distanceCheck =0;
  }
  while (distIRFR <=150) { //checks
    distIRFRCheck = 1;
  }
  else {
    distIRFRCheck =0;
  }
  while (distIRFL <=150) {
    distIRFLCheck = 1;
  }
  else {
    distIRFLCheck =0;
  }
  while (distIRR <=150) {
    distIRRCheck = 1;
  }
  else {
    distIRRCheck =0;
  }
  while (distIRL <=150) {
    distIRLCheck = 1;
  }
  else {
    distIRLCheck =0;
  }
  while (distPhoto1 <=250) {
    PhotoCheck1 = 1;
  }
  else {
    photoCheck1 =0;
  }
   while (distPhoto2 <=250) {
    photoCheck2 = 1;
  }
  else {
    photoCheck2 =0;
  }
   while (distPhoto3 <=250) {
    photoCheck3 = 1;
  }
  else {
    photoCheck3 =0;
  }
   while (distPhoto4 <=250) {
    photoCheck4 = 1;
  }
  else {
    photoCheck4 =0;
  }
  //multiple sensors of ours tell us if there is something in front of us, probably relying primarily on ultrasound.
  int frontCheck = distanceCheck *ultrasoundWeight + distIRFRCheck *infraredFrontRightWeight + distIRFLCheck*infraredFrontLeftWeight 
  int fireCheck = photoCheck1Weight*photoCheck1 + photoCheck2Weight*photoCheck2 + photoCheck3Weight*photoCheck3 + photoCheck3Weight*photoCheck4
  if (fireCheck >= 0.6) {
    extinguish();
  } 
  else if (frontCheck >= 0.6 or distIRLCheck =1 or distIRRCheck =1) { //check if there is an object in front too close to activate avoid to move past it
    avoid();
    //avoid(frontCheck, distIRLCheck, distIRRCheck) could do that way to pass what state it should enter.
  }
  else  {
    locate();
  }
  
}
