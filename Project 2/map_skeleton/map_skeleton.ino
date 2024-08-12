#define MAP_SIZE 100 // Size of the occupancy grid map
#define CELL_SIZE 5   // Size of each cell in cm

bool grid[MAP_SIZE][MAP_SIZE]; // Occupancy grid map
int sensorReadings[8];         // Ultrasonic sensor readings

void updateOccupancyGrid()
{
  // Loop through each cell in the map
  for (int x = 0; x < MAP_SIZE; x++)
  {
    for (int y = 0; y < MAP_SIZE; y++)
    {
      // Calculate the distance from the cell to the robot using sensor readings
      int dist = calculateDistance(x, y);

      // If the distance is less than a threshold, mark the cell as occupied
      if (dist < THRESHOLD)
      {
        grid[x][y] = true;
      }
    }
  }
}

int calculateDistance(int x, int y)
{
  // Calculate the distance from the cell to the robot using sensor readings
  // You will need to use trigonometry to calculate the distance based on the sensor readings
  // This function will depend on the specific configuration of your ultrasonic sensors
}

void printOccupancyGrid()
{
  // Print the occupancy grid map
  for (int y = 0; y < MAP_SIZE; y++)
  {
    for (int x = 0; x < MAP_SIZE; x++)
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

void setup()
{
  // Initialize ultrasonic sensors and other hardware
  // ...

  // Initialize the occupancy grid map
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
  // Read ultrasonic sensor data
  // ...

  // Update the occupancy grid map based on the sensor data
  updateOccupancyGrid();

  // Print the occupancy grid map
  printOccupancyGrid();
}
