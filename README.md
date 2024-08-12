# Dynamic-Pathfinding-Robot
Two projects on dynamic robotic control for a final year engineering course, Mechatronics Design Projects. Both projects were completed in our group of four using an arduino, several motors, 3D printed structural parts and housings, and an array of sensors. These projects involved software development for Arduino in C++, mechanical prototyping, and breadboard electronics implementation.



![robotfront-mh](https://github.com/user-attachments/assets/bc1d33a2-1bf1-4274-acd2-72cbc48bbdac)
Front sensor array (Project 2)

![Breadboard](https://github.com/user-attachments/assets/24968d39-1125-461a-8f43-9b191a9b3271)
Final Breadboard Layout (Project 2)

![firetruck robot](https://github.com/user-attachments/assets/9eacfd02-4f29-4615-b001-63ee16186101)
"Artist's" rendition, back from when AI gen images were new and interesting.

## Project 1

The first project required our robot to cover every square in a grid, as observed by an overhead computer vision system, after being placed randomly in an X by Y walled area. The scenario given was that the robot was intended to fertilize a field, and needed to cover the entire area in an even pattern.

The primary difficulty came from implementing control logic to overcome faulty hardware. Both the motors and sensors we were requierd to use for the project were (intentionally?) extremely unreliable; the motors would provide wildly varying torque and speed outputs for the same voltage input, and the sensors would give a high percentage of faulty outputs, including . We eventually overcame these issues by implementing a PID controller for each motor, along with several automatic scaling and correction factors to allow the robot to drive straight. The sensors required a couple of signal processing algorithms, including comparisons to previous and expected outputs, in order to produce useable data and guide the robot in the correct direction.

## Project 2

The second project was more complicated, and required us to build upon the learnings of the first. Instead of a static environment, the robot was placed into an environment with moving obstacles, and tasked with finding and approaching several light sources in order to blow on them with a small fan, completed in the least timme possible. The scenario was an automated firefighting robot in an active disaster zone.

This project required all of the same control systems as the first project, with additional logic to allow the robot to scan the environment, detect the light sources, and travel towards them, dynamically avoiding obstacles. This required several control patterns, implemented using a finite state control system, which enabled the robot to prioritize the correct action in a given situation. The first issue we had to overcome was the physical design of the robot, in order to efficiently scan the environment and avoid obstacles. We found that a partially rotating sensor array paired with the smallest frame practical gave us the best results. The second issue we overcame was on the control side, developing our algorithms in such a way as to allow the robot to break out of edge cases, such as when an obstacle would move in front of the robot such that it seemed to be blocked on all sides, or when the robot would continuously gain and lose sight of the target lights. We eventually solved this through rearchitecting the control flow to limit the number of executions of each state logic before attempting an alternative action.
