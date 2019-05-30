#include <iostream>
#include <signal.h>
#include <math.h>

#include <libplayerc++/playerc++.h>

#include "Vector2D.hh"

using namespace std;
using namespace PlayerCc;

bool programRunning = true;
void programShutdown(int );

// TODO: Implement the following functions:
bool doorDetection(const RangerProxy &, Vector2D & );

double computeStimulusRight(const RangerProxy & laser);
double computeStimulusLeft(const RangerProxy & laser);

double wanderLinearVelocity(double leftStimulus, double rightStimulus); 
double wanderAngularVelocity(double leftStimulus, double rightStimulus); 
void wander(const RangerProxy & lasers, Position2dProxy & base);

double targetLinearVelocityCalc(const Vector2D & target, const Vector2D & robot); // Linear velocity to reach door
double targetAngularVelocityCalc(const Vector2D & target, const Vector2D & robot, double robotHeading); // Angular velocity to reach door

double maxRadius = 3.0; // Max Radius - Used in computing stimuli
double maxVelocity = 0.3; // Max Linear Velocity - Real robot max velocity would be around 0.3 m/s
double sigma = 30 * M_PI / 180; // Sigma used in Wandering

double computeStimulusRight(const RangerProxy & laser)
{
  double retVal = 0;
  double wSum = 0.0;
  double weight = 0.0; // auxiliary variable
  
  int beams = laser.GetRangeCount();
  for (int i = 0; i < beams; ++i)
    {
      double theta = laser.GetMinAngle() + i * laser.GetAngularRes(); //Get value for angle theta for each laser scan
      if (theta < 0) // beams on the left have positive heading, right have negative
		{
			/*
			* The aux variable equals
			* the sum of the values of
			* the angle -theta squared
			* divided by sigma squared
			*/
		  weight = exp(- (theta / theta) * (sigma / sigma)); 
		  
		  // Add weight to the sum of the weights
		  wSum += weight;
		  // normR (normalized reading) holds temp value for normalizing i (reading), by dividing by max radius (rm)
		  double normR = laser[i] / maxRadius;
		  // ternary operation - if normalized reading is greater than 1 set it to 1
		  //otherwise value is set to 0
		  normR = (normR > 1) ? 1 : normR;
		  // retVal will contain a sum of the weighted normalized values.
		  retVal += (weight * normR);
		}
    }
  // sum of the weight * normalized reading divided by sum of the weights
  retVal /= wSum;

  return retVal;
}

double computeStimulusLeft(const RangerProxy & laser)
{
  double retVal = 0;
  double wSum = 0.0;
  double weight = 0.0; // auxiliary variable
  
  int beams = laser.GetRangeCount();
  for (int i = 0; i < beams; ++i)
    {
      double theta = laser.GetMinAngle() + i * laser.GetAngularRes(); //Get value for angle theta for each laser scan
      if (theta > 0) // beams on the left have positive heading, right have negative
		{
			/*
			* The aux variable equals
			* the sum of the values of
			* the angle -theta squared
			* divided by sigma squared
			*/
		  weight = exp(- (theta / theta) * (sigma / sigma)); 
		  
		  // Add weight to the sum of the weights
		  wSum += weight;
		  // normR (normalized reading) holds temp value for normalizing i (reading), by dividing by max radius (rm)
		  double normR = laser[i] / maxRadius;
		  // ternary operation - if normalized reading is greater than 1 set it to 1
		  // otherwise value is set to 0
		  normR = (normR > 1) ? 1 : normR;
		  // retVal will contain a sum of the weighted normalized values.
		  retVal += (weight * normR);
		}
    }
  // sum of the weight * normalized reading divided by sum of the weights
  retVal /= wSum;

  return retVal;
}

// Calculate the Linear velocity based on L/R Stimuli
double wanderLinearVelocity(double leftStimulus, double rightStimulus)  
{
	// maxSpeed * (LStim + RStim)/2
  return maxVelocity * ((leftStimulus + rightStimulus) / 2);
}

// Calculate the Angular velocity based on L/R Stimuli
double wanderAngularVelocity(double leftStimulus, double rightStimulus) 
{
	// maxSpeed * (LStim - RStim) divided by wheel base width
  return maxVelocity * ((leftStimulus - rightStimulus) / 0.4);
}

void wander(const RangerProxy & lasers, Position2dProxy & base)
{
	// Calculate stimulus
    double stimulusLeft = computeStimulusLeft(lasers);
    double stimulusRight = computeStimulusRight(lasers);

    // Calculate velocity and turning rate
    double v = wanderLinearVelocity(stimulusLeft, stimulusRight);
    double w = wanderAngularVelocity(stimulusLeft, stimulusRight);

    // Apply calculated values
    base.SetSpeed(v, 0, w);
}

double targetLinearVelocityCalc(const Vector2D & target, const Vector2D & robot)
{
  double linearVelocity = 0.0;
  Vector2D d;
  double maxDistance = 2; 
  //double vMax = 0.3; // Real robot max velocity would be around 0.3 m/s - DECLARED GLOBALLY
  
  d = target - robot; // Difference between the robot position and target position - 2 coordinates
  double distanceToTarget = d.Length(); // Get length of line between two points
  
  /* if distance to target > max distance then
  *  velocity = max velocity
  *  else calculate proportional velocity
  *  velocity = max velocity * distance to target / max distance 
  */
  if (distanceToTarget > maxDistance)
    linearVelocity = maxVelocity;
  else
    linearVelocity = maxVelocity * distanceToTarget / maxDistance;
  
  return linearVelocity;
}

double targetAngularVelocityCalc(const Vector2D & target, const Vector2D & robot, double robotHeading)
{
  double kw = 0.8; // Represents controller value in equation
  
  Vector2D d;
  
  d =  target - robot; // Difference between the robot position and target position - 2 coordinates

  double theta = d.Angle() - robotHeading; // 
  
  /* if theta is greater than pi
  * theta = theta - 2*pi
  * end if
  * if theta is less than negative pi
  * theta = theta + 2*pi
  * end if
  */
  if (theta > M_PI)
    theta -= (2 * M_PI);
  if (theta < -M_PI)
    theta += (2 * M_PI);
  
  kw = kw * theta;
  
  return kw;
}

// Does a door exist in scan? If yes, get midpoint
bool doorDetection(const RangerProxy & laser, Vector2D & midpoint) {
  // Door found flag
  bool doorFound = false; 
  // Minimum detection threshold between two scans i and i-1
  double minJump = 1; 

  // Flags for door discovered
  bool foundLeftSide = false, foundRightSide = false; 
  
  // Beginning (right) and end (left) of door indexes
  int rightStartIndex, leftStartIndex; 

  // Coordinates of the door sides
  Vector2D rightCoordinates, leftCoordinates; 

  // From the second laser scan to the last, until a door is found
  for(int i = 1; !doorFound && i < laser.GetRangeCount(); i++) {
    // Difference between current scan and the previous
    double diff = laser[i] - laser[i - 1];

    // If the difference is greater than the minimum jump
    if(diff > minJump) {
      foundRightSide = true; // Set right side flag
      rightStartIndex = i - 1; // Set right side index


      // Get the angle that right side finishes at
      double angleRight = laser.GetMinAngle() + rightStartIndex * laser.GetAngularRes();
      // Get coordinates
      double x = laser[rightStartIndex] * cos(angleRight);
      double y = laser[rightStartIndex] * sin(angleRight);

      // Set the coordinates for the right side point
      rightCoordinates.X(x);
      rightCoordinates.Y(y);
    }

    // If the difference is less than negative min threshold
    // && right side has been discovered
    if(diff < -minJump && foundRightSide) {
	  // Set door found flag
      doorFound = true;
	  // Set left side index	  
      leftStartIndex = i; 

      // Get the angle that left side finishes at
      double angleLeft = laser.GetMinAngle() + leftStartIndex * laser.GetAngularRes();
      // Calculate coordinates
      double x = laser[leftStartIndex] * cos(angleLeft);
      double y = laser[leftStartIndex] * sin(angleLeft);

      // Set the coordinates for the left side point
      leftCoordinates.X(x);
      leftCoordinates.Y(y);
    }
  }

  midpoint = (rightCoordinates + leftCoordinates)/2; // Calculate midpoint between right and left sides of door
  return doorFound; // Return door found flag
}

int main(int argn, char *argv[])
{
  PlayerClient robotClient("localhost");

  RangerProxy laser(&robotClient, 1);
  Position2dProxy base(&robotClient, 0);

  // for a clean shutdown
  signal(SIGINT, programShutdown);

  // Robot pose, position and heading.
  Vector2D robot;
  double robotHeading;

  // Wait until we get the pose of the robot from the server.
  do {
    robotClient.Read();
  }
  while (!laser.IsFresh());
  
  // Enable robot motors (else the robot will not move).
  base.SetMotorEnable(true);
  while (programRunning)
    {
      robotClient.Read();
      // TODO: Here goes your solution to the coursework
	  
	  // Midpoint of the found door
      Vector2D midPoint;
      // If a door is detected
      if(doorDetection(laser, midPoint)) {
        // Calculate velocity proportional to the midpoint
        double v = targetLinearVelocityCalc(midPoint, robot);
        // Calculate turning speed proportional to the midpoint
        double w = targetAngularVelocityCalc(midPoint, robot, robotHeading);

        // Apply calculated values
        base.SetSpeed(v, w);
      } else {
        // Wander using collision avoidance
        wander(laser, base);
      }

      
    }
  
  // Disable robot motors.
  base.SetMotorEnable(false);

  return 0;
}

void programShutdown(int s)
{
  programRunning = false;
}
