////////////////////////////////////////////////
// Odometry Source File
////////////////////////////////////////////////
// Purpose:
//     This file containts the implementaion
// to run our the odometry system to provide
// others systems with the x,y locations
// and orientatiosn of the robot at a giving time.
//
// Authors:
//     Hunter Guidry, 
//     Michael Boudreaux,
//     Richard Truong,
//     Jose
//     ..
//
// Date Created: 12/24/2022
// Date Modified: 12/27/2022
//

#include "odometry.h"

std::ofstream odometryValues("odometryTest.csv", std::ofstream::out);

// The Odometry class constructor
Odometry::Odometry()
{
  std::cout << "[Odometry] Initializing..." <<std::endl;
  x_global = 0;
  y_global = 0;
  globalTheta = 0;
  //prevLeftEncoder = RotationY.position(degrees);
  //prevMiddleEncoder = RotationX.position(degrees);
  
}

// Update Odometry calculations by perform
// a single round of the odometry algorithm
void Odometry::performCalculations()
{
  //////////////////////////////////
  // Put Odometry Algorithm here
  //////////////////////////////////
  // Initalize encoder values

 

  if(ROBOT_ID == LEARN_BOT_ID)
  {
    curLeftEncoder = EncoderLeft.position(degrees);
    curMiddleEncoder = EncoderMiddle.position(degrees);
  }
  else
  {
    curLeftEncoder = RotationY.position(degrees)*-1; //- y_offset;
    curMiddleEncoder = RotationX.position(degrees); //- x_offset;
  }

  curLeftEncoder = (curLeftEncoder / 360) * (M_PI * 2.75);
  curMiddleEncoder = (curMiddleEncoder / 360) * (M_PI * 2.75);

  // Find the difference between the current and previous encoder values
  differenceLeft = curLeftEncoder - prevLeftEncoder;
  differenceMiddle = curMiddleEncoder - prevMiddleEncoder;
  

  // dist traveled converted to inches
  //deltaLeft = ((differenceLeft * CONVERT_TO_RADIANS) / CONVERT_TO_REVOLUTIONS) * diameter * M_PI;
  //deltaCenter = ((differenceMiddle * CONVERT_TO_RADIANS) / CONVERT_TO_REVOLUTIONS) * diameter * M_PI;

  // calculate total distance traveled
  //deltaTotalLeft += deltaLeft;

  // Calculate new orientation
  curOrientation = (inertiall.rotation() * M_PI) / 180.0;

  // Calculate change in orientation
  deltaTheta =  curOrientation - prevAbsOrientation;

  // Calculating offset. Conditional to prevent nan values.
  local_x = (deltaTheta + (differenceMiddle / centerDistance)) * centerDistance;
  local_y = differenceLeft;
  /*
  if(deltaTheta == 0)
  {
    local_x = deltaCenter;
    local_y = deltaLeft;
  }
  else
  {
    OFFSET_FORMULA = (2*sin(deltaTheta/2));
    local_x = OFFSET_FORMULA*((deltaCenter/deltaTheta) + centerDistance);
    local_y = OFFSET_FORMULA*((deltaLeft/deltaTheta) + leftDistance);
  }

  // Calculating average orientation
  averageOrientation = prevAbsOrientation - (deltaTheta/2);

  // calculate global offset
  // convert first to polar coords
  localPolarPosition = sqrt((pow(local_x,2.0)+(pow(local_y,2.0))));




  if(local_x == 0 || local_y == 0)
  {
    globalTheta = 0;
  }
  else
  {
    globalTheta = atan2(local_y,0); 
  }


  // Add the average orientation to globalOrientation
  globalTheta = globalTheta + averageOrientation;

  // Convert back to rectangular
  x_offset = localPolarPosition * cos(globalTheta);
  y_offset = localPolarPosition * sin(globalTheta);

  x_global += x_offset;
  y_global += y_offset;
  */
  x_global += (local_y * sin(prevAbsOrientation + deltaTheta/2)) + (local_x * cos(prevAbsOrientation + deltaTheta/2)); 
  y_global += (local_y * cos(prevAbsOrientation + deltaTheta/2)) - (local_x* sin(prevAbsOrientation + deltaTheta/2));
  globalTheta = curOrientation;

  
  

  // Update prev values
  prevLeftEncoder = curLeftEncoder;
  prevMiddleEncoder = curMiddleEncoder;
  prevAbsOrientation = curOrientation;
  prevX = x_global;
  prevY = y_global;
}


// Continusly run odometry in the background
// to get location and orientation while others robot
// systems continue
void Odometry::runOdometry()
{
  task::sleep(1);
  while(isOdometryEnabled)
  {
    this->performCalculations();

    // Print values if odometry debug is enabled
    if(isDebugEnabled)
    {
      this->printValuesTerminal();
      this->printValuesScreen();
      this->printValuesController();
    }
    task::sleep(10); // Run the task at 50 Hz
  }
}

// This method will run the odometry
// Must be start as a task to run properly
// alongside other 
void Odometry::start()
{
  isOdometryEnabled = true;
  std::cout << "[Odometry] Starting..." << std::endl;
  this->runOdometry();
}

// This method will flag the odometry task
// to stop running
void Odometry::stop()
{
  isOdometryEnabled = false;
  std::cout << "[Odometry] Stopping..." <<std::endl;
}

void Odometry::enableDebug()
{
  isDebugEnabled = true;
}

void Odometry::disableDebug()
{
  isDebugEnabled = false;
}

double Odometry::getX()
{
  return x_global;
}

double Odometry::getY()
{
  return y_global;
}

double Odometry::getOrientation()
{
  return curOrientation;
}

void Odometry::printValuesTerminal()
{
  std::cout <<
  "[Odometry] "
  "X: " << getX() << "    " <<
  "Y: " << getY() << "    " <<
  "O: " << getOrientation() << "    " <<
  std::endl;
}

void Odometry::printValuesScreen()
{
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.clearLine(1, black);
  Brain.Screen.print("Odometry Debug");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.clearLine(2, black);
  Brain.Screen.print("X: %f", getX());
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.clearLine(3, black);
  Brain.Screen.print("Y: %f", getY());
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.clearLine(4, black);
  Brain.Screen.print("Theta: %f", getOrientation());
}

void Odometry::printValuesController()
{
  Controller1.Screen.clearLine(1);
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("X: %f", getX());
  Controller1.Screen.clearLine(2);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Y: %f", getY());
  Controller1.Screen.clearLine(1);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Theta: %f", getOrientation());
}

int Odometry::saveToSdOdometry()
{
  std::cout << "[Odometry] SD Card: Saved values" << std::endl;
  odometryValues << getX() << ",";
  odometryValues << getY() << ",";
  odometryValues << getOrientation() << ",";
  odometryValues << std::endl;
  odometryValues.flush();
  wait(100,msec);
  return 0;
}

int Odometry::closeSD()
{
  odometryValues.close();
  std::cout << "[Odometry] SD Card: Closed" << std::endl;
  return 0;
}

void Odometry::setMeasurements(double wheelDiameter_, double centerDistance_, double leftDistance_, double rightDistance_)
{
  this->diameter = wheelDiameter_;
  this->centerDistance = centerDistance_;
  this->leftDistance = leftDistance_;
  this->rightDistance = rightDistance_;
}

void Odometry::reset()
{
  
}