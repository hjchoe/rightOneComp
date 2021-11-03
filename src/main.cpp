/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]         
// Controller1          controller           
// rightMotor           motor         1           
// leftMotor            motor         2               
// liftMotor            motor         3
// lowArmMotor          motor         4        
// highArmMotor         motor         5
// donutArmMotor        motor         6
// donutPickerMotor     motor         7
// visionSens           vision        20                  
// ---- END VEXCODE CONFIGURED DEVICES ----

// |-------------------- Library Imports --------------------|

#include <stdio.h>
#include "vex.h"
using namespace vex;

// |-------------------- Initialize Global Variables --------------------|

int highCenterX = 158;          // half the number of pixels of the vision sensor
int highBottomY = 175;          // bottom of vision sensor field of view
int lowBottomY = 190;

double speedMultiplierX = 0.35;  // multiplier used when calculating speed based on distance from object on x-axis
double speedMultiplierY = 0.75;  // multiplier used when calculating speed based on distance from object on y-axis

double liftAngle = 70;
int liftSpeed = 50;
int armSpeed = 50;
int donutSpeed = 100;

bool canDrive = true;

int focusobj;

competition Competition;  // A global instance of competition

task driveMotors;
task calibrateliftMotor;
task centerTOgoal;
task runArmMotors;

// |-------------------- Class Definitions --------------------|

class driveTrain
{
  public:
  
  driveTrain()
  {
    canDrive = true;

    refreshMotors();

    Controller1.Axis3.changed(setDirectionVelocity);
    Controller1.Axis4.changed(setDirectionVelocity);
  }

  static void setDirectionVelocity()
  {
    int updown = Controller1.Axis3.position() ^ 3 / 20000;
    int leftright = Controller1.Axis4.position() ^ 3 / 20000;

    int leftVelocity = updown, rightVelocity = updown;

    if (updown == 0)
    {
      if (leftright > 0)
      {
        leftVelocity += abs(leftright);
        rightVelocity -= abs(leftright);
      }
      else if (leftright < 0)
      {
        rightVelocity += abs(leftright);
        leftVelocity -= abs(leftright);
      }
    }
    else if (updown > 0)
    {
      if (leftright > 0)
      {
        leftVelocity += abs(leftright);
      }
      else if (leftright < 0)
      {
        rightVelocity += abs(leftright);
      }
    }
    else if (updown < 0)
    {
      if (leftright > 0)
      {
        leftVelocity -= abs(leftright);
      }
      else if (leftright < 0)
      {
        rightVelocity -= abs(leftright);
      }
    }

    leftMotor.setVelocity(leftVelocity, velocityUnits::pct);
    rightMotor.setVelocity(rightVelocity, velocityUnits::pct);
  }

  void refreshMotors()
  {
    leftMotor.setVelocity(0, velocityUnits::pct);
    rightMotor.setVelocity(0, velocityUnits::pct);
  }
};

class lift
{
  public:

  lift()
  {
    calibrateliftMotor = task(liftRunMotors, vex::task::taskPriorityHigh);

    Controller1.ButtonUp.pressed(liftUp);
    Controller1.ButtonUp.released(liftStop);
    Controller1.ButtonDown.pressed(liftDown);
    Controller1.ButtonUp.released(liftStop);
  }

  static void liftUp()
  {
    liftMotor.setVelocity(liftSpeed, velocityUnits::pct);
  }

  static void liftDown()
  {
    liftMotor.setVelocity(-liftSpeed, velocityUnits::pct);
  }

  static void liftStop()
  {
    liftMotor.setVelocity(0, velocityUnits::pct);
  }

  static int liftRunMotors()
  {
    while (true)
    {
      liftMotor.spin(forward);

      wait(25, msec);
    }
    return 0;
  }
};

class arm
{
  public:

  static bool suckState;
  static bool spitState;

  arm()
  {
    runArmMotors = task(runArm, vex::task::taskPriorityHigh);

    lowArmStop();
    highArmStop();
    donutStop();
    suckState = false;
    spitState = false;

    donutArmMotor.setVelocity(0, velocityUnits::pct);

    Controller1.ButtonR1.pressed(lowArmUp);
    Controller1.ButtonR1.released(lowArmStop);
    Controller1.ButtonR2.pressed(lowArmDown);
    Controller1.ButtonR2.released(lowArmStop);

    Controller1.ButtonL1.pressed(highArmUp);
    Controller1.ButtonL1.released(highArmStop);
    Controller1.ButtonL2.pressed(highArmDown);
    Controller1.ButtonL2.released(highArmStop);

    Controller1.ButtonX.pressed(donutSuck);
    Controller1.ButtonA.pressed(donutSpit);

    Controller1.Axis2.changed(setDonutArmVelocity);
  }

  static void setDonutArmVelocity()
  {
    int velocity = Controller1.Axis2.position() ^ 3 / 20000;

    donutArmMotor.setVelocity(velocity, velocityUnits::pct);
  }

  static void lowArmUp()
  {
    lowArmMotor.setVelocity(armSpeed, velocityUnits::pct);
  }

  static void lowArmStop()
  {
    lowArmMotor.setVelocity(0, velocityUnits::pct);
  }

  static void lowArmDown()
  {
    lowArmMotor.setVelocity(-armSpeed, velocityUnits::pct);
  }

  static void highArmUp()
  {
    highArmMotor.setVelocity(armSpeed, velocityUnits::pct);
  }

  static void highArmStop()
  {
    highArmMotor.setVelocity(0, velocityUnits::pct);
  }

  static void highArmDown()
  {
    highArmMotor.setVelocity(-armSpeed, velocityUnits::pct);
  }

  static void donutSuck()
  {
    if (suckState)
    {
      donutPickerMotor.setVelocity(0, velocityUnits::pct);
      suckState = false;
      spitState = false;
    }
    else if (!suckState)
    {
      donutPickerMotor.setVelocity(donutSpeed, velocityUnits::pct);
      suckState = true;
      spitState = false;
    }    
  }

  static void donutSpit()
  {
    if (spitState)
    {
      donutPickerMotor.setVelocity(0, velocityUnits::pct);
      spitState = false;
      suckState = false;
    }
    else if (!spitState)
    {
      donutPickerMotor.setVelocity(-donutSpeed, velocityUnits::pct);
      spitState = true;
      suckState = false;
    }
  }

  static void donutStop()
  {
    donutPickerMotor.setVelocity(0, velocityUnits::pct);
  }

  static int runArm()
  {
    while (true)
    {
      lowArmMotor.spin(forward);
      highArmMotor.spin(forward);
      donutArmMotor.spin(forward);
      donutPickerMotor.spin(forward);

      wait(25, msec);
    }
    return 0;
  }
};
bool arm::suckState = false;
bool arm::spitState = false;

class centerAssistTool
{
  public:

  centerAssistTool()
  {
    Controller1.ButtonY.pressed(centerStart);
    Controller1.ButtonY.released(centerStop);

    centerTOgoal = task(focus, vex::task::taskPriorityHigh);
    centerTOgoal.suspend();
  }

  static void centerStart()
  {
    driveMotors.suspend();
    centerTOgoal.resume();
  }

  static void centerStop()
  {
    driveMotors.resume();
    centerTOgoal.suspend();
  }

  static void snap(int sig, int* Objects)
  {
    // take snapshot with vision camera and return number of signature objects in frame
    switch (sig)
    {
      case 0:
        *Objects = visionSens.takeSnapshot(REDGOAL);
        break;
      case 1:
        *Objects = visionSens.takeSnapshot(BLUEGOAL);
        break;
      case 2:
        *Objects = visionSens.takeSnapshot(YELLOWGOAL);
        break;
    }
  }

  // find(object): function that returns a boolean of whether the object is in vision
  static void scan(int sig, bool* State) // sig parameter is the object to detect
  {
    int Objects;

    snap(sig, &Objects);

    if (Objects != 0) *State = true;
    else *State = false;
  }

  static void findFocusObj(int* sig)
  {
    int redArea;
    int blueArea;
    int yellowArea;

    visionSens.takeSnapshot(REDGOAL);
    redArea = (visionSens.largestObject.width*visionSens.largestObject.height);

    visionSens.takeSnapshot(BLUEGOAL);
    blueArea = (visionSens.largestObject.width*visionSens.largestObject.height);

    visionSens.takeSnapshot(YELLOWGOAL);
    yellowArea = (visionSens.largestObject.width*visionSens.largestObject.height);

    *sig = 0;
    if (blueArea > redArea)
    {
      *sig = 1;
      if (yellowArea > blueArea)
      {
        *sig = 2;
      }
    }
    else if (yellowArea > redArea)
    {
      *sig = 2;
    }
  }

  // focus(object): function that rotates the robot until the object in vision is centered on the x-axis
  static int focus()
  {
    // initialize variables
    int x;

    double speed = 0;
    bool linedUp = false;
    bool objectSeenState = false;

    // while loop until robot is lined up with object in vision on the x-axis
    while (!linedUp)
    {
      // take snapshot with vision camera and return number of signature objects in frame

      if (focusobj == -1) findFocusObj(&focusobj);
      scan(focusobj, &objectSeenState);

      if (!objectSeenState) break;

      x = visionSens.largestObject.centerX;

      // if the object's x-coordinate is to the left of the center of vision
      if (x > highCenterX + 10)       // 10 is added to the center x value to give a 10 pixel band to the target
      {  
        // calculate speed by multiplying the speedMultiplier to the distance of object from the center on the x-axis
        speed = speedMultiplierX * (x - highCenterX);

        // turn off right motor and set left motor velocity to the calculated speed to turn right
        leftMotor.setVelocity(speed, velocityUnits::pct);
        rightMotor.setVelocity(0, velocityUnits::pct);
      }
      // if the object's x-coordinate is to the right of the center of vision
      else if (x < highCenterX - 10)  // 10 is subtracted to the center x value to give a 10 pixel band to the target
      {      
        // calculate speed by multiplying the x-speedMultiplier to the distance of object from the center on the x-axis
        speed = speedMultiplierX * (highCenterX - x);

        // turn off left motor and set right motor velocity to the calculated speed to turn left
        leftMotor.setVelocity(0, velocityUnits::pct);
        rightMotor.setVelocity(speed, velocityUnits::pct);
      }
      // if object is in vision and centered on the x-axis
      else
      {
        // set motor velocities to 0 (stop motors)
        leftMotor.setVelocity(0, velocityUnits::pct);
        rightMotor.setVelocity(0, velocityUnits::pct);

        // end while loop
        linedUp = true;
        focusobj = -1;  //reset signature
      }
      
      // spin both motors at set velocities
      rightMotor.spin(forward);
      leftMotor.spin(forward);

      // while loop delay (50 milliseconds)
      wait(50, msec);
    }
    return 0;
  }
};

// |-------------------- Function Definitions --------------------|

int drivetrainMotorsCallback()
{
  while (true)
  {
    leftMotor.spin(forward);
    rightMotor.spin(forward);

    //printf("\nleft motor: %f | right motor: %f\n", leftMotor.velocity(velocityUnits::pct), rightMotor.velocity(velocityUnits::pct));

    wait(25, msec);
  }
  return 0;
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
  rightMotor.setBrake(brakeType::brake);
  leftMotor.setBrake(brakeType::brake);
  vexcodeInit();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
  centerAssistTool cat;
  lift l;
  int autoSpeed = 100;

  // setup arm
  lowArmMotor.spinFor(1.4, rotationUnits::rev, autoSpeed, velocityUnits::pct, true);
  highArmMotor.spinFor(-2, rotationUnits::rev, autoSpeed, velocityUnits::pct, false);
  donutArmMotor.spinFor(1.2, rotationUnits::rev, autoSpeed, velocityUnits::pct, true);

  wait(1000, msec);

  // drop two and keep one ring
  donutPickerMotor.spinFor(3, rotationUnits::rev, autoSpeed, velocityUnits::pct, true);

  // bring up arm to drive
  highArmMotor.spinFor(2, rotationUnits::rev, autoSpeed, velocityUnits::pct, false);
  donutArmMotor.spinFor(-1.25, rotationUnits::rev, autoSpeed, velocityUnits::pct, false);

  // drive forward to goal
  leftMotor.spinFor(1, rotationUnits::rev, 50, velocityUnits::pct, false);
  rightMotor.spinFor(1, rotationUnits::rev, 50, velocityUnits::pct, true);

  // focus to goal
  cat.focus();

  // drive forward to goal
  leftMotor.spinFor(1.5, rotationUnits::rev, 100, velocityUnits::pct, false);
  rightMotor.spinFor(1.5, rotationUnits::rev, 100, velocityUnits::pct, true);

  // pickup goal 1
  l.liftUp();

  // drive right into zone
  rightMotor.spinFor(5, rotationUnits::rev, 100, velocityUnits::pct, true);

  // drop goal 1
  l.liftDown();

  // backup away
  leftMotor.spinFor(-2, rotationUnits::rev, 100, velocityUnits::pct, false);
  rightMotor.spinFor(-2, rotationUnits::rev, 100, velocityUnits::pct, true);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void pre_user(void)
{
  rightMotor.setBrake(brakeType::coast);
  leftMotor.setBrake(brakeType::coast);
}

void usercontrol(void)
{
  pre_user();
  driveTrain dt;
  lift l;
  arm a;
  centerAssistTool cat;
  driveMotors = task(drivetrainMotorsCallback, vex::task::taskPriorityHigh);

  while (1)
  {
    wait(20, msec);
  }
}

int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}