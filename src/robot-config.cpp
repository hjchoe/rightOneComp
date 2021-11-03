#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// VEXcode device constructors

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller(primary);

/*vex-signature-config:begin*/
signature REDGOAL = signature(3, 6729, 9593, 8161, -1795, -885, -1340, 4.4, 0);
signature BLUEGOAL = signature(4, -3145, -1333, -2239, 5461, 11189, 8325, 2.2, 0);
signature YELLOWGOAL = signature(5, 149, 2571, 1360, -4359, -3459, -3909, 2.4, 0);
/*vex-signature-config:end*/

/*vex-vision-config:begin*/
vision visionSens = vision(PORT20, 50, BLUEGOAL, REDGOAL, YELLOWGOAL);
/*vex-vision-config:end*/

/*vex-motor-config:begin*/
motor rightMotor = motor(PORT1, ratio18_1, true);
motor leftMotor = motor(PORT2, ratio18_1, false);
motor liftMotor = motor(PORT3, ratio36_1, false);
motor lowArmMotor = motor(PORT4, ratio36_1, false);
motor highArmMotor = motor(PORT5, ratio36_1, true);
motor donutArmMotor = motor(PORT6, ratio36_1, false);
motor donutPickerMotor = motor(PORT7, ratio36_1, true);
/*vex-motor-config:end*/

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void )
{
  liftMotor.setBrake(brakeType::hold);
  lowArmMotor.setBrake(brakeType::hold);
  highArmMotor.setBrake(brakeType::hold);
  donutArmMotor.setBrake(brakeType::hold);
  donutPickerMotor.setBrake(brakeType::hold);
}