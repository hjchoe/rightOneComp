using namespace vex;

extern brain Brain;
extern controller Controller1;

using signature = vision::signature;

// VEXcode devices
extern signature REDGOAL;
extern signature BLUEGOAL;
extern signature YELLOWGOAL;

extern vision visionSens;

extern motor rightMotor;
extern motor leftMotor;
extern motor liftMotor;
extern motor lowArmMotor;
extern motor highArmMotor;
extern motor donutArmMotor;
extern motor donutPickerMotor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );