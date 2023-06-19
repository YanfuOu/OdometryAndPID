#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftDrive1 = motor(PORT1, ratio18_1, true);
motor leftDrive2 = motor(PORT2, ratio18_1, false);
motor rightDrive1 = motor(PORT3, ratio18_1, true);
motor rightDrive2 = motor(PORT5, ratio18_1, false);
digital_out frontGrabPneumatic = digital_out(Brain.ThreeWirePort.A);
rotation RotationLeft = rotation(PORT20, true);
rotation RotationRight = rotation(PORT19, false);
motor Conveyer = motor(PORT7, ratio36_1, false);
motor rightDrive3 = motor(PORT16, ratio18_1, false);
motor leftDrive3 = motor(PORT14, ratio18_1, true);
digital_out backGrabberPneum = digital_out(Brain.ThreeWirePort.B);
motor frontArm = motor(PORT9, ratio18_1, false);
rotation RotationBack = rotation(PORT11, true);
inertial InertialSensor = inertial(PORT17);
limit backGrabberLimitS = limit(Brain.ThreeWirePort.C);
limit frontGrabberLimitS12 = limit(Brain.ThreeWirePort.D);
limit backGrabberLimitS2 = limit(Brain.ThreeWirePort.E);
limit frontGrabberLimitS2 = limit(Brain.ThreeWirePort.F);
rotation armRotation = rotation(PORT15, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}