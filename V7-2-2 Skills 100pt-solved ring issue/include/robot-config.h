using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftDrive1;
extern motor leftDrive2;
extern motor rightDrive1;
extern motor rightDrive2;
extern digital_out frontGrabPneumatic;
extern rotation RotationLeft;
extern rotation RotationRight;
extern motor Conveyer;
extern motor rightDrive3;
extern motor leftDrive3;
extern digital_out backGrabberPneum;
extern motor frontArm;
extern rotation RotationBack;
extern inertial InertialSensor;
extern limit backGrabberLimitS;
extern limit frontGrabberLimitS12;
extern limit backGrabberLimitS2;
extern limit frontGrabberLimitS2;
extern rotation armRotation;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );