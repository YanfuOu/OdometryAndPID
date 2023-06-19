#include "vex.h" 
void calibration_and_testing()
{
  double rightDeg; double leftDeg; double backDeg; 
  //Resets the position of the absolute encoder to the value of zero.
    RotationRight.resetPosition();RotationLeft.resetPosition();RotationBack.resetPosition(); 
  while(true)
  {
    //sets the values and prints it out
    rightDeg = RotationRight.position(degrees);leftDeg = RotationLeft.position(degrees);backDeg = RotationBack.position(degrees);
    Brain.Screen.setCursor(12, 27); Brain.Screen.print("rightDeg");Brain.Screen.setCursor(12, 36); Brain.Screen.print(rightDeg ); 
    Brain.Screen.setCursor(11, 27); Brain.Screen.print("leftDeg");Brain.Screen.setCursor(11, 36); Brain.Screen.print(leftDeg ); 
    Brain.Screen.setCursor(10, 27); Brain.Screen.print("backDeg");Brain.Screen.setCursor(10, 36); Brain.Screen.print(backDeg ); 
  }

}