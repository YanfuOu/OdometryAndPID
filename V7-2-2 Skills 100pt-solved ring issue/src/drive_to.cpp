#include "vex.h"
#include "turn_to.h"

double test = 0; //does nothing

//#include "main.cpp"
//the program calculates the amount of power needed to move to a given target
// ---------------------------PID Forward distance loop ------------------
double drive_toCalc(double KP, double KI, double KD, bool isForward, double target_distanceX, double target_distanceY, bool extra2, double XGlobal, double yGlobal) {

  //PID
  /* decent tuning values double KP = 0.05; double KI = 0.05; double KD = 0.02;
  */
  //if the front limit switch sensed the goal made contact
        //sensing using back limit switch 
    if(backGrabberLimitS.pressing() || backGrabberLimitS2.pressing())
    {
      backGrabberPneum.set(false);//close
      //backGrabberPneum = true; 
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");

    }
    else
    {
        //keeps the arm down
        //backArm.spin(fwd, 20, pct); 
    }
    
  //sensing using front limit switch 
    if(frontGrabberLimitS12.pressing() )
    {
      frontGrabPneumatic.set(false); //closes
      Brain.Screen.setCursor(7,2); Brain.Screen.setPenColor(blue); Brain.Screen.print("pneu true");
      //return 0; 
    }
    else
    {
      frontArm.spin(fwd, -20, pct); 
    }

  double intergral = 0;
  double derivative = 0;
  double error = 500;
  double previous_error = 0; 
  double power1 = 0;
  double max_error = 7;

  double drive_CurrentDeg; 

  double error_bound = 180;

  double counter = 0; 

  //double target_distance = distance_toCalc(target_distanceX, target_distanceY, false,  XGlobal, yGlobal);
  double delta_X = 0;
  double delta_Y = 0;
  double distance_to_point = 0; 

  //change in x, y
  delta_X = target_distanceX - XGlobal;
  delta_Y = target_distanceY - yGlobal; 

  //printf("drive delta_X %f\n", delta_X);
  //printf("drive delta_Y %f\n", delta_Y);

  //applying the distance formula
  distance_to_point = sqrt( pow(delta_X, 2) + pow(delta_Y, 2)); //only provides the magnitude

  //printf("drive target_distance %f\n", distance_to_point);

  //wheel radius =2"; d[in inches] = r* theata[rads]; d = r * ( theata[deg] * pi / 180); ( 180/ r * pi) * d[inches] = target deg; 28.64 * d[inches] = target_degrees; in inches

  //double target_degrees = distance_to_point * 28.648; 

  //------sets the directions of the vector
  //if the robot is driving backwards(aka back is front)
  if(isForward == false)
  {
    distance_to_point *= -1;
    
  }
  else
  {

    
    //if the distance is ever negative(when it has to go backward); flip the sign of distance_to_point
    if( delta_Y < 0)
    {
      if(target_distanceX < 0) //if delta_X is also negative(when in quad 3), don't flip it
      {
        printf("##########target_distanceX is <0");
      }
      else if(target_distanceX > 0)
      {
        printf("##########target_distanceX is >0");
      }
      else
      {

        printf("^^^^^^^^^^drive_to: driving backwards");
        distance_to_point *= -1; //flipp the sign if negative
      }
    }
  }

  double target_degrees = distance_to_point; 
  double rightDeg = RotationRight.position(degrees);

  //while loop should run until within max_error range
  //since the outside loop already continueouly sends in values and utilizes this to calculate, there is no need for this to be a while loop
  //this program only computes and returns the power amount and does not do anything
  while(fabs(error) > max_error){
    /*rightDeg = RotationRight.position(degrees);
    //calculates current degrees by averaging both wheels
    drive_CurrentDeg = rightDeg; 
    error = target_degrees - drive_CurrentDeg; */
    
    //since target_distance passed in is how much the robot must travel, it basically does what the calculations for error does
    error = target_degrees;

    //printf("drive error %f\n", error);

    //when within the error bound, let KI grow to give a final push
    if( fabs(error) < error_bound){ //caution
      intergral += error;
    }
    //reset intergral if it turns negative
    if(error*previous_error < 0){
      intergral = 0;
    }
    //calculates the derivative(for self correction)
    derivative = previous_error - error; 

    previous_error = error; 

    //calculates power
    power1 = error*KP + intergral*KI + derivative*KD ; 
    //Brain.Screen.setCursor(10, 3); Brain.Screen.print("drive power:");
    //Brain.Screen.setCursor(10, 20); Brain.Screen.print(power1);

    printf("drive power1 %f\n", power1);
    printf("drive max_error: %f\n", max_error);
    printf("drive error: %f\n", error);  
    //return 0 if within range
    if(fabs(error) < max_error)
    {
      Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(yellow); Brain.Screen.print("out of PID");
      printf("------------------------driving done!!------------------- ");
      return 0;
    }
 


     //since this program only does calculations, no need for movement
    //gets it out of loop if stuck CAUTION: SAFETY MECHANISM
    if(counter > 10000)
    {
      error  = max_error; 
      return power1 = 0; 
      printf("------------------------driving done!!------------------- ");
    }
    /*if(delta_X < 0.15*target_distanceX )
    {
      if(delta_Y < 0.15*target_distanceY)
      {
        Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(green); Brain.Screen.print("out of PID");
        printf("------------------------turning done!! into exit loop-------------------\n ");
        return 0;
      }
    } */
    return power1; 

    counter ++; 

    //CAUTION
    //drive_CurrentDeg = 0;
  };

  //once outside the loop, stop and hold the motor
  leftDrive1.stop(hold);
  leftDrive2.stop(hold);
  leftDrive3.stop(hold);
  rightDrive1.stop(hold);
  rightDrive2.stop(hold); 
  rightDrive3.stop(hold); 
  Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(blue); Brain.Screen.print("out of PID");

  return 0; //return 0 to trip stopping mechanism

  //resets everything
  error = 0; previous_error = 0; intergral = 0; derivative = 0; power1 = 0; target_degrees = 0; target_degrees = 0; 
  
  
};