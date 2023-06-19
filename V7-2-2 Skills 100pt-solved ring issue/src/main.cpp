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
// leftDrive1           motor         1               
// leftDrive2           motor         2               
// rightDrive1          motor         3               
// rightDrive2          motor         5               
// frontGrabPneumatic   digital_out   A               
// RotationLeft         rotation      20              
// RotationRight        rotation      19              
// Conveyer             motor         7               
// rightDrive3          motor         16              
// leftDrive3           motor         14              
// backGrabberPneum     digital_out   B               
// frontArm             motor         9               
// RotationBack         rotation      11              
// InertialSensor       inertial      17              
// backGrabberLimitS    limit         C               
// frontGrabberLimitS12 limit         D               
// backGrabberLimitS2   limit         E               
// frontGrabberLimitS2  limit         F               
// armRotation          rotation      15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "calibration.h"
#include "turn_to.h"
#include "drive_to.h"
#include "block1_turn_to.h"
#include "Vision1.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

//---conveyer-----------------
int IsIntake; 

int zuck()
{
  while(true)
  {
    
    while(IsIntake == 1)
    {
      Conveyer.spin(directionType::fwd, 10, voltageUnits::volt);
      //intakeL.spin(directionType::fwd, 10, voltageUnits::volt);
      //upperRoller.spin(directionType::fwd, 5, voltageUnits::volt);
      
      
    }
    while(IsIntake == -1)
    {
      Conveyer.spin(directionType::fwd, -10, voltageUnits::volt);
      //intakeL.spin(directionType::fwd, -10, voltageUnits::volt);
      //upperRoller.spin(directionType::fwd, -5, voltageUnits::volt);
    }
    while(IsIntake == 0)
    {
      Conveyer.stop(brakeType::brake);
      //intakeL.stop(brakeType::brake);
    }
    
  }
  return 1;
}


bool isDriving = false; 
bool isVisionLock = false;
double goalType = 1.0; //goalType = 1 is blue; 2 is yellow; 3 is red // default is blue

double sweepPower = 0; 
int sweepCounter = 0; 

int VisionDrivingTo()
{

    double xScale = 480.0/310;//Scaling the vision sensor range to the V5 Brain Screen
  double yScale = 240.0/212; 
  int counter = 0;

  double x = 0;
  double y = 0; 
  double width;
  double height;
  int center = 158;// The x coordinate for the center of the vision sensor
  int OKError = 100; //Used to set a range of values to count is being just in 
  double area = 0; 

  double deltaError; 
  double p_constant = 0.25; //play around with corrections constants
  double drive_power; 

  
  while(true)
  {
    printf("isVisionLock: %d", isVisionLock);
    while(isVisionLock == true)
    {

      //Vision1.takeSnapshot(BLUEBLOCK); //default is blue goal

      if(goalType == 1.0)
      {
        Vision1.takeSnapshot(BLUEBLOCK);
        printf("-----------BLUE GOAL-----------");
      }
      if(goalType == 2.0)
      {
        Vision1.takeSnapshot(YELLOWBLOCK2);
        printf("-----------YELLOW GOAL-----------");
      }
      if(goalType == 3.0)
      {
        Vision1.takeSnapshot(REDBLOCK);
        printf("-----------RED GOAL-----------");
      }      
      
      //if (Vision1.largestObject.exists)
      //{
      /*Brain.Screen.print("Vision Sensor: x: %d", Vision1.largestObject.originX);
      Brain.Screen.print(" Y: %d", Vision1.largestObject.originY);
      Brain.Screen.print(" W %d", Vision1.largestObject.width);
      Brain.Screen.print(" H: %d", Vision1.largestObject.height); */
      printf("goalType %f\n ", goalType); 
      x = Vision1.largestObject.centerX;
      y = Vision1.largestObject.originY;
      width = Vision1.largestObject.width;
      height = Vision1.largestObject.height; 
      //Brain.Screen.print(" x %d y%d Width %d Height %d ",x, y, width, height);
      Brain.Screen.setFillColor(color::blue);
      Brain.Screen.drawRectangle( x*xScale , y*yScale , width*xScale , height*yScale );
      vex::task::sleep(100);

      area = x*xScale + y*yScale + width*xScale + height*yScale; 
      printf("x %f ", x); 
      printf("y %f \n", y); 
      printf("width %f ", width); 
      printf("height %f \n", height); 
      printf("area %f ", area); 
      printf("\n -----------\n");

      Brain.Screen.setCursor(2,1); Brain.Screen.print(" area %d" , area);;

      deltaError = center -x; 


      drive_power = deltaError*p_constant;
      printf("deltaerror %f\n", deltaError);
      printf("drive_Power %f\n", drive_power);

      if(backGrabberLimitS.pressing() || backGrabberLimitS2.pressing())
      {
        backGrabberPneum.set(true);
        Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
        printf("------------backGrabber triggered ------- ");
        isVisionLock = false;

      }
      //if the front limit switch sensed the goal made contact
      if(frontGrabberLimitS12.pressing())
      {
        frontGrabPneumatic.set(false);
        Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
        printf("------------front triggered ------- ");
        isVisionLock = false;

    }
      if(x< (center- OKError) ) //If the object is to the left of center
      {

        rightDrive1.spin(directionType::fwd, 20, velocityUnits::pct);
        rightDrive2.spin(directionType::fwd, 20, velocityUnits::pct);
        rightDrive3.spin(directionType::fwd, 20, velocityUnits::pct);
        leftDrive1.spin(directionType::rev, 20, velocityUnits::pct);
        leftDrive2.spin(directionType::rev, 20, velocityUnits::pct);
        leftDrive3.spin(directionType::rev, 20, velocityUnits::pct);
        printf("turning left \n");
        
        //----------------------side sweeping-----------------
        if(x == 0) //when nothing is detected, sweep 
        {
          sweepCounter += 1.0; //increasing the sweeping counter

          sweepPower = cos(sweepCounter /3.0 + 0.95) *40 ; 
          printf("sweepCounter %d\n", sweepCounter);
          printf("sweepPower %f\n", sweepPower);

          rightDrive1.spin(directionType::fwd, sweepPower, velocityUnits::pct);
          rightDrive2.spin(directionType::fwd, sweepPower, velocityUnits::pct);
          rightDrive3.spin(directionType::fwd, sweepPower, velocityUnits::pct);
          leftDrive1.spin(directionType::rev, sweepPower, velocityUnits::pct);
          leftDrive2.spin(directionType::rev, sweepPower, velocityUnits::pct);
          leftDrive3.spin(directionType::rev, sweepPower, velocityUnits::pct);

        }

      } 
      else if (x> (center + OKError) ) //If the object is to the right of center
      {
        rightDrive1.spin(directionType::rev, 20, velocityUnits::pct);
        rightDrive2.spin(directionType::rev, 20, velocityUnits::pct);
        rightDrive3.spin(directionType::rev, 20, velocityUnits::pct);
        leftDrive1.spin(directionType::fwd, 20, velocityUnits::pct);
        leftDrive2.spin(directionType::fwd, 20, velocityUnits::pct);
        leftDrive3.spin(directionType::fwd, 20, velocityUnits::pct);

        printf("turning right \n"); 
      } 
      else //The object is not to the right of center and not to the left of center
      {
        leftDrive1.stop(brakeType::hold);
        leftDrive2.stop(brakeType::hold);
        leftDrive3.stop(brakeType::hold);
        rightDrive1.stop(brakeType::hold);
        rightDrive2.stop(brakeType::hold);
        rightDrive3.stop(brakeType::hold);

        printf("facing the goal \n");

        if(area < 5033812879) //play around with the area
        {
          if(area < 1633812879)
          {
            printf("driving toward the goal DEAD STRAIGHT \n");
            rightDrive1.spin(directionType::fwd, 70, velocityUnits::pct);
            rightDrive2.spin(directionType::fwd, 70, velocityUnits::pct);
            rightDrive3.spin(directionType::fwd, 70, velocityUnits::pct);
            leftDrive1.spin(directionType::fwd, 70, velocityUnits::pct);
            leftDrive2.spin(directionType::fwd, 70, velocityUnits::pct);
            leftDrive3.spin(directionType::fwd, 70, velocityUnits::pct);
          }

          else
          {
            printf("driving toward the goal W CORRECTIONS \n");
            rightDrive1.spin(directionType::rev, 25 - drive_power, velocityUnits::pct); //play around with the base drive power
            rightDrive2.spin(directionType::rev, 25 - drive_power, velocityUnits::pct);
            rightDrive3.spin(directionType::rev, 25 - drive_power, velocityUnits::pct);
            leftDrive1.spin(directionType::rev, 25 + drive_power, velocityUnits::pct);
            leftDrive2.spin(directionType::rev, 25 + drive_power, velocityUnits::pct);
            leftDrive3.spin(directionType::rev, 25 + drive_power, velocityUnits::pct);

          }
          
        }
      } 

      Brain.Screen.clearScreen(); 

      //}
      /*else
      {
      Brain.Screen.print("Vision Sensor: Color Signature Not Found!");
      }*/
      //task::sleep(20);
    }

    while(isVisionLock == false)
    {
      //printf("isDriving: %d", isDriving);
      if(isDriving == false )
      {
        leftDrive1.stop(brakeType::coast);
      leftDrive2.stop(brakeType::coast);
      leftDrive3.stop(brakeType::coast);
      rightDrive1.stop(brakeType::coast);
      rightDrive2.stop(brakeType::coast);
      rightDrive3.stop(brakeType::coast);
      }
      vex::task::sleep(20);
      
    }
  }

  return 1; 
}

void callibrateInertialSensor()
{
  InertialSensor.calibrate();
  while(InertialSensor.isCalibrating()){
    Brain.Screen.setCursor(2, 3); Brain.Screen.print("calibrating..."); task::sleep(200);
  };
  RotationLeft.resetPosition();
  RotationRight.resetPosition();
  armRotation.resetPosition();
  Brain.Screen.setCursor(2, 3); Brain.Screen.print("done calabriting!");
  
  vexcodeInit();

}

//----------------------odometry variable---------------------
//----------------------odometry variable---------------------
//double leftDeg;  //degs left wheel rotated  
  double rightDeg; //degs right wheel rotated  
  double backDeg; //degs back wheel rotated  

  double prevLeftDeg = 0; //previous left encoder degs; it starts with 0
  double prevRightDeg = 0; 
  double prevBackDeg = 0;

  double deltaLeftDeg; // change in encoders' values since last cycle
  double deltaRightDeg;
  double deltaBackDeg;

  double deltaL; //distance wheel traveled per cycle
  double deltaR;
  double deltaS;

  double lastLeftDeg = 0; //left and right encoder values since the last reset; assume it's was 0
  double lastRightDeg = 0; 

  double deltaLr; //the total change in the left and right encoder values since the last reset, and convert to distance of wheel travel;
  double deltaRr; 

  double lastTheta; //global orientation at last rest
  double thetaOne; //change in absolute orientation
  double thetaNot = 0; //previous theta
  double deltaTheta; //change in angle 
  double thetaM; //average orientation
  double starting_thetaM; //useless

  double inertialDeg;//used to find deltaTheta
  double inertialDegPrev = 0;

  double localX; //local x coordinate
  double localY;
  double prevLocalX; //previous local X coordinate
  double prevLocalY;
  double deltaXLocal; //change in local x coordinate from previous
  double deltaYLocal;
  double deltaXGlobal; //change in global x coordinate
  double deltaYGlobal;
  double lastXGlobal; //previous global x coordinate
  double lastYGlobal;
  double XGlobal; //current global x coordinate
  double yGlobal;

  //---declaration of constants---
  double wheelRadius = 1.375;

  double Sleft = 4.5;//now: 9 across; 5 to each //was all 7 //(inches) left distance from tracking center to Left Wheel
  double Sright = 4.75; //span of 2 wheels: 14"
  double Sback = 0.95; //increases this, increases the xGlobal value //was 2.5 //was 7.5" //13" to 17.5"; distance: 4.5" //backwards distance from tracking center

    double fwdAmount;
    double turnAmount;
    double strafAmount; 

    double fwdPower;
    double turnPower; 
    double strafPower;


bool IsOdom = false; 

//----------------------odometry task-------------------------
int odom()
{
      //Resets the position of the absolute encoder to the value of zero.
    RotationRight.resetPosition();
    RotationLeft.resetPosition();
    RotationBack.resetPosition(); 
    //CAUTION: remember to calibrate all sensors in pre-auton!

  while(IsOdom == true)
{

      //step 1: stores current encoder values in local varibles
      //leftDeg = RotationLeft.position(degrees);
      rightDeg = RotationRight.position(degrees);
     // leftDeg = RotationLeft.position(degrees);
      backDeg = RotationBack.position(degrees);

      inertialDeg = InertialSensor.rotation(rotationUnits::deg);

      //convert inertialDeg to rads
      inertialDeg *= M_PI / 180; 
      Brain.Screen.setCursor(3, 20); Brain.Screen.print("inertialDeg");
      Brain.Screen.setCursor(3, 37); Brain.Screen.print(inertialDeg );


      //----------------this whole block is used to calculate deltaTheta-----------------
      
      //step 2: calculates the change in encoders's value
      //deltaLeftDeg = leftDeg - prevLeftDeg; // deltaL
      deltaRightDeg = rightDeg - prevRightDeg; //deltaR
      deltaBackDeg = backDeg - prevBackDeg; //deltaS
        //convert the chnage to distance wheel traveled
        deltaL = deltaLeftDeg * M_PI/ 180 * wheelRadius; 
        deltaR = deltaRightDeg * M_PI/ 180 * wheelRadius; 
        deltaS = deltaBackDeg * M_PI/ 180 * wheelRadius; 

      //step 3: update previous values of encoders
      //prevLeftDeg = leftDeg;
      prevRightDeg = rightDeg;
      prevBackDeg = backDeg;

      /*
      //step 4: Calculate the total change in the left and right encoder values 
        //since the last reset, and convert to distance of wheel travel; 
        //call these Δ𝐿𝑟 and Δ𝑅𝑟
      deltaLr = (leftDeg - lastLeftDeg) * M_PI/ 180 * wheelRadius;
      deltaRr = (rightDeg - lastRightDeg) * M_PI/ 180 * wheelRadius;

            //updates thetaNot to find the difference to get deltaTheta
      thetaNot = thetaOne; 

      //step 5: Calculate new absolute orientation 𝜃1 = 𝜃𝑟+ (Δ𝐿𝑟−Δ𝑅𝑟)/(𝑠𝐿+𝑠𝑅); 
        //please note that the second term will be in radians,
      thetaOne = (deltaLr - deltaRr) / ( Sleft + Sright);
      */


      //step 6: calculate the change in angle; also heading: current - prev
      //new way of calculating deltaTheta: 
      deltaTheta = inertialDeg - inertialDegPrev; 
      //deltaTheta = thetaOne - thetaNot; //old way of calculating deltaTheta
      /*printf("thetaOne: %f\n", thetaOne );
      printf("thetaNot: %f\n", thetaNot );
      printf("deltaTheta: %f\n", deltaTheta ); */

      //after the calculation, updates inertialDegPrev
      //CAUTION: careful of when this should be updated
      inertialDegPrev = inertialDeg; 


      //printf("thetaNot: %f\n", thetaNot );
      //step 7: If Δ𝜃=0 (i.e. Δ𝐿=Δ𝑅), then calculate the local offset Δ𝑑𝑙⃗⃗⃗ =[Δ𝑆Δ𝑅]

      //CAUTION(REVIEW)
      //deltaL == deltaR
      if(  deltaTheta == 0.0f) //when deltaTheta is 0(to avoid dividing by 0); basically when it's going forward //was deltaL == deltaR
      {
        localX = deltaS;
        localY = deltaR;
        //localY = (deltaR + deltaL)/2.0f;

      }

      //step 8: Otherwise, calculate the local offset Δ𝑑𝑙⃗⃗⃗ (equation 6)
      else
      {
        localX = 2 * sin( deltaTheta/2 ) * ( (deltaS / deltaTheta) + Sback ); 
        localY = 2* sin( deltaTheta /2 ) * ( (deltaR / deltaTheta) + Sright );
      }
      /*printf("localX: %f\n", localX );
      printf("localY: %f\n", localY );
      printf("----------------\n"); 
      printf("deltaTheta: %f\n", deltaTheta); */
      //printf("Sleft + Sright: %f\n", Sleft + Sright);

      /*Brain.Screen.setCursor(1, 3); Brain.Screen.print("localX");
      Brain.Screen.setCursor(1, 20); Brain.Screen.print(localX );

      Brain.Screen.setCursor(2, 3); Brain.Screen.print("localY");
      Brain.Screen.setCursor(2, 20); Brain.Screen.print(localY ); */

      //step 9: Calculate the average orientation 𝜃𝑚=𝜃0+Δ𝜃2 (in radians)
      //NEW: thetaM is basically inertial sensor reading
      thetaM = inertialDeg; 
      //thetaM = average angle of the robot during it's arc
      //warning: not sure + or -
      //thetaM = thetaNot + deltaTheta /2;
      //thetaM = -thetaM; 

      //thetaM *= 57.29;
      Brain.Screen.setCursor(2, 30); Brain.Screen.print("thetaM");
      Brain.Screen.setCursor(2, 37); Brain.Screen.print(thetaM );

      //step 10: calculate global offset Δ𝑑 as Δ𝑑𝑙⃗⃗⃗ rotated by −𝜃𝑚;
      //this can be done by converting your existing Cartesian coordinates to polar coordinates, changing the angle, then converting back
      deltaXGlobal = (localY * sin(thetaM)) + (localX * cos(thetaM));//look more carefully 
      deltaYGlobal = (localY * cos(thetaM)) - (localX * sin(thetaM));
      //--------------------
      /*Brain.Screen.setCursor(3, 3); Brain.Screen.print("deltaXGlobal");
      Brain.Screen.setCursor(3, 20); Brain.Screen.print(deltaXGlobal );
      Brain.Screen.setCursor(4, 3); Brain.Screen.print("deltaYGlobal");
      Brain.Screen.setCursor(4, 20); Brain.Screen.print(deltaYGlobal );*/
      //printing things out!!!
      /*printf("deltaXGlobal: %f\n", deltaXGlobal);printf("deltaYGlobal: %f\n", deltaYGlobal);
      //printf("localX: %f\n", localX);printf("localY: %f\n", localY);
      printf("lastXGlobal: %f\n", lastXGlobal);printf("lastYGlobal: %f\n", lastYGlobal);
      printf("XGlobal: %f\n", XGlobal);printf("YGlobal: %f\n", yGlobal);
      printf("thetaM: %f\n", thetaM); */
      
      //printf("\n"); 

      bool isProblemX = isnan(deltaXGlobal);
      bool isProblemY = isnan(yGlobal);

      if(isProblemX)
      {
        printf("-----------------------------warning deltaXGlobal is the problem-------------------------------");
      }
      if(isProblemY)
      {
        printf("-----------------------------warning y is the problem-------------------------------");
      }


      //step 11: calculate new abosulte position 𝑑1⃗⃗⃗⃗ =𝑑0⃗⃗⃗⃗ +Δ𝑑
      XGlobal = lastXGlobal + deltaXGlobal;
      yGlobal = lastYGlobal + deltaYGlobal; 


      Brain.Screen.setCursor(1, 20); Brain.Screen.print("(");
      Brain.Screen.setCursor(1, 23); Brain.Screen.print(XGlobal );
      //Brain.Screen.setCursor(1, 27); Brain.Screen.print("counter" );Brain.Screen.setCursor(1, 36); Brain.Screen.print(counterOdom );

      Brain.Screen.setCursor(1, 33); Brain.Screen.print(",");
      Brain.Screen.setCursor(1, 35); Brain.Screen.print(yGlobal ); 


      //updates lastXGlobal and lastYGlobal
      lastXGlobal = XGlobal;
      lastYGlobal = yGlobal; 

      //CAUTION: flips the sign of thetaM after all calculations are done, needs this to adopt to new movement program
      //thetaM *= -1; //no need for adaptation now that thetaM is not negativized above
      
      //counterOdom++;
      //update frequency of 20 miliseconds
      task::sleep(20);
  
  
  } //always run task while true loop

// the if statement
  Brain.Screen.setCursor(9, 3); Brain.Screen.setFillColor(red); Brain.Screen.print("out of odom loop");


  return 1;
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



//--------------------------PID for arms------
// ---------------------------PID Forward distance loop ------------------
void front_arm_up_loop(double KP, double KI, double KD, double target_distance, bool extra, double isFrontArm, int isBackArm) {
  //if it is activating front arm, isFrontArm enter 1;
  //if it is the back arm, isBackArm enter 1
  //PID
  /* decent tuning values double KP = 0.05; double KI = 0.05; double KD = 0.02;
  */
  //RotationRight.resetPosition();RotationLeft.resetPosition();RotationBack.resetPosition(); 

  double intergral = 0;
  double derivative = 0;
  double error = 500;
  double previous_error = 0; 
  double power1 = 0;
  double max_error = 5;

  double drive_CurrentDeg; 

  double error_bound = 180;

  double counter = 0; 

  //wheel radius =2"; d[in inches] = r* theata[rads]; d = r * ( theata[deg] * pi / 180); ( 180/ r * pi) * d[inches] = target deg; 28.64 * d[inches] = target_degrees; in inches

  //double target_degrees; //= target_distance * 28.648; 


  //while loop should run until within max_error range
  while(fabs(error) > max_error){

    
    //calculates current degrees by averaging both wheels
    if(isFrontArm == 1)
    {
      
      drive_CurrentDeg = armRotation.position(degrees);
    }
    
    if(isBackArm == 1)
    {
      //CAUTION! This code is commented out
      //drive_CurrentDeg = backArm.rotation(rotationUnits::deg);
    }
    printf("target_degrees %f\n", target_distance);
    printf("arm_currentDeg %f\n", drive_CurrentDeg);

    error = target_distance - drive_CurrentDeg; 

    //when within the error bound, let KI grow to give a final push
    if( fabs(error) < error_bound){
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
    Brain.Screen.setCursor(10, 3); Brain.Screen.print("Current power:");
    Brain.Screen.setCursor(10, 20); Brain.Screen.print(power1);
    printf("power1: %f\n", power1);

    printf("error: %f\n", error);
    //gets it out of loop if stuck
    if(counter > 5000)
    {
      error  = max_error; 
    }
    printf("counter: %f\n", counter);

    if(isFrontArm ==1)
    {
      
      frontArm.spin(fwd, power1, pct);
    }
    if(isBackArm ==1)
    {
      //CAUTION: THIS CODE IS COMMENTED OUT
      //backArm.spin(fwd, -power1, pct);
    }
    


    counter ++; 


    drive_CurrentDeg = 0;
  };

  //once outside the loop, stop and hold the motor
  //frontArm.stop(hold);
  frontArm.stop(hold);


  //resets everything
  error = 0; previous_error = 0; intergral = 0; derivative = 0; power1 = 0; drive_CurrentDeg = 0; target_distance = 0;
  Brain.Screen.setCursor(10, 3); Brain.Screen.setFillColor(green); Brain.Screen.print("Done!");
  printf("done!!");
  
};



//---------------------------Odom movement program---------------
bool isEnterBlock2Calc(double target_X, double target_Y)
{
  bool isXWithinRange = false; bool isXWithinRange2 = false;
  bool isYWithinRange = false; bool isYWithinRange2 = false; 

  double lowerErrorBound = 7;
  double higherErrorBound = 7;


  if(XGlobal > (target_X - lowerErrorBound) && XGlobal < (target_X + higherErrorBound))//do not enter if thetaM is within +-8% of target_orientation
  {
    isXWithinRange = true; 
  }

  //----------------yGlobal-------
  //this condition only works if x is positive because inequalities flip if negative
  if(yGlobal > (target_Y - lowerErrorBound) && yGlobal < (target_Y + higherErrorBound))//do not enter if thetaM is within +-8% of target_orientation
  {
    isYWithinRange = true; 
  }



  //calculates conditions for isEnterBlock2; only enter if both isXWithinRange and isYWithinRange are true; 
  if(isXWithinRange == true || isXWithinRange2 == true)
  {
    if(isYWithinRange == true || isYWithinRange2 == true)
    {
      return false;//close enough to target coordinate
    }
    else
    {
      return true; //second condition is not satisfied, get into the loop
      
    }
  }
  else
  {
    return true; //not close enough, get into the loop
  }
}

bool isEnterBlock1and3Calc(double target_orientation)
{
  double lowerErrorBound = 0.2;
  double higherErrorBound = 0.2;

  //this condition only works if thetaM is positive because inequalities flip if negative
  if(thetaM > (target_orientation - lowerErrorBound) && thetaM < (target_orientation + higherErrorBound))//do not enter if thetaM is within +-15% of target_orientation
  {
    return false; 
  }
  
  //when neither conditions are reached; return true to keep the loop going
  return true; 

}
//---------------------------Odom movement program---------------
void move_to(double target_X, double target_Y, bool extra, double target_orientation, double terminal_orientation, bool isDrivingForward, bool isFacingForward, double driveSpeed,  bool isGrabAndGo )
{

  //sets values to get it into loop
  double turn_power = 0.0;
  double drive_power = 5.0; //leave default to 0

  double turn_power_block1 = 0;
  double turn_power_block3 = 0;
  
  bool isEnterBlock1 = true; //default is to enter block1
  bool isEnterBlock2 = true; //default is to enter block1
  bool isEnterBlock3 = true; //default is to enter block3

  
  //-------------block1----------------------------------------

  isEnterBlock1 = isEnterBlock1and3Calc( target_orientation);
  

  while(isEnterBlock1 == true)
  {
    printf("------------into block1------------"); 
    turn_power_block1 = block1_turn_to(24, 2, 2, false, target_orientation, thetaM); 
    leftDrive1.spin(fwd, turn_power_block1, pct);//caution, might have to reverse turn directions
    leftDrive2.spin(fwd, turn_power_block1, pct);
    leftDrive3.spin(fwd, turn_power_block1, pct);
    rightDrive1.spin(fwd, -turn_power_block1, pct);
    rightDrive2.spin(fwd, -turn_power_block1, pct);
    rightDrive3.spin(fwd, -turn_power_block1, pct);

    //revaluates the loop condition
    isEnterBlock1 = isEnterBlock1and3Calc( target_orientation);
  }
  //once done, stop everything
    leftDrive1.stop(brakeType::hold);
  leftDrive2.stop(brakeType::hold);
  leftDrive3.stop(brakeType::hold);
  rightDrive1.stop(brakeType::hold);
  rightDrive2.stop(brakeType::hold); 
  rightDrive3.stop(brakeType::hold); 
  Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(blue); Brain.Screen.print("block 1 done");
  printf("----Done with block 1------------------------------");
  task::sleep(50); 




  
  //turning toward target_orientation loop once at the target coordinate
  
  //------------------------block 2--------------------------------
  //*********************************************************
  

  
  //checking block2 conditions
  isEnterBlock2 = isEnterBlock2Calc( target_X, target_Y);
  //going to a point loop
  //block 2 condition was fabs(turn_power) >3 || fabs(drive_power) >3 
  while(isEnterBlock2 == true)
  {

    //CAUTION: flipped thetaM because odometry is backward
    //thetaM *= -1;
    printf("main ThetaM %f\n", thetaM);

    turn_power = turn_toCalc(24, 2, 3, isDrivingForward, isFacingForward, target_X, target_Y, false, XGlobal, yGlobal, thetaM);//potential tip: try reversing thetaM (-thetaM)
    //turn_power = 0; 

    //continuously updates it
    drive_power = drive_toCalc(driveSpeed, 0.3, 0.3, isDrivingForward, target_X, target_Y, false, XGlobal, yGlobal); //CAUTION:  loop header is a set value; not set to upperloop param
    if(fabs(turn_power) <3 )
    {

      printf("------------------into driving loop!!-----------\n"); 
      drive_power = drive_toCalc(driveSpeed, 0.3, 0.3, isDrivingForward, target_X, target_Y, false, XGlobal, yGlobal); //CAUTION:  loop header is a set value; not set to upperloop param
      //drive_toCalc(double KP, double KI, double KD, bool extra, double target_distanceX, double target_distanceY, bool extra2, double XGlobal, double yGlobal)
    }
    else
    {
      
      if(drive_power < 0)
      {
        drive_power = -50;
      }
      else
      {
        drive_power = 50; 
      }
      
    }

    printf("-----------------main----------\n");
    printf("main turn_power %f\n", turn_power);
    printf("main drive_power %f\n", drive_power);

    printf("XGlobal %f\n", XGlobal);
    printf("yGlobal %f\n", yGlobal); 

    //CAUTION: might be at the wrong place
    //updates isEnterBlock2 conditions
    //isEnterBlock2 = isEnterBlock2Calc( target_X, target_Y);

    

    leftDrive1.spin(fwd, drive_power + turn_power, pct);//caution, might have to reverse turn directions
    leftDrive2.spin(fwd, drive_power + turn_power, pct);
    leftDrive3.spin(fwd, drive_power + turn_power, pct);
    rightDrive1.spin(fwd, drive_power - turn_power, pct);
    rightDrive2.spin(fwd, drive_power - turn_power, pct);
    rightDrive3.spin(fwd, drive_power - turn_power, pct);

    //checking block2 conditions
    isEnterBlock2 = isEnterBlock2Calc( target_X, target_Y);

    //if the back limit switch sensed the goal made contact
    if(backGrabberLimitS.pressing() || backGrabberLimitS2.pressing())
    {
      backGrabberPneum.set(true);
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
      printf("------------backGrabber triggered ------- ");
      if(isGrabAndGo == true)
      {
        isEnterBlock2 = false;
        printf("------------backGrabber entered grab and go! ------- ");
      }

    }
    //if the front limit switch sensed the goal made contact
    if(frontGrabberLimitS12.pressing())
    {
      frontGrabPneumatic.set(false);
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
      printf("------------front triggered ------- ");
      if(isGrabAndGo == true)
      {
        isEnterBlock2 = false; 
        isEnterBlock3 = false; 
        printf("------------front entered grab and go! ------- ");
      }

    }

  } 

  leftDrive1.stop(brakeType::hold);
  leftDrive2.stop(brakeType::hold);
  leftDrive3.stop(brakeType::hold);
  rightDrive1.stop(brakeType::hold);
  rightDrive2.stop(brakeType::hold); 
  rightDrive3.stop(brakeType::hold); 
  Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(yellow); Brain.Screen.print("out of turning_test");
  printf("----Done with move_to------------------------------");
  task::sleep(300); 


    
  
  //-------------block3----------------------------------------
  
  isEnterBlock3 = isEnterBlock1and3Calc( terminal_orientation);


  while(isEnterBlock3 == true)
  {
    printf("------------into block3------------"); 
    turn_power_block3 = block1_turn_to(120, 2, 3, false, terminal_orientation, thetaM); 
    leftDrive1.spin(fwd, turn_power_block3, pct);//caution, might have to reverse turn directions
    leftDrive2.spin(fwd, turn_power_block3, pct);
    leftDrive3.spin(fwd, turn_power_block3, pct);
    rightDrive1.spin(fwd, -turn_power_block3, pct);
    rightDrive2.spin(fwd, -turn_power_block3, pct);
    rightDrive3.spin(fwd, -turn_power_block3, pct);

    //revaluates the loop condition
  isEnterBlock3 = isEnterBlock1and3Calc( terminal_orientation);
  
  }
  
  //once done, stop everything
    leftDrive1.stop(brakeType::hold);
  leftDrive2.stop(brakeType::hold);
  leftDrive3.stop(brakeType::hold);
  rightDrive1.stop(brakeType::hold);
  rightDrive2.stop(brakeType::hold); 
  rightDrive3.stop(brakeType::hold); 
  Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(red); Brain.Screen.print("block 3 done");
  printf("----Done with block 3------------------------------");
  task::sleep(50); 
  


}

void goal_secure_w_vision(int goalTypeInFunction)
{
  int timeout_counter = 0; 
  bool is_goal_secure_w_vision = !frontGrabberLimitS12.pressing() && timeout_counter < 400; //20 seconds per loop, 20*300 loops = 2000msc = 6 seconds
  goalType = goalTypeInFunction; //sets the global variable
  while(is_goal_secure_w_vision)
  {
    isVisionLock = true; 
    isDriving = false;
    timeout_counter ++; 
    is_goal_secure_w_vision = !frontGrabberLimitS12.pressing() && timeout_counter < 100 && yGlobal < 75;
    printf("is_goal_secure_w_vision: %d", is_goal_secure_w_vision);
    

    task::sleep(20);
  }
  isVisionLock = false; //once done, stops the vision lock
  isDriving = true; 

}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  callibrateInertialSensor();
  Brain.Screen.setCursor(2,2); Brain.Screen.print("printing");

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here

  task myTask(odom);
  task myTask2(VisionDrivingTo);
  task myTask3(zuck); 

  IsOdom = true; 
  isDriving = true; 
    //Resets the position of the absolute encoder to the value of zero.
    RotationRight.resetPosition();
    RotationLeft.resetPosition();
    RotationBack.resetPosition();

    IsIntake = 1; //spin the intake
  //calibration_and_testing(); 
  //------------actual auton---------
//void move_to(double target_X, double target_Y, bool extra, double target_orientation, double terminal_orientation, bool isDrivingForward, bool facingForward, double driveSpeed,  bool isGrabAndGo )
    
    frontGrabPneumatic.set(true); //opens front

    backGrabberPneum.set(true);//opens, and gets blue goal
    move_to(0, -25, false, 0, 0, false, false, 2, true); 
    backGrabberPneum.set(false);//close, and gets blue goal

    move_to(0, -25, false, 2.58, 2.58, false, false, 2, false); //turning around
    front_arm_up_loop(4, 0.02, 0.1, 810, false, 1, 1);//front arm up

    //move_to(25, -65, false, 2.58, 2.58, true, true, 2, true); //gets side neutral goal 

    move_to(15, -45, false, 2.58, 2.58, true, true, 2, true); //approaches side neutral goal //pull back

    front_arm_up_loop(4, 0.02, 0.1, 0, false, 1, 1);//front arm down

    move_to(25, -65, false, 2.58, 2.58, true, true, 2, true);//forward //gets side neutral goal 

    frontGrabPneumatic.set(false); //closes front
    front_arm_up_loop(4, 0.02, 0.1, 810, false, 1, 1);//front arm up 


    move_to(50, -105, false, 2.35, 3.14, true, true, 3, false); //toward the red platform 

    //-----------------red platform-----------------
    //frontGrabPneumatic.set(true); //closes front
    front_arm_up_loop(4, 0.02, 0.1, 300, false, 1, 1);//front arm down
    frontGrabPneumatic.set(true); //opens front
    front_arm_up_loop(4, 0.02, 0.1, 600, false, 1, 1);//front arm up 

    move_to(50, -85, false, 3.14, 0.2, false, false, 3, false); //away from the red platform 
    front_arm_up_loop(4, 0.02, 0.1, 0, false, 1, 1);//front arm down  

    
    move_to(50, -40, false, 0.2, 0.2, true, true, 3, false); //gets middle mogol

    goalType = 2.0;
    goal_secure_w_vision(2); //secure the neutral with vision sensor

    frontGrabPneumatic.set(false); //close front
    front_arm_up_loop(4, 0.02, 0.1, 810, false, 1, 1);//front arm up 

    //------------blue platform----------
    move_to(50, -15, false, 0, 0, true, true, 3, false); //puts on blue platform
    front_arm_up_loop(4, 0.02, 0.1, 300, false, 1, 1);//front arm down 
    frontGrabPneumatic.set(true); //opens front
    front_arm_up_loop(4, 0.02, 0.1, 600, false, 1, 1);//front arm up 
    //----------end of blue platform--------

    move_to(50, -65, false, 0, 1.57, false, false, 3, false); //back away from platform 
    
    //move_to(80, -65, false, 1.57, 1.57, false, false, 3, false); //gets 3rd neural goal
    





    //move_to(5, -25, false, 3.14, 0, false, true, 3, false); //toward the red platform */

    //move_to(40, -65, false, 2.65, 2.65, false, true, 4); //good value 65 //turning for neutral

    //move_to(0, 15, false, 0, 0, true, true, 4); 
    /*frontGrabPneumatic.set(false);  //opens front pnumatics
    move_to(0, 65, false, 0, 0, true, true, 4); //good value 65 //grab and go activated
    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$Command 1 done&&&&&&&&&&&&");
    goal_secure_w_vision(2); //goalType = 1 is blue; 2 is yellow; 3 is red // default is blue

    frontGrabPneumatic.set(true); //close front pnumatics
    isDriving = true; 

    move_to(0, 10, false, 0, 1.57, true, false, 4); //pulling back
    printf("$$$$$$$$$$$$$$$$$$$$$$$$$$Command 2 done&&&&&&&&&&&&");
    //move_to(0, 10, false, 0.78, 0, false, false, 4); //side toss
    frontGrabPneumatic.set(false);  //opens front pnumatics//side toss

    
    move_to(35, 75, false, 0.61, 0.61, true, true, 4); //going for middle neutral goal
    goal_secure_w_vision(2); //goalType = 1 is blue; 2 is yellow; 3 is red // default is blue
    frontGrabPneumatic.set(true); //close front pnumatics 
  
    move_to(0, 0, false, 0.60, -1.77, true, false, 4); //bring it back  

    
    backGrabberPneum.set(true);//opens, and gets win point goal
    move_to(30, 5, false, -1.77, -1.77, false, true, 4); //winpoint goal 

    backGrabberPneum.set(false);//closes, and dunmps rings into win point goal
    IsIntake = 1; //drop rings  

    vex::task::sleep(2500);
    backGrabberPneum.set(true);//opens, and gets win point goal */

    /*move_to(0, 20, false, 0, -1.57, true, false, 4); //bring it back  
    backGrabberPneum.set(true);//opens, and gets win point goal
    move_to(40, 20, false, -1.57, -1.37, false, true, 4); */
    //move_to(70, 15, false, -1.57, -1.57, false, true, 4); //move toward winpoint goal



    //void move_to(double target_X, double target_Y, bool extra, double target_orientation, double terminal_orientation, bool isForward, bool isGrabAndGo, double driveSpeed )
    //move_to(-20, -20, false, 0.75, 0, false, false, 4); //bring it back 
    //move_to(10, 25, false, -3.14, -3.14, false, false, 4); //bring it back 
  
    
  

  // ..........................................................................
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


void usercontrol(void) {

    /*task myTask1(odom);

  IsOdom = true; */



    /*task myTask(odom);

  IsOdom = true; 
    //Resets the position of the absolute encoder to the value of zero.
    RotationRight.resetPosition();
    RotationLeft.resetPosition();
    RotationBack.resetPosition();

    move_to(0, 20, false, 0, 0, true, true, 2); */

  task myTask10(zuck);
  task myTask11(VisionDrivingTo);


      // temperature check button code
  int box1_originX = 20;
  int box1_width = 80;
  int box1_originY = 80;
  int box1_height = 50;

  int box1_endX = box1_originX + box1_width;
  int box1_endY = box1_originY + box1_height;

  double driveSpeed = 1.0;

    double fwdAmount;
  double turnAmount;


  double fwdPower;
  double turnPower; 

  bool isBackGrabberClose = false; 
  bool isFrontGrabberClose = false; 

    //Resets the position of the absolute encoder to the value of zero.
    RotationRight.resetPosition();
    RotationLeft.resetPosition();
    RotationBack.resetPosition();

  //task myTask2(goalGrabDown);

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    fwdAmount = Controller1.Axis3.position();
    turnAmount = Controller1.Axis4.position();

    //Check temp box
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(box1_originX, box1_originY, box1_width, box1_height); Brain.Screen.setFillColor(transparent);
    Brain.Screen.setCursor(5, -10); Brain.Screen.print("Check Temp"); //coordinate order (y, x) ?
    //wait(20, msec);

    //exponential power control
    fwdPower = pow(fwdAmount,3) / 100000; 
    turnPower = pow(turnAmount, 3) / 100000; 

    //drive backward button
    if(Controller1.ButtonDown.pressing())
    {
      driveSpeed = -2;
    }
    else
    {

    }

    Brain.Screen.setCursor(4, 10); Brain.Screen.print("fwdPower");
    Brain.Screen.setCursor(4, 10); Brain.Screen.print(fwdPower);
 
    if(fabs(fwdPower) > 1 || fabs(turnPower)> 1)
    {//second and 4th flipped
      leftDrive1.spin(directionType::fwd, fwdPower*driveSpeed + turnPower, voltageUnits::volt);
      leftDrive2.spin(directionType::fwd, fwdPower*driveSpeed + turnPower, voltageUnits::volt);
      leftDrive3.spin(directionType::fwd, fwdPower*driveSpeed + turnPower, voltageUnits::volt);
      rightDrive1.spin(directionType::fwd, fwdPower*driveSpeed - turnPower, voltageUnits::volt);
      rightDrive2.spin(directionType::fwd, fwdPower*driveSpeed - turnPower, voltageUnits::volt);
      rightDrive3.spin(directionType::fwd, fwdPower*driveSpeed - turnPower, voltageUnits::volt);

      Brain.Screen.setCursor(7,2); Brain.Screen.print("voltageright:");
      Brain.Screen.setCursor(7,20); Brain.Screen.print(rightDrive1.voltage());

      Brain.Screen.setCursor(8,2); Brain.Screen.print("voltageLeft:");
      Brain.Screen.setCursor(8,20); Brain.Screen.print(leftDrive1.voltage());
      isDriving = true; 

    }
    else
    {
      if(isVisionLock == false)
      {
        leftDrive1.stop(brakeType::hold);
        leftDrive2.stop(brakeType::hold);
        leftDrive3.stop(brakeType::hold);
        rightDrive1.stop(brakeType::hold);
        rightDrive2.stop(brakeType::hold);
        rightDrive3.stop(brakeType::hold);

      }
      
    }

    //----------goal locking using vision sensor
    if(Controller1.ButtonLeft.pressing())
    {
      Brain.Screen.setCursor(3,1); Brain.Screen.print("ButtonLeft Pressed");
      isVisionLock = true; 
    }
    else if(Controller1.ButtonRight.pressing())
    {
      Brain.Screen.setCursor(3,1); Brain.Screen.print("ButtonLeft not pressed");
      isVisionLock = false; 
    }
    else
    {
      
    }

    //-------------------CONVEYER----------------
//"X" is up, "A" is down
    //-----------conveyer-------
    if(Controller1.ButtonL1.pressing())
    {
      Brain.Screen.setCursor(2, 2), Brain.Screen.print("ButtonUp Pressed");
      IsIntake = 1; 
      
    }
    else if(Controller1.ButtonUp.pressing())
    {
      Brain.Screen.setCursor(2, 2), Brain.Screen.print("ButtonX Pressed");
      IsIntake = 0;
    
    }
    else if (Controller1.ButtonL2.pressing()) 
    {
      Brain.Screen.setCursor(2, 2), Brain.Screen.print("ButtonDown Pressed");
      IsIntake = -1;
      
      
    }
    else
    {
      
    }

    //sensing using front limit switch 
    if(frontGrabberLimitS12.pressing())
    {
      isFrontGrabberClose = true; 
      Brain.Screen.setCursor(7,2); Brain.Screen.setPenColor(blue); Brain.Screen.print("LS = true");

    }

    //button override
    if(Controller1.ButtonR1.pressing())
    {
      //backGrabberPneum.set(true);
      isFrontGrabberClose = false; //opens
      
    }
    //putting goalgrabber down//open
    else if(Controller1.ButtonR2.pressing())
    {
      //backGrabberPneum.set(false);
      isFrontGrabberClose = true; //closes

    }
    else
    {
      //IsGrabberDown = 0;
      //goalGrabber.stop(brakeType::brake);

    }

    //---activate the fornt pnumatics
    //the pneumatics
    if(isFrontGrabberClose == true)
    {
      frontGrabPneumatic.set(false);
      Brain.Screen.setCursor(7,2); Brain.Screen.setPenColor(blue); Brain.Screen.print("pneu true");
    }
    //putting goalgrabber down//open
    else if(isFrontGrabberClose == false)
    {
      frontGrabPneumatic.set(true);
      Brain.Screen.setCursor(8,2); Brain.Screen.setPenColor(red); Brain.Screen.print("pneu false");

    }
    else
    {
      //IsGrabberDown = 0;
      //goalGrabber.stop(brakeType::brake);

    }




//---------------------front goal grabber--------
    //the arm
    //up - "Y"
    if(Controller1.ButtonX.pressing())
    {
      frontArm.spin(directionType::fwd, 15, voltageUnits::volt);
      
    }
    //down - "B"
    else if(Controller1.ButtonA.pressing())
    {
      frontArm.spin(directionType::fwd, -15, voltageUnits::volt);
    
    }
    else
    {
      frontArm.stop(brakeType::brake);
    }

    //sensing using back limit switch 
    if(backGrabberLimitS.pressing() || backGrabberLimitS2.pressing())
    {
      isBackGrabberClose = true; 
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");

    }
    

    //button override
    if(Controller1.ButtonY.pressing())
    {
      //backGrabberPneum.set(true);
      isBackGrabberClose = false; //opens
      
    }
    //putting goalgrabber down//open
    else if(Controller1.ButtonB.pressing())
    {
      //backGrabberPneum.set(false);
      isBackGrabberClose = true; 

    }
    else
    {
      //IsGrabberDown = 0;
      //goalGrabber.stop(brakeType::brake);

    }

    //---activate the pnumatics
    //the pneumatics
    if(isBackGrabberClose == true)
    {
      backGrabberPneum.set(false); 
      Brain.Screen.setCursor(7,2); Brain.Screen.setPenColor(blue); Brain.Screen.print("pneu true");
    }
    //putting goalgrabber down//open
    else if(isBackGrabberClose == false)
    {
      backGrabberPneum.set(true);
      Brain.Screen.setCursor(8,2); Brain.Screen.setPenColor(red); Brain.Screen.print("pneu false");

    }
    else
    {
      //IsGrabberDown = 0;
      //goalGrabber.stop(brakeType::brake);

    }
    //Brain.Screen.clearScreen();




    
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    //-----------add ons-------------------

    //motor temperature checking button
      //if the brain screen is pressed
    if (Brain.Screen.pressing()){

      int X = Brain.Screen.xPosition();//X pos of press
      int Y = Brain.Screen.yPosition();// Y pos of press

    //displaying motor temp
    if ((X >= box1_originX && X <= box1_endX) && (Y >= box1_originY && Y <= box1_endY))
      {
        Brain.Screen.clearScreen();
        
        Brain.Screen.setCursor(1, 2); Brain.Screen.print("LD1: "); if(leftDrive1.temperature() > 50){Brain.Screen.setFillColor(color::orange);};
        Brain.Screen.setCursor(1, 7); Brain.Screen.print(leftDrive1.temperature());

        Brain.Screen.setCursor(1, 14); Brain.Screen.print("LD2: ");
        Brain.Screen.setCursor(1, 19); Brain.Screen.print(leftDrive2.temperature());

        Brain.Screen.setCursor(1, 27); Brain.Screen.print("LD3: ");
        Brain.Screen.setCursor(1, 32); Brain.Screen.print(leftDrive3.temperature());

        Brain.Screen.setCursor(2, 2); Brain.Screen.print("RD1: ");
        Brain.Screen.setCursor(2, 7); Brain.Screen.print(rightDrive1.temperature());

        Brain.Screen.setCursor(2, 14); Brain.Screen.print("RD2: ");
        Brain.Screen.setCursor(2, 19); Brain.Screen.print(rightDrive2.temperature());

        Brain.Screen.setCursor(2, 27); Brain.Screen.print("RD3: ");
        Brain.Screen.setCursor(2, 32); Brain.Screen.print(rightDrive3.temperature());

        Brain.Screen.setCursor(3, 2); Brain.Screen.print("Conveyer: ");
        Brain.Screen.setCursor(3, 7); Brain.Screen.print(Conveyer.temperature());

        Brain.Screen.setCursor(3, 14); Brain.Screen.print("F Arm: ");
        Brain.Screen.setCursor(3, 19); Brain.Screen.print(frontArm.temperature());

        task::sleep(5000);
      }
      else
      {
        
      }
    //Brain.Screen.clearScreen();

    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
  
}
