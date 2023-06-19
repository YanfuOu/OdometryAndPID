#include "vex.h"
//#include "main.cpp"
//calculates the power needed to turn toward a target point from the current position. 
//-------------------------------Turn To loop TESt-------------------------------
//loop header: instead of #include "main" which causes lots of problems, I'll simply send in XGlobal, YGlobal, and thetaM
//this functions serves as a calculator, rather than execution of movements. This is to keep the main.cpp clean :)
//bool extra1 and bool extra2 does nothing, it merely serves as a visual divider when putting arguements in
double turn_toCalc(double KP, double KI, double KD, bool isDrivingForward, bool isFacingForwards, double target_distanceX, double target_distanceY, bool isTurnRight, double XGlobal, double yGlobal, double thetaM) {
  
    //if the front limit switch sensed the goal made contact
    if(backGrabberLimitS.pressing() || backGrabberLimitS2.pressing())
    {
      backGrabberPneum.set(false);
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
      //return 0; 
    }
    
  //sensing using front limit switch 
    if(frontGrabberLimitS12.pressing())
    {
      frontGrabPneumatic.set(false); //closes
      Brain.Screen.setCursor(7,2); Brain.Screen.setPenColor(blue); Brain.Screen.print("pneu true");
      //return 0; 
    }
    
  //CAUTION: everything is in radians now!
  double intergral = 0;
  double derivative = 0;
 
  double error = 3; //great lakes tourney: 6degs = 0.10rads
  double previous_error = 0; 
  double power2 = 0;
  double max_error = 0.0175;//loop tripping mechenism //newest: 8deg = 0.0175rads //new:15degs = 0.26 //great lakes tourney: 5deg = 0.087rads

  double error_bound = 0.209;//newest: 12deg = 0.209rads //new: 20degs = 0.36 //great lakes tourney: 10deg = 0.17rads
  double var_current_degrees;
  double target = 0; 
  double changeInY = 0;
  double changeInX = 0; 

  int revsAboutCenter;

  double error_X;
  double error_Y;

  //target_distanceX = -target_distanceX; //revert the x target so turning to the right is positive; currently doing target = -target instead

  changeInY = target_distanceY-yGlobal;
  changeInX = target_distanceX - XGlobal; 
  printf("turn changInY: %f\n", changeInY );
  printf("turn changeInX: %f\n", changeInX );
  //target = inverse tan( (YTarget - YCurrent)/(XTarget-Xcurrent) );
  //DOUBLE CHECK; atan should give you radians(?)

  //printf("turn target1 y/x: %f\n", (changeInY) / (changeInX) );
  if(changeInY == 0.0) //avoid divide by 0 error
  {
    if(changeInX < 0) //if going to the right, reverse it
    {
      target = 1.5708;//set target to -pi/2 
    }
    else
    {
      target = -1.5708; //set target to pi/2 if positive
    }

  }
  else
  {
    target = atan( (changeInX) / (changeInY)  );//CAUTION: FLIPPED X AND Y! //CAUTION! If changeInX = 0, it will calculate atan(infinity) = pi/2 
    //printf("turn target2: %f\n", target );

  }
  
 
  target = fabs(target); //take the magnitute

  printf("turn target2: %f\n", target ); 

  if(isFacingForwards == true)
  {


        //------------quad 2------
    if( changeInX <= 0 && changeInY > 0 ) //signature for quad 3
    {
      printf("****in quad 2**** \n");
      //target *= -1;
      target = -target; 

    }
       //CAUTION
    //if it is in quad III, make sure the target is negative and adds pi/2 to it
    if( changeInX < 0 && changeInY <= 0 ) //signature for quad 3
    {
      printf("****in quad 3**** \n");
      //target *= -1;
      target = 3.1415 + target; 

    }

    if( changeInX >= 0 && changeInY <= 0 ) //signature for quad 4
    {
      printf("****in quad 4 driving forward**** \n");
      //target *= -1;
      target = 3.1415 - target;  

    }
    //------------end of if facing forward loop-----------
  }

  
 //if the target/current degrees is over 180 degrees, compensates for it
  /*revsAboutCenter = thetaM / 3.14;//get number of revolutions it has made for reference, casted it to int to get revs but not thetaM's percise position
  target += revsAboutCenter * 3.14; */
  
  printf("turn target3: %f\n", target ); 

  if(isFacingForwards == false)
  {

    //-----------------quad 1----------
    if( changeInX > 0 && changeInY > 0 ) //signature for quad 3
    {
      printf("****in quad 2**** \n");
      //target *= -1;
      target = 3.1415 + target; //1st would be 3rd

    }
        //------------quad 2------
    if( changeInX <= 0 && changeInY > 0 ) //signature for quad 3
    {
      printf("****in quad 2**** \n");
      //target *= -1;
      //target = -target; 
      target = 3.1415 - target;  //2nd is 4th

    }
       //CAUTION
    //if it is in quad III, make sure the target is negative and adds pi/2 to it
    if( changeInX < 0 && changeInY <= 0 ) //signature for quad 3
    {
      printf("****in quad 3**** \n");
      //target *= -1;
      //target = 3.1415 + target; 
      //3rd would be 1st quad

    }

    if( changeInX >= 0 && changeInY <= 0 ) //signature for quad 4
    {
      printf("****in quad 4 facing backwards**** \n");
      //target *= -1;
      //target = 3.1415 - target;  
       target = -target; //4th is 2nd quad

    }
    //------------end of if facing backwards loop-----------
  }


  //this is for TERMINAL CORRECTION/ low error correction, minor corrections! 
    //instead of setting the negative, use the value of y to detect if going forward or backwards
    if(fabs(changeInY) < 7) //if close enough and negative
    {
        if(changeInY < 0)
        {
          target = -target; 
          printf("----------------TURN GOING BACKWARDS---------------------\n");

        }
    }

    printf("turn target4: %f\n", target );
    
    //printf("turn target5: %f\n", target );

    //target = target*M_PI/180; // converts to radians
    //double target_degrees = target*180/M_PI; //DON'T USE THIS; only for display purposes


  //if going backwards; reverse everything that was forward
  /*if(isForward == false)
  {
   target *= -1; //reverse it //CAUTION: be aware when it's changeInY == 0, might reverse it too many times
  } */
  
  
  if(fabs(error) > max_error){

    printf("turn target5: %f\n", target);
    printf("thetaM: %f\n", thetaM);
     //var_current_degrees= thetaM; //CAUTION. By flipping the current degrees, it pretends CCW turn is (-) //was: unflipped
     //CAUTION: COMPETITION ROBOT TRACKING WHEELS' DISTANCES FROM TRACKING CENTER ARE NOT UPDATED!
    Brain.Screen.clearScreen(); 
    //Brain.Screen.setCursor(2, 3); Brain.Screen.print("Current degrees:");
    Brain.Screen.setCursor(2, 20); Brain.Screen.print(var_current_degrees);

    //Brain.Screen.setCursor(3, 3); Brain.Screen.print("target:");
    Brain.Screen.setCursor(3, 20); Brain.Screen.print(target);

    

    error = target - thetaM; 

    printf("turn error: %f\n", error);
    //Brain.Screen.setCursor(4, 3); Brain.Screen.print("Current error:");
    //Brain.Screen.setCursor(4, 20); Brain.Screen.print(error);
    //task::sleep(200);
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
    //current intergral
    //Brain.Screen.setCursor(6, 3); Brain.Screen.print("Current intergral:");
    //Brain.Screen.setCursor(6, 20); Brain.Screen.print(intergral);
    //current derivative
    //Brain.Screen.setCursor(8, 3); Brain.Screen.print("Current derivative:");
    //Brain.Screen.setCursor(8, 20); Brain.Screen.print(derivative);
    previous_error = error; 

    power2 = error*KP + intergral*KI + derivative*KD ; 
    //printf("turn power2: %f\n", power2);

    //printf("turn max_error: %f\n", max_error);
    //printf("turn error: %f\n", error);

    /*
    //return 0 if within range
    if(fabs(error) < max_error)
    {
      Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(green); Brain.Screen.print("out of PID");
      printf("------------------------turning done!!-------------------\n ");
      return 0;
    } */

    //return 0 if both x and y differences are within 5% range
    //prevents werid movements when close to target
    error_X = fabs(target_distanceX - XGlobal);
    error_Y = fabs(target_distanceY - yGlobal);

    printf("turn error_X: %f\n", error_X);
    printf("turn error_Y: %f\n", error_Y);
    if(error_X < 0.15*target_distanceX )
    {
      if(error_Y < 0.15*target_distanceY)
      {
        Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(green); Brain.Screen.print("out of PID");
        printf("------------------------turning done!! into exit loop-------------------\n ");
        return 0;
      }
    }
    
    return power2; 
    
    //task::sleep(20); //don't loop it or run motors; simply return the values and let the loop in main take care of it
  } //the loop ending bracket

  //if doesn't get into the if statement: 
  //when outside the loop, return 0 to trip the loop

    return 0; 

  leftDrive1.stop(brake);
  rightDrive1.stop(brake);
  leftDrive2.stop(brake);
  rightDrive2.stop(brake);
  Brain.Screen.setCursor(11, 3); Brain.Screen.setFillColor(yellow); Brain.Screen.print("out of PID");
  printf("------------------------turning done!!didn't even enter loop!-------");
  vex::task::sleep(20);
  //resets everything
  error = 0; previous_error = 0; intergral = 0; derivative = 0; power2 = 0; 

  //return power2; 
  
};