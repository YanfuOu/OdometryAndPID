#include "vex.h"
//-------------------------------PID Turning loop -------------------------------

double block1_turn_to(double KP, double KI, double KD, bool extra, double target, double thetaM) {
  
    //CAUTION: everything is in radians now!
  double intergral = 0;
  double derivative = 0;
 
  double error = 3; //great lakes tourney: 6degs = 0.10rads
  double previous_error = 0; 
  double power2 = 0;
  double max_error = 0.0175;//loop tripping mechenism //newest: 8deg = 0.0175rads //new:15degs = 0.26 //great lakes tourney: 5deg = 0.087rads

  double error_bound = 0.209;//newest: 12deg = 0.209rads //new: 20degs = 0.36 //great lakes tourney: 10deg = 0.17rads
  double var_current_degrees;
  //double target = 0; target is passed in
  double changeInY = 0;
  double changeInX = 0; 

  double error_X;
  double error_Y;

  printf("turn target4: %f\n", target );
  
  //printf("turn target5: %f\n", target );

  //target = target*M_PI/180; // converts to radians
  //double target_degrees = target*180/M_PI; //DON'T USE THIS; only for display purposes

  
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

    /* ---------- close enough safety loop------------------
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
    */
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
  vex::task::sleep(20);
  printf("------------------------turning done!! didn't even enter loop!-------");

  //resets everything
  error = 0; previous_error = 0; intergral = 0; derivative = 0; power2 = 0; 
  
};