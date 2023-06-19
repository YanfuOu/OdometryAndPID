/*
//-----------------goal grabber activation--------------------
//this task moves the goal grabber up and down to activate it. This is done while auton is doing something else. 
int IsGrabberDown;
double grabberDeg;

//goal grabber task

int goalGrabDown()
{
  Grabber2.resetPosition();

  while(true)
  {
    grabberDeg = Grabber2.rotation(deg);
    //Brain.Screen.setCursor(9,2); Brain.Screen.print("OuterLoop grabberDeg:");
    //Brain.Screen.setCursor(9,20); Brain.Screen.print(grabberDeg);
    //lifting goalgrabber up
    while(IsGrabberDown ==1) //was >-520
    {
      grabberDeg = Grabber2.rotation(deg);
      Brain.Screen.setCursor(9,2); Brain.Screen.print("GrabberUp grabberDeg:");
      Brain.Screen.setCursor(9,30); Brain.Screen.print(grabberDeg);
      while(grabberDeg < 90)
      {
        grabberDeg = Grabber2.rotation(deg);
        Grabber2.spin(directionType::fwd, -10, voltageUnits::volt);
      }
      Grabber2.stop(brakeType::brake);

    }
    //putting goalbrabber down
    while(IsGrabberDown == -1)
    {
      grabberDeg = Grabber2.rotation(deg);
      //Brain.Screen.setCursor(9,2); Brain.Screen.print("GrabberDown grabberDeg:");
      //Brain.Screen.setCursor(9,30); Brain.Screen.print(grabberDeg);
      while(grabberDeg >-600) //flipped
      {
        grabberDeg = Grabber2.rotation(deg);
        Grabber2.spin(directionType::fwd, 10, voltageUnits::volt);
        Brain.Screen.setCursor(10,2); Brain.Screen.setFillColor(green); Brain.Screen.print("Going down:");
        Brain.Screen.setCursor(10,30); Brain.Screen.print(grabberDeg);
      }
      Grabber2.stop(brakeType::brake);


    }

  }

  return 1;
} 
*/

/*
double armDeg;
int isArmDowm; 

//--------------activate arm------------------.
//similar to the task above, not sure why there are 2 of them
int armActivation()
{
  frontArm.resetPosition();

  while(true)
  {
    armDeg = frontArm.rotation(deg);
    //Brain.Screen.setCursor(9,2); Brain.Screen.print("OuterLoop grabberDeg:");
    //Brain.Screen.setCursor(9,20); Brain.Screen.print(grabberDeg);
    //lifting goalgrabber up
    while(isArmDowm ==1) //was >-520
    {
      armDeg = frontArm.rotation(deg);
      Brain.Screen.setCursor(9,2); Brain.Screen.print("GrabberUp armDeg:");
      Brain.Screen.setCursor(9,30); Brain.Screen.print(armDeg);
      while(armDeg < 90)
      {
        armDeg = frontArm.rotation(deg);
        Grabber2.spin(directionType::fwd, -10, voltageUnits::volt);
      }
      frontArm.stop(brakeType::brake);

    }
    //putting goalbrabber down
    while(isArmDowm == -1)
    {
      armDeg = frontArm.rotation(deg);
      //Brain.Screen.setCursor(9,2); Brain.Screen.print("GrabberDown grabberDeg:");
      //Brain.Screen.setCursor(9,30); Brain.Screen.print(grabberDeg);
      while(armDeg >-600) //flipped
      {
        armDeg = frontArm.rotation(deg);
        Grabber2.spin(directionType::fwd, 10, voltageUnits::volt);
        Brain.Screen.setCursor(10,2); Brain.Screen.setFillColor(green); Brain.Screen.print("Going down:");
        Brain.Screen.setCursor(10,30); Brain.Screen.print(armDeg);
      }
      frontArm.stop(brakeType::brake);


    }

  }

  return 1;
} 
*/

/*
// ---------------------------PID Forward distance loop ------------------
void forward_loop(double KP, double KI, double KD, double target_distance, bool isReleaseGoal, double LRflippersSpeeds, int upperFlipperSpeed) {

  //PID
  / decent tuning values double KP = 0.05; double KI = 0.05; double KD = 0.02;

  RotationRight.resetPosition();RotationLeft.resetPosition();RotationBack.resetPosition(); 

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

  double target_degrees = target_distance * 28.648; 


  //while loop should run until within max_error range
  while(fabs(error) > max_error){

        //if the back limit switch sensed the goal made contact
    if(backGrabberLimitS.pressing() || backGrabberLimitS2.pressing())
    {
      if(isReleaseGoal == false)
      {
      backGrabberPneum.set(false);
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
      printf("------------backGrabber triggered ------- ");

        error = max_error;
        printf("------------backGrabber entered grab and go! ------- ");
      }
      

    }
    //if the front limit switch sensed the goal made contact
    if(frontGrabberLimitS12.pressing())
    {
      if(isReleaseGoal ==false)
      {
        frontGrabPneumatic.set(true);
      Brain.Screen.setCursor(6,2); Brain.Screen.setPenColor(green); Brain.Screen.print("LS = true");
      printf("------------front triggered ------- ");

        error = max_error;
        printf("------------front entered grab and go! ------- ");

      }
      
    }

    //calculates current degrees by averaging both wheels
    drive_CurrentDeg = ( RotationLeft.position(degrees) + RotationRight.position(degrees) ) /2; 

    error = target_degrees - drive_CurrentDeg; 

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
    if(counter > 10000)
    {
      error  = max_error; 
    }
    printf("counter: %f\n", counter);

    leftDrive1.spin(fwd, power1, pct);
    leftDrive2.spin(fwd, power1, pct);
    leftDrive3.spin(fwd, power1, pct);
    rightDrive1.spin(fwd, power1, pct);
    rightDrive2.spin(fwd, power1, pct);
    rightDrive3.spin(fwd, power1, pct);

    counter ++; 


    drive_CurrentDeg = 0;
  };

  //once outside the loop, stop and hold the motor
  leftDrive1.stop(hold);
  leftDrive2.stop(hold);
  leftDrive3.stop(hold);
  rightDrive1.stop(hold);
  rightDrive2.stop(hold); 
  rightDrive3.stop(hold); 

  //resets everything
  error = 0; previous_error = 0; intergral = 0; derivative = 0; power1 = 0; target_degrees = 0; target_distance = 0;
  Brain.Screen.setCursor(10, 3); Brain.Screen.setFillColor(green); Brain.Screen.print("Done!");
  printf("done!!");
  
};
*/

/*

//-------------------------------PID Turning loop -------------------------------
void turning_loop(double KP, double KI, double KD,double target) {
  double intergral = 0;
  double derivative = 0;
 
  double error = 6;
  double previous_error = 0; 
  double power2 = 0;
  double max_error = 5;

  double error_bound = 10;
  
  while(fabs(error) > max_error){
    double var_current_degrees = InertialSensor.rotation(rotationUnits::deg);
    Brain.Screen.clearScreen(); 
    Brain.Screen.setCursor(2, 3); Brain.Screen.print("Current degrees:");
    Brain.Screen.setCursor(2, 20); Brain.Screen.print(var_current_degrees);
    task::sleep(200);

    error = target - var_current_degrees; 
    Brain.Screen.setCursor(4, 3); Brain.Screen.print("Current error:");
    Brain.Screen.setCursor(4, 20); Brain.Screen.print(error);

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
    Brain.Screen.setCursor(6, 3); Brain.Screen.print("Current intergral:");
    Brain.Screen.setCursor(6, 20); Brain.Screen.print(intergral);
    //current derivative
    Brain.Screen.setCursor(8, 3); Brain.Screen.print("Current derivative:");
    Brain.Screen.setCursor(8, 20); Brain.Screen.print(derivative);
    previous_error = error; 

    power2 = error*KP + intergral*KI + derivative*KD ; 
    Brain.Screen.setCursor(10, 3); Brain.Screen.print("Current power:");
    Brain.Screen.setCursor(10, 20); Brain.Screen.print(power2);
    leftDrive1.spin(fwd, power2, pct);
    leftDrive2.spin(fwd, power2, pct);
    leftDrive3.spin(fwd, power2, pct);
    rightDrive1.spin(fwd, -power2, pct);
    rightDrive2.spin(fwd, -power2, pct);
    rightDrive3.spin(fwd, -power2, pct);
    
    task::sleep(20);
  };

  leftDrive1.stop(brake);
  leftDrive2.stop(brake);
  leftDrive3.stop(brake);
  rightDrive1.stop(brake);
  rightDrive2.stop(brake);
  rightDrive3.stop(brake);

  //resets everything
  error = 0; previous_error = 0; intergral = 0; derivative = 0; power2 = 0; 
  
};
*/