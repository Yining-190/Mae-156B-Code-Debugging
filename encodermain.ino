// math constants and conversions
const float Pi = 3.14159265;
const float rad2deg = 180/Pi;
const float deg2rad = Pi/180;

// Angle combination struct to define lower and upper bound of a slice of a pie.
struct AngleCombination {
  float angle1;
  float angle2;
};


// Initialization of array to take pie slices
AngleCombination angle_combination_arr[2];

// Initialization of array that will be returned by G(z) to jump
float jump_arr[5];

// Check if G(z) has commanded a change in q
bool state_changed = false;

// Check if the arduino is currently predicting the servo position as the servo is transitioning.
volatile bool predictingServo = false;

// Encoder interrupt pins
const int outputA = 2; //Arduino interupt pins are 2 and 3
const int outputB = 3;
unsigned long myTime;

const int resolution = 250; //ppr

volatile long counter = 0; //encoder position through pulse counts   /*NOT NEEDED WITH CAM*/
volatile float encoderPos; //encoder position (pendulum angle relative to frame)              
/*THIS MIGHT BE A PROBLEM SINCE THE ENCODER POSITION IS NOT IN LINE IN THE FIRST FRAME*/

volatile int aState; // current state of pinA (high or low)
volatile int bState; // current state of pinB
volatile int aStatePrevious; // previous state of pinA
volatile int bStatePrevious; // previous state of pinB

volatile int phase; // quadrature phases 1-4
volatile int lastphase; // previous quadrature phase
volatile int error = 0; // number of errors (skipped quadrature phases)

volatile float absolutePos_rad; // Absolute position of pendulum in radians
volatile float absolutePos_rad_previous = 0; // Previously measured position of pendulum in radians. Will be used in the backward difference method to estimate sign of pendulum angular velocity.
volatile float absolutePos_deg; // Absolute position of pendulum in degrees

volatile float x1;
volatile float x2;
const float counts2degree = 0.36; // ratio to convert encoder counts to degree
const float counts2rad = Pi/500; // ratio to convert encoder counts to radians

const int servo_pin = 9; // Pin where arduino outputs servo command (PWM signal)
const int frame_zero = 146; //servo state where frame angle = 0
const float slope = 1.3953488; // 180/(frame_zero - frame_oneEighty)  
const int servo_speed = 150; //how many milliseconds to travel 60 degrees // 150 is the best one // 108 Measured

volatile int servoState; //servo PWM input state (14 to 148)
/*14 TO 148*/
volatile int servoStateInput; // Servo command received from serial.input
/*14 TO 148*/
volatile float servoPos;  //servo position in degrees relative to vertical
/*IF IN RE;ATION TO VERICAL THEN SHOULD BE 0 OR +-180*/
volatile int moveTime;
int dc_motor_on = 0; // State variable that tracks whether DC motor is on (0 = off, 1 = on)

// for servo predictions:
volatile float servoPos_prediction; //predicted position of servo motor
volatile float timePassed;
float startTime; //time when servo starts moving
float startingPos; //starting position of servo
float servo_displacement; //how much servo moves
float servoTransientTime; //how long servo is moving
float goToDeg; //position commanded to go
float goToDeg_prev = 0; // position commanded to go previously

volatile float freq; // oscillation freq
volatile int motorCounter; //encoder counts for DC motor

int k_set_1 = 0; // Jumpset k value 
int k_set_2 = 0; // Jumpset k value

#include <Servo.h>
Servo myservo;

#include "CytronMotorDriver.h"

// Configure the DC motor driver.
CytronMD motor(PWM_DIR, 6, 7);  // PWM = Pin 6, DIR = Pin 7.
unsigned long currentTime; //current time
unsigned long previousTime = 0; //time since last DC motor velocity recording

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() { 
  // set encoder pins as inputs
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);

  digitalWrite(outputA, HIGH);
  digitalWrite(outputB, HIGH);

  attachInterrupt(digitalPinToInterrupt(outputA), quadratureDecoder, CHANGE); // Interrupt for encoder A signal
  attachInterrupt(digitalPinToInterrupt(outputB), quadratureDecoder, CHANGE); // Interrupt for encoder B signal
  attachInterrupt(digitalPinToInterrupt(5), motorSpeed, CHANGE);  // Interrupt for Brushless DC Motor Encoder

  //attachInterrupt(digitalPinToInterrupt(outputA), closedLoopTest(100000), CHANGE);
   
  Serial.begin (460800);

  // Reads the initial state of the outputA
  aState = digitalRead(outputA); // Reads the "current" state of outputA
  bState = digitalRead(outputB); // Reads the "current" state of outputB
 
  //quadrature phases
  if (aState == LOW && bState == LOW){
    lastphase = 1;
  } else if (aState == LOW && bState == HIGH){
    lastphase = 2;
  } else if (aState == HIGH && bState == HIGH){
    lastphase = 3;
  } else {
    lastphase = 4;
  }

  myservo.attach(servo_pin); // Attach servo to pin 9
  myservo.write(frame_zero); // Home servo to 0 deg position
  /*FRAME ZERO IS SUPPOSED TO CORRESPOND TO 0 DEGREES*/
  servoState = frame_zero;
  delay(4000); // Wait for it to home

  // Zero 
  counter = 0;
  servoPos = 0;

  lcd.init(); // initialize the lcd 
  // Print a message to the LCD.
  lcd.print("Hybrid Kapitza's");
  lcd.setCursor(0, 1);
  lcd.print("Pendulum");
  lcd.backlight();
} 

void loop() { 

  //turn servo through Serial input
  if (Serial.available())
  {
    servoStateInput = Serial.parseInt();
    // Input -1 into serial monitor to turn off dc motor
    if (servoStateInput == -1) {
      motor.setSpeed(0);
    }
    else if (servoStateInput > 0){
      servoState = servoStateInput;
      myservo.write(servoState);
    }
  }

  // Calculates the current position and sign of velocity of the pendulum
  calculatePositions();

  //openLoopTest(50000,5000,23); //test for 50s, change angle every 5s, change by 23 degrees every time


  // Print time, pendulum angle and state variables q and k
  myTime = millis();
  Serial.print(myTime);
  Serial.print(" ");
  Serial.print(absolutePos_deg);
  Serial.print(" ");
  Serial.print(servoPos);
  Serial.print(" ");
  Serial.print(jump_arr[2]); // State variable q
  Serial.print(" ");
  Serial.println(jump_arr[3]); // State variable k
  Serial.print(" ");
  // Serial.print(D(jump_arr, 0.01));
  // Serial.print(" ");
  // Serial.println(!C(jump_arr));

  closedLoopTest(200000); // Test which calls jump array

  // Servo prediction code
  if (predictingServo == true){
    if (millis() - startTime < servoTransientTime){
      currentTime = millis();
      timePassed = currentTime - startTime;
      // Integration of servo velocity to obtain servo predition
      if (servo_displacement > 0){
        servoPos_prediction = startingPos + (timePassed/(servo_speed/60));
      }
      else{
        servoPos_prediction = startingPos - (timePassed/(servo_speed/60));
      }

      servoPos = servoPos_prediction;
    } else{
      predictingServo = false;
      calculatePositions();
    }
  }

  if (myTime % 1000 == 0) {
    freq = (motorCounter)/(16*6.25); //calculate oscillation frequency of pendulum
    motorCounter = 0; //reset counter
  } 
  
}


void openLoopTest(long runtime,int timeChange,int angleChange) {
  
  currentTime = millis();

  //DC motor is on during runtime (except for start)
  if (currentTime < runtime) {
    motor.setSpeed(255);  // Run forward at full speed.

    if (currentTime < 2500) {
      servoState = 180; //set pendulum to 42degs
      motor.setSpeed(0); //DC motor off
      counter = 0; //reset pendulum reference
      moveTime = 2500 + timeChange;
    } else {
      if ((currentTime > moveTime) && servoState > 42 ){
        moveTime = moveTime + timeChange;
        if (servoState - angleChange > 42){ //test if vertical position has be achieved
          servoState = servoState - angleChange; //change servo position by angleChange
        } else {
          servoState = 42; //to get to vertical stabilization          }
        }
      }
    }
    myservo.write(servoState);
  } else {
    motor.setSpeed(0);     
    //end of test, turn DC motor off
  }

}

void calculatePositions(){  //no need for quadrature decoder with camera angle. send cam angle in radians directly. update x2 and absposradprev. turn this into task
  counter = counter % 1000; // not allow multiple revolutions
  encoderPos = counts2degree*(float)counter; //convert counter into degrees

  if(predictingServo == false){
    servoPos = slope*((float)frame_zero-(float)servoState); // remap servoState into an angular position
  }

  absolutePos_deg = (encoderPos + servoPos); //add angle from encoder with angle from servo
  if (absolutePos_deg > 360){
    absolutePos_deg = absolutePos_deg - 360;
  } 
  absolutePos_rad = deg2rad*absolutePos_deg;
  absolutePos_rad = atan2(sin(absolutePos_rad), cos(absolutePos_rad)); // Remap to [-pi, pi]

  // if(myTime % 10 == 0){
    x2 = absolutePos_rad - absolutePos_rad_previous; // Calculate sign of pendulum angle velocity through backward difference
    absolutePos_rad_previous = absolutePos_rad;
  // }
}

void moveServo_prediction(float goToDeg){
  predictingServo = true;
  // calculatePositions(); //doublecheck by recalculating values

  startTime = millis(); //start of servo transient
  startingPos = servoPos;
  servo_displacement = goToDeg - servoPos; //distance servo must move
  servoTransientTime = (servo_displacement/60.0)*servo_speed; //estimate of time servo will be moving in milliseconds ADD abs() if you want to predict servo when negative velocity

  servoState = round((float)frame_zero-goToDeg/slope); //convert from goToDeg to servoState
  myservo.write(servoState); //send signal to move servo 
}

void closedLoopTest(long vibration_time){
  currentTime = millis();

  // Initialization of test
  if (dc_motor_on == 0){
    jump_arr[2] = 1; // Initialize q = 1
    jump_arr[3] = 1; // Initialize k = 1
    counter = 0; // Initialize encoder counter to 0
    motor.setSpeed(255);
    delay(2000); // Wait for motor to get up to speed
    dc_motor_on = 1;
  }

  // Turn of closed loop if time above argument vibration_time
  if (currentTime > vibration_time){
    motor.setSpeed(0);
    servoState = frame_zero;
    myservo.write(servoState);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Turning off");
    lcd.setCursor(0, 1);
    lcd.print("closed loop");
    lcd.backlight();
  }

  /* ------------------Changing k to 4 when pendulum hits inverted position-------------------
  --------------Correction to Hybrid Control Code to respond to outside disturbance---------*/
  
  // if (abs(absolutePos_deg - 180.0) < 5 && jump_arr[2] == 0){
  //   jump_arr[3] = -4;
  // }

  // if (abs(absolutePos_rad - 3*Pi/8) < 0.075 &&  jump_arr[2] == 3 && x2 < 0){
  //   jump_arr[2] = 2;
  //   jump_arr[3] = 4;
  // }

  // if (jump_arr[2] == 1 && jump_arr[3] == -1){
  //   jump_arr[2] = 1;
  //   jump_arr[3] = 1;
  // }

  
  if (jump_arr[2] == -1 && jump_arr[3] == -3 && abs(absolutePos_deg - 110) < 10){
    jump_arr[2] = 3;
    jump_arr[3] = 3;
  }

  if (jump_arr[2] == 3 && jump_arr[3] == 3 && abs(absolutePos_deg + 30) < 10){
    jump_arr[2] = -1;
    jump_arr[3] = -3;
  }

  if (jump_arr[2] == 0 && jump_arr[3] == 4 && abs(absolutePos_deg) < 10){
    jump_arr[2] = 1;
    jump_arr[3] = 1;
  }

  if (jump_arr[2] == 0 && jump_arr[3] == -4 && abs(absolutePos_deg) < 10){
    jump_arr[2] = 1;
    jump_arr[3] = 1;
  }
  
  

  
  //--------------------------------------end of corrections-----------------------------------
  //-------------------------------------------------------------------------------------------

  // Prepare jump_array which will be put into G(z)
  jump_arr[0] = absolutePos_rad;
  jump_arr[1] = x2;
  jump_arr[4] = currentTime;


  //------ Debugging Code ------------------------//

  // if (D(jump_arr, 0.01) == 1){
  //   Serial.println("D changed!");
  // }
  // if (C(jump_arr) == 1){
  //   Serial.println("C changed!");
  // }
  // Serial.println(jump_arr[2]);

  // Serial.println(dx1p(absolutePos_rad,3));
  // Serial.println(D(jump_arr, 0.01));
  // Serial.print(" ");
  // Serial.println(C(jump_arr));
  // delay(2000);

  //------ Debugging Code ------------------------//

  //Check if G(z) should be called
  if(D(jump_arr, 0.01) && !C(jump_arr)){
    G(jump_arr);
    state_changed = true;

    //------ Debugging Code ------------------------//

    // Serial.print(myTime);
    // Serial.print(" ");
    // Serial.print(rad2deg*absolutePos_rad);
    // Serial.print(" ");
    // Serial.print(servoPos);
    // Serial.print(" ");
    // Serial.print(jump_arr[2]);
    // Serial.print(" ");
    // Serial.print(jump_arr[3]);
    // Serial.print(" ");
    // Serial.print(D(jump_arr, 0.01));
    // Serial.print(" ");
    // Serial.println(!C(jump_arr));

    //------ Debugging Code ------------------------//
  }
  else{
    state_changed = false;
  }

  goToDeg = rad2deg*psi(jump_arr[2]);

  if(int(goToDeg) != int(goToDeg_prev)){
    // Command servo to jump to new state with predicition
    moveServo_prediction(goToDeg);
  }
  goToDeg_prev = goToDeg;
}

void quadratureDecoder() {
  
  aState = digitalRead(outputA); // Reads the current state of outputA
  bState = digitalRead(outputB); // Reads the current state of outputB

  // define four quadrature phases from A and B states
  if (aState == LOW && bState == LOW){
    phase = 1;
  } else if (aState == LOW && bState == HIGH){
    phase = 2;
  } else if (aState == HIGH && bState == HIGH){
    phase = 3;
  } else {
    phase = 4;
  }

  if (phase != lastphase){     
    // Forward direction order: (1,2,3,4,1,2...)
    if ((phase == lastphase + 1) || (phase == lastphase-3)) {
      counter ++;
    // Backward direction order: (4,3,2,1,4,3...)
    } else if ((phase == lastphase -1) || (phase == lastphase + 3)){
      counter --;
    // error if skipped step    example: (1,2,4,1)
    } else { 
      error ++;
      Serial.print("e ");
      Serial.println(error);
    }

  } 
  // Updates the previous state of the outputA with the current state
  lastphase = phase; //MIGHT NEED LASTANGLE = ANGLE;

 }

 void motorSpeed() {
  motorCounter++;
 }

// Psi function provided by Daniel
float psi(int q){

  switch(q){
    case -3:
      return Pi/4;

    case -2:
      return Pi/2;

    case -1:
      return 3*Pi/4;
    
    case 0:
      return Pi;
    
    case 1:
      return Pi/4;

    case 2:
      return Pi/2;

    case 3:
      return 3*Pi/4;
  }
}


// Lowercase theta p function provided by Daniel
float lower_theta_p(int p){

  switch(p){
    case -3:
      return -5*Pi/8;

    case -2:
      return -3*Pi/8;

    case -1:
      return -Pi/8;
    
    case 0:
      return 0.0;
    
    case 1:
      return Pi/8;

    case 2:
      return 3*Pi/8;

    case 3:
      return 5*Pi/8;
  }
}

// Uppercase theta p function provided by Daniel
// DISCLAIMER: C++ does not return arrays. You must pass a pointer to an array into the function and edit it.
// Takes pointer to AngleCombination array of length 2 and edits that array.
void upper_theta_p(int q, AngleCombination pArray[]){
  switch(q){
    case -3:
      pArray[0].angle1 = -5*Pi/8;
      pArray[0].angle2 = -3*Pi/8;
      pArray[1].angle1 = NULL;
      pArray[1].angle2 = NULL;
      break;
    case -2:
      pArray[0].angle1 = -3*Pi/8;
      pArray[0].angle2 = -Pi/8;
      pArray[1].angle1 = NULL;
      pArray[1].angle2 = NULL;
      break;
    case -1:
      pArray[0].angle1 = -Pi/8;
      pArray[0].angle2 = 0;
      pArray[1].angle1 = NULL;
      pArray[1].angle2 = NULL;
      break;
    case 0:
      pArray[0].angle1 = -Pi;
      pArray[0].angle2 = -5*Pi/8;
      pArray[1].angle1 = 5*Pi/8;
      pArray[1].angle2 = Pi;
      break;
    
    case 1:
      pArray[0].angle1 = 0;
      pArray[0].angle2 = Pi/8;
      pArray[1].angle1 = NULL;
      pArray[1].angle2 = NULL;
      break;

    case 2:
      pArray[0].angle1 = Pi/8;
      pArray[0].angle2 = 3*Pi/8;
      pArray[1].angle1 = NULL;
      pArray[1].angle2 = NULL;
      break;
    case 3:
      pArray[0].angle1 = 3*Pi/8;
      pArray[0].angle2 = 5*Pi/8;
      pArray[1].angle1 = NULL;
      pArray[1].angle2 = NULL;
      break;
  }
}

// Kp_up function provided by Daniel
void kp_up(int p){

  switch(p){
    case -3:
      k_set_1 = -3;
      k_set_2 = -1;
      break;

    case -2:
      k_set_1 = -4;
      k_set_2 = -2;
      break;

    case -1:
      k_set_1 = -3;
      k_set_2 = -1;
      break;
    
    case 0:
      k_set_1 = -4;
      k_set_2 = -4;
      break;
    
    case 1:
      k_set_1 = 1;
      k_set_2 = 3;
      break;

    case 2:
      k_set_1 = 2;
      k_set_2 = 4;
      break;

    case 3:
      k_set_1 = 3;
      k_set_2 = -1;
      break;
  }
}

// Kp_down function provided by Daniel
int kp_down(int p){

  switch(p){
    case -3:
      return 4;


    case -2:
      return -1;

    case -1:
      return -4;
    
    case 1:
      return 4;

    case 2:
      return -1;

    case 3:
      return -4;
  }
}

//dx1p function provided by Daniel
bool dx1p(float x1, int p){
  // Epsilon is the tolerance given to the transition angles when crossreferencing with the pendulum angle
  float epsilon = 0.05;
  if (x1 >= lower_theta_p(p)-epsilon && x1 <= lower_theta_p(p) + epsilon){
    return true;
  }
  return false;
}


// Input pArray and edit the values for the jump. Takes jump array instead of z
void G(float pArray[]){
  float x1 = pArray[0]; // x1 received from Jump Array
  float x1_plus = x1;
  float x2 = pArray[1]; // x2 received from Jump Array
  float x2_plus = x2;
  int q = int(pArray[2]); // q received from Jump Array
  int k = int(pArray[3]); // k received from Jump Array
  float t = pArray[4]; // t received from Jump Array
  float t_plus = t;

  int sq = (q>0) - (q<=0); // Sign of q calculated
  int sx2 = (x2>0) - (x2<=0); // THIS HAS BEEN EDITED FROM DANIEL'S CODE

  // Serial.print(sx2);
  // Serial.print(" ");

  int q_plus = 0;
  int k_plus = 0;

  // if (dx1p(x1,0)){ // If angle of the pendulum is around 0, 
  //   q_plus = 1; // New q is 1
  //   k_plus = -sq; // k_plus is the negative of the sign of q
  // }

  if (dx1p(x1,3)){ // If the angle of the pendulum is around 112.5
    int s_plus = (x2>=0) ? 0 : 3; 
    q_plus = s_plus; // Change configuration to 0 (180deg) if the angular velocity is positive (CCW) otherwise stay at configuration 3
    k_plus = q+sq; // k formula
  }

  else if (dx1p(x1,-3)){ // If the angle of the pendulum is around -112.5
    int s_minus = (x2<=0) ? 0 : -3;
    q_plus = s_minus; // Change configuration to 0 (180deg) if the angular velocity is positive (CCW) otherwise stay at configuration -3
    k_plus = q+sq;
  }

  else{ // If the angle of the pendulum is not around 0, 112.5, -112.5
    if (q == -1 && k == -3){
      q_plus = 1;
      k_plus = 1;
    }
    else if (q == 0 && k==2){
      q_plus = 1;
      k_plus = 1;
    }
    else{
      q_plus = (q+sx2)%4;
      k_plus = q + sq;
    }
  }

  // Serial.print(q_plus);
  // Serial.print(" ");
  // Serial.println(k_plus);
  // delay(5000);

  pArray[0] = x1_plus;
  pArray[1] = x2_plus;
  pArray[2] = q_plus;
  pArray[3] = k_plus;
  pArray[4] = t_plus;
}


// Cp code provided by Daniel
// Takes the jump array instead of z
bool Cp(float pArray[],int p){
  float x1 = pArray[0];
  float x2 = pArray[1];
  int q = int(pArray[2]);
  int k = int(pArray[3]);
  float t = pArray[4];

  upper_theta_p(p, angle_combination_arr);
  bool x1Cp = false;

  if(p != 0){
    if(x1>=angle_combination_arr[0].angle1 && x1<=angle_combination_arr[0].angle2){
      x1Cp = true;
    }
  }
  else{
    if((x1>=angle_combination_arr[0].angle1 && x1<=angle_combination_arr[0].angle2)||(x1>=angle_combination_arr[1].angle1 && x1<=angle_combination_arr[1].angle2)){
      x1Cp = true;
    }
  }

  bool x2Cp = true;
  bool qCp = (q==p);

  kp_up(p);

  bool kCp = false;

  if(k == k_set_1 || k == k_set_2){
    kCp = true;
  }

  //------ Debugging Code ------------------------//

  // Serial.print(x1Cp);
  // Serial.print(" ");
  // Serial.print(x2Cp);
  // Serial.print(" ");
  // Serial.print(qCp);
  // Serial.print(" ");
  // Serial.println(kCp);
  // delay(1000);

  //------ Debugging Code ------------------------//

  return x1Cp && x2Cp && qCp && kCp;
}

// C code provided by Daniel
// Takes jump array instead of z
bool C(float pArray[]){
  for (int j=-3; j<=3; j++){
    // Serial.println(j);
    if(Cp(pArray,j)){
      return true;
    }
  }

  return false;
}

// Dp_up code provided by Daniel
// Takes jump array instead of z
bool Dp_up(float pArray[], int p){
  float x1 = pArray[0];
  float x2 = pArray[1];
  int q = int(pArray[2]);
  int k = int(pArray[3]);
  float t = pArray[4];

  int sq = (q>0) - (q<=0);

  bool x2Dp = (x2*sq >= 0);

  bool qDp = false;

  if (p==0){
    qDp |= (q==-1);
  }
  else{
    qDp |= (q==p);
  }

  // Serial.print(q);
  // Serial.print(" ");
  // Serial.println(p);
  // delay(100);

  bool kDp = false;

  if (p==0){
    if (k==-3){
      kDp = true;
    }
  }
  else{
    kp_up(q);
    if(k == k_set_1 || k == k_set_2){
      kDp = true;
    }
  }

  //------ Debugging Code ------------------------//

  // Serial.print(k);
  // Serial.print(" ");
  // Serial.print(k_set_1);
  // Serial.print(" ");
  // Serial.print(k_set_2);
  // Serial.print(" ");
  // // Serial.print(x2Dp);
  // // Serial.print(" ");
  // // Serial.print(qDp);
  // // Serial.print(" ");
  // Serial.println(kDp);
  // delay(1000);

  //------ Debugging Code ------------------------//
  return x2Dp && qDp && kDp;
}

// Dp_down code provided by Daniel
// Takes jump array instead of z
bool Dp_down(float pArray[], int p){
  float x1 = pArray[0];
  float x2 = pArray[1];
  int q = int(pArray[2]);
  int k = int(pArray[3]);
  float t = pArray[4];

  int sp = (p>0) - (p<=0);
  bool x2Dp = (x2*sp <= 0);

  bool qDp = false;

  if (p==0){
    qDp |= (q==1);
  }
  else{
    qDp |= (q==((p + sp) %4));
  }

  bool kDp = false;

  if (p==0){
    if (k==3){
      kDp = true;
    }
  }
  else{
    if(k == kp_down(p)){
      kDp = true;
    }
  }
  //------ Debugging Code ------------------------//

  // Serial.println(k);
  // Serial.print(x2Dp);
  // Serial.print(" ");
  // Serial.print(qDp);
  // Serial.print(" ");
  // Serial.println(kDp);
  // delay(2000);

  //------ Debugging Code ------------------------//
  return x2Dp && qDp && kDp;
}

// D code provided by Daniel
// Takes jump array instead of z
bool D(float pArray[], float epsilon){
  float x1 = pArray[0];
  float x2 = pArray[1];
  int q = int(pArray[2]);
  int k = int(pArray[3]);
  float t = pArray[4];

  for (int i=-3; i<=3; i++){
    // Serial.println(i);
    bool Dp_up_v = Dp_up(pArray,i);
    bool Dp_down_v = Dp_down(pArray,i);
    bool dx1pv = dx1p(x1, i);
    bool zDp  = dx1pv && (Dp_up_v || Dp_down_v);
    //------ Debugging Code ------------------------//

    // Serial.println(i);
    // Serial.print(" ");
    // Serial.print(Dp_up_v);
    // Serial.print(" ");
    // Serial.print(Dp_down_v);
    // Serial.print(" ");
    // Serial.println(dx1pv);
    // delay(2000);
  
    //------ Debugging Code ------------------------//



    if(zDp){
      return true;
    }
  }
  return false;
}