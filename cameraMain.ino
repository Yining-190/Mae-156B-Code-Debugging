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

unsigned long myTime;

const int resolution = 250; //ppr

volatile float absolutePos_rad; // Absolute position of pendulum in radians
volatile float absolutePos_rad_previous = 0; // Previously measured position of pendulum in radians. Will be used in the backward difference method to estimate sign of pendulum angular velocity.
volatile float absolutePos_deg; // Absolute position of pendulum in degrees

volatile float x1;
volatile float x2;

const int servo_pin = 9; // Pin where arduino outputs servo command (PWM signal)
const int frame_zero = 157; //servo state where frame angle = 0
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

#define INTERRUPT_PIN 7 // Pin connected to Arduino A's SIGNAL_PIN

volatile bool newSignal = false;

String receivedString = ""; // Buffer to store the incoming data
bool newData = false;       // Flag to indicate new data received

void setup() { 
  //pinMode(INTERRUPT_PIN, INPUT); // Configure the pin as input
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), signalHandler, RISING);

  //Serial.begin(115200); // Communication with the serial monitor
  Serial.begin(460800);  // Communication with Arduino A
  //Serial.println("Arduino B Ready");
   
  //Serial.begin(460800);

  myservo.attach(servo_pin); // Attach servo to pin 9
  myservo.write(frame_zero); // Home servo to 0 deg position
  /*FRAME ZERO IS SUPPOSED TO CORRESPOND TO 0 DEGREES*/
  servoState = frame_zero;
  delay(4000); // Wait for it to home

  // Zero 
  servoPos = 0;

  lcd.init(); // initialize the lcd 
  // Print a message to the LCD.
  lcd.print("Hybrid Kapitza's");
  lcd.setCursor(0, 1);
  lcd.print("Pendulum");
  lcd.backlight();
} 

void loop() { 

  // Calculates the current position and sign of velocity of the pendulum
  calculatePositions();

  closedLoopTest(200000); // Test which calls jump array

  //delay(5000);
  
}

// Function to calculate positions and angular velocity
void calculatePositions() {
  if (Serial.available()) {
    String angleStr = Serial.readStringUntil('\n');
    float angle = angleStr.toFloat();

    // Echo back for debugging
    Serial.println(angleStr);
    
    absolutePos_deg = angle;
    absolutePos_rad = deg2rad * absolutePos_deg;
  }

    // Reset the buffer
    receivedString = "";
  //}
  x2 = absolutePos_rad - absolutePos_rad_previous;  // Backward difference for angular velocity
  absolutePos_rad_previous = absolutePos_rad;
}

void closedLoopTest(long vibration_time){
  currentTime = millis();

  // Initialization of test
  if (dc_motor_on == 0){
    jump_arr[2] = 1; // Initialize q = 1
    jump_arr[3] = 1; // Initialize k = 1
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

  // Prepare jump_array which will be put into G(z)
  jump_arr[0] = absolutePos_rad;
  jump_arr[1] = x2;
  jump_arr[4] = currentTime;

  //Check if G(z) should be called
  if(D(jump_arr, 0.01) && !C(jump_arr)){
    G(jump_arr);
    state_changed = true;
  }

  else{
    state_changed = false;
  }

  goToDeg = rad2deg*psi(jump_arr[2]);

  if(int(goToDeg) != int(goToDeg_prev)){
    // Command servo to jump to new state with predicition
    moveServo(goToDeg);
  }
  goToDeg_prev = goToDeg;
  updateLCD();
  //Serial.println("Ran closedLoopTest");
}

void moveServo(float goToDeg){
  // calculatePositions(); //doublecheck by recalculating values
  
  servoState = round((float)frame_zero-goToDeg/slope); //convert from goToDeg to servoState
  myservo.write(servoState); //send signal to move servo 
}

// Update LCD for debugging
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  lcd.print(absolutePos_deg, 2);
  lcd.setCursor(0, 1);
  lcd.print("Servo: ");
  lcd.print(goToDeg_prev, 2);
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

  pArray[0] = x1_plus;
  pArray[1] = x2_plus;
  pArray[2] = q_plus;
  pArray[3] = k_plus;
  pArray[4] = t_plus;
}

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

  return x1Cp && x2Cp && qCp && kCp;
}

bool C(float pArray[]){
  for (int j=-3; j<=3; j++){
    // Serial.println(j);
    if(Cp(pArray,j)){
      return true;
    }
  }

  return false;
}

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

  return x2Dp && qDp && kDp;
}

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

  return x2Dp && qDp && kDp;
}

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
    
    if(zDp){
      return true;
    }
  }
  return false;
}
