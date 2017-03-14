/**********************************************************************************************************************************************************************************************
*********************************************************SLAM-BOT'S BLOODHOUND PROJECT MOTION FUNCTION PROGRAM*********************************************************************************
********************************************************KEMP HARTZOG, CHRIS HARRIS, THOMAS MILLER, CORY LANDETA********************************************************************************
**********************************************************************************************************************************************************************************************/

/***************************************************************************LIBRARIES*********************************************************************************************************/

#include <Wire.h>

//matrix library
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

//Ultrasonics library
#include <NewPing.h>

//motor shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//servo library
#include <Servo.h>

/*****************************************************************************INITIALIZATION**************************************************************************************************/

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

//create matrix object
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *M1Motor = AFMS.getMotor(1); Adafruit_DCMotor *M2Motor = AFMS.getMotor(2);
Adafruit_DCMotor *M3Motor = AFMS.getMotor(3); Adafruit_DCMotor *M4Motor = AFMS.getMotor(4);

Servo grabber;

int grabber_start    = 75;
int grabber_cache    = 0;
int grabber_finished = 180;

//Setup variables for moving one block and initial speed
int oneBlock = 306;  int Ninety  = 24;
int M1Speed  = 50 ;  int M2Speed = 50; 
int M3Speed  = 50 ;  int M4Speed = 50;

//interrupt pins and motor ticks count
int M1tick = 0; int M2tick = 0; int M3tick= 0 ; int M4tick = 0;
int M1interrupt = 19; int M2interrupt = 18;
int M3interrupt =  3; int M4interrupt =  2;

//pin for knocker & tic tracer
int knocker = 7;
int tic = 12;

//initialize x,y cordinates
int x_pos = 0;  //int x_finish = 0;
int y_pos = 0;  //int y_finish = 0;

//Ultrasonic initialization
#define MAX_DISTANCE   122  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//Beta side trigger and echo
#define BravoR_TRIGGER     24  
#define BravoR_ECHO        25 
NewPing BravoR(BravoR_TRIGGER, BravoR_ECHO, MAX_DISTANCE);   
#define BravoL_TRIGGER     26  
#define BravoL_ECHO        27 
NewPing BravoL(BravoL_TRIGGER, BravoL_ECHO, MAX_DISTANCE); 

//Alpha side trigger and echo
#define AlphaR_TRIGGER    28  
#define AlphaR_ECHO       29 
NewPing AlphaR(AlphaR_TRIGGER, AlphaR_ECHO, MAX_DISTANCE);  
#define AlphaL_TRIGGER    30  
#define AlphaL_ECHO       31 
NewPing AlphaL(AlphaL_TRIGGER, AlphaL_ECHO, MAX_DISTANCE);

//Delta side trigger and echo
#define DeltaR_TRIGGER    32  
#define DeltaR_ECHO       33 
NewPing DeltaR(DeltaR_TRIGGER, DeltaR_ECHO, MAX_DISTANCE);  
#define DeltaL_TRIGGER    34  
#define DeltaL_ECHO       35  
NewPing DeltaL(DeltaL_TRIGGER, DeltaL_ECHO, MAX_DISTANCE);

//Charlie side triggera and echo
#define CharlieR_TRIGGER  36  
#define CharlieR_ECHO     37  
NewPing CharlieR(CharlieR_TRIGGER, CharlieR_ECHO, MAX_DISTANCE);  
#define CharlieL_TRIGGER  22  
#define CharlieL_ECHO     23
NewPing CharlieL(CharlieL_TRIGGER, CharlieL_ECHO, MAX_DISTANCE);  

int obstaclex[10];
int obstacley[10];
int obnum;

/**************************************************************************START OF PROGRAM***************************************************************************************************/

//Line dancing moves
void setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz

  //start matrix and ready light
  matrix.begin(0x71); // matrix at 71 address
  matrix.clear();
  matrix.drawPixel(0, 0, LED_YELLOW);
  matrix.writeDisplay();
  delay(500);

  //start tic tracer
  pinMode(tic, OUTPUT);
  digitalWrite(tic, HIGH);
  delay(500);
  digitalWrite(tic, LOW);

  grabber.attach(10);
  grabber.write(grabber_start);
  
  //Set speed that will be used
  M1Motor->setSpeed(M1Speed); M2Motor->setSpeed(M2Speed); 
  M3Motor->setSpeed(M3Speed); M4Motor->setSpeed(M4Speed);

  //setup for motor interrupt pins
  pinMode(M1interrupt, INPUT_PULLUP); pinMode(M2interrupt, INPUT_PULLUP);
  pinMode(M3interrupt, INPUT_PULLUP); pinMode(M4interrupt, INPUT_PULLUP);

  //interrupt function attachment and call modes
  attachInterrupt(digitalPinToInterrupt(19),M1count,RISING);
  attachInterrupt(digitalPinToInterrupt(18),M2count,RISING);
  attachInterrupt(digitalPinToInterrupt(3),M3count,RISING);
  attachInterrupt(digitalPinToInterrupt(2),M4count,RISING);

  //knocker setup
  pinMode(knocker, OUTPUT);

  //port setup for printing
  Serial.begin(57600);
  
  Right(oneBlock/2);
  unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  if(AlphaL_time > 0 && AlphaL_time < 800){
    Right(oneBlock/2);
    x_pos = 1;
    y_pos = 0;
  }
  else{
    Forward(oneBlock);
    Right(oneBlock/2);
    Knock();
    x_pos = 1;
    y_pos = 1;
  }

/**************************BEGIN GRID SEARCH************************************/  
  
  while (row < 5 || col < 5)            //ends upon arrival at F2 (row 5, col 5)
{
  if (col % 2 == 1 && row < 5)        //odd column, go forward
  {
    Go_to( col , row + 1 );
//    if (row < 
  }
  else if (col % 2 == 1 && row == 5)   //top of odd column, go right
  {
    Go_to( col + 1 , row );
  }
  else if (col %2 == 0 && row > 1)    //even column, go backward
  {
    Go_to( col , row - 1 );
  }
  else if (col %2 == 0 && row == 1)    //bottom of even column, go right
  {
    Go_to( col + 1 , row );
  }
}
/*******************************END GRID SEARCH***************************/

//   Go_to(1,2);
//   Knock();
//   Go_to(1,3);
//   Knock();
//   Go_to(1,4);
//   Knock();
//   Go_to(1,5);




//  Go_to(1,1);
//  Knock();
//  Go_to(1,2);
//  Knock();
//Right(oneBlock);
/*
  //block(0,0)
  Start(oneBlock);

  //block(1,1)
  align_Bravo(1);
  offset_Bravo(1);
  offset_Charlie(1);
  align_Charlie(1);
  Knock();
  
  if(hollow){
    if(wire){
      matrix.drawPixel(x, y, LED_RED);
      matrix.writeDisplay();
      delay(500);
    }
    else(){
      matrix.drawPixel(x, y, LED_GREEN);
      matrix.writeDisplay();
      delay(500);
    }
  }
  
  Forward(oneBlock);
  align_Bravo(1);
  Knock();
  matrix.drawPixel(y, x, LED_GREEN);
  matrix.writeDisplay();
  delay(500);
  Forward(oneBlock);

  //block(1,3)
  align_Bravo(1);
  offset_Alpha(3);
  offset_Bravo(1);
  align_Bravo(1);
  Knock();
  Forward(oneBlock);
  align_Bravo(1);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Forward(oneBlock);

  //block(1,5)
  align_Bravo(1);
  offset_Bravo(1);
  offset_Alpha(1);
  align_Alpha(1);
  Knock();
  Right(oneBlock);
  align_Alpha(1);
  offset_Bravo(2);

  //block(2,5)
  align_Alpha(1);
  offset_Bravo(2);
  offset_Alpha(1);
  align_Alpha(1);
  Knock();
  Backward(oneBlock);
  align_Bravo(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Backward(oneBlock);
  
  //block(2,3)
  align_Bravo(2);
  offset_Alpha(3);
  offset_Bravo(2);
  align_Bravo(2);
  Knock();
  Backward(oneBlock);
  align_Bravo(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Backward(oneBlock);

  //block(2,1)
  align_Charlie(1);
  offset_Bravo(2);
  offset_Charlie(1);
  align_Charlie(1);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Right(oneBlock);
  align_Charlie(1);
  offset_Bravo(3);


  //block(3,1)
  align_Charlie(1);
  offset_Bravo(3);
  offset_Charlie(1);
  align_Charlie(1);
  Knock();
  Forward(oneBlock);
  
  //block(3,2)
  align_Charlie(2);
  offset_Delta(3);
  offset_Charlie(2);
  align_Charlie(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Forward(oneBlock);
  align_Bravo(3);
  Knock();
  Forward(oneBlock);

  //Block(3,4)
  align_Alpha(2);
  offset_Bravo(3);
  offset_Alpha(2);
  align_Alpha(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Forward(oneBlock);

  //block(3,5)
  align_Alpha(1);
  offset_Delta(3);
  offset_Alpha(1);
  align_Alpha(1);
  Knock();
  Right(oneBlock);
  align_Alpha(1);
  offset_Delta(2);

  //block(4,5)
  align_Alpha(1);
  offset_Delta(2);
  offset_Alpha(1);
  align_Alpha(1);
  Knock();
  Backward(oneBlock);
  align_Delta(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Backward(oneBlock);

  //block(4,3)
  align_Delta(2);
  offset_Alpha(3);
  offset_Delta(2);
  align_Delta(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Backward(oneBlock);
  align_Delta(2);
  Knock();
  matrix.drawPixel(y, x, LED_RED);
  matrix.writeDisplay();
  delay(500);
  Backward(oneBlock);

  //block(4,1)
  align_Charlie(1);
  offset_Delta(2);
  offset_Charlie(1);
  align_Charlie(1);
  Knock();
  Right(oneBlock);
  align_Charlie(1);
  offset_Delta(1);

  //block(5,1)
  align_Delta(1);
  offset_Delta(1);
  offset_Charlie(1);
  align_Charlie(1);
  Knock();
  Forward(oneBlock);
  align_Delta(1);
  Knock();
  Forward(oneBlock);

  //block(5,3)
  align_Delta(1);
  offset_Alpha(3);
  offset_Delta(1);
  align_Delta(1);
  Knock();
  Forward(oneBlock);
  align_Delta(1);
  Knock();
  Forward(oneBlock);

  //block(5,5)
  align_Delta(1);
  offset_Delta(1);
  offset_Alpha(1);
  align_Alpha(1);
  Knock();
  End(oneBlock);

  //block(4,4)
  align_Delta(2);
  offset_Delta(2);
  offset_Alpha(2);
  align_Alpha(2);
  End(oneBlock*2.2);

  //block(2,2)
  align_Bravo(2);
  offset_Bravo(2);
  offset_Charlie(2);
  align_Charlie(2);
  End(oneBlock*2.2);

  //block(0,0)
  align_Bravo(1);

  */

  
  
  
}

/**********************************************************************START OF FUNCTIONS(DANCE MOVES)****************************************************************************************/

//knock funtion that knocks and returns
void Knock(){
  digitalWrite(knocker,HIGH);   delay(200);//open mosfet to drive solenoid
  digitalWrite(knocker,LOW);    delay(200);//close mosfet to return solenoid
}

void Camera(){
  align_Alpha(1);
  M1Motor->setSpeed(M1Speed); M3Motor->setSpeed(M3Speed);
  int goal = 1123;//goal distance based on multiple parameter from blocks between wall
  unsigned int AlphaR_time = AlphaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction
  int differenceR = AlphaR_time - goal;
  int differenceL = AlphaL_time - goal;
  differenceR = abs(differenceR);
  differenceL = abs(differenceL);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){
    AlphaR_time = AlphaR.ping(); // Send ping, get ping time in microseconds (uS).
    AlphaL_time = AlphaL.ping(); // Send ping, get ping time in microseconds (uS)
    //difference from goal and absolute value for error correction
    differenceR = AlphaR_time - goal;
    differenceL = AlphaL_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall
    differenceR = abs(differenceR);
    differenceL = abs(differenceL);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0){
      M1Motor->run(FORWARD);  M3Motor->run(BACKWARD);
    }
    if(movement<0){
      M1Motor->run(BACKWARD);  M3Motor->run(FORWARD);
    }  
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
align_Alpha(1);
  
}

/*
void Camera(){
  M1Motor->setSpeed(M1Speed); M2Motor->setSpeed(M2Speed); 
  M3Motor->setSpeed(M3Speed); M4Motor->setSpeed(M4Speed);
  while(finish_pin == LOW){
    if(forward_pin == HIGH){
      M1Motor->run(FORWARD);  M3Motor->run(BACKWARD);
    }
    if(backward_pin == HIGH){
      M1Motor->run(BACKWARD); M3Motor->run(FORWARD);
    }
    if(right_pin == HIGH){
      M2Motor->run(FORWARD);  M4Motor->run(BACKWARD);
    }
    if(left_pin == HIGH){
      M2Motor->run(BACKWARD); M4Motor->run(FORWARD);
    }
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
}

*/

//Move Right 12" so many Blocks times
void Right(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M2tick = 0; M4tick = 0; //encoder counts
  M2Motor->run(FORWARD);  M4Motor->run(BACKWARD); //start motors moving
  //loop for distance based on amount of encoder ticks
  while(M2tick<Blocks && M4tick<Blocks){  
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<200; i++){
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M2tick);
    Serial.print(M4tick);
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  // turn off motors
  x_pos++;
  delay(200);
}

//Move Left 12" so many Blocks times
void Left(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M2tick = 0; M4tick = 0; //encoder counts
  M2Motor->run(BACKWARD); M4Motor->run(FORWARD);  //start motors moving
  //loop for distance based on amount of encoder ticks
  while(M2tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<200; i++){
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M2tick);
    Serial.print(M4tick);
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
  x_pos--;
  delay(200);
}

//Move Backward 12" so many Blocks times
void Backward(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M1tick = 0; M3tick = 0; //encoder counts
  M1Motor->run(BACKWARD); M3Motor->run(FORWARD);  //start motors moving
  //loop for distance based on amount of encoder ticks
  while(M1tick<Blocks && M3tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<200; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M1tick);
    Serial.print(M3tick);
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  y_pos--;
  delay(200);
}

//Move Forward 12" so many Blocks times
void Forward(int Blocks){
  unsigned int AlphaR_time = AlphaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  delay(200);
  unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  delay(200);
  int A_time = (AlphaR_time + AlphaL_time)/2;
  if(A_time>0 && A_time<800){
    obstaclex[obnum] = x_pos;
    obstacley[obnum] = y_pos++;
    
    if(x_pos>3 && y_pos>3){
      align_Delta(1);
      Left(oneBlock);
      Forward(oneBlock);
      align_Delta(1);
      //offset_Delta(obstacle);
      align_Delta(1);
      if(y_pos == 5){
        //last block return home
    }
    if(x_pos<4 && y_pos<4){
    align_Bravo(1);
    Right(oneBlock);
    Forward(oneBlock);
    align_Bravo(1);
    //offset_Bravo(obstacle);
    align_Bravo(1);
    Forward(oneBlock);
    Left(oneBlock);
    align_Charlie(1);
    //offset_Charlie(obstacle);
    align_Charlie(1);
    }

    if(x_pos<4 && y_pos>3){
      
    }
      
    }
  }
  Blocks = Blocks;
  int i = 0;
  M1tick = 0; M3tick = 0; //encoder counts
  M1Motor->run(FORWARD);  M3Motor->run(BACKWARD); //start motors moving
  //loop for distance based on amount of encoder ticks
  while(M1tick<Blocks && M3tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<200; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M1tick);
    Serial.print(M3tick);
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  y_pos++;
  delay(200);
}

//Slide Movement for leaving home space
void Start(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M1tick = 0; M2tick = 0; M3tick = 0; M4tick = 0; //encoder counts
  //start motors moving
  M1Motor->run(FORWARD); M3Motor->run(BACKWARD);
  M2Motor->run(FORWARD); M4Motor->run(BACKWARD);
  //loop for distance based on amount of encoder ticks
  while(M1tick<Blocks && M2tick<Blocks && M3tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<200; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M1tick);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
  //x++;
 // y++;
  delay(200);
}

void End(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M1tick = 0; M2tick = 0; M3tick = 0; M4tick = 0; //encoder counts
  //start motors moving
  M1Motor->run(BACKWARD); M3Motor->run(FORWARD);
  M2Motor->run(BACKWARD); M4Motor->run(FORWARD);
  //loop for distance based on amount of encoder ticks
  while(M1tick<Blocks && M2tick<Blocks && M3tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<200; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M1tick);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
  delay(200);
}



//Rotate 90 degrees clockwise so many times
void turnCW(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M1tick = 0; M2tick = 0; M3tick = 0; M4tick = 0; //encoder counts
  //start motors moving
  M1Motor->run(FORWARD);  M2Motor->run(FORWARD);
  M3Motor->run(FORWARD); M4Motor->run(FORWARD);
  while(M1tick<Blocks && M2tick<Blocks && M3tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=40; i<100; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delay(10);
      }
    }
    Serial.print(M1tick);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);  
  delay(100);
}

//Rotate 90 degrees counter-clockwise so many times
void turnCCW(int Blocks){
  Blocks = Blocks;
  int i = 0;
  M1tick = 0; M2tick = 0; M3tick = 0; M4tick = 0; //encoder counts
  //start motors moving
  M1Motor->run(BACKWARD); M2Motor->run(BACKWARD);
  M3Motor->run(BACKWARD); M4Motor->run(BACKWARD);
  while(M1tick<Blocks && M2tick<Blocks && M3tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=40; i<100; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delay(10);
      }
    }
    Serial.print(M1tick);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
  delay(100);
}

//adjust offset from wall based on ultrasonic readings
void offset_Alpha(int multiple){
  M1Motor->setSpeed(M1Speed); M3Motor->setSpeed(M3Speed);
  int goal = multiple*1780 + 287;//goal distance based on multiple parameter from blocks between wall
  unsigned int AlphaR_time = AlphaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction
  int differenceR = AlphaR_time - goal;
  int differenceL = AlphaL_time - goal;
  differenceR = abs(differenceR);
  differenceL = abs(differenceL);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){
    AlphaR_time = AlphaR.ping(); // Send ping, get ping time in microseconds (uS).
    AlphaL_time = AlphaL.ping(); // Send ping, get ping time in microseconds (uS)
    //difference from goal and absolute value for error correction
    differenceR = AlphaR_time - goal;
    differenceL = AlphaL_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall
    differenceR = abs(differenceR);
    differenceL = abs(differenceL);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0){
      M1Motor->run(FORWARD);  M3Motor->run(BACKWARD);
    }
    if(movement<0){
      M1Motor->run(BACKWARD);  M3Motor->run(FORWARD);
    }  
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors

  
}


//adjust offset from wall based on ultrasonic readings
void offset_Bravo(int multiple){
  M2Motor->setSpeed(M2Speed); M4Motor->setSpeed(M4Speed); 
  int goal = multiple*1780 + 215; //goal distance based on multiple parameter from blocks between wall
  unsigned int BravoR_time = BravoR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int BravoL_time = BravoL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction
  int differenceR = BravoR_time - goal;
  int differenceL = BravoL_time - goal;
  differenceR = abs(differenceR);
  differenceL = abs(differenceL);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){
    BravoR_time = BravoR.ping(); // Send ping, get ping time in microseconds (uS).
    BravoL_time = BravoL.ping(); // Send ping, get ping time in microseconds (uS)
    //difference from goal and absolute value for error correction
    differenceR = BravoR_time - goal;
    differenceL = BravoL_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall
    differenceR = abs(differenceR);
    differenceL = abs(differenceL);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0){
      M2Motor->run(BACKWARD);  M4Motor->run(FORWARD);
    }
    if(movement<0){
      M2Motor->run(FORWARD);  M4Motor->run(BACKWARD);
    }  
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
}


//adjust offset from wall based on ultrasonic readings
void offset_Charlie(int multiple){
  M1Motor->setSpeed(M1Speed); M3Motor->setSpeed(M3Speed);
  int goal = multiple*1780 + 287;//goal distance based on multiple parameter from blocks between wall
  unsigned int CharlieR_time = CharlieR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int CharlieL_time = CharlieL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction
  int differenceR = CharlieR_time - goal;
  int differenceL = CharlieL_time - goal;
  differenceR = abs(differenceR);
  differenceL = abs(differenceL);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){
    CharlieR_time = CharlieR.ping(); // Send ping, get ping time in microseconds (uS).
    CharlieL_time = CharlieL.ping(); // Send ping, get ping time in microseconds (uS)
    //difference from goal and absolute value for error correction
    differenceR = CharlieR_time - goal;
    differenceL = CharlieL_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall
    differenceR = abs(differenceR);
    differenceL = abs(differenceL);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0){
      M1Motor->run(BACKWARD);  M3Motor->run(FORWARD);
    }
    if(movement<0){
      M1Motor->run(FORWARD);  M3Motor->run(BACKWARD);
    }  
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
}

//adjust offset from wall based on ultrasonic readings
void offset_Delta(int multiple){
  M2Motor->setSpeed(M2Speed); M4Motor->setSpeed(M4Speed); 
  int goal = multiple*1780 + 287; //goal distance based on multiple parameter from blocks between wall
  unsigned int DeltaR_time = DeltaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int DeltaL_time = DeltaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction
  int differenceR = DeltaR_time - goal;
  int differenceL = DeltaL_time - goal;
  differenceR = abs(differenceR);
  differenceL = abs(differenceL);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){
    DeltaR_time = DeltaR.ping(); // Send ping, get ping time in microseconds (uS).
    DeltaL_time = DeltaL.ping(); // Send ping, get ping time in microseconds (uS)
    //difference from goal and absolute value for error correction
    differenceR = DeltaR_time - goal;
    differenceL = DeltaL_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall
    differenceR = abs(differenceR);
    differenceL = abs(differenceL);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0){
      M2Motor->run(FORWARD);  M4Motor->run(BACKWARD);
    }
    if(movement<0){
      M2Motor->run(BACKWARD);  M4Motor->run(FORWARD);
    }  
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
}

//align when not parrallel with wall
void align_Alpha(int multiple){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int AlphaR_time = AlphaR.ping_median(5);
  unsigned int AlphaL_time = AlphaL.ping_median(5);
           int difference  = AlphaR_time - AlphaL_time;
               difference  = abs(difference);
  //loop that turns until parrallel with wall  
           int goal        = 10*multiple;    
           int cw          = 0; 
           int ccw         = 0;      
  while(difference > goal){
    AlphaR_time = AlphaR.ping();
    AlphaL_time = AlphaL.ping();
    difference  = AlphaR_time - AlphaL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){
        break;
      }
      M1Motor->run(FORWARD); M2Motor->run(FORWARD);
      M3Motor->run(FORWARD); M4Motor->run(FORWARD);
    }
    if(difference<0){
      ccw = 1;
      M1Motor->run(BACKWARD); M2Motor->run(BACKWARD);
      M3Motor->run(BACKWARD); M4Motor->run(BACKWARD);
    }
    difference = abs(difference);
  }
  //turn off motors
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
}

//align when not parrallel with wall
void align_Bravo(int multiple){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int BravoR_time    = BravoR.ping_median(5);
  unsigned int BravoL_time    = BravoL.ping_median(5);
           int difference     = BravoR_time - BravoL_time;
               difference     = abs(difference);         
  //loop that turns until parrallel with wall
           int goal           = 10*multiple;  
           int cw             = 0;
           int ccw            = 0;           
  while(difference > goal){
    BravoR_time = BravoR.ping();
    BravoL_time = BravoL.ping();
    difference  = BravoR_time - BravoL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){
        break;
      }
      M1Motor->run(FORWARD); M2Motor->run(FORWARD);
      M3Motor->run(FORWARD); M4Motor->run(FORWARD);
    }
    if(difference<0){
      ccw = 1;
      M1Motor->run(BACKWARD); M2Motor->run(BACKWARD);
      M3Motor->run(BACKWARD); M4Motor->run(BACKWARD);
    }
    difference = abs(difference);
  }
  //turn off motors
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
}

//align when not parrallel with wall
void align_Charlie(int multiple){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int CharlieR_time = CharlieR.ping_median(5);
  unsigned int CharlieL_time = CharlieL.ping_median(5);
           int difference  = CharlieR_time - CharlieL_time;
           int goal        = 10*multiple;
           int cw          = 0;
           int ccw         = 0;
               difference  = abs(difference);
  //loop that turns until parrallel with wall             
  while(difference > goal){
    CharlieR_time = CharlieR.ping();
    CharlieL_time = CharlieL.ping();
    difference  = CharlieR_time - CharlieL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){
        break;
      }
      M1Motor->run(FORWARD); M2Motor->run(FORWARD);
      M3Motor->run(FORWARD); M4Motor->run(FORWARD);
    }
    if(difference<0){
      ccw = 1;
      M1Motor->run(BACKWARD); M2Motor->run(BACKWARD);
      M3Motor->run(BACKWARD); M4Motor->run(BACKWARD);
    }
    difference = abs(difference);
  }
  //turn off motors
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
}

//align when not parrallel with wall
void align_Delta(int multiple){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int DeltaR_time = DeltaR.ping_median(5);
  unsigned int DeltaL_time = DeltaL.ping_median(5);
           int difference  = DeltaR_time - DeltaL_time;
           int goal        = 10*multiple;
           int cw          = 0;
           int ccw         = 0;
               difference  = abs(difference);
  //loop that turns until parrallel with wall             
  while(difference > goal){
    DeltaR_time = DeltaR.ping();
    DeltaL_time = DeltaL.ping();
    difference  = DeltaR_time - DeltaL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){
        break;
      }
      M1Motor->run(FORWARD); M2Motor->run(FORWARD);
      M3Motor->run(FORWARD); M4Motor->run(FORWARD);
    }
    if(difference<0){
      ccw = 1;
      M1Motor->run(BACKWARD); M2Motor->run(BACKWARD);
      M3Motor->run(BACKWARD); M4Motor->run(BACKWARD);
    }
    difference = abs(difference);
  }
  //turn off motors
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
}

void Go_to(int x_finish, int y_finish){
  Serial.println("inloop");
  //int y_finish = 0;
  int x_difference = x_finish - x_pos;
  int y_difference = y_finish - y_pos;
  while(y_difference != 0){
    Serial.println("y_loop");
    Serial.println(y_difference);
    while(x_difference != 0){
      Serial.println("x_loop");
      Serial.println(x_difference);
      if(x_difference>0){
        align_Bravo(x_pos);
        Right(oneBlock);
        offset_Bravo(x_pos);
        offset_Charlie(y_pos);
        align_Bravo(x_pos);
      }
      if(x_difference<0){
        align_Bravo(x_pos);
        Left(oneBlock);
        offset_Bravo(x_pos);
        offset_Charlie(y_pos);
        align_Bravo(x_pos);
      }
      Serial.println(x_pos);
      Serial.println(x_difference);
      x_difference = x_finish - x_pos;
    }
    if(y_difference>0){
      align_Bravo(x_pos);
      Forward(oneBlock);
      offset_Bravo(x_pos);
      offset_Charlie(y_pos);
      align_Bravo(x_pos);
    }
    if(y_difference<0){
      align_Bravo(x_pos);
      Backward(oneBlock);
      offset_Bravo(x_pos);
      offset_Charlie(y_pos);
      align_Bravo(x_pos);
    }
    y_difference = y_finish - y_pos;
  }
}

void knowledge(){
  int offsetx_final = 6;
  int offsety_final = 6;
  int differencex_final = 0;
  int differencey_final = 0;
  for(int i=0; i<obnum; i++){
    if(y_pos == obstacley[i]){
      int x_offset = obstaclex[i];
      int offsetx_difference = x_offset - x_pos;
      int offsetx_change = abs(offsetx_difference);
      if(offsetx_change<offsetx_final){
        offsetx_final = offsetx_change;
        differencex_final = offsetx_difference;
      }
    }
    if(x_pos == obstaclex[i]){
      int y_offset = obstacley[i];
      int offsety_difference = offsety_difference - y_pos;
      int offsety_change = abs(offsety_difference);
      if(offsety_change<offsety_final){
        offsety_final = offsety_change;
        differencey_final = offsety_difference;
      }
    }
  }
  if(offsetx_final < 6){
    if(differencex_final<0){
      align_Bravo(1);
      offset_Bravo(offsetx_final);
      align_Bravo(1);
    }
    if(differencex_final>0){
      align_Delta(1);
      offset_Delta(offsetx_final);
      align_Delta(1);
    }
  }
  if(offsety_final < 6){
    if(differencey_final<0){
      align_Charlie(1);
      offset_Charlie(offsety_final);
      align_Charlie(1);
    }
    if(differencey_final>0){
      align_Alpha(1);
      offset_Alpha(offsety_final);
      align_Alpha(1);
    }
      
  }
}

 
/**************************************************************************************INTERRUPT FUNCTIONS************************************************************************************/
 //functions that will count the rising edge of interrupts for each motor
void M1count(){
  M1tick++;}
void M2count(){
  M2tick++;}
void M3count(){
  M3tick++;}
void M4count(){
  M4tick++;}

/****************************************************************************************CONTINUOUS LOOP**************************************************************************************/

void loop() {

}
