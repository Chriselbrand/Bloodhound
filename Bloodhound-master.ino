/**********************************************************************************************************************************************************************************************
*********************************************************SLAM-BOT'S BLOODHOUND PROJECT MOTION FUNCTION PROGRAM*********************************************************************************
********************************************************KEMP HARTZOG, CHRIS HARRIS, THOMAS MILLER, CORY LANDETA********************************************************************************
**********************************************************************************************************************************************************************************************/

/***************************************************************************LIBRARIES & SETUP*********************************************************************************************************/

#include <Wire.h>

//matrix library
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

//create matrix object
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

//Ultrasonics library
#include <NewPing.h>

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
#define AlphaR_TRIGGER     28  
#define AlphaR_ECHO        29 
NewPing AlphaR(AlphaR_TRIGGER, AlphaR_ECHO, MAX_DISTANCE);  
#define AlphaL_TRIGGER     30  
#define AlphaL_ECHO        31 
NewPing AlphaL(AlphaL_TRIGGER, AlphaL_ECHO, MAX_DISTANCE);

//Delta side trigger and echo
#define DeltaR_TRIGGER     32  
#define DeltaR_ECHO        33 
NewPing DeltaR(DeltaR_TRIGGER, DeltaR_ECHO, MAX_DISTANCE);  
#define DeltaL_TRIGGER     34  
#define DeltaL_ECHO        35  
NewPing DeltaL(DeltaL_TRIGGER, DeltaL_ECHO, MAX_DISTANCE);

//Charlie side triggera and echo
#define CharlieR_TRIGGER   36  
#define CharlieR_ECHO      37  
NewPing CharlieR(CharlieR_TRIGGER, CharlieR_ECHO, MAX_DISTANCE);  
#define CharlieL_TRIGGER   22  
#define CharlieL_ECHO      23
NewPing CharlieL(CharlieL_TRIGGER, CharlieL_ECHO, MAX_DISTANCE);  

//definitions of map matrix designators
#define UNKNOWN             0
#define VOID                1
#define OBJECTIVE           2
#define DEAD_END            3

//motor shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *M1Motor = AFMS.getMotor(1); Adafruit_DCMotor *M2Motor = AFMS.getMotor(2);
Adafruit_DCMotor *M3Motor = AFMS.getMotor(3); Adafruit_DCMotor *M4Motor = AFMS.getMotor(4);

//Setup variables for moving one block and initial speed
int oneBlock = 306;  int Ninety  = 24;
int M1Speed  =  50;  int M2Speed = 50; 
int M3Speed  =  50;  int M4Speed = 50;

//interrupt pins and motor ticks count
int M1tick = 0; int M2tick = 0; int M3tick= 0 ; int M4tick = 0;
#define M1interrupt 19 
#define M2interrupt 18
#define M3interrupt  3 
#define M4interrupt  2

//servo library
#include <Servo.h>

//declare the grabber as servo
Servo grabber;

//initialize grabber position to be used in code
//#define grabber_pin 40
int grabber_start    =  75;
int grabber_cache    =   0;
int grabber_finished = 180;

//Knocker pin
#define knocker 7

//tic tracer pin
#define tic 12

/*****************************************************************************INITIALIZATION**************************************************************************************************/

//initialize x,y cordinates
int x_pos = 0;        int y_pos = 0;  

//arrays to use for obstacles
int obstaclex[10];    int obstacley[10];
int obnum;// number of obstacles avoided

//arrays to use for determining under obstacle
int map[10][10];

/**************************************************************************START OF PROGRAM***************************************************************************************************/

//Line dancing moves
void setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz

  //start matrix and ready light
  matrix.begin(0x71); // matrix at 71 address
  matrix.clear();
  matrix.drawPixel(0, 0, LED_YELLOW);
  matrix.writeDisplay();  delay(500);

  //start tic tracer
  pinMode(tic, OUTPUT);
  digitalWrite(tic, HIGH);
  delay(500);
  digitalWrite(tic, LOW);
  
  //knocker setup
  pinMode(knocker, OUTPUT);
  
  //initalize grabber pin and set to start position
  //grabber.attach(grabber_pin);
  grabber.write(grabber_start);
  
  //Set speed that will be used
  M1Motor->setSpeed(M1Speed); M2Motor->setSpeed(M2Speed); 
  M3Motor->setSpeed(M3Speed); M4Motor->setSpeed(M4Speed);

  //setup for motor interrupt pins
  pinMode(M1interrupt, INPUT_PULLUP); pinMode(M2interrupt, INPUT_PULLUP);
  pinMode(M3interrupt, INPUT_PULLUP); pinMode(M4interrupt, INPUT_PULLUP);

  //interrupt function attachment and call modes
  attachInterrupt(digitalPinToInterrupt(M1interrupt),M1count,RISING);
  attachInterrupt(digitalPinToInterrupt(M2interrupt),M2count,RISING);
  attachInterrupt(digitalPinToInterrupt(M3interrupt),M3count,RISING);
  attachInterrupt(digitalPinToInterrupt(M4interrupt),M4count,RISING);

  //port setup for printing
  Serial.begin(57600);
  
  //start program that will determine if obstacle in (1,1) position
  Right(oneBlock/2);
  unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //if block found
  if(AlphaL_time > 0 && AlphaL_time < 800){
    Right(oneBlock/2);
    align_Bravo(1);
    x_pos = 1;        y_pos = 0;
  }
  //no block
  else{
    align_Bravo(1);
    Forward(oneBlock);
    Right(oneBlock/2);
    offset_Bravo(1);
    offset_Charlie(1);
    align_Bravo(1);
    x_pos = 1;        y_pos = 1;
    Knock();    
    matrix.drawPixel(y_pos, x_pos, LED_RED);
    matrix.writeDisplay();       delay(500);
  }

/**************************BEGIN GRID SEARCH************************************/  
  
  while (x_pos < 5 || y_pos < 5)            //ends upon arrival at F2 (row 5, col 5)
{
  if (x_pos % 2 == 1 && y_pos < 5)        //odd column, go forward
  {
    ////update obstacle matrix in front and to right
    ////check obstacle matrix for next block
    Go_to( x_pos , y_pos + 1 );
    Knock();
    //int currentBlock = ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    map[x_pos][y_pos] = currentBlock
    matrix.drawPixel(y_pos, x_pos, LED_RED);
    matrix.writeDisplay();       delay(500);
//    if (row < 
  }
  else if (x_pos % 2 == 1 && y_pos == 5)   //top of odd column, go right
  {
    Go_to( x_pos + 1 , y_pos );
    Knock();
    //int currentBlock = ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    map[x_pos][y_pos] = currentBlock
    matrix.drawPixel(y_pos, x_pos, LED_GREEN);
    matrix.writeDisplay();         delay(500);
  }
  else if (x_pos %2 == 0 && y_pos > 1)    //even column, go backward
  {
    Go_to( x_pos , y_pos - 1 );
    Knock();
    //int currentBlock = ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    map[x_pos][y_pos] = currentBlock
    matrix.drawPixel(y_pos, x_pos, LED_GREEN);
    matrix.writeDisplay();         delay(500);
  }
  else if (x_pos %2 == 0 && y_pos == 1)    //bottom of even column, go right
  {
    Go_to( x_pos + 1 , y_pos );
    Knock();
    //int currentBlock = ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    map[x_pos][y_pos] = currentBlock
    matrix.drawPixel(y_pos, x_pos, LED_RED);
    matrix.writeDisplay();       delay(500);
  }
}
/*******************************END GRID SEARCH***************************/
  
  //go to, uncover, and read cache die
  
  //return function since last space was (5,5) we are finished
  
}
  
/**********************************************************************START OF FUNCTIONS(DANCE MOVES)****************************************************************************************/

//knock funtion that knocks and returns
void Knock(){
  digitalWrite(knocker,HIGH);   delay(200);//open mosfet to drive solenoid
  digitalWrite(knocker,LOW);    delay(200);//close mosfet to return solenoid
}
// function for placement of camera to cache lid
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


//Move Right 12" so many Blocks times
void Right(int Blocks){
  //Blocks = Blocks;
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
    Serial.print(M2tick);     Serial.print(M4tick);
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  // turn off motors
  x_pos++;
  delay(200);
}

//Move Left 12" so many Blocks times
void Left(int Blocks){
  //Blocks = Blocks;
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
    Serial.print(M2tick);   Serial.print(M4tick);
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
  x_pos--;
  delay(200);
}

//Move Backward 12" so many Blocks times
void Backward(int Blocks){
  //Blocks = Blocks;
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
    Serial.print(M1tick);   Serial.print(M3tick);
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
    obstaclex[x_pos] = 1;
    obstacley[y_pos++] = 1;
    
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
  //Blocks = Blocks;
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
    Serial.print(M1tick); Serial.print(M3tick);
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  y_pos++;
  delay(200);
}

//Rotate 90 degrees clockwise so many times
void turnCW(int Blocks){
  //Blocks = Blocks;
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
  //Blocks = Blocks;
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
  int goal = multiple*1780 + 215;//goal distance based on multiple parameter from blocks between wall
  unsigned int AlphaR_time = AlphaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction  
  int differenceL = AlphaL_time - goal; int differenceR = AlphaR_time - goal;  
      differenceL = abs(differenceL);       differenceR = abs(differenceR);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){   
    AlphaL_time = AlphaL.ping();            AlphaR_time = AlphaR.ping(); 
    differenceL = AlphaL_time - goal;       differenceR = AlphaR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall    
    differenceL = abs(differenceL);         differenceR = abs(differenceR);
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
  int differenceL = BravoL_time - goal; int differenceR = BravoR_time - goal;
  differenceL = abs(differenceL);           differenceR = abs(differenceR);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){   
    BravoL_time = BravoL.ping();            BravoR_time = BravoR.ping();
    differenceL = BravoL_time - goal;       differenceR = BravoR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall   
    differenceL = abs(differenceL);         differenceR = abs(differenceR);
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
  int goal = multiple*1780 + 215;//goal distance based on multiple parameter from blocks between wall
  unsigned int CharlieR_time = CharlieR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int CharlieL_time = CharlieL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction
  int differenceL = CharlieL_time - goal; int differenceR = CharlieR_time - goal;
  differenceL = abs(differenceL);             differenceR = abs(differenceR);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){    
    CharlieL_time = CharlieL.ping();        CharlieR_time = CharlieR.ping(); 
    differenceL = CharlieL_time - goal;       differenceR = CharlieR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall    
    differenceL = abs(differenceL);           differenceR = abs(differenceR);
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
  int goal = multiple*1780 + 215; //goal distance based on multiple parameter from blocks between wall
  unsigned int DeltaR_time = DeltaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int DeltaL_time = DeltaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction 
  int differenceL = DeltaL_time - goal; int differenceR = DeltaR_time - goal;
  differenceL = abs(differenceL);           differenceR = abs(differenceR);
  //loop for error correction
  while(differenceR > 50 && differenceL > 50){    
    DeltaL_time = DeltaL.ping();            DeltaR_time = DeltaR.ping();    
    differenceL = DeltaL_time - goal;       differenceR = DeltaR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall   
    differenceL = abs(differenceL);         differenceR = abs(differenceR);
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
           int ccw         = 0;      
  while(difference > goal){   
    AlphaL_time = AlphaL.ping();   AlphaR_time = AlphaR.ping();
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
           int ccw            = 0;           
  while(difference > goal){   
    BravoL_time = BravoL.ping();  BravoR_time = BravoR.ping();
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
           int ccw         = 0;
               difference  = abs(difference);
  //loop that turns until parrallel with wall             
  while(difference > goal){    
    CharlieL_time = CharlieL.ping();  CharlieR_time = CharlieR.ping();
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
           int ccw         = 0;
               difference  = abs(difference);
  //loop that turns until parrallel with wall             
  while(difference > goal){    
    DeltaL_time = DeltaL.ping();  DeltaR_time = DeltaR.ping();
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

//function that will go to an x,y position based on your current x,y position
void Go_to(int x_finish, int y_finish){
  Serial.println("inloop");
  int x_difference = x_finish - x_pos;
  int y_difference = y_finish - y_pos;
  while(x_difference != 0){
    Serial.println("x_loop");
    Serial.println(x_difference);
    if(x_difference>0){
      if(x_pos<3){
        align_Bravo(x_pos);
        Right(oneBlock);
        offset_Bravo(x_pos);
        align_Bravo(x_pos);
      }
      else{
        align_Delta(6 - x_pos);
        Right(oneBlock);
        offset_Delta(6 - x_pos);
        align_Delta(6 - x_pos);
      }
    }
    if(x_difference<0){
      if(x_pos<3){
        align_Bravo(x_pos);
        Left(oneBlock);
        offset_Bravo(x_pos);
        align_Bravo(x_pos);
      }
      else{
        align_Delta(6 - x_pos);
        Left(oneBlock);
        offset_Delta(6 - x_pos);
        align_Delta(6 - x_pos);
      }
    }
      x_difference = x_finish - x_pos;
    }
  while(y_difference != 0){
    Serial.println("y_loop");
    Serial.println(y_difference);
    
    if(y_difference>0){
      if(y_pos<3){
        align_Charlie(y_pos);
        Forward(oneBlock);
        offset_Charlie(y_pos);
        align_Charlie(y_pos);
      }
      else{
        align_Alpha(6 - y_pos);
        Forward(oneBlock);
        offset_Alpha(6 - y_pos);
        align_Alpha(6 - y_pos);
      }
    }
    if(y_difference<0){
      if(y_pos<4){
        align_Charlie(y_pos);
        Backward(oneBlock);
        offset_Charlie(y_pos);
        align_Charlie(y_pos);
      }
      else{
        align_Alpha(6 - y_pos);
        Backward(oneBlock);
        offset_Alpha(6 - y_pos);
        align_Alpha(6 - y_pos);
      }
    }
    y_difference = y_finish - y_pos;
  }
}

  // function that will determine the number needed for offset and align based on obstacles
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
void M1count(){M1tick++;}
void M2count(){M2tick++;}
void M3count(){M3tick++;}
void M4count(){M4tick++;}

/****************************************************************************************CONTINUOUS LOOP**************************************************************************************/

void loop() {

}
