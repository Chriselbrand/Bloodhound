/**********************************************************************************************************************************************************************************************
*********************************************************SLAM-BOT'S BLOODHOUND PROJECT MOTION FUNCTION PROGRAM*********************************************************************************
********************************************************KEMP HARTZOG, CHRIS HARRIS, THOMAS MILLER, CORY LANDETA********************************************************************************
**********************************************************************************************************************************************************************************************/

/***************************************************************************LIBRARIES & SETUP*********************************************************************************************************/

#include <Wire.h>  //i2c communication

//neo pixel library
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//neopixel setup
#define neopixel   6
#define numpixels 64
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numpixels, neopixel, NEO_GRB + NEO_KHZ800);       
int neo_pos = 0;              
                    
                          //   Kemp <-> Cory
#define UP       53       //   blue <-> blue
#define DOWN     51       //  green <-> green
#define LEFT     49       // yellow <-> yellow
#define RIGHT    47       // orange <-> orange
#define TRANSMIT 43       //  brown <-> brown
#define RECEIVE  45       //    red <-> red

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

//motor shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *M1Motor = AFMS.getMotor(1); Adafruit_DCMotor *M2Motor = AFMS.getMotor(2);
Adafruit_DCMotor *M3Motor = AFMS.getMotor(3); Adafruit_DCMotor *M4Motor = AFMS.getMotor(4);

//Setup variables for moving one block and initial speed
int oneBlock = 306;  int Ninety  = 135;
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
#define grabber_pin 10
int grabber_start    =  45;
int grabber_cache    =   0;
int grabber_finished = 90;

//pin for green start button
#define Starter 39

/*****************************************************************************INITIALIZATION**************************************************************************************************/

//initialize x,y cordinates
int x_pos = 0;        int y_pos = 0;  

/**************************************************************************START OF PROGRAM***************************************************************************************************/

//Line dancing moves
void setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz

  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
    //end of trinket special code
  
  //start neopixel and ready ligh
  pixels.begin();
  pixels.setBrightness(20);
  pixels.setPixelColor(neo_pos,pixels.Color(255,180,0)); //show yellow on (0,0)
  pixels.show();
  delay(500);

  //initalize grabber pin and set to start position
  grabber.attach(grabber_pin );    
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

  //set function and initial value of arduino <-> pi communication pins
  pinMode(RECEIVE,INPUT);      pinMode(TRANSMIT,OUTPUT); 
  pinMode(UP     ,INPUT);      pinMode(DOWN    ,INPUT );
  pinMode(LEFT   ,INPUT);      pinMode(RIGHT   ,INPUT );
  
  //set green button for input
  pinMode(Starter,INPUT);

  //offset_camera();
  grabber.write(180);
  delay(100);
  Cory();
  
  //loop that goes until green button is pressed
  bool dontGO=true;
  int GO=0;
  delay(200);
  while(dontGO){GO=digitalRead(Starter);
  Serial.println(GO);
    if(GO==1){dontGO=false;}
    delay(200);
  }
  Serial.println("start");
  
  //warm up strecth
  Start();    Locate();
  

/**************************BEGIN GRID SEARCH************************************/  

  while(y_pos<5||x_pos<5){            //ends upon arrival at F2 (row 5, col 5)
    //odd column electric slide forward
    if (x_pos%2==1&&y_pos<5)       {Go_to(x_pos,y_pos+1);    
      Calibrate();  Locate();
    }
    //top of odd column, slide to the right
    else if(x_pos%2==1&&y_pos==5)  {Go_to(x_pos+1,y_pos);    
      Calibrate();  Locate();
    }
    //even column, electric slide backward
    else if(x_pos%2==0&&y_pos>1)   {Go_to(x_pos,y_pos-1);    
      Calibrate();  Locate();
    }
    //bottom of even column, slide to the right
    else if(x_pos%2==0&&y_pos==1)  {Go_to(x_pos+1,y_pos);
      Calibrate();  Locate();
    }
  }

  /*******************************END GRID SEARCH***************************/
  
  //Congratulations ET Everythings unlimited with the new TMobile ONE plan
  Home();
}
  
/**********************************************************************START OF FUNCTIONS(DANCE MOVES)****************************************************************************************/

//start program, Move to (1,1)
void Start(){
  Right(oneBlock/2);        align_Bravo();       
  Forward(oneBlock);        align_Delta();    
  Right(oneBlock/2);
  x_pos = 1;   y_pos = 1;   Calibrate();
  }

//return to (0,0) from already known location
void Home(){
  Go_to(1,5);               Calibrate();         
  Go_to(1,1);               Calibrate();         
  Left(oneBlock/2);         align_Delta();
  Backward(oneBlock);  
  Left(oneBlock/2);    
  x_pos = 0;   y_pos = 0;   Calibrate();
  }

//function that will go to an x,y position based on your current x,y position
void Go_to(int x_finish, int y_finish){
  int x_difference = x_finish - x_pos;
  int y_difference = y_finish - y_pos;
  while(x_difference != 0){
    if(x_difference>0)  {Right(oneBlock);}
    else                { Left(oneBlock);}
      x_difference = x_finish - x_pos;
    }
  while(y_difference != 0){
    if(y_difference>0)  { Forward(oneBlock);}
    else                {Backward(oneBlock);}
    y_difference = y_finish - y_pos;
  }
}

//Move Right 12" so many Blocks times
void Right(int Blocks){
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
  delay(50);
}

//Move Left 12" so many Blocks times
void Left(int Blocks){
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
  delay(50);
}

//Move Backward 12" so many Blocks times
void Backward(int Blocks){
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
  delay(50);
}

//Move Forward 12" so many Blocks times
void Forward(int Blocks){
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
  delay(50);
}

//Rotate 90 degrees clockwise so many times
void turnCW(int Blocks){
  int i = 0;
  M1tick = 0; M2tick = 0; M3tick = 0; M4tick = 0; //encoder counts
  //start motors moving
  M1Motor->run(FORWARD);  M2Motor->run(FORWARD);
  M3Motor->run(FORWARD); M4Motor->run(FORWARD);
  while(M1tick<Blocks && M2tick<Blocks && M3tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<150; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M1tick);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);  
  delay(50);
}

//Rotate 90 degrees counter-clockwise so many times
void turnCCW(int Blocks){
  int i = 0;
  M1tick = 0; M2tick = 0; M3tick = 0; M4tick = 0; //encoder counts
  //start motors moving
  M1Motor->run(BACKWARD); M2Motor->run(BACKWARD);
  M3Motor->run(BACKWARD); M4Motor->run(BACKWARD);
  while(M1tick<Blocks && M2tick<Blocks && M3tick<Blocks && M4tick<Blocks){
    if(i == 0){
      //adjust speed for acceleration
      for(i=50; i<150; i++){
        M1Motor->setSpeed(i); M3Motor->setSpeed(i);
        M2Motor->setSpeed(i); M4Motor->setSpeed(i);
        delayMicroseconds(100);
      }
    }
    Serial.print(M1tick);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
  delay(50);
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
  int gone=0;
  while(differenceR > 50 && differenceL > 50){   
    AlphaL_time = AlphaL.ping();            AlphaR_time = AlphaR.ping(); 
    differenceL = AlphaL_time - goal;       differenceR = AlphaR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall    
    differenceL = abs(differenceL);         differenceR = abs(differenceR);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0&&gone==0){gone=1;
      M1Motor->run(FORWARD);  M3Motor->run(BACKWARD);
    }
    if(movement<0&&gone==0){gone=1;
      M1Motor->run(BACKWARD);  M3Motor->run(FORWARD);
    }  
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  delay(50);
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
  int gone=0;
  while(differenceR > 50 && differenceL > 50){   
    BravoL_time = BravoL.ping();            BravoR_time = BravoR.ping();
    differenceL = BravoL_time - goal;       differenceR = BravoR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall   
    differenceL = abs(differenceL);         differenceR = abs(differenceR);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0&&gone==0){gone=1;
      M2Motor->run(BACKWARD);  M4Motor->run(FORWARD);
    }
    if(movement<0&&gone==0){gone=1;
      M2Motor->run(FORWARD);  M4Motor->run(BACKWARD);
    }  
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
  delay(50);
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
  int gone=0;
  while(differenceR > 50 && differenceL > 50){    
    CharlieL_time = CharlieL.ping();        CharlieR_time = CharlieR.ping(); 
    differenceL = CharlieL_time - goal;       differenceR = CharlieR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall    
    differenceL = abs(differenceL);           differenceR = abs(differenceR);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0&&gone==0){gone=1;
      M1Motor->run(BACKWARD);  M3Motor->run(FORWARD);
    }
    if(movement<0&&gone==0){gone=1;
      M1Motor->run(FORWARD);  M3Motor->run(BACKWARD);
    }  
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  delay(50);
}

//adjust offset from wall based on ultrasonic readings
void offset_Delta(int multiple){
  M2Motor->setSpeed(M2Speed); M4Motor->setSpeed(M4Speed); 
  int goal = multiple*1780 + 215; //goal distance based on multiple parameter from blocks between wall
  unsigned int DeltaR_time = DeltaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
  unsigned int DeltaL_time = DeltaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
  //difference from goal and absolute value for error correction 
  int differenceL = DeltaL_time - goal;    int differenceR = DeltaR_time - goal;
      differenceL =   abs(differenceL);        differenceR =   abs(differenceR);
  //loop for error correction
  int gone=0;
  while(differenceR > 50 && differenceL > 50){    
    DeltaL_time =      DeltaL.ping();       DeltaR_time =      DeltaR.ping();    
    differenceL = DeltaL_time - goal;       differenceR = DeltaR_time - goal;
    int movement = (differenceR + differenceL)/2; //movement to give motion towards or away from wall   
    differenceL = abs(differenceL);         differenceR = abs(differenceR);
    //motion adjustment based on positive or negative movement to or from wall
    if(movement>0&&gone==0){gone=1;
      M2Motor->run(FORWARD);  M4Motor->run(BACKWARD);
    }
    if(movement<0&&gone==0){gone=1;
      M2Motor->run(BACKWARD);  M4Motor->run(FORWARD);
    }  
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
  delay(50);
}

//align when not parrallel with wall
void align_Alpha(){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int AlphaR_time = AlphaR.ping_median(5);
  unsigned int AlphaL_time = AlphaL.ping_median(5);
           int difference  = AlphaR_time - AlphaL_time;
               difference  = abs(difference);
  //loop that turns until parrallel with wall  
           int goal        = 10;     
           int ccw         = 0;      
  while(difference > goal){   
    AlphaL_time = AlphaL.ping();   AlphaR_time = AlphaR.ping();
    difference  = AlphaR_time - AlphaL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){break;}
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
  delay(50);
}

//align when not parrallel with wall
void align_Bravo(){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int BravoR_time    = BravoR.ping_median(5);
  unsigned int BravoL_time    = BravoL.ping_median(5);
           int difference     = BravoR_time - BravoL_time;
               difference     = abs(difference);         
  //loop that turns until parrallel with wall
           int goal           = 10;  
           int ccw            = 0;           
  while(difference > goal){   
    BravoL_time = BravoL.ping();  BravoR_time = BravoR.ping();
    difference  = BravoR_time - BravoL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){break;}
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
  delay(50);
}

//align when not parrallel with wall
void align_Charlie(){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int CharlieR_time = CharlieR.ping_median(5);
  unsigned int CharlieL_time = CharlieL.ping_median(5);
           int difference  = CharlieR_time - CharlieL_time;
           int goal        = 10;
           int ccw         = 0;
               difference  = abs(difference);
  //loop that turns until parrallel with wall             
  while(difference > goal){    
    CharlieL_time = CharlieL.ping();  CharlieR_time = CharlieR.ping();
    difference  = CharlieR_time - CharlieL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){break;}
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
  delay(50);
}

//align when not parrallel with wall
void align_Delta(){
  //set motor speed
  M1Motor->setSpeed(M1Speed-10); M3Motor->setSpeed(M3Speed-10);
  M2Motor->setSpeed(M2Speed-10); M4Motor->setSpeed(M4Speed-10);
  //get time and difference between two sensors
  unsigned int DeltaR_time = DeltaR.ping_median(5);
  unsigned int DeltaL_time = DeltaL.ping_median(5);
           int difference  = DeltaR_time - DeltaL_time;
           int goal        = 10;
           int ccw         = 0;
               difference  = abs(difference);
  //loop that turns until parrallel with wall             
  while(difference > goal){    
    DeltaL_time = DeltaL.ping();  DeltaR_time = DeltaR.ping();
    difference  = DeltaR_time - DeltaL_time;
    //when difference is positive turn cw, when negative turn ccw
    if(difference>0){
      if(ccw == 1){break;}
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
  delay(50);
}

//Calibrate to block
void Calibrate(){
  if(x_pos<3)   {align_Bravo();   offset_Bravo(x_pos);     align_Bravo();}
  else          {align_Delta();   offset_Delta(6-x_pos);   align_Delta();}
  
  if(y_pos<4)   {offset_Charlie(y_pos);                  align_Charlie();}
  else          {offset_Alpha(6-y_pos);                    align_Alpha();}
}

//set neo_pos based ont the x_pos and y_pos values to light up correct led
void Locate(){
  if(x_pos==0){neo_pos=y_pos;}
  if(x_pos==1){neo_pos=y_pos+8;}
  if(x_pos==2){neo_pos=y_pos+16;}
  if(x_pos==3){neo_pos=y_pos+24;}
  if(x_pos==4){neo_pos=y_pos+32;}
  if(x_pos==5){neo_pos=y_pos+40;}
  if(x_pos==6){neo_pos=y_pos+48;}
  pixels.setPixelColor(neo_pos,pixels.Color(255,0,0)); //show yellow on (0,0)
  pixels.show();        delay(500);
}


void find_obstacles(){
  unsigned int AlphaL_time    = AlphaL.ping_median(5);
  unsigned int AlphaR_time    = AlphaR.ping_median(5);
           int Alpha_obstacle = (AlphaL_time + AlphaR_time)/2;
  unsigned int DeltaL_time    = DeltaL.ping_median(5);
  unsigned int DeltaR_time    = DeltaR.ping_median(5); 
           int Delta_obstacle = (DeltaL_time + DeltaR_time)/2;

  if(Alpha_obstacle>2300 && Alpha_obstacle<2900){
    Serial.println("three blocks away");
    Forward(oneBlock*2);
  }
  if(Alpha_obstacle>580 && Alpha_obstacle<2300){
    Serial.println("two blocks away");
    Forward(oneBlock);
  }
  if(Alpha_obstacle>0 && Alpha_obstacle<580){
    Serial.println("one block away");
  }
  delay(1000);

  Serial.println(Alpha_obstacle);

  if(Delta_obstacle>4050 && Delta_obstacle<5770){
    Serial.println("four blocks away");
  }
  if(Delta_obstacle>2300 && Delta_obstacle<4050){
    Serial.println("three blocks away");
  }
  if(Delta_obstacle>580 && Delta_obstacle<2300){
    Serial.println("two blocks away");
  }
  if(Delta_obstacle>0 && Delta_obstacle<580){
    Serial.println("one block away");
  }
  delay(1000);
}

/*******************************************************COMM WITH CORY*****************************************************************/

// function for placement of camera to cache lid
void Camera(){
  if(x_pos<2)           {turnCW(3*Ninety);       
                align_Alpha();    offset_Alpha(x_pos)  ;
    if(y_pos<4){align_Bravo();    offset_Bravo(y_pos)  ;}
    else       {align_Delta();    offset_Delta(6-y_pos);}
  }
  else if(x_pos>4)      {turnCW(Ninety);         
                align_Alpha();    offset_Alpha(6-x_pos);
    if(y_pos<4){align_Delta();    offset_Delta(y_pos)  ;}
    else       {align_Bravo();    offset_Bravo(6-y_pos);}
  }
  else if(y_pos<4)      {turnCW(Ninety*2);
                align_Alpha();    offset_Alpha(y_pos)  ;
    if(x_pos<4){align_Delta();    offset_Delta(x_pos)  ;}
    else       {align_Bravo();    offset_Bravo(6-x_pos);}
  }
  else if(y_pos>4)      {//no need to turn
                align_Alpha();    offset_Alpha(y_pos)  ;
    if(x_pos<4){align_Bravo();    offset_Bravo(x_pos)  ;}
    else       {align_Delta();    offset_Delta(6-x_pos);}
   }
   align_Alpha();
   offset_camera();//get to a close position on cache
}

//moves close to lid
void offset_camera(){
  M1Motor->setSpeed(M1Speed); M3Motor->setSpeed(M3Speed);
  int goal = 980;//goal distance based on multiple parameter from blocks between wall
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
    if(movement>0) {M1Motor->run(FORWARD );  M3Motor->run(BACKWARD);}
    if(movement<0) {M1Motor->run(BACKWARD);  M3Motor->run(FORWARD );}  
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  delay(50);
}

void Cory(){
  bool Cory=true;
  
  digitalWrite(TRANSMIT,HIGH);
  delay(100);
  digitalWrite(TRANSMIT,LOW);
  
  while(Cory){
  int move_up   = digitalRead(UP  );  int move_down  = digitalRead(DOWN );  
  int move_left = digitalRead(LEFT);  int move_right = digitalRead(RIGHT);
                                            
    //move based on high pin from cory
    if(move_up   )   {M1Motor->run(FORWARD );  M3Motor->run(BACKWARD);  delay(100);  M1Motor->run(RELEASE); M3Motor->run(RELEASE);} //forward 1/5 in
    if(move_down )   {M1Motor->run(BACKWARD);  M3Motor->run(FORWARD );  delay(100);  M1Motor->run(RELEASE); M3Motor->run(RELEASE);} //backward
    if(move_right)   {M2Motor->run(FORWARD );  M4Motor->run(BACKWARD);  delay(100);  M2Motor->run(RELEASE); M4Motor->run(RELEASE);} //right
    if(move_left )   {M2Motor->run(BACKWARD);  M4Motor->run(FORWARD );  delay(100);  M2Motor->run(RELEASE); M4Motor->run(RELEASE);} //left
    int reading = digitalRead(RECEIVE);
    if(reading==1){Cory=false;}// get out of loop when cory is finished
  }
  //delay(3000);
  //grab lid and place on porch with security guard
  grabber.write(180); 
  delay(1000);

  for(int down = 180; down >=5; down -=1){
    grabber.write(down);
    delay(10);
  }
  grabber.write(0);
  delay(500);

  for(int pos = 0; pos < 180; pos += 1){
    grabber.write(pos);
    delay(10);
  }
  delay(1000);

  //This tells Cory that servo is moved
  digitalWrite(TRANSMIT, HIGH);
  delay(100);
  digitalWrite(TRANSMIT, LOW );

  //This loop will wait until Pi is done with die
  Cory=true;
  while(Cory){
    int reading = digitalRead(RECEIVE);
    if(reading==1){Cory=false;}
    delay(200);
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



