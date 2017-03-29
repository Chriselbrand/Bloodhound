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
#define TRANSMIT 52       //  brown <-> brown
#define RECEIVE  50       //    red <-> red

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
int grabber_start    = 180;

//pin for green start button
#define Starter 39

int obstacle_map[7][7] ={{0000000},
                         {0000000},
                         {0000000},
                         {0000000},
                         {0000000},
                         {0000000},
                         {0000000}};

int grid_map[7][7] ;                          //map of detected grid locations

//pins for Teensy
#define ThomReady     3
#define teensy_hollow 4
#define teensy_wire   5

//definitions of map matrix designators
#define UNKNOWN_SQUARE      0
#define SOLID               1
#define OBJECTIVE           2
#define DEAD_END            3

/*****************************************************************************INITIALIZATION**************************************************************************************************/

//initialize x,y cordinates
int x_pos = 0;        int y_pos = 0;  
int x_counter = 0;    int y_counter = 0;

int left_stuck=0;    int right_stuck=0;   int up_stuck=0;   int down_stuck=0;

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
  pinMode(M3interrupt, INPUT); pinMode(M4interrupt, INPUT_PULLUP);


pinMode(ThomReady,      INPUT);
pinMode(teensy_hollow,  INPUT);
pinMode(teensy_wire,    INPUT);

  

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


  
  //loop that goes until green button is pressed
  int starterread = digitalRead(Starter);
  while(starterread != 1){
    delay(1);
    starterread = digitalRead(Starter);
    delay(10);
    if (digitalRead(Starter) != starterread) starterread = 0;
    Serial.println(starterread);
  }
  
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
  M1Motor->setSpeed(240); M3Motor->setSpeed(240);
  Forward(oneBlock/2);
  unsigned int DeltaR_time = DeltaR.ping_median(9);
  //if block is detected
  if(DeltaR_time>200 && DeltaR_time<600){
  Forward(oneBlock/2);    align_Bravo();
  offset_Charlie(1)  ;    align_Bravo();
  Forward(oneBlock)  ;    align_Bravo();
  offset_Charlie(2)  ;    align_Bravo();
  Right(oneBlock)    ;    align_Bravo();
  offset_Bravo(1)    ;    align_Bravo();
  x_pos = 1;    y_pos = 2;
  Go_to(x_pos,y_pos); 
  }
  else{
  Right(oneBlock)    ;    align_Bravo();
  offset_Bravo(1)    ;    align_Bravo();
  Forward(oneBlock/2);    align_Bravo();
  x_pos = 1;    y_pos = 1;
  Go_to(x_pos,y_pos); 
  }
}

//return to (0,0) from already known location
void Home(){
  int check_x = 0;
  int check_y = 0;
  for(int i =1;i<x_pos;i++){
  check_x = check_x + obstacle_map[i][y_pos];
  }
  if(check_x == 0)       {Go_to(1,y_pos); }

  for(int i =1;i<y_pos;i++){
    check_y = check_y + obstacle_map[x_pos][i];
  }
  if(check_y == 0)       {Go_to(x_pos,1);}
  
 while(x_pos>3 && y_pos>3){
  Go_to(3,y_pos);
  Go_to(x_pos,3);
 }

  for(int i =1;i<x_pos;i++){
  check_x = check_x + obstacle_map[i][y_pos];
  }
  if(check_x == 0)        {Go_to(1,y_pos);}
  
  for(int i =1;i<y_pos;i++){
    check_y = check_y + obstacle_map[x_pos][i];
  }
  if(check_y == 0)   {Go_to(x_pos,1);}

  while(x_pos>2 && y_pos>2){
  Go_to(3,y_pos);
  Go_to(x_pos,3);
  }

  for(int i =1;i<x_pos;i++){
  check_x = check_x + obstacle_map[i][y_pos];
  }
  if(check_x == 0)       {Go_to(1,y_pos);}
  
  for(int i =1;i<y_pos;i++){
    check_y = check_y + obstacle_map[x_pos][i];
  }
  if(check_y == 0)  {Go_to(x_pos,1);}

  if(obstacle_map[1][1] == 1){
    Go_to(1,2);   Calibrate();
    Left(oneBlock);    align_Bravo();
    offset_Bravo(0);    align_Bravo();
    Backward(oneBlock);    align_Charlie();
    offset_Charlie(1); align_Charlie();
    Backward(oneBlock); Calibrate();
    }

  else if(obstacle_map[1][2] == 1){
    Go_to(1,3);   Calibrate();
    Left(oneBlock);    align_Bravo();
    offset_Bravo(0);    align_Bravo();
    Backward(oneBlock);    align_Charlie();
    offset_Charlie(2); align_Charlie();
    Backward(oneBlock); align_Charlie();
    offset_Charlie(1);  align_Charlie(); 
    Backward(oneBlock); Calibrate();
  }
    
  else if(obstacle_map[2][1] == 1){
    Go_to(3,1);   Calibrate();
    Backward(oneBlock);    align_Charlie();
    offset_Bravo(3);    align_Bravo();
    Left(oneBlock);    align_Charlie();
    offset_Bravo(2); align_Charlie();
    Left(oneBlock); align_Charlie();
    offset_Bravo(1);  align_Charlie(); 
    Left(oneBlock); Calibrate();
  }
}




 

void look_around(){
  delay(100);
  //check forward
  unsigned int AlphaR_time = AlphaR.ping_median(9);   
  unsigned int AlphaL_time = AlphaL.ping_median(9);    
           int A_time = (AlphaR_time + AlphaL_time)/2;
  if(A_time>200 && A_time<600){
    //save obstacle spot to be used later
    obstacle_map[x_pos][y_pos+1] = 1;
  }
  //check backward
  unsigned int CharlieR_time = CharlieR.ping_median(9);    
  unsigned int CharlieL_time = CharlieL.ping_median(9); 
           int C_time = (CharlieR_time + CharlieL_time)/2;
  if(C_time>200 && C_time<00){
    obstacle_map[x_pos][y_pos-1] = 1;
  }

  
  //check right
  unsigned int DeltaR_time = DeltaR.ping_median(9);  
  unsigned int DeltaL_time = DeltaL.ping_median(9);   
           int D_time = (DeltaR_time + DeltaL_time)/2;
  if(D_time>200 && D_time<600){
    obstacle_map[x_pos+1][y_pos] = 1;
  }
  delay(100);
 
}
/* the most hard to understand yet very useful function, your x and y position are already known and updated in the motion functions. based on where you are you can tell go to where you 
 *  would like to be and it will bring you there. it only moves in one increment at a time that way it always knows if the next movement is safe based on any 1's in the obstacle map that
 *  indicate an obstacle. the first look around function just looks in the first three squares around you to check and update any obstacles. the stuck variables are if you have tried going in
 *  two directions they will know and give a seperate course since the previous logic has you stuck(failsafe). the x and y counter will let you know if there is obstacel in that direction and
 *  it will adjust your movement based on if the variables are 1. the x and y difference is for when you go more than one square the loop will get going in the direction until you have arrived
 *  where you told go_to to go.
 */
void Go_to(int x_finish, int y_finish){
  look_around();
  //counts if there was object in x or y direction
  x_counter = 0;
  y_counter = 0;
  //difference from position to desired position
  int x_difference = x_finish - x_pos;    int y_difference = y_finish - y_pos;
  //while desired xpos not met
  while(x_difference != 0){
    //if need to go right
    if(x_difference>0)  {
      //if obstacle in the right direction stop where you are and update counter to adjust to new location
        if(obstacle_map[x_pos+1][y_pos] == 1 && x_counter == 0){
          x_finish = x_pos;//will make x_difference 0 and get out of loop
          x_counter = 1;//update counter to activate the change of finished position
        }
        //go right if no objects
        else{Right(oneBlock);    look_around();}//look around after every move to find obstacles
    }
    //if need to go left
    if(x_difference<0){
      //if obstacle in the left direction stop where you are and update counter to adjust to new location
        if(obstacle_map[x_pos-1][y_pos]==1 && x_counter == 0){
          x_finish = x_pos;//will make x_difference 0 and get out of loop
          x_counter = 1;//update counter to activate the change of finished position
        }
        //go left if no objects
        else{Left(oneBlock);    look_around; left_stuck = 0;}//look around after every move to find obstacles
  }
  //update difference to see if you are where you want to be, if not continue loop until you are at the finished position
  x_difference = x_finish - x_pos;
  }
  //if there was a block in x direction update the movement to where you go around obstacle
  if(x_counter>0){
    //if in the upper quadrant go down then go to right
    if(y_pos>3){
      Go_to(x_pos,y_pos-1);  Go_to(x_pos+1,y_pos);
      y_difference = 0;//bypass next loop to be finished
    }
    //if in the lower quadrant go up then go right
    else if(y_pos<4){
      Go_to(x_pos,y_pos+1); Go_to(x_pos+1,y_pos);
      y_difference = 0;//bypass next loop to be finished
    }
  }
  x_counter = 0;//update counter just incase
  //while y is not at desired location
while(y_difference != 0){
  //if need to go forward
    if(y_difference>0)  {
      //if obstacle in the forward direction stop where you are and update the counter to change finished location
        if(obstacle_map[x_pos][y_pos+1]==1){
          y_finish=y_pos;//make ydifference 0 and exit loop
          y_counter = 1;
        }
        //go forward since no blocks
        else{Forward(oneBlock); look_around();}//look around after every movement to search for obstacles
    }
    //if need to go backward
    if(y_difference<0){
      //look for objects in way
        if(obstacle_map[x_pos][y_pos-1]==1){
          y_finish=y_pos;
          y_counter = 1;
        }
        //go backward since no blocks
        else{Backward(oneBlock); look_around();}
      }
    //update difference to see if you are where you want to be
    y_difference = y_finish - y_pos;//check to see if you are at the finished y position
  }
  //if there was block which way were you going and hug object to get to position
  //based on the x_pos you want to go the same way that you are currently going so you dont miss squares
  if(y_counter>0){
    //if moving in forward direction
    if(x_pos==1 || x_pos==3){
      if(y_pos<4){//not at last block
        if(y_pos==2&&x_pos==1){Go_to(x_pos-1,y_pos+2);          Go_to(x_pos+1,y_pos);}
        
        else if(y_pos==1&&x_pos==3){Go_to(x_pos-1,y_pos+2);     Go_to(x_pos+1,y_pos);}
        
        else if(obstacle_map[x_pos-1][y_pos]==1&&obstacle_map[x_pos+1][y_pos]==1){
          Go_to(x_pos,y_pos-1);     Go_to(x_pos+2,y_pos+2); 
          Go_to(x_pos-1,y_pos+1);   Go_to(x_pos-1,y_pos);
        }
        else{Go_to(x_pos+1,y_pos+2);     Go_to(x_pos-1,y_pos);}
      }
      else{Go_to(x_pos+1,y_pos+1);}//at last block
    }
    //if moving in the backward direction
    else if(x_pos==2||x_pos==4){
      if(y_pos>2){//not at last block
        if(obstacle_map[x_pos-1][y_pos]==1){Go_to(x_pos+1,y_pos-2);       Go_to(x_pos-1,y_pos);}
        
        else if(obstacle_map[x_pos+1][y_pos]==1){Go_to(x_pos-1,y_pos-2);  Go_to(x_pos+1,y_pos);}
        
        else if(obstacle_map[x_pos-1][y_pos]==1&&obstacle_map[x_pos+1][y_pos]==1){
          Go_to(x_pos,y_pos+1);     Go_to(x_pos+2,y_pos-2); 
          Go_to(x_pos-1,y_pos-2);   Go_to(x_pos-1,y_pos);
        }
        else{
          if(x_pos==2){Go_to(x_pos+1,y_pos-2);   Go_to(x_pos-1,y_pos);}
          
          else{Go_to(x_pos-1,y_pos-2);           Go_to(x_pos+1,y_pos);}
       }
      }
      else{Go_to(x_pos+1,y_pos-1);}//at last block
    }
    else if(x_pos==5){//at the last column
      if(y_pos<4){Go_to(x_pos-1,y_pos+2);        Go_to(x_pos+1,y_pos);}
      
      else{Home();}
    }
    y_counter = 0;
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
//    Serial.print(M2tick);     Serial.print(M4tick);
    delayMicroseconds(350);
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  // turn off motors
  x_pos++;
  delay(200);
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
//    Serial.print(M2tick);   Serial.print(M4tick);
    delayMicroseconds(350);
  }
  M2Motor->run(RELEASE);  M4Motor->run(RELEASE);  //turn off motors
  x_pos--;
  delay(200);
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
//    Serial.print(M1tick);   Serial.print(M3tick);
    delayMicroseconds(350);
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  y_pos--;
  delay(200);
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
//    Serial.print(M1tick); Serial.print(M3tick);
  delayMicroseconds(350);
  }
  M1Motor->run(RELEASE);  M3Motor->run(RELEASE);  //turn off motors
  y_pos++;
  delay(200);
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
//    Serial.print(M1tick);
    delayMicroseconds(175);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);  
  delay(200);
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
//    Serial.print(M1tick);
    delayMicroseconds(175);
  }
  M1Motor->run(RELEASE); M2Motor->run(RELEASE);
  M3Motor->run(RELEASE); M4Motor->run(RELEASE);
  delay(200);
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
  delay(200);
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
  delay(200);
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
  delay(200);
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
  delay(200);
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
  delay(200);
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
  delay(200);
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
  delay(200);
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
  delay(200);
}

//Calibrate to block
//will use the matrix that has obstacle locations to see a change in offset based on a reference point away that is no longer the wall
void Calibrate(){
  if(x_pos<3)   {align_Bravo();   
    if(obstacle_map[x_pos-1][y_pos]==1){
      offset_Bravo(0);    align_Bravo();
    }
    else{offset_Bravo(x_pos);}
  }
   
  else if(x_pos ==3)  {align_Delta();   
    if(obstacle_map[x_pos+1][y_pos]==1){offset_Delta(0);}
  align_Delta();
  }

  else{align_Delta();
    if(obstacle_map[x_pos+1][y_pos]==1){offset_Delta(0);}
    else{offset_Delta(6-x_pos);}
  align_Delta();
  }
  
  if(y_pos<3)   {
    if(obstacle_map[x_pos][y_pos-1]==1){offset_Charlie(0);}
    else{offset_Charlie(y_pos);}
    align_Charlie();
    }
  
  
  else if(y_pos>3)  {
   if(obstacle_map[x_pos][y_pos+1]==1){offset_Alpha(0);}
   else{offset_Alpha(6-y_pos);}
   align_Alpha();
}
delay(200);
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


void blockStatus(){
  
  bool solid = false;
  bool OT = false;
  bool dead_end = false;
  
  delay(500);

  while(digitalRead(ThomReady)){
    delay(100);
  }

  bool hollow       = digitalRead(teensy_hollow);
  bool wire         = digitalRead(teensy_wire);
 
  if (wire == true){
    grid_map[x_pos][y_pos] = OBJECTIVE;
  } else if (hollow == true) {
    grid_map[x_pos][y_pos] = DEAD_END;
  } else {
    grid_map[x_pos][y_pos] = SOLID; 
  }
  
}





/*
void grid_stuck(){
  int gridlock = right_stuck + left_stuck + up_stuck + down_stuck;
  if(gridlock>1){
    if(right_stuck == 1 && up_stuck == 1){
      

    else if(right_stuck == 1 && down_stuck ==1){

    else if(left_stuck == 1 && up_stuck == 1){

    else if(left_stuck == 1 && down_stuck == 1){
      
    }
    }
    }
    }
  
  }
}
*/







/*******************************************************COMM WITH CORY*****************************************************************/

// function for placement of camera to cache lid
void Camera(int cache_x, int cache_y ){
  
  if(cache_x==0)      {Go_to(cache_x+1,cache_y);
   
                        turnCW(3*Ninety);       
                align_Alpha();    offset_Alpha(x_pos)  ;
    if(y_pos<4){align_Bravo();    offset_Bravo(y_pos)  ;}
    else       {align_Delta();    offset_Delta(6-y_pos);}
  }
  
  else if(cache_x==6)   {Go_to(cache_x-1,cache_y);
    
                           turnCW(Ninety);         
                align_Alpha();    offset_Alpha(6-x_pos);
    if(y_pos<4){align_Delta();    offset_Delta(y_pos)  ;}
    else       {align_Bravo();    offset_Bravo(6-y_pos);}
  }
  
  else if(cache_y==0)   {Go_to(cache_x,cache_y+1);
    
                           turnCW(Ninety*2);
                align_Alpha();    offset_Alpha(y_pos)  ;
    if(x_pos<4){align_Delta();    offset_Delta(x_pos)  ;}
    else       {align_Bravo();    offset_Bravo(6-x_pos);}
  }
  else if(cache_y==6)      {Go_to(cache_x,cache_y-1);
    
                          //no need to turn
                align_Alpha();    offset_Alpha(6-y_pos)  ;
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
  delay(200);
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
  //grab lid and place on porch with security guard
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



