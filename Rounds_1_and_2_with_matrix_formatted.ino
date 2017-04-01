/**********************************************************************************************************************************************************************************************
*********************************************************SLAM-BOT'S BLOODHOUND PROJECT MOTION FUNCTION PROGRAM*********************************************************************************
********************************************************KEMP HARTZOG, CHRIS HARRIS, THOMAS MILLER, CORY LANDETA********************************************************************************
**********************************************************************************************************************************************************************************************/

/***************************************************************************LIBRARIES & SETUP*********************************************************************************************************/

#include <Wire.h> //i2c communication
#include <elapsedMillis.h>
//neo pixel library
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include <gamma.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

//neomatrix setup
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, 6 /*pin*/,
    NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE);

//color definitions
#define UNKNOWN_SQUARE 0
#define DEAD_END_COLOR 0x001F //blue
#define OBJECTIVE_COLOR 0xF800 //red
#define YELLOW_COLOR 0xFFE0 //yellow

//   Kemp <-> Cory
#define UP 53 //   blue <-> blue
#define DOWN 51 //  green <-> green
#define LEFT 49 // yellow <-> yellow
#define RIGHT 47 // orange <-> orange
#define TRANSMIT 52 //  brown <-> brown
#define RECEIVE 50 //    red <-> red

//Ultrasonics library
#include <NewPing.h>

//Ultrasonic initialization
#define MAX_DISTANCE 280 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//Beta side trigger and echo
#define BravoR_TRIGGER 24
#define BravoR_ECHO 25
NewPing BravoR(BravoR_TRIGGER, BravoR_ECHO, MAX_DISTANCE);
#define BravoL_TRIGGER 26
#define BravoL_ECHO 27
NewPing BravoL(BravoL_TRIGGER, BravoL_ECHO, MAX_DISTANCE);

//Alpha side trigger and echo
#define AlphaR_TRIGGER 28
#define AlphaR_ECHO 29
NewPing AlphaR(AlphaR_TRIGGER, AlphaR_ECHO, MAX_DISTANCE);
#define AlphaL_TRIGGER 30
#define AlphaL_ECHO 31
NewPing AlphaL(AlphaL_TRIGGER, AlphaL_ECHO, MAX_DISTANCE);

//Delta side trigger and echo
#define DeltaR_TRIGGER 32
#define DeltaR_ECHO 33
NewPing DeltaR(DeltaR_TRIGGER, DeltaR_ECHO, MAX_DISTANCE);
#define DeltaL_TRIGGER 34
#define DeltaL_ECHO 35
NewPing DeltaL(DeltaL_TRIGGER, DeltaL_ECHO, MAX_DISTANCE);

//Charlie side triggera and echo
#define CharlieR_TRIGGER 36
#define CharlieR_ECHO 37
NewPing CharlieR(CharlieR_TRIGGER, CharlieR_ECHO, MAX_DISTANCE);
#define CharlieL_TRIGGER 22
#define CharlieL_ECHO 23
NewPing CharlieL(CharlieL_TRIGGER, CharlieL_ECHO, MAX_DISTANCE);

//pins for Teensy
#define ThomCall 5
#define ThomReady 7
#define teensy_hollow 4
#define teensy_wire A3

//definitions of map matrix designators
#define UNKNOWN_SQUARE 0
#define SOLID 1
#define OBJECTIVE 2
#define DEAD_END 3
#define START 4
#define CACHE 5

//motor shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor* M1Motor = AFMS.getMotor(1);
Adafruit_DCMotor* M2Motor = AFMS.getMotor(2);
Adafruit_DCMotor* M3Motor = AFMS.getMotor(3);
Adafruit_DCMotor* M4Motor = AFMS.getMotor(4);

//Setup variables for moving one block and initial speed
int oneBlock = 306;
int Ninety = 135;
int M1Speed = 50;
int M2Speed = 50;
int M3Speed = 50;
int M4Speed = 50;

//interrupt pins and motor ticks count
int M1tick = 0;
int M2tick = 0;
int M3tick = 0;
int M4tick = 0;
#define M1interrupt 19
#define M2interrupt 18
#define M3interrupt 3
#define M4interrupt 2

//servo library
#include <Servo.h>

//declare the grabber as servo
Servo grabber;

//elapsedMillis timers
elapsedMillis thomTeensyTimer;

//initialize grabber position to be used in code
#define grabber_pin 10
int grabber_start = 180;

//pin for green start button
#define Starter 39

//tic tracer threshold
#define tic_thresh 450

/*****************************************************************************INITIALIZATION**************************************************************************************************/

//initialize x,y cordinates
int x_pos = 0;
int y_pos = 0;

//arrays to use for determining under obstacle
int grid_map[7][7]; //map of detected grid locations

/**************************************************************************START OF PROGRAM***************************************************************************************************/

//SET THIS = 1 TO ENABLE SERIAL MONITOR TROUBLESHOOTING. 0 TO DISABLE.
bool verboseSerial = 1;

//Line dancing moves
void setup()
{
    AFMS.begin(); // create with the default frequency 1.6KHz

#if defined(__AVR_ATtiny85__)
    if (F_CPU == 16000000)
        clock_prescale_set(clock_div_1);
#endif
    //end of trinket special code

    //initalize grabber pin and set to start position
    grabber.attach(grabber_pin);
    grabber.write(40);

    //Set speed that will be used
    M1Motor->setSpeed(M1Speed);
    M2Motor->setSpeed(M2Speed);
    M3Motor->setSpeed(M3Speed);
    M4Motor->setSpeed(M4Speed);

    //setup for motor interrupt pins
    pinMode(M1interrupt, INPUT);
    pinMode(M2interrupt, INPUT);
    pinMode(M3interrupt, INPUT);
    pinMode(M4interrupt, INPUT);

    //interrupt function attachment and call modes
    attachInterrupt(digitalPinToInterrupt(M1interrupt), M1count, RISING);
    attachInterrupt(digitalPinToInterrupt(M2interrupt), M2count, RISING);
    attachInterrupt(digitalPinToInterrupt(M3interrupt), M3count, RISING);
    attachInterrupt(digitalPinToInterrupt(M4interrupt), M4count, RISING);

    //port setup for printing
    Serial.begin(115200);

    //set function and initial value of arduino <-> pi communication pins
    pinMode(RECEIVE, INPUT);
    pinMode(TRANSMIT, OUTPUT);
    pinMode(UP, INPUT);
    pinMode(DOWN, INPUT);
    pinMode(LEFT, INPUT);
    pinMode(RIGHT, INPUT);

    //set green button for input
    pinMode(Starter, INPUT);

    //set pins to Teensy
    pinMode(ThomCall, OUTPUT);
    pinMode(ThomReady, INPUT);
    pinMode(teensy_hollow, INPUT);
    pinMode(teensy_wire, INPUT);

    matrix.begin();
    matrix.clear();
    matrix.setBrightness(10);
    grid_map[0][0] = START;
    matrix.drawPixel(0, 0, YELLOW_COLOR);
    matrix.show();


    //loop that goes until green button is pressed
    int starterread = digitalRead(Starter);
    while (starterread != 1) {
        if (verboseSerial == 1)
            Serial.println("Waiting For Green Button");
        delay(1);
        starterread = digitalRead(Starter);
        delay(10);
        if (digitalRead(Starter) != starterread)
            starterread = 0;
//        Serial.println(starterread);
    }
    delay(300);

    //warm up strecth
    Start(); //Locate();
    blockStatus();
    if (verboseSerial == 1)
        Serial.println("Start pixel drawn");

    /**************************BEGIN GRID SEARCH************************************/

    while (x_pos < 5 || y_pos < 5) { //ends upon arrival at F2 (row 5, col 5)
        //odd column electric slide forward
        if (x_pos % 2 == 1 && y_pos < 5) {
            Go_to(x_pos, y_pos + 1);
            Calibrate(); //Locate();
            blockStatus();
        }
        //top of odd column, slide to the right
        else if (x_pos % 2 == 1 && y_pos == 5) {
            Go_to(x_pos + 1, y_pos);
            Calibrate(); //Locate();
            blockStatus();
        }
        //even column, electric slide backward
        else if (x_pos % 2 == 0 && y_pos > 1) {
            Go_to(x_pos, y_pos - 1);
            Calibrate(); //Locate();
            blockStatus();
        }
        //bottom of even column, slide to the right
        else if (x_pos % 2 == 0 && y_pos == 1) {
            Go_to(x_pos + 1, y_pos);
            Calibrate(); //Locate();
            blockStatus();
        }

        //    if (verboseSerial == 1) {
        //      Serial.print("(X,Y) and Value:");
        //      Serial.print(x_pos);
        //      Serial.print(" ");
        //      Serial.print(y_pos);
        //      Serial.print(" ");
        //      Serial.println(grid_map[x_pos][y_pos]);
        //    }
    }

    /*******************************END GRID SEARCH***************************/

    /*************************FIND STATUS OF EDGE SQUARES*********************/

    for (int x = 1; x <= 5; x++) {
        if (grid_map[x][1] == SOLID) {
            grid_map[x][0] = SOLID;
            Serial.print(x);
            Serial.println(" 0: SOLID");
        }
        if (grid_map[x][1] == OBJECTIVE) {

            int adjacentObj1 = 0;

            if (grid_map[x - 1][1] == OBJECTIVE) {
                adjacentObj1++;
            }
            if (grid_map[x + 1][1] == OBJECTIVE) {
                adjacentObj1++;
            }
            if (grid_map[x][2] == OBJECTIVE) {
                adjacentObj1++;
            }

            if (adjacentObj1 == 1) {
                grid_map[x][0] = CACHE;
                Serial.print(x);
                Serial.println(" 0: CACHE");
                matrix.drawPixel(x, 0, OBJECTIVE_COLOR);
            }
            else if (adjacentObj1 == 2) {
                grid_map[x][0] = SOLID;
                Serial.print(x);
                Serial.println(" 0: SOLID BUT NEXT TO OT");
            }

            if (grid_map[x][5] == SOLID) {
                grid_map[x][6] = SOLID;
                Serial.print(x);
                Serial.println(" 0: SOLID");
            }
        }
        if (grid_map[x][5] == OBJECTIVE) {

            int adjacentObj5 = 0;

            if (grid_map[x - 1][5] == OBJECTIVE) {
                adjacentObj5++;
            }
            if (grid_map[x + 1][5] == OBJECTIVE) {
                adjacentObj5++;
            }
            if (grid_map[x][4] == OBJECTIVE) {
                adjacentObj5++;
            }

            if (adjacentObj5 == 1) {
                grid_map[x][6] = CACHE;
                Serial.print(x);
                Serial.println(" 6: CACHE");
                matrix.drawPixel(x, 6, OBJECTIVE_COLOR);
            }
            else if (adjacentObj5 == 2) {
                grid_map[x][6] = SOLID;
                Serial.print(x);
                Serial.println(" 6: SOLID BUT NEXT TO OT");
            }
        }
    }

    for (int y = 1; y <= 5; y++) {
        if (grid_map[1][y] == SOLID) {
            grid_map[0][y] = SOLID;
            Serial.print("0 ");
            Serial.print(y);
            Serial.println(": SOLID");
        }
        if (grid_map[1][y] == OBJECTIVE) {

            int adjacentObj1 = 0;

            if (grid_map[1][y - 1] == OBJECTIVE) {
                adjacentObj1++;
            }
            if (grid_map[1][y + 1] == OBJECTIVE) {
                adjacentObj1++;
            }
            if (grid_map[2][y] == OBJECTIVE) {
                adjacentObj1++;
            }

            if (adjacentObj1 == 1) {
                grid_map[0][y] = CACHE;
                Serial.print("0 ");
                Serial.print(y);
                Serial.println(": CACHE");
                matrix.drawPixel(0, y, OBJECTIVE_COLOR);
            }
            else if (adjacentObj1 == 2) {
                grid_map[0][y] = SOLID;
                Serial.print("0 ");
                Serial.print(y);
                Serial.println(": SOLID BUT NEXT TO OT");
            }
        }
        if (grid_map[5][y] == OBJECTIVE) {

            int adjacentObj5 = 0;

            if (grid_map[5][y - 1] == OBJECTIVE) {
                adjacentObj5++;
            }
            if (grid_map[5][y + 1] == OBJECTIVE) {
                adjacentObj5++;
            }
            if (grid_map[4][y] == OBJECTIVE) {
                adjacentObj5++;
            }

            if (adjacentObj5 == 1) {
                grid_map[6][y] = CACHE;
                Serial.print("6 ");
                Serial.print(y);
                Serial.println(": CACHE");
                matrix.drawPixel(6, y, OBJECTIVE_COLOR);
            }
            else if (adjacentObj5 == 2) {
                grid_map[6][y] = SOLID;
                Serial.print("6 ");
                Serial.print(y);
                Serial.println(": SOLID BUT NEXT TO OT");
            }
        }
    }

    matrix.show();

    /******************************GO TO A TUNNEL END**************************/

    for (int y = 0; y < 7; y++) {
        for (int x = 0; x < 7; x++) {
            if (grid_map[x][y] == CACHE) {
                if (x == 0) {
                    Go_to(x + 1, y);
                    Camera(x, y);
                    Cory();
                }
                if (x == 6) {
                    Go_to(x - 1, y);
                    Camera(x, y);
                }
                if (y == 0) {
                    Go_to(x, y + 1);
                    Camera(x, y);
                }
                if (y == 6) {
                    Go_to(x, y - 1);
                    Camera(x, y);
                }
            }
        }
    }
    
    /******************************LED MATRIX OUTPUT**************************/

    //  for (int y_pos = 0 ; y_pos<7 ; y_pos++) {
    //    for (int x_pos = 0 ; x_pos<7 ; x_pos++) {
    //      matrix.drawPixel(x_pos,y_pos,grid_map[x_pos][y_pos]);
    //    }
    //  }
    //
    //matrix.show();

    //Congratulations ET Everythings unlimited with the new TMobile ONE plan
//    Home();
}

/**********************************************************************START OF FUNCTIONS(DANCE MOVES)****************************************************************************************/

void blockStatus()
{

    delay(200);

    digitalWrite(ThomCall, HIGH);
    while (!digitalRead(ThomReady)) {
        delay(1);
    }
    while (digitalRead(ThomReady)) {
        delay(1);
    }
    while (!digitalRead(ThomReady)) {
        delay(1);
    }

    //  while(!digitalRead(ThomReady)){
    //    delay(200);
    //  }
    digitalWrite(ThomCall, LOW);
    bool hollow = digitalRead(teensy_hollow);
    int wire = digitalRead(teensy_wire);
    Serial.println(wire);
    if (wire == 1) {
        grid_map[x_pos][y_pos] = OBJECTIVE;
        matrix.drawPixel(x_pos, y_pos, OBJECTIVE_COLOR);
        Serial.print("(");
        Serial.print(x_pos);
        Serial.print(",");
        Serial.print(y_pos);
        Serial.println("): OBJECTIVE");
    }
    else if (hollow == true) {
        grid_map[x_pos][y_pos] = DEAD_END;
        matrix.drawPixel(x_pos, y_pos, DEAD_END_COLOR);
        Serial.print("(");
        Serial.print(x_pos);
        Serial.print(",");
        Serial.print(y_pos);
        Serial.println("): DEAD END");
    }
    else {
        grid_map[x_pos][y_pos] = SOLID;
        Serial.print("(");
        Serial.print(x_pos);
        Serial.print(",");
        Serial.print(y_pos);
        Serial.println("): SOLID");
    }

    matrix.show();
}

//start program, Move to (1,1)
void Start()
{
    Right(oneBlock / 2);
    align_Bravo();
    Forward(oneBlock);
    align_Delta();
    Right(oneBlock / 2);
    x_pos = 1;
    y_pos = 1;
    Calibrate();
}

//return to (0,0) from already known location
void Home()
{
    Go_to(1, 5);
    Calibrate();
    Go_to(1, 1);
    Calibrate();
    Left(oneBlock / 2);
    align_Delta();
    Backward(oneBlock);
    Left(oneBlock / 2);
    x_pos = 0;
    y_pos = 0;
    Calibrate();
}

//function that will go to an x,y position based on your current x,y position
void Go_to(int x_finish, int y_finish)
{
    int x_difference = x_finish - x_pos;
    int y_difference = y_finish - y_pos;
    while (x_difference != 0) {
        if (x_difference > 0) {
            Right(oneBlock);
        }
        else {
            Left(oneBlock);
        }
        x_difference = x_finish - x_pos;
    }
    while (y_difference != 0) {
        if (y_difference > 0) {
            Forward(oneBlock);
        }
        else {
            Backward(oneBlock);
        }
        y_difference = y_finish - y_pos;
    }
}

//Move Right 12" so many Blocks times
void Right(int Blocks)
{
    int i = 0;
    M2tick = 0;
    M4tick = 0; //encoder counts
    M2Motor->run(FORWARD);
    M4Motor->run(BACKWARD); //start motors moving
    //loop for distance based on amount of encoder ticks
    while (M2tick < Blocks && M4tick < Blocks) {
        if (i == 0) {
            //adjust speed for acceleration
            for (i = 50; i < 200; i++) {
                M2Motor->setSpeed(i);
                M4Motor->setSpeed(i);
                delayMicroseconds(100);
            }
        }
        //    Serial.print(M2tick);     Serial.print(M4tick);
        delayMicroseconds(350);
    }
    M2Motor->run(RELEASE);
    M4Motor->run(RELEASE); // turn off motors
    x_pos++;
    delay(50);
}

//Move Left 12" so many Blocks times
void Left(int Blocks)
{
    int i = 0;
    M2tick = 0;
    M4tick = 0; //encoder counts
    M2Motor->run(BACKWARD);
    M4Motor->run(FORWARD); //start motors moving
    //loop for distance based on amount of encoder ticks
    while (M2tick < Blocks && M4tick < Blocks) {
        if (i == 0) {
            //adjust speed for acceleration
            for (i = 50; i < 200; i++) {
                M2Motor->setSpeed(i);
                M4Motor->setSpeed(i);
                delayMicroseconds(100);
            }
        }
        //    Serial.print(M2tick);   Serial.print(M4tick);
        delayMicroseconds(350);
    }
    M2Motor->run(RELEASE);
    M4Motor->run(RELEASE); //turn off motors
    x_pos--;
    delay(50);
}

//Move Backward 12" so many Blocks times
void Backward(int Blocks)
{
    int i = 0;
    M1tick = 0;
    M3tick = 0; //encoder counts
    M1Motor->run(BACKWARD);
    M3Motor->run(FORWARD); //start motors moving
    //loop for distance based on amount of encoder ticks
    while (M1tick < Blocks && M3tick < Blocks) {
        if (i == 0) {
            //adjust speed for acceleration
            for (i = 50; i < 200; i++) {
                M1Motor->setSpeed(i);
                M3Motor->setSpeed(i);
                delayMicroseconds(100);
            }
        }
        //    Serial.print(M1tick);   Serial.print(M3tick);
        delayMicroseconds(350);
    }
    M1Motor->run(RELEASE);
    M3Motor->run(RELEASE); //turn off motors
    y_pos--;
    delay(50);
}

//Move Forward 12" so many Blocks times
void Forward(int Blocks)
{
    int i = 0;
    M1tick = 0;
    M3tick = 0; //encoder counts
    M1Motor->run(FORWARD);
    M3Motor->run(BACKWARD); //start motors moving
    //loop for distance based on amount of encoder ticks
    while (M1tick < Blocks && M3tick < Blocks) {
        if (i == 0) {
            //adjust speed for acceleration
            for (i = 50; i < 200; i++) {
                M1Motor->setSpeed(i);
                M3Motor->setSpeed(i);
                delayMicroseconds(100);
            }
        }
        //    Serial.print(M1tick); Serial.print(M3tick);
        delayMicroseconds(350);
    }
    M1Motor->run(RELEASE);
    M3Motor->run(RELEASE); //turn off motors
    y_pos++;
    delay(50);
}

//Rotate 90 degrees clockwise so many times
void turnCW(int Blocks)
{
    int i = 0;
    M1tick = 0;
    M2tick = 0;
    M3tick = 0;
    M4tick = 0; //encoder counts
    //start motors moving
    M1Motor->run(FORWARD);
    M2Motor->run(FORWARD);
    M3Motor->run(FORWARD);
    M4Motor->run(FORWARD);
    while (M1tick < Blocks && M2tick < Blocks && M3tick < Blocks && M4tick < Blocks) {
        if (i == 0) {
            //adjust speed for acceleration
            for (i = 50; i < 150; i++) {
                M1Motor->setSpeed(i);
                M3Motor->setSpeed(i);
                M2Motor->setSpeed(i);
                M4Motor->setSpeed(i);
                delayMicroseconds(100);
            }
        }
        //    Serial.print(M1tick);
        delayMicroseconds(175);
    }
    M1Motor->run(RELEASE);
    M2Motor->run(RELEASE);
    M3Motor->run(RELEASE);
    M4Motor->run(RELEASE);
    delay(50);
}

//Rotate 90 degrees counter-clockwise so many times
void turnCCW(int Blocks)
{
    int i = 0;
    M1tick = 0;
    M2tick = 0;
    M3tick = 0;
    M4tick = 0; //encoder counts
    //start motors moving
    M1Motor->run(BACKWARD);
    M2Motor->run(BACKWARD);
    M3Motor->run(BACKWARD);
    M4Motor->run(BACKWARD);
    while (M1tick < Blocks && M2tick < Blocks && M3tick < Blocks && M4tick < Blocks) {
        if (i == 0) {
            //adjust speed for acceleration
            for (i = 50; i < 150; i++) {
                M1Motor->setSpeed(i);
                M3Motor->setSpeed(i);
                M2Motor->setSpeed(i);
                M4Motor->setSpeed(i);
                delayMicroseconds(100);
            }
        }
        //    Serial.print(M1tick);
        delayMicroseconds(175);
    }
    M1Motor->run(RELEASE);
    M2Motor->run(RELEASE);
    M3Motor->run(RELEASE);
    M4Motor->run(RELEASE);
    delay(50);
}

//adjust offset from wall based on ultrasonic readings
void offset_Alpha(int multiple)
{
    M1Motor->setSpeed(M1Speed);
    M3Motor->setSpeed(M3Speed);
    int goal = multiple * 1780 + 215; //goal distance based on multiple parameter from blocks between wall
    unsigned int AlphaR_time = AlphaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
    unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
    //difference from goal and absolute value for error correction
    int differenceL = AlphaL_time - goal;
    int differenceR = AlphaR_time - goal;
    differenceL = abs(differenceL);
    differenceR = abs(differenceR);
    //loop for error correction
    int gone = 0;
    int old_movement;
    int positive = 0;
    int negative = 0;
    while (differenceR > 50 && differenceL > 50) {
        AlphaL_time = AlphaL.ping();
        AlphaR_time = AlphaR.ping();
        differenceL = AlphaL_time - goal;
        differenceR = AlphaR_time - goal;
        int movement = (differenceR + differenceL) / 2; //movement to give motion towards or away from wall
        differenceL = abs(differenceL);
        differenceR = abs(differenceR);
        //motion adjustment based on positive or negative movement to or from wall
        if(movement > old_movement && positive == 1){
           offset_Alpha(multiple);
           break;
        }
        if(old_movement>movement && negative == 1){
          offset_Alpha(multiple);
          break;
        }
        if (movement > 0 && gone == 0) {
          old_movement = movement;
            positive=1;
            gone = 1;
              M1Motor->run(FORWARD);
              M3Motor->run(BACKWARD);
          }
          if (movement < 0 && gone == 0) {
              negative = 1;
              gone = 1;
              M1Motor->run(BACKWARD);
              M3Motor->run(FORWARD);
          }
        }
    M1Motor->run(RELEASE);
    M3Motor->run(RELEASE); //turn off motors
    delay(50);
}

//adjust offset from wall based on ultrasonic readings
void offset_Bravo(int multiple)
{
    M2Motor->setSpeed(M2Speed);
    M4Motor->setSpeed(M4Speed);
    int goal = multiple * 1780 + 215; //goal distance based on multiple parameter from blocks between wall
    unsigned int BravoR_time = BravoR.ping_median(5); // Send ping, get ping time in microseconds (uS).
    unsigned int BravoL_time = BravoL.ping_median(5); // Send ping, get ping time in microseconds (uS).
    //difference from goal and absolute value for error correction
    int differenceL = BravoL_time - goal;
    int differenceR = BravoR_time - goal;
    differenceL = abs(differenceL);
    differenceR = abs(differenceR);
    //loop for error correction
    int gone = 0;
    int old_movement;
    int positive = 0;
    int negative = 0;
    while (differenceR > 50 && differenceL > 50) {
        BravoL_time = BravoL.ping();
        BravoR_time = BravoR.ping();
        differenceL = BravoL_time - goal;
        differenceR = BravoR_time - goal;
        int movement = (differenceR + differenceL) / 2; //movement to give motion towards or away from wall
        differenceL = abs(differenceL);
        differenceR = abs(differenceR);
        //motion adjustment based on positive or negative movement to or from wall
        if (movement > old_movement && positive == 1) {
          offset_Bravo(multiple);
          break;
        }
        if (old_movement>movement && negative == 1){
          offset_Bravo(multiple);
          break;
        }
        if (movement > 0 && gone == 0) {
            old_movement = movement;
            positive = 1;
            gone = 1;
            M2Motor->run(BACKWARD);
            M4Motor->run(FORWARD);
        }
        if (movement < 0 && gone == 0) {
            old_movement = movement;
            negative = 1;
            gone = 1;
            M2Motor->run(FORWARD);
            M4Motor->run(BACKWARD);
        }
    }
    M2Motor->run(RELEASE);
    M4Motor->run(RELEASE); //turn off motors
    delay(50);
}

//adjust offset from wall based on ultrasonic readings
void offset_Charlie(int multiple)
{
    M1Motor->setSpeed(M1Speed);
    M3Motor->setSpeed(M3Speed);
    int goal = multiple * 1780 + 215; //goal distance based on multiple parameter from blocks between wall
    unsigned int CharlieR_time = CharlieR.ping_median(5); // Send ping, get ping time in microseconds (uS).
    unsigned int CharlieL_time = CharlieL.ping_median(5); // Send ping, get ping time in microseconds (uS).
    //difference from goal and absolute value for error correction
    int differenceL = CharlieL_time - goal;
    int differenceR = CharlieR_time - goal;
    differenceL = abs(differenceL);
    differenceR = abs(differenceR);
    //loop for error correction
    int gone = 0;
    int old_movement;
    int positive = 0;
    int negative = 0;
    while (differenceR > 50 && differenceL > 50) {
        CharlieL_time = CharlieL.ping();
        CharlieR_time = CharlieR.ping();
        differenceL = CharlieL_time - goal;
        differenceR = CharlieR_time - goal;
        int movement = (differenceR + differenceL) / 2; //movement to give motion towards or away from wall
        differenceL = abs(differenceL);
        differenceR = abs(differenceR);
        //motion adjustment based on positive or negative movement to or from wall
        if (movement > old_movement && positive == 1) {
          offset_Charlie(multiple);
          break;
        }
        if (old_movement > movement && negative == 1){
          offset_Charlie(multiple);
          break;
        }
        if (movement > 0 && gone == 0) {
            old_movement = movement;
            positive = 1;
            gone = 1;
            M1Motor->run(BACKWARD);
            M3Motor->run(FORWARD);
        }
        if (movement < 0 && gone == 0) {
            old_movement = movement;
            negative = 1;
            gone = 1;
            M1Motor->run(FORWARD);
            M3Motor->run(BACKWARD);
        }
    }
    M1Motor->run(RELEASE);
    M3Motor->run(RELEASE); //turn off motors
    delay(50);
}

//adjust offset from wall based on ultrasonic readings
void offset_Delta(int multiple)
{
    M2Motor->setSpeed(M2Speed);
    M4Motor->setSpeed(M4Speed);
    int goal = multiple * 1780 + 215; //goal distance based on multiple parameter from blocks between wall
    unsigned int DeltaR_time = DeltaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
    unsigned int DeltaL_time = DeltaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
    //difference from goal and absolute value for error correction
    int differenceL = DeltaL_time - goal;
    int differenceR = DeltaR_time - goal;
    differenceL = abs(differenceL);
    differenceR = abs(differenceR);
    //loop for error correction
    int gone = 0;
    int old_movement;
    int positive = 0;
    int negative = 0;
    while (differenceR > 50 && differenceL > 50) {
        DeltaL_time = DeltaL.ping();
        DeltaR_time = DeltaR.ping();
        differenceL = DeltaL_time - goal;
        differenceR = DeltaR_time - goal;
        int movement = (differenceR + differenceL) / 2; //movement to give motion towards or away from wall
        differenceL = abs(differenceL);
        differenceR = abs(differenceR);
        //motion adjustment based on positive or negative movement to or from wall
        if (movement > old_movement && positive == 1){
          offset_Delta(multiple);
          break;
        }
        if (old_movement > movement && negative == 1){
          offset_Delta(multiple);
          break;
        }
        if (movement > 0 && gone == 0) {
            old_movement = movement;
            positive = 1;
            gone = 1;
            M2Motor->run(FORWARD);
            M4Motor->run(BACKWARD);
        }
        if (movement < 0 && gone == 0) {
            old_movement = movement;
            negative = 1;
            gone = 1;
            M2Motor->run(BACKWARD);
            M4Motor->run(FORWARD);
        }
    }
    M2Motor->run(RELEASE);
    M4Motor->run(RELEASE); //turn off motors
    delay(50);
}

//align when not parrallel with wall
void align_Alpha()
{
    //set motor speed
    M1Motor->setSpeed(M1Speed - 10);
    M3Motor->setSpeed(M3Speed - 10);
    M2Motor->setSpeed(M2Speed - 10);
    M4Motor->setSpeed(M4Speed - 10);
    //get time and difference between two sensors
    unsigned int AlphaR_time = AlphaR.ping_median(5);
    unsigned int AlphaL_time = AlphaL.ping_median(5);
    int difference = AlphaR_time - AlphaL_time;
    difference = abs(difference);
    //loop that turns until parrallel with wall
    int goal = 10;
    int ccw = 0;
    while (difference > goal) {
        AlphaL_time = AlphaL.ping();
        AlphaR_time = AlphaR.ping();
        difference = AlphaR_time - AlphaL_time;
        //when difference is positive turn cw, when negative turn ccw
        if (difference > 0) {
            if (ccw == 1) {
                break;
            }
            M1Motor->run(FORWARD);
            M2Motor->run(FORWARD);
            M3Motor->run(FORWARD);
            M4Motor->run(FORWARD);
        }

        if (difference < 0) {
            ccw = 1;
            M1Motor->run(BACKWARD);
            M2Motor->run(BACKWARD);
            M3Motor->run(BACKWARD);
            M4Motor->run(BACKWARD);
        }

        difference = abs(difference);
    }
    //turn off motors
    M1Motor->run(RELEASE);
    M2Motor->run(RELEASE);
    M3Motor->run(RELEASE);
    M4Motor->run(RELEASE);
    delay(50);
}

//align when not parrallel with wall
void align_Bravo()
{
    //set motor speed
    M1Motor->setSpeed(M1Speed - 10);
    M3Motor->setSpeed(M3Speed - 10);
    M2Motor->setSpeed(M2Speed - 10);
    M4Motor->setSpeed(M4Speed - 10);
    //get time and difference between two sensors
    unsigned int BravoR_time = BravoR.ping_median(5);
    unsigned int BravoL_time = BravoL.ping_median(5);
    int difference = BravoR_time - BravoL_time;
    difference = abs(difference);
    //loop that turns until parrallel with wall
    int goal = 10;
    int ccw = 0;
    while (difference > goal) {
        BravoL_time = BravoL.ping();
        BravoR_time = BravoR.ping();
        difference = BravoR_time - BravoL_time;
        //when difference is positive turn cw, when negative turn ccw
        if (difference > 0) {
            if (ccw == 1) {
                break;
            }
            M1Motor->run(FORWARD);
            M2Motor->run(FORWARD);
            M3Motor->run(FORWARD);
            M4Motor->run(FORWARD);
        }

        if (difference < 0) {
            ccw = 1;
            M1Motor->run(BACKWARD);
            M2Motor->run(BACKWARD);
            M3Motor->run(BACKWARD);
            M4Motor->run(BACKWARD);
        }

        difference = abs(difference);
    }
    //turn off motors
    M1Motor->run(RELEASE);
    M2Motor->run(RELEASE);
    M3Motor->run(RELEASE);
    M4Motor->run(RELEASE);
    delay(50);
}

//align when not parrallel with wall
void align_Charlie()
{
    //set motor speed
    M1Motor->setSpeed(M1Speed - 10);
    M3Motor->setSpeed(M3Speed - 10);
    M2Motor->setSpeed(M2Speed - 10);
    M4Motor->setSpeed(M4Speed - 10);
    //get time and difference between two sensors
    unsigned int CharlieR_time = CharlieR.ping_median(5);
    unsigned int CharlieL_time = CharlieL.ping_median(5);
    int difference = CharlieR_time - CharlieL_time;
    int goal = 10;
    int ccw = 0;
    difference = abs(difference);
    //loop that turns until parrallel with wall
    while (difference > goal) {
        CharlieL_time = CharlieL.ping();
        CharlieR_time = CharlieR.ping();
        difference = CharlieR_time - CharlieL_time;
        //when difference is positive turn cw, when negative turn ccw
        if (difference > 0) {
            if (ccw == 1) {
                break;
            }
            M1Motor->run(FORWARD);
            M2Motor->run(FORWARD);
            M3Motor->run(FORWARD);
            M4Motor->run(FORWARD);
        }

        if (difference < 0) {
            ccw = 1;
            M1Motor->run(BACKWARD);
            M2Motor->run(BACKWARD);
            M3Motor->run(BACKWARD);
            M4Motor->run(BACKWARD);
        }

        difference = abs(difference);
    }
    //turn off motors
    M1Motor->run(RELEASE);
    M2Motor->run(RELEASE);
    M3Motor->run(RELEASE);
    M4Motor->run(RELEASE);
    delay(50);
}

//align when not parrallel with wall
void align_Delta()
{
    //set motor speed
    M1Motor->setSpeed(M1Speed - 10);
    M3Motor->setSpeed(M3Speed - 10);
    M2Motor->setSpeed(M2Speed - 10);
    M4Motor->setSpeed(M4Speed - 10);
    //get time and difference between two sensors
    unsigned int DeltaR_time = DeltaR.ping_median(5);
    unsigned int DeltaL_time = DeltaL.ping_median(5);
    int difference = DeltaR_time - DeltaL_time;
    int goal = 10;
    int ccw = 0;
    difference = abs(difference);
    //loop that turns until parrallel with wall
    while (difference > goal) {
        DeltaL_time = DeltaL.ping();
        DeltaR_time = DeltaR.ping();
        difference = DeltaR_time - DeltaL_time;
        //when difference is positive turn cw, when negative turn ccw
        if (difference > 0) {
            if (ccw == 1) {
                break;
            }
            M1Motor->run(FORWARD);
            M2Motor->run(FORWARD);
            M3Motor->run(FORWARD);
            M4Motor->run(FORWARD);
        }

        if (difference < 0) {
            ccw = 1;
            M1Motor->run(BACKWARD);
            M2Motor->run(BACKWARD);
            M3Motor->run(BACKWARD);
            M4Motor->run(BACKWARD);
        }

        difference = abs(difference);
    }
    //turn off motors
    M1Motor->run(RELEASE);
    M2Motor->run(RELEASE);
    M3Motor->run(RELEASE);
    M4Motor->run(RELEASE);
    delay(50);
}

//Calibrate to block
void Calibrate()
{
    if (verboseSerial == 1)
        Serial.println("Calibrate() called");
    if (x_pos < 3) {
        align_Bravo();
        offset_Bravo(x_pos);
        align_Bravo();
    }
    else {
        align_Delta();
        offset_Delta(6 - x_pos);
        align_Delta();
    }

    if (y_pos < 4) {
        offset_Charlie(y_pos);
        align_Charlie();
    }
    else {
        offset_Alpha(6 - y_pos);
        align_Alpha();
    }
}

void find_obstacles()
{
    unsigned int AlphaL_time = AlphaL.ping_median(5);
    unsigned int AlphaR_time = AlphaR.ping_median(5);
    int Alpha_obstacle = (AlphaL_time + AlphaR_time) / 2;
    unsigned int DeltaL_time = DeltaL.ping_median(5);
    unsigned int DeltaR_time = DeltaR.ping_median(5);
    int Delta_obstacle = (DeltaL_time + DeltaR_time) / 2;

    if (Alpha_obstacle > 2300 && Alpha_obstacle < 2900) {
        Serial.println("three blocks away");
        Forward(oneBlock * 2);
    }
    if (Alpha_obstacle > 580 && Alpha_obstacle < 2300) {
        Serial.println("two blocks away");
        Forward(oneBlock);
    }
    if (Alpha_obstacle > 0 && Alpha_obstacle < 580) {
        Serial.println("one block away");
    }
    delay(1000);

    Serial.println(Alpha_obstacle);

    if (Delta_obstacle > 4050 && Delta_obstacle < 5770) {
        Serial.println("four blocks away");
    }
    if (Delta_obstacle > 2300 && Delta_obstacle < 4050) {
        Serial.println("three blocks away");
    }
    if (Delta_obstacle > 580 && Delta_obstacle < 2300) {
        Serial.println("two blocks away");
    }
    if (Delta_obstacle > 0 && Delta_obstacle < 580) {
        Serial.println("one block away");
    }
    delay(1000);
}

/*******************************************************COMM WITH CORY*****************************************************************/

// function for placement of camera to cache lid
void Camera(int cache_x, int cache_y)
{

    if (cache_x == 0) {
        Go_to(cache_x + 1, cache_y);    Calibrate();

        turnCW(3 * Ninety);
        align_Alpha();
        offset_Alpha(x_pos);
        if (y_pos < 4) {
            align_Bravo();
            offset_Bravo(y_pos);
        }
        else {
            align_Delta();
            offset_Delta(6 - y_pos);
        }
    }

    else if (cache_x == 6) {
        Go_to(cache_x - 1, cache_y);    Calibrate();

        turnCW(Ninety);
        align_Alpha();
        offset_Alpha(6 - x_pos);
        if (y_pos < 4) {
            align_Delta();
            offset_Delta(y_pos);
        }
        else {
            align_Bravo();
            offset_Bravo(6 - y_pos);
        }
    }

    else if (cache_y == 0) {
        Go_to(cache_x, cache_y + 1);    Calibrate();

        turnCW(Ninety * 2);
        align_Alpha();
        offset_Alpha(y_pos);
        if (x_pos < 4) {
            align_Delta();
            offset_Delta(x_pos);
        }
        else {
            align_Bravo();
            offset_Bravo(6 - x_pos);
        }
    }
    else if (cache_y == 6) {
        Go_to(cache_x, cache_y - 1);  Calibrate();

        //no need to turn
        align_Alpha();
        offset_Alpha(6 - y_pos);
        if (x_pos < 4) {
            align_Bravo();
            offset_Bravo(x_pos);
        }
        else {
            align_Delta();
            offset_Delta(6 - x_pos);
        }
    }
    align_Alpha(); offset_Alpha(1);
    align_Alpha();
    delay(200);
    offset_camera(); //get to a close position on cache
    delay(200);
    Cory();
    Serial.println("Made it out of Cory()");
    delay(200);
    matrix.show();
    Serial.println("Matrix re-displayed");
    align_Alpha();
    offset_Alpha(1);
    if(y_pos==1){turnCW(Ninety*2); Serial.println("Turned 180 degrees."); delay(200); Calibrate();}
    else if(x_pos==5){turnCW(Ninety*3); Serial.println("Turned 270 degrees."); delay(200); Calibrate();}
    else if(x_pos==1){turnCW(Ninety*1); Serial.println("Turned 90 degrees."); delay(200); Calibrate();}
    Serial.print("this is my x,y"); Serial.print(x_pos); Serial.println(y_pos);
    Go_to(1,1);  Calibrate();
    Left(oneBlock/2);  Backward(oneBlock);
    align_Bravo(); Left(oneBlock/2);
    Calibrate();
}

//moves close to lid
void offset_camera()
{
    M1Motor->setSpeed(M1Speed);
    M3Motor->setSpeed(M3Speed);
    int goal = 1200; //goal distance based on multiple parameter from blocks between wall
    unsigned int AlphaR_time = AlphaR.ping_median(5); // Send ping, get ping time in microseconds (uS).
    unsigned int AlphaL_time = AlphaL.ping_median(5); // Send ping, get ping time in microseconds (uS).
    Serial.println(AlphaR_time);
    Serial.println(AlphaL_time);
    //difference from goal and absolute value for error correction
    int differenceL = AlphaL_time - goal;
    int differenceR = AlphaR_time - goal;
    differenceL = abs(differenceL);
    differenceR = abs(differenceR);
    //loop for error correction
    while (differenceR > 50 && differenceL > 50) {
      Serial.println("in loop");
        AlphaL_time = AlphaL.ping();
        AlphaR_time = AlphaR.ping();
        differenceL = AlphaL_time - goal;
        differenceR = AlphaR_time - goal;
        int movement = (differenceR + differenceL) / 2; //movement to give motion towards or away from wall
        differenceL = abs(differenceL);
        differenceR = abs(differenceR);
        //motion adjustment based on positive or negative movement to or from wall
        if (movement > 0) {
            M1Motor->run(FORWARD);
            M3Motor->run(BACKWARD);
        }
        if (movement < 0) {
            M1Motor->run(BACKWARD);
            M3Motor->run(FORWARD);
        }
    }
        M1Motor->run(RELEASE);
        M3Motor->run(RELEASE);
        delay(200);
}

void Cory()
{
    bool cory = true;

    digitalWrite(TRANSMIT, HIGH);
    delay(100);
    digitalWrite(TRANSMIT, LOW);

 

    while (cory) {
        int move_up = digitalRead(UP);
        int move_down = digitalRead(DOWN);
        int move_left = digitalRead(LEFT);
        int move_right = digitalRead(RIGHT);

        //move based on high pin from cory
        if (move_up) {
            M1Motor->run(FORWARD);
            M3Motor->run(BACKWARD);
            delay(100);
            M1Motor->run(RELEASE);
            M3Motor->run(RELEASE);
        } //forward 1/5 in
        if (move_down) {
            M1Motor->run(BACKWARD);
            M3Motor->run(FORWARD);
            delay(100);
            M1Motor->run(RELEASE);
            M3Motor->run(RELEASE);
        } //backward
        if (move_right) {
            M2Motor->run(FORWARD);
            M4Motor->run(BACKWARD);
            delay(100);
            M2Motor->run(RELEASE);
            M4Motor->run(RELEASE);
        } //right
        if (move_left) {
            M2Motor->run(BACKWARD);
            M4Motor->run(FORWARD);
            delay(100);
            M2Motor->run(RELEASE);
            M4Motor->run(RELEASE);
        } //left
        if (digitalRead(RECEIVE)) {
            cory = false;
        } // get out of loop when cory is finished
    }
    //delay(3000);
    //grab lid and place on porch with security guard
    delay(1000);

    for (int down = 0; down <= 180; down += 1) {
        grabber.write(down);
        delay(10);
    }
    delay(500);

    for (int pos = 180; pos >= 0; pos -= 1) {
        grabber.write(pos);
        delay(10);
    }
    delay(1000);
    delay(4000);
    delay(4000);

    //This tells Cory that servo is moved
    //digitalWrite(TRANSMIT, HIGH);
    //delay(100);
    //Serial.println("Waiting for Cory's slow ass...");
    
    //This loop will wait until Pi is done with die
    //while (!digitalRead(RECEIVE)) {
      //Serial.println("inloop")
      //delay(10);
    //}
    //Serial.println("FINALLY!");
}


/**************************************************************************************INTERRUPT FUNCTIONS************************************************************************************/
//functions that will count the rising edge of interrupts for each motor
void M1count() { M1tick++; }
void M2count() { M2tick++; }
void M3count() { M3tick++; }
void M4count() { M4tick++; }

/****************************************************************************************CONTINUOUS LOOP**************************************************************************************/

void loop()
{
}

