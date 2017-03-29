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

void setup() {
  // put your setup code here, to run once:
    Serial.begin(57600);


}

void loop() {

  //find_obstacles(){
  unsigned int AlphaL_time = AlphaL.ping_median(5);
  unsigned int AlphaR_time = AlphaR.ping_median(5);
  int Alpha_obstacle = (AlphaL_time + AlphaR_time)/2;
  unsigned int DeltaL_time = DeltaL.ping_median(5);
  unsigned int DeltaR_time = DeltaR.ping_median(5); 
  int Delta_obstacle = (DeltaL_time + DeltaR_time)/2;
  unsigned int BravoL_time = BravoL.ping_median(5);
  unsigned int BravoR_time = BravoR.ping_median(5);
  int Bravo_obstacle = (BravoL_time + BravoR_time)/2;
  unsigned int CharlieL_time = CharlieL.ping_median(5);
  unsigned int CharlieR_time = CharlieR.ping_median(5); 
  int Charlie_obstacle = (CharlieL_time + CharlieR_time)/2;
  Serial.print(Alpha_obstacle);Serial.print("  ||  ");Serial.print(Bravo_obstacle);Serial.print("  ||  ");Serial.print(Charlie_obstacle);Serial.print("  ||  ");Serial.println(Delta_obstacle);  


/*
  if(Alpha_obstacle>4050 && Alpha_obstacle<5770){
    Serial.println("four blocks away");
  }
  if(Alpha_obstacle>2300 && Alpha_obstacle<4050){
    Serial.println("three blocks away");
  }
  if(Alpha_obstacle>580 && Alpha_obstacle<2300){
    Serial.println("two blocks away");
  }
  if(Alpha_obstacle>0 && Alpha_obstacle<580){
    Serial.println("one block away");
  }
  delay(1000);

  Serial.println(Alpha_obstacle);

  if(Delta_obstacle>4050 && Delta_obstacle<5770){
    Serial.println("four blocks away");
  }
  if(Delta_obstacle>2300 && Delta_obstacle<2700){
    Serial.println("three blocks away");
  }
  if(Delta_obstacle>580 && Delta_obstacle<2300){
    Serial.println("two blocks away");
  }
  if(Delta_obstacle>0 && Delta_obstacle<580){
    Serial.println("one block away");
  }

  */
  delay(1000);
//}
  //NewPing AlphaR(AlphaR_TRIGGER, AlphaR_ECHO, MAX_DISTANCE);  
//  unsigned int CharlieL_time = CharlieL.ping_median(11); // Send ping, get ping time in microseconds (uS).
//  unsigned int CharlieR_time = CharlieR.ping_median(11);
//  unsigned int BetaL_time = BetaL.ping_median(11); // Send ping, get ping time in microseconds (uS).
//  unsigned int BetaR_time = BetaR.ping_median(11);
//  unsigned int AlphaL_time = AlphaL.ping_median(11); // Send ping, get ping time in microseconds (uS).
//  unsigned int AlphaR_time = AlphaR.ping_median(11);
//  unsigned int DeltaL_time = DeltaL.ping_median(11); // Send ping, get ping time in microseconds (uS).
//  unsigned int DeltaR_time = DeltaR.ping_median(11);
//  Serial.print("Charlie_L   "); Serial.print("Charlie_R   "); Serial.print("Beta_L    "); Serial.print("Beta_R  "); Serial.print("Alpha_L  "); Serial.print("Alpha_R  "); Serial.print("Delta_L  "); Serial.println("Delta_R  ");
//  Serial.print(CharlieL_time); Serial.print("        "); Serial.print(CharlieR_time); Serial.print("           "); Serial.print(BetaL_time); Serial.print("         "); Serial.print(BetaR_time);Serial.print("    "); Serial.print(AlphaL_time); Serial.print("     "); Serial.print(AlphaR_time); Serial.print("     "); Serial.print(DeltaL_time); Serial.print("        "); Serial.println(DeltaR_time);
//  //pinMode(AlphaR_ECHO,OUTPUT);
  //digitalWrite(AlphaR_ECHO,LOW);
  //pinMode(AlphaR_ECHO,INPUT);
//  Serial.print("AlphaR: ");
  //Serial.println(BetaR_time); // Convert ping time to distance in cm and print result (0 = outside set distance range)
//  Serial.println("us  ");
  //NewPing AlphaL(AlphaL_TRIGGER, AlphaL_ECHO, MAX_DISTANCE);  
  //pinMode(AlphaL_ECHO,OUTPUT);
  //digitalWrite(AlphaL_ECHO,LOW);
  //pinMode(AlphaL_ECHO,INPUT);
  //Serial.print("AlphaL: ");
  //Serial.println(BetaL_time); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  //Serial.println("us  ");
  // put your main code here, to run repeatedly:

}
