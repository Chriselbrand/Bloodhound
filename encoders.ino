int M1tick = 0; int M2tick = 0; int M3tick= 0 ; int M4tick = 0;
int M1interrupt = 19; int M2interrupt = 18;
int M3interrupt = 3; int M4interrupt = 2;

void setup(){
Serial.begin(9600);



pinMode(M1interrupt, INPUT_PULLUP); pinMode(M2interrupt, INPUT_PULLUP);
pinMode(M3interrupt, INPUT); pinMode(M4interrupt, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(19),M1count,RISING);
attachInterrupt(digitalPinToInterrupt(18),M2count,RISING);
attachInterrupt(digitalPinToInterrupt(3),M3count,RISING);
attachInterrupt(digitalPinToInterrupt(2),M4count,RISING);
}

void M1count(){
  M1tick++;
}

void M2count(){
  M2tick++;
}

void M3count(){
  M3tick++;
}

void M4count(){
  M4tick++;
}

void loop(){
  Serial.print(M1tick);
  Serial.print(M2tick);
  Serial.print(M3tick);
  Serial.println(M4tick);

}

