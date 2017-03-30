

// UNO IEEE Robotics Competition 2017: Listen for 60Hz from Klein NCVT-2 Tic Tracer 
// and listen for hollow/solid signatures from Electret Microphone
// --By Thomas Miller    v1.0 11/23/2016
// Modified for Bloodhound Robot - 03/16/17 - Initial Mod
//
// How?: Analog data samples fill an array, ARM CMSIS DSP Library driven Fast Fourier Transform
// is performed on the data. Output is the magnitudes[] array. The array is a distribution of 'bins'
// containing frequency ranges. i.e. At 1000Hz sample rate and 256 FFT size, Bin 15 is ~58-62 Hz.
// The magnitude result is with respect to the amplitude of the signal at that frequency.
//
// What does this do?:
// This is an example code which cycles through the inputs, taking readings and serial printing them.
//
// What about YOUR TEAM? 
// Your implementation depends on how you want to establish communication rapport, depending upon your 
// coding abilities and the nuances of your particular robot design. I'll lay out some of the simple
// methods, but the sky is the limit in how many ways YOU can make this more elegant!
//
// Tic Tracer:
// One easy method would be to take Tic Tracer readings continuously, outputting them as a voltage via
// analogWrite- reading that voltage from your other Arduino to interpret as a distance to the wire.
// This can be happening continuously, so that the voltage relative to wire distance is always sitting
// on that analogWrite pin, for whenever your Arduino wants to read it.
// Hint: Use 'map' to scale the Tic Tracer magnitudes to ADC voltage output values.
//
// Microphone:
// Since knocking happens in an instance, one very easy way to do it is to have your other Arduino
// send a knock to the motor, and turn on a digital pin, have a condition in THIS Teensy code that
// IF it sees that pin is on, switch over and start listening to the microphone for the KNOCK.
// and IF it sees the very distinct frequency bins for HOLLOW pass some threshold, have the Teensy
// turn on a digital pin. Read that digital pin the Teensy turned on at your Arduino. You've now told
// the Arduino we heard a HOLLOW knock. Then, let it go back to Tic Tracer work.

// This code is based on the example code from Adafruit written by Tony DiCola for the Fun with FFT guide.
// A lot more background can be found here: http://learn.adafruit.com/fft-fun-with-fourier-transforms/
// This code is written for the Teensy 3.2 microcontroller, by Paul Stoffregen at PJRC


#define ARM_MATH_CM4
#include <arm_math.h>

/////////////////////////////
//CONFIGURATION VALUE INPUT//
/////////////////////////////

int SAMPLE_RATE_HZ = 1000;     //Tic or Knock Audio input sampling rate in Hertz. (Range: 100-9000Hz)
                               //A lower number allows a tighter range of frequencies in each FFT output bin.
const int FFT_SIZE = 256;      //Size of Fast Fourier Transform. Max: 256. Defines number of 'bins' of frequency resolution.
                               //i.e. (SampleRate[1000]/FFTsize[256])=4Hz per-'bin' distribution. 600/256=2Hz per bin.
                               //For faster FFTs, try lowering this size, but also lower the sample rate so you don't look
                               //at too wide a range of frequency-per-bin.

//Input Device Pin Configuration        //If using the example loop I made, have these in successive # order (i.e. TIC1=14,TIC2=15,MIC1=16,MIC2=17)
const int TIC1_TRACER_PIN = 15;         //Plug the output of your Tic Tracer into this I/O pin on the Teensy.
const int MIC1_AUDIO_PIN =  14;         //Plug the output of your Electret Microphone into this I/O pin on the Teensy.

//Analog Input Signal Read Settings
const int ANALOG_READ_RESOLUTION = 12; //This is the ADC reading resolution. I.E.: 10-bit=0-1024 (UNO/MEGA maximum), 13-bit=0-8192 (Teensy Usable Maximum), 16-bit=0-65536 (Teensy Actual Maximum Using Precision AREF Resistor)
const int ANALOG_READ_AVERAGING = 20; //If you want to average X number of readings together PER analog read, before putting that number into the array that will be used to perform the FFT, this is where you adjust raw analog sample reading averaging.

//Other Things
const int POWER_LED_PIN = 13;    //The Teensy's on-board power LED uses this pin. It's optional, but you can use it to signal you things, or just to say "I'm powered up".
const int TIC1_ON_PULSE_PIN = 18; //Turn on the Tic Tracer. Turn it on only for a short period. Delay value between 500-1500ms works. Simulates pressing the button on the Tic Tracer.
const int SHOW_ME_THE_KNOCKERS = 19; //Boolean Digital I/O pin to Start reading the "knocking" solenoid to see if its hollow or solid
const int KNOCKER_SOLENOID_PULSE_PIN = 23; //pulse to MOSFET to trigger knocking solenoid

//MEGA Communication
const int MEGA_READINGREADY = 1;
const int MEGA_READREQUEST = 2;
const int MEGA_WIREFOUND = 16;
const int MEGA_HOLLOWFOUND = 4;



//Things You Might Want to Research Before Messing With
IntervalTimer samplingTimer;        //Initializes the Teensy's IntervalTimer function as samplingTimer. You can have up to 4 of these interrupt based timers on a Teensy. 
                                    //IntervalTimer uses interrupts to call a function at a precise time. It is initialized (in pseudocode) as IntervalTimer.begin(function-to-run,microseconds-between-runs).
float samples[FFT_SIZE*2];          //This is the sample array of analog data you're pumping through the FFT algorithm. For Nyquist-compliant sampling, sample count is 2x the samples you're trying to discretize.
float magnitudes[FFT_SIZE];         //This is the array of magnitude values output by the FFT algorithm.
                                    //These magnitudes are the magnitudes in each 'bin'. If 60Hz falls in Bin 15, you should call on magnitudes[15]
                                    //yourbinmagnitudevalue = 20.0*log10(magnitudes[##]) can convert it to decibels if you want. 
int sampleCounter = 0;              //sampleCounter global variable says how many analog samples have been logged to the samples array, resets each time sampling completes.
int sampleSource = TIC1_TRACER_PIN; //This tells the sampleBegin() function which source to sample from. By default I set it to the TIC1_INPUT_PIN (Digital 14)
float micADCtotal = 0;
int ticMagnitude = 0;   //Tic Tracer latest magnitude value
float micMagSmooth[FFT_SIZE];
int micMagSmoothCount = 0;
int ticMagSmoothCount = 0;
int ticMagSmooth = 0;
elapsedMillis knockLength; //this resettable elapsed delay allows the knock pulse to be stopped
elapsedMillis readReadyTimer;
elapsedMillis ticOnPulseTimer;
elapsedMillis getUnstuckTimer;
//////////END CONFIGURATION VALUE INPUT//////////////

void setup() {

  Serial.begin(115200); //Set up serial port and set communication baud rate to 115200
  pinMode(TIC1_TRACER_PIN, INPUT);    //initialize the I/O pin as input
  pinMode(MIC1_AUDIO_PIN, INPUT);     //initialize the I/O pin as input
  pinMode(TIC1_ON_PULSE_PIN, OUTPUT); //initialize the Tic Tracer on/off as output
  ticOnPulseTimer = 0;
  analogWriteResolution(11);
  analogReadResolution(ANALOG_READ_RESOLUTION); //Tells the Teensy to perform analog reads at the resolution you set above.
  analogReadAveraging(ANALOG_READ_AVERAGING);   //Tells the Teensy to average X number of samples per each sample.
  pinMode(POWER_LED_PIN, OUTPUT); //Initialize the onboard LED. You can also use this LED for troubleshooting or communicating status' to you.
  pinMode(POWER_LED_PIN, HIGH);   //Turn on the Teensy's onboard LED.
  pinMode(SHOW_ME_THE_KNOCKERS, INPUT); //This pin requests a knock from the Teensy
  pinMode(KNOCKER_SOLENOID_PULSE_PIN, OUTPUT); //Send a knock pin is an output to solenoid
  samplingBegin(); //First things first, gather samples before we even get into the loop.
  pinMode(MEGA_READINGREADY, OUTPUT);
  pinMode(MEGA_READREQUEST, INPUT);
  pinMode(MEGA_WIREFOUND, OUTPUT);
  pinMode(MEGA_HOLLOWFOUND, OUTPUT);
  }

void loop() {
delay(50);
  if(readReadyTimer > 200) digitalWrite(MEGA_READINGREADY, LOW);
  if(ticOnPulseTimer > 1500 && ticOnPulseTimer < 3000){
    digitalWrite(TIC1_ON_PULSE_PIN, HIGH);
  }
  else if (ticOnPulseTimer > 5000 && ticOnPulseTimer < 6000){
      digitalWrite(TIC1_ON_PULSE_PIN, LOW);
  }
  if(samplingIsDone()){
    //////////////////// Run FFT on sample data.////////////////////// https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__cfft__radix4__instance__f32.html
    arm_cfft_radix4_instance_f32 fft_inst;                        //Calls the ARM FFT Library, sets up fft_inst instance for a CFFT of radix4 (base4) at single precision floating point 32-bit numbers.
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);          //Tells the FFT Library you're using instance 'fft_inst', the FFT Bin Size is 'FFT_SIZE', ifftFlag '0' tells it to perform a forward direction FFT (as opposed to inverse '1'), and bitReverseFlag '1' says we want the output bits to stream in the reverse direction.
    arm_cfft_radix4_f32(&fft_inst, samples);                      //Control structure initialization for FFT function
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);             // Calculate magnitude of complex numbers output by the FFT. 'samples' is interleaved as [real, imag, real, imag, real, imag,...] and the first bin is the total bin magnitude (basically ignore it).
    //////////// END FFT Code: Output is magnitudes[] array //////////

      if(sampleSource == TIC1_TRACER_PIN){
        samplingBegin();
        ticMagnitude = magnitudes[15]; //take reading from TIC1_TRACER_PIN reading and store it here before going on to read the microphone.
        twoReadingCombineTIC();
        sampleSource = MIC1_AUDIO_PIN; //change to +1 if using 2nd tic tracer
      }
      else if(sampleSource == MIC1_AUDIO_PIN){
        //Now sample the Microphone://
        knockLength = 0;
        getUnstuckTimer = 0;
        samplingBegin();
        twoReadingCombineMIC();
        sampleSource = TIC1_TRACER_PIN;
      }
      else Serial.println("Invalid Source Configuration");


  }
      //////END samplingIsDone() FFT functions////
    
}



void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(sampleSource);
//  micADCtotal = micADCtotal + (samples[sampleCounter]/100);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;

bool megaRequest = digitalRead(MEGA_READREQUEST);
if(megaRequest == 1){
  if(sampleSource == MIC1_AUDIO_PIN){
        if((knockLength > 1) && (knockLength < 20)){
           if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 0){
           digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, HIGH);
           }
        }
        if((knockLength > 20) && (knockLength < 100)){
          if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 1){
            digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, LOW);
          }
        }
        if((knockLength > 100) && (knockLength < 120)){ //length of readings is like 255
           if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 0){
           digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, HIGH);
           }
        }
        if((knockLength > 120) && (knockLength < 200)){ //length of readings is like 255
          if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 1){
            digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, LOW);
          }
        }
        if((knockLength > 200) && (knockLength < 220)){ //length of readings is like 255
           if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 0){
           digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, HIGH);
           }
        }
        if(knockLength > 221){ //length of readings is like 255
          if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 1){
            digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, LOW);
          }
        }
  }
}
helpMeImStuck();

  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  knockLength = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
  knockLength = 0;
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}
//////////END SAMPLING FUNCTIONS////////////

void hollowOrSolid() {

///////SERIAL OUTPUT FOR TESTING////////////////
    Serial.print(millis());   
    Serial.print(",");             
    Serial.print(ticMagnitude);
    Serial.print(",");
    Serial.print(ticMagSmooth);
    Serial.print(",");
//    analogWrite(A14, ticMagSmooth);
    if(ticMagSmooth > 3000){
      digitalWrite(MEGA_WIREFOUND, HIGH);
            digitalWrite(POWER_LED_PIN, !digitalRead(POWER_LED_PIN)); //Toggle Teensy LED
      Serial.print("WIRES!");
    }
    else{
      digitalWrite(MEGA_WIREFOUND, LOW);
      Serial.print("NOWIRE");
    }
    Serial.print(",");
    int magsum = 0;
    for (int i = 1; i < (FFT_SIZE/2); i++){
        magsum = magsum + micMagSmooth[i];   
    }
    Serial.print(magsum);
    Serial.print(",");
    int hollowcheck = 0;
        for (int i = 30; i < 62; i++){
        hollowcheck = hollowcheck + micMagSmooth[i];   
    }
    if(hollowcheck > 700000){
      digitalWrite(MEGA_HOLLOWFOUND, HIGH);
      Serial.print("HOLLOW: ");Serial.print(hollowcheck);

    }
    else {
      Serial.print("SOLID : ");Serial.print(hollowcheck);
      digitalWrite(MEGA_HOLLOWFOUND, LOW);
    }
    Serial.print(",");
    int solidcheck = 0;
        for (int i = 62; i < 90; i++){
        solidcheck = solidcheck + micMagSmooth[i];   
    }
    if(solidcheck > 650000){
      Serial.print("SOLID : ");Serial.print(solidcheck);
    }
    else {
      Serial.print("HOLLOW: ");Serial.print(solidcheck);
    }
    Serial.print(",");
    Serial.print(micADCtotal);
    micADCtotal = 0;
    int i;                                 //Declare FOR loop count variable
    for (i=0; i < 129; i = i + 1){         //FOR loop i=## is lower bin to print,  i < ## is upper bin to print, plus 1.
      int magOut = (int)(micMagSmooth[i]);   //Declare freq as the value in magnitude array value [i]
      Serial.print(",");                   //Tell me which bin that magnitude was from
      //Serial.print(i);                   //
      //Serial.print(",");                 //Now tell me the magnitude
      if(magOut < 3000){
        magOut = (i);
        Serial.print("0");
      }
      else Serial.print(magOut/1000);                //
                                           //(The overuse of commas makes it easier to Excel import as CSV)
    }                                      //
      Serial.println(",");                 //
    digitalWrite(MEGA_READINGREADY, HIGH);
    readReadyTimer = 0;
    if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 1){
        digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, LOW);
    }
/////////////////////////////////////////////////////////////////////////////////////////////
}

void twoReadingCombineMIC() {

  int j;
    for (j = 0; j < (FFT_SIZE/2); j = j + 1) {
       micMagSmooth[j] = magnitudes[j];
    }
  micMagSmoothCount++;
  if(micMagSmoothCount == 2){
    int k;
    for (k = 0; k < (FFT_SIZE/2); k = k + 1) {
       micMagSmooth[k] = micMagSmooth[k] + magnitudes[k];
    }
    hollowOrSolid();
    micMagSmoothCount = 0;
    int i;
    for (i = 0; i < (FFT_SIZE/2); i = i + 1) {
       micMagSmooth[i] = 0;
    }
  }
}

void twoReadingCombineTIC() {
  if(ticMagSmoothCount == 0){
    ticMagSmooth = 0;
  }
  ticMagSmooth += magnitudes[15];
//  Serial.print("TICMAGSMOOTH:  ");Serial.println(ticMagSmooth);
  ticMagSmoothCount++;
  if(ticMagSmoothCount == 2){
    ticMagSmoothCount = 0;
    }
}

void helpMeImStuck(){
  bool megaRequest = digitalRead(MEGA_READREQUEST);
  if(getUnstuckTimer > 4000){
  megaRequest = digitalRead(MEGA_READREQUEST);
  int escapeWhile = 0;
  while(megaRequest == 1){
    megaRequest = digitalRead(MEGA_READREQUEST);
       if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 0){
        digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, HIGH);
       }
       delay(150);
       if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 1){
        digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, LOW);
       }
       delay(200);
       escapeWhile++;
       if(escapeWhile == 10) {
        megaRequest = 0;
        escapeWhile = 0;
       }
  }
       if(digitalRead(KNOCKER_SOLENOID_PULSE_PIN) == 1){
        digitalWrite(KNOCKER_SOLENOID_PULSE_PIN, LOW);
       }
   if(megaRequest == 0) getUnstuckTimer = 0;
}
}

