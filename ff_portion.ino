#include "arduinoFFT.h"

#define SAMPLES 256             //Must be a power of 2
//the sample rate
#define FS 8000

double vReal[SAMPLES];
double vImag[SAMPLES];

#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;
 
void setup() {
  // put your setup code here, to run once:
   sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

}

void loop() {
  // put your main code here, to run repeatedly:
      for(int i=0; i<SAMPLES; i++)
    {
       if(i == 0 or i == 1){
       tdiff =  micros();  // time between the first and second sample
       }
        microseconds = micros();    //Overflows after around 70 minutes!
     
        vReal[i] = analogRead(INSERT DATA); //sampling
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        } 
 
      double  nfft = i; //number of samples 
      FFT = ZeroFFT( vReal,SAMPLES) 
      double F = (2/tdiff)*FFT*(1/microseconds);
      //matlab line 69 ? (nice)
      mag = abs(F)   //magnitude of the FFT 
      for(int i=0; i<SAMPLES/2; i++){ // for running magnitudes and frequency and magnitude
    
        cur_freq(i) =  FFT_BIN(i, FS, DATA_SIZE)
        Serial.print(FFT_BIN(i, FS, DATA_SIZE));
        Serial.print(" Hz: ");

        //print the corresponding FFT output
        Serial.println(vReal[i]);
      magnitude = vReal[i]  // current magnitude
  }
      
      
         
      
      
      
        
    }
  

}
