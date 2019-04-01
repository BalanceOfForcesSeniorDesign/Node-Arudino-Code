#include <RF24.h>
#include <RadioConfig.h>
#include <Filters.h>


#define CSN_PIN 10
#define CE_PIN 11
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN


int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A2;   //analog pin 1

// filters out changes faster that 5 Hz.
float lowpassFrequency = 5.0;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter1( LOWPASS, lowpassFrequency );  
FilterOnePole lowpassFilter2( LOWPASS, lowpassFrequency );  


int diff = 0;

void setup(void)
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  setupAckRadio(2, radio);
}

void loop() {

  while(!radio.available())
{
  lowpassFilter1.input(analogRead(flexiForcePin1));
  lowpassFilter2.input(analogRead(flexiForcePin2));
  diff =  (lowpassFilter1.output()) - (lowpassFilter2.output());
  digitalWrite(13, LOW);    // turn the LED off by making the voltage HIGH
  Serial.println(diff);
}
    //Serial.println("Pinged for voltage difference");
    radio.writeAckPayload(1, &diff, sizeof(diff)); // Send back the ratio
    byte gotByte;
    radio.read(&gotByte, 1); // Recieve the packet
    digitalWrite(13, HIGH);    // turn the LED off by making the voltage HIGH

}
