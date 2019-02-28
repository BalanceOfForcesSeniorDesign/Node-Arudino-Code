#include <RF24.h>
#include <RadioConfig.h>


#define CSN_PIN 10
#define CE_PIN 11
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN


int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A1;   //analog pin 1

float diff = 0;

void setup(void)
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  setupAckRadio(1, radio);
}

void loop() {

  while(!radio.available())
{
  float flexiForceReading1 = analogRead(flexiForcePin1);
  float flexiForceReading2 = analogRead(flexiForcePin2);
  diff =  (flexiForceReading1) - (flexiForceReading2);
  digitalWrite(13, LOW);    // turn the LED off by making the voltage HIGH
}
    Serial.println("Pinged for voltage difference");
    radio.writeAckPayload(1, &diff, sizeof(diff)); // Send back the ratio
    byte gotByte;
    radio.read(&gotByte, 1); // Recieve the packet
    digitalWrite(13, HIGH);    // turn the LED off by making the voltage HIGH
}
