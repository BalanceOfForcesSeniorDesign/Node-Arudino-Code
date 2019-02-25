#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#define CSN_PIN 10
#define CE_PIN 11


RF24 radio(CE_PIN, CSN_PIN); // CE, CSN   

RF24Network network(radio);          // Network uses that radio

const uint16_t node_P = 01;    // Address of our node in Octal format ( 04,031, etc)
const uint16_t node_B = 021;   // Address of the other node in Octal format

int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A2;   //analog pin 1

const unsigned long interval = 1; //ms  // How often to send 'hello world to the other unit

unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already

float diff = 0;

typedef struct payload_t {
  float diff;
};

void setup(void)
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  SPI.begin();
  radio.begin();
  network.begin(node_B);
}

void loop() {
  
  network.update();                          // Check the network regularly

  float flexiForceReading1 = analogRead(flexiForcePin1); 
  float flexiForceReading2 = analogRead(flexiForcePin2); 
  diff =  (flexiForceReading1)-(flexiForceReading2);
  
  unsigned long now = millis();              // If it's time to send a message, send it!
  if ( now - last_sent >= interval  )
  {
    last_sent = now;

    Serial.print("Sending...");
    payload_t payload = {diff};
    RF24NetworkHeader header(node_P);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok){
      Serial.println("ok.");
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else{
      Serial.println("failed.");
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    }
  }
}
