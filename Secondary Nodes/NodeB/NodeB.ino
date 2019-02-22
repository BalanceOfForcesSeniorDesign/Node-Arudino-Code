#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <NodeConfig.h>
#include <RF24Network.h>

#define CSN_PIN 10
#define CE_PIN 11


RF24 radio(CE_PIN, CSN_PIN); // CE, CSN
RF24Network network(radio); 

const uint16_t this_node = 0x02;        // Address of our node in Octal format
const uint16_t other_node = 0x00;       // Address of the other node in Octal format
const unsigned long interval = 1; //ms  // How often to send 'hello world to the other unit
unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already

int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A2;   //analog pin 1

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif


float diff = 0;

typedef struct payload_t {
  float diff;
};



void setup() 
{

  Serial.begin(115200);
  Serial.println("Starting the radio chip");
 // radio.begin();
 // radio.setPALevel(RF24_PA_MIN);
 // radio.openWritingPipe(NODE_ADR[1]);
 // radio.setRetries(15, 15);
  pinMode(13, OUTPUT);
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);
  Serial.println("Finished setting up the RF chip.");
}

void loop() 
{
  network.update();                          // Check the network regularly
  
  float flexiForceReading1 = analogRead(flexiForcePin1); 
  float flexiForceReading2 = analogRead(flexiForcePin2); 

 // Serial.print(flexiForceReading1);
 // Serial.print(flexiForceReading2);

  diff =  (flexiForceReading1)-(flexiForceReading2);


  unsigned long now = millis();              // If it's time to send a message, send it!
  if ( now - last_sent >= interval  )
  {
    last_sent = now;
    Serial.print("Sending...");
    Serial.print(diff);
    struct payload_t payload;
    payload.diff = diff;
    RF24NetworkHeader header(/*to node*/ other_node);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");
  }
  
 // sendPacket(send_packet);

  
}


/*void sendPacket(struct send_data packet)
{
 bool rslt;
 radio.stopListening();
 rslt = radio.write(&packet, sizeof(packet));
 radio.startListening();
 Serial.print("Data Sent ");
 if (rslt) {
 Serial.println("  Acknowledge received");
 digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
 } else {
            Serial.println("  Tx failed");
             digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
        }
        
 }*/

 
