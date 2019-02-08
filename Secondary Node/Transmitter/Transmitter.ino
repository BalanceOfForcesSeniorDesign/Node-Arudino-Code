#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#define CSN_PIN 10
#define CE_PIN 11


RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A1;   //analog pin 1

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

const byte rxAddr[6] = "00002";

float ratio = 0;

typedef struct send_data {
  float ratio;
};



void setup() 
{
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting the radio chip");
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(rxAddr);
  radio.setRetries(15, 15);

  Serial.println("Finished setting up the RF chip.");
}

void loop() 
{
  float flexiForceReading1 = analogRead(flexiForcePin1); 
  float flexiForceReading2 = analogRead(flexiForcePin2); 

  Serial.print(flexiForceReading1);
  Serial.print(flexiForceReading2);


  struct send_data send_packet;
  ratio =  (flexiForceReading1)-(flexiForceReading2);
  send_packet.ratio = ratio;
  sendPacket(send_packet);

  delay(10);
  Serial.println(ratio);
  
}


void sendPacket(struct send_data packet)
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
        
 }

 
