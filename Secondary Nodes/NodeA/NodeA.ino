#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <SPI.h>
#include <NodeConfig.h>


#define CSN_PIN 10
#define CE_PIN 11


RF24 radio(CE_PIN, CSN_PIN); // CE, CSN
RF24Network network(radio);
RF24Mesh mesh(radio, network);

int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A2;   //analog pin 1

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif


float diff = 0;
uint32_t displayTimer = 0;

typedef struct send_data {
  float diff;
};

#define nodeID 1

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting the radio chip");

  pinMode(13, OUTPUT);
  mesh.setNodeID(nodeID);
    // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin();
  //radio.begin();
  //radio.setPALevel(RF24_PA_MIN);
  //radio.openWritingPipe(NODE_ADR[0]);
  //radio.setRetries(15, 15);

  Serial.println("Finished setting up the RF chip.");
}

void loop() 
{
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  float flexiForceReading1 = analogRead(flexiForcePin1); 
  float flexiForceReading2 = analogRead(flexiForcePin2); 

  Serial.print(flexiForceReading1);
  Serial.print(flexiForceReading2);


  struct send_data send_packet;
  diff =  (flexiForceReading1)-(flexiForceReading2);
  send_packet.diff = diff;

  mesh.update();

    // Send to the master node every second
  if (millis() - displayTimer >= 1000) {
    displayTimer = millis();

    // Send an 'M' type message containing the current millis()
    if (!mesh.write(&send_packet, 'D', sizeof(send_packet))) {

      // If a write fails, check connectivity to the mesh network
      if ( ! mesh.checkConnection() ) {
        //refresh the network address
        Serial.println("Renewing Address");
        mesh.renewAddress();
      } else {
        Serial.println("Send fail, Test OK");
      }
    } else {
      Serial.print("Send OK: "); Serial.println(displayTimer);
    }
                 digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  }



  
 // send_packet.ratio = ratio;
 // sendPacket(send_packet);

  //delay(10);
  //Serial.println(ratio);
  
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

 
