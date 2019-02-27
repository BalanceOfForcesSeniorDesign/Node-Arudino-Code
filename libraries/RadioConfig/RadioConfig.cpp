#include "RadioConfig.h"


const uint8_t addresses[][6] = { "NodeP","NodeA","NodeB","NodeS"};

void setupRadio(RF24 radio)
{
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX);         //PA level to output
  radio.setDataRate(RF24_2MBPS);         //Set data rate as specified in user options
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(15,15);                 // Smallest time between retries, max no. of retries
  radio.enableDynamicPayloads();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging

}


void setupAckRadio(uint8_t node, RF24 radio)
{
  setupRadio(radio);
  if (node == 1)
  {
    radio.openReadingPipe(1,addresses[1]);
  }
  else if (node == 2)
  {
    radio.openReadingPipe(1,addresses[2]);
  }
  radio.startListening();
}



void openRadioToNode(uint8_t node,RF24 radio)
{
    if (node == 1)
  {
  radio.openWritingPipe(addresses[1]);
  }
  else if (node == 2)
  {
   radio.openWritingPipe(addresses[2]);
  }

}