#ifndef RadioConfig_h
#define RadioConfig_h

#include <Arduino.h>
#include <RF24.h>





void setupRadio(RF24 radio);
void setupAckRadio(uint8_t node, RF24 radio);
void openRadioToNode(uint8_t node,RF24 radio);



#endif