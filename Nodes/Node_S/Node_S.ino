#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>


RF24 radio(7,8);                // nRF24L01(+) radio attached using Getting Started board 

RF24Network network(radio);      // Network uses that radio
const uint16_t node_P = 00;    // Address of our node in Octal format ( 04,031, etc)

unsigned long time;

struct payload_t {                 // Structure of our payload
  float ax,ay,az,gx,gy,gz,nodeA_diff,nodeB_diff;
};

float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,nodeA_diff,nodeB_diff;

void setup(void)
{
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(node_P);
}

void loop(void){
  
  network.update();                  // Check the network regularly
  time = millis();
  
  while ( network.available() ) {     // Is there anything ready for us?
    
    RF24NetworkHeader header;        // If so, grab it and print it out
    payload_t payload;
    network.read(header,&payload,sizeof(payload));
    acc_x = payload.ax;
    acc_y = payload.ay;
    acc_z = payload.az;
    gyro_x = payload.gx;
    gyro_y = payload.gy;
    gyro_z = payload.gz;
    nodeA_diff = payload.nodeA_diff;
    nodeB_diff = payload.nodeB_diff;

  }

    Serial.print(time);
    Serial.print(",");
    Serial.print(acc_x);
    Serial.print(",");
    Serial.print(acc_y);
    Serial.print(",");
    Serial.print(acc_z);
    Serial.print(",");
    Serial.print(gyro_x);
    Serial.print(",");
    Serial.print(gyro_y);
    Serial.print(",");
    Serial.print(gyro_z);
    Serial.print(",");
    Serial.print(nodeA_diff);
    Serial.print(",");
    Serial.print(nodeB_diff);
    Serial.println();
}
