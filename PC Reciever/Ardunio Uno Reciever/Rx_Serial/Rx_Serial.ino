
#include <Plotter.h>
#include <RF24.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <nRF24L01.h>
#include <NodeConfig.h>
#include "Plotter.h"
RF24 radio(7, 8);

unsigned long time;

typedef struct data {
  float ax,ay,az,gx,gy,gz,node1_diff;
};

float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,node1_diff,node2_diff;

// Also declare plotter as global
//Plotter p;


void setup() 
{
    Serial.begin(115200);
    radio.begin();
    radio.openReadingPipe(0, RX_PC_ADR);
    radio.startListening();



}

void loop()
{
  time = millis();
    if (radio.available())
  {
   struct data recieve_data;
   radio.read(&recieve_data, sizeof(recieve_data));
   acc_x = recieve_data.ax;
   acc_y = recieve_data.ay;
   acc_z = recieve_data.az;
   gyro_x = recieve_data.gx;
   gyro_y = recieve_data.gy;
   gyro_z = recieve_data.gz;
   node1_diff = recieve_data.node1_diff;
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
    Serial.print(node1_diff);
    Serial.println();

}
