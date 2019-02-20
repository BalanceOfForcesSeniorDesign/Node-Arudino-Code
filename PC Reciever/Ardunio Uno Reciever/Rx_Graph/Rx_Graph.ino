
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

typedef struct data {
  float ax,ay,az,gx,gy,gz,ratio;
};

float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,ratio;

// Also declare plotter as global
Plotter p;


void setup()
{
    // Start plotter
    p.Begin();
    radio.begin();
    radio.openReadingPipe(0, RX_PC_ADR);
    radio.startListening();
  
    p.AddTimeGraph( "Acceleration X", 1000, "m/s^2", acc_x );
    p.SetColor(0,"red");
    p.AddTimeGraph( "Acceleration Y", 1000, "m/s^2", acc_y );
    p.SetColor(1,"green");
    p.AddTimeGraph( "Acceleration Z", 1000, "m/s^2", acc_z );
    p.SetColor(2,"blue");
    p.AddTimeGraph( "Gyroscope X", 1000, "d/s", gyro_x );
    p.SetColor(3,"red");
    p.AddTimeGraph( "Gyroscope Y", 1000, "d/s", gyro_y );
    p.SetColor(4,"green");
    p.AddTimeGraph( "Gyroscope Z", 1000, "d/s", gyro_z );
    p.SetColor(5,"blue");
    p.AddTimeGraph( "Voltage Ratio", 1000, "ratio", ratio );
    p.SetColor(6,"red");

}

void loop()
{
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
   ratio = recieve_data.ratio;
  }

    // Plot
    p.Plot();
}
