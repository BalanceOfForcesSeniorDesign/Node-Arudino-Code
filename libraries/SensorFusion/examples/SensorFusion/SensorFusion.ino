// IMPORTANT
// this is not a working example, this is just to show how to set the filter
// if you need a working example please see: https://www.hackster.io/vincenzo-g/sensor-fusion-algorithms-made-simple-6c55f6


#include "SensorFusion.h" //SF
SF filter;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;


void setup() {

  Serial.begin(115200); //serial to display data
  // your IMU begin code goes here
}

void loop() {

  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD

  deltat = filter.deltatUpdate(); //this have to be done before calling the filter update
  //choose only one of these two:
  filter.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  //filter.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = filter.getPitch();
  roll = filter.getRoll();    //you could also use getRollRadians() ecc
  yaw = filter.getYaw();

  Serial.print("Pitch:\t"); Serial.println(pitch);
  Serial.print("Roll:\t"); Serial.println(roll);
  Serial.print("Yaw:\t"); Serial.println(yaw);
  Serial.println();
}
