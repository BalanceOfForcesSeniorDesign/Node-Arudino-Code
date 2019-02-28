#include <RF24.h>
#include <RadioConfig.h>
#include <nRF24L01.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define PIN 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);


#define CSN_PIN 11
#define CE_PIN 10
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN




float ax, ay, az, gx, gy, gz, nodeA_diff, nodeB_diff;

// Payload structure
struct payload_t {                 
  float ax, ay, az, gx, gy, gz, nodeA_diff, nodeB_diff;
};


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup(void)
{
  Serial.begin(115200);

  lsm.begin();
  setupSensor();

  strip.begin();
  strip.setBrightness(64);
  strip.show();

  pinMode(13, OUTPUT);
  setupRadio(radio);
  radio.stopListening(); // Needed for a write only operation
}

void loop(void) {


  

    // Read IMU data
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;

    strip.setPixelColor(0, (int)abs(gx), (int)abs(gy), (int)abs(gz));
    strip.show();


    byte sendByte;

    // Ping Node A
    openRadioToNode(1, radio);
    if (radio.write(&sendByte, 1)) {
      if (radio.isAckPayloadAvailable()) radio.read(&nodeA_diff, sizeof(nodeA_diff));
    }
    
    // Ping Node B
    openRadioToNode(2, radio); 
    if (radio.write(&sendByte, 1)) {
      if (radio.isAckPayloadAvailable()) radio.read(&nodeB_diff, sizeof(nodeB_diff));
    }

    // Send message to PC reciever
    Serial.println("Sending message....");
    openRadioToNode(3, radio); // Switch to transmit to the PC node
    payload_t packet = {ax, ay, az, gx, gy, gz, nodeA_diff, nodeB_diff};
    if (radio.write(&packet, sizeof(packet)))
    {
      digitalWrite(13, HIGH);    // turn the LED on by making the voltage HIGH
    }
    else
    {
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    }

}
