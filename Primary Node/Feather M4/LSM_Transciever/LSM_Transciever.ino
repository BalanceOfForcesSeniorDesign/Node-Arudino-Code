#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

#define PIN 8
  // Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);

#define CSN_PIN 11
#define CE_PIN 10
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN



const byte rxAddr[6] = "00001";
const byte Node1Addr[6] = "00002";


float ratio = 0;

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


typedef struct send_data {
  float ax,ay,az,gx,gy,gz,ratio;
};

typedef struct recieve_data {
  float ratio;
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


void setup() 
{
  Serial.begin(115200);
  
  Serial.println("Looking for the LSM9DS1.....");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  // helper to just set the default scaling we want, see above!
  setupSensor();
  
  strip.begin();
  strip.setBrightness(64);
  strip.show();

  pinMode(13, OUTPUT);

  Serial.println("Setup NeoPixel");


  
  radio.begin();

  radio.openWritingPipe(rxAddr);
  radio.openReadingPipe(1, Node1Addr);
  radio.setRetries(15, 15);
  radio.startListening();

  Serial.println("Finished setting up the RF chip.");
}

void loop() 
{
  lsm.read();  
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  struct send_data send_packet;
  send_packet.ax = a.acceleration.x;
  send_packet.ay = a.acceleration.y;
  send_packet.az = a.acceleration.z;
  send_packet.gx = g.gyro.x;
  send_packet.gy = g.gyro.y;
  send_packet.gz = g.gyro.z;
  send_packet.ratio = ratio;

  
  sendPacket(send_packet);

  //delay(10);
  
  strip.setPixelColor(0,(int)abs(g.gyro.x),(int)abs(g.gyro.y),(int)abs(g.gyro.z));
  strip.show();
  
  struct recieve_data recieve_packet;
  recieve_packet = recievePacket();
  ratio = recieve_packet.ratio;

}


void sendPacket(struct send_data packet)
{
 bool rslt;
 radio.stopListening();
 rslt = radio.write(&packet, sizeof(packet));
 radio.startListening();
 //Serial.print("Data Sent ");
 if (rslt) {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
// Serial.println("  Acknowledge received");
 } else {
       //  Serial.println("  Tx failed");
         digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
        }
        
 }

struct recieve_data recievePacket()
 {
 // Serial.println("Looking for Data...");
  if (radio.available())
  {
   struct recieve_data packet;
   radio.read(&packet, sizeof(packet));
  // Serial.print("Recieved packet! : ");
 //  Serial.println(packet.ratio);
   return packet;
   
  }
 }
 
 
