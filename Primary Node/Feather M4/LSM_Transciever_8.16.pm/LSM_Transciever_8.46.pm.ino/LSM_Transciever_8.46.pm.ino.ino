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



const uint8_t rxAddr[6] = "00001";
const uint8_t  NodeAAddr[6] = "00002";
const uint8_t NodeBAddr[6] = "00003";



float ax,ay,az,gx,gy,gz,node1_ratio,node2_ratio;

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
  float ax,ay,az,gx,gy,gz,node1_ratio,node2_ratio;
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

  radio.openReadingPipe(1, NodeAAddr);
  radio.openReadingPipe(2, NodeBAddr);


  radio.openWritingPipe(rxAddr);

  
  radio.setRetries(15, 15);
  radio.startListening();

  Serial.println("Finished setting up the RF chip.");
}


void loop() 
{

  readIMUData();

  struct send_data send_packet;
  send_packet.ax = ax;
  send_packet.ay = ay;
  send_packet.az = az;
  send_packet.gx = gx;
  send_packet.gy = gy;
  send_packet.gz = gz;
  send_packet.node1_ratio = node1_ratio;
  send_packet.node2_ratio = node2_ratio;

  
  sendPacket(send_packet);

  digitalWrite(13, HIGH);  
  delay(1);
  digitalWrite(13, LOW);  
  
  strip.setPixelColor(0,(int)abs(gx),(int)abs(gy),(int)abs(gz));
  strip.show();
  
  struct recieve_data packet1,packet2;

  packet1 = recievePacket(1);
  packet2 = recievePacket(2);
  delay(1);
  
  node1_ratio = packet1.ratio;
  node2_ratio = packet2.ratio;


  

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

recieve_data recievePacket(uint8_t pipe_number)
 {
  Serial.println("Looking for Data...");
     Serial.println(pipe_number);
  if (radio.available(&pipe_number))
  {
   struct recieve_data packet;

   radio.read(&packet, sizeof(packet));
   Serial.print("Recieved packet! : ");
   Serial.println(packet.ratio);
   return packet;
   
  }
 }

 void readIMUData()
 {
 lsm.read();  
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
 }
 
 
 
