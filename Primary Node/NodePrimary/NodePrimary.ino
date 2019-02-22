#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <nRF24L01.h>
#include <RF24.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <NodeConfig.h>
#include <RF24Network.h>

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
RF24Network network(radio);  


const uint16_t this_node = 00;        // Address of our node in Octal format
const uint16_t node_A = 01;
const uint16_t node_B = 02;
const uint16_t node_PC = 04;

unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already

float nodeAdiff,nodeBdiff;
uint8_t recieve_pipe_num;

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
  float ax,ay,az,gx,gy,gz,nodeAdiff,nodeBdiff;
};

typedef struct recieve_data {
  float diff;
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
  
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ this_node);

  
  //radio.begin();
  //radio.setPALevel(RF24_PA_MAX);
  //radio.openWritingPipe(RX_PC_ADR);
  //radio.openReadingPipe(1, NODE_ADR[0]);
  //radio.openReadingPipe(2, NODE_ADR[1]);
  //radio.setRetries(15, 15);
  //radio.startListening();

  Serial.println("Finished setting up the RF chip.");
}

void loop() 
{
    network.update(); 
    
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
  send_packet.nodeAdiff = nodeAdiff;
  send_packet.nodeBdiff = nodeBdiff;


  unsigned long now = millis(); 
 last_sent = now;
    Serial.print("Sending...");
    RF24NetworkHeader header(/*to node*/ node_PC);
    bool ok = network.write(header,&send_packet,sizeof(send_packet));
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");




while(network.available()){
RF24NetworkHeader header;


  
}
  
 // sendPacket(send_packet);

  //delay(10);
  
  strip.setPixelColor(0,(int)abs(g.gyro.x),(int)abs(g.gyro.y),(int)abs(g.gyro.z));
  strip.show();





  
  
  //struct recieve_data recieve_packet;
  //recieve_packet = recievePacket();
  //if (recieve_pipe_num == 1)
  //{
   // nodeAdiff = recieve_packet.diff;
 // }
 // else
  //{
   // nodeBdiff = recieve_packet.diff;
 // }
  

}


void sendPacket(struct send_data packet)
{
 bool rslt;
 radio.stopListening();
 rslt = radio.write(&packet, sizeof(packet));
 radio.startListening();
 Serial.print("Data Sent ");
 if (rslt) {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
 Serial.println("  Acknowledge received");
 } else {
       //  Serial.println("  Tx failed");
         digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
        }
        
 }

struct recieve_data recievePacket()
 {
  Serial.println("Looking for Data...");
  while (radio.available(&recieve_pipe_num))
  {
   struct recieve_data packet;
   radio.read(&packet, sizeof(packet));
   Serial.print("Recieved packet! from  ");
   Serial.print(recieve_pipe_num);
   Serial.print(" : ");
   Serial.println(packet.diff);
   return packet;
   
  }
 }
 
 
