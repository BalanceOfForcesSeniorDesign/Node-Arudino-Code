#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <nRF24L01.h>

#define CSN_PIN 11
#define CE_PIN 10

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

#define PIN 8
  // Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);

RF24 radio(CE_PIN, CSN_PIN); // CE, CSN

RF24Network network(radio);      // Network uses that radio

const uint16_t node_S = 00;   // Address of the other node in Octal format
const uint16_t node_P = 01;    // Address of our node in Octal format ( 04,031, etc)
const uint16_t node_A = 011;   // Address of the other node in Octal format
const uint16_t node_B = 021;   // Address of the other node in Octal format




const unsigned long interval = 4; //ms  // How often to send 'hello world to the other unit

unsigned long last_sent;             // When did we last send?
unsigned long packets_sent;          // How many have we sent already


float ax,ay,az,gx,gy,gz,nodeA_diff,nodeB_diff;

struct payload_t {                 // Structure of our payload
  float ax,ay,az,gx,gy,gz,nodeA_diff,nodeB_diff;
};

struct payload_recieve {                 // Structure of our payload
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

void setup(void)
{
 // Serial.begin(115200);
 // Serial.println("Looking for the LSM9DS1.....");
 // 
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
  //  Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
//  Serial.println("Found LSM9DS1 9DOF");
  // helper to just set the default scaling we want, see above!
  setupSensor();
  
  strip.begin();
  strip.setBrightness(64);
  strip.show();

  pinMode(13, OUTPUT);
 
  SPI.begin();
  radio.begin();
  network.begin(/*node address*/ node_P);
}

void loop(void){
  
  network.update();                  // Check the network regularly

  
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

  strip.setPixelColor(0,(int)abs(gx),(int)abs(gy),(int)abs(gz));
  strip.show();


  

    unsigned long now = millis();              // If it's time to send a message, send it!

  if ( now - last_sent >= interval  )
  {
    last_sent = now;

  //  Serial.print("Sending...");
    payload_t payload = {ax,ay,az,gx,gy,gz,nodeA_diff,nodeB_diff};
    RF24NetworkHeader header(/*to node*/ node_S);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok)
    {
  //    Serial.println("ok.");
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else
    {
  //    Serial.println("failed.");
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    }
  }
  

 while ( network.available() ) {     // Is there anything ready for us?
    
    RF24NetworkHeader header;        // If so, grab it and print it out
    payload_recieve payload;
    network.read(header,&payload,sizeof(payload));
 //   Serial.print("Received packet: ");
  //  Serial.print(header.from_node);
    if (header.from_node == node_B)
    {
    nodeB_diff = payload.diff;
    }
    else if (header.from_node == node_A)
    {
    nodeA_diff = payload.diff;
    }
  }
}
