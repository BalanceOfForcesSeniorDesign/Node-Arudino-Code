#include <RF24.h>
#include <RadioConfig.h>
#include <nRF24L01.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include <string.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// NeoPixel
#define PIN 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);

#define BUZZER_PIN 12
#define LED_PIN 13

// Radio Pins
#define CSN_PIN 11
#define CE_PIN 10
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN
byte sendByte;

// Timing
#define DATA_ACQ_INTERVALS_US 4000 // us = 4ms
#define PC_TX_INTERVALS_US 2000 // us = 2ms

int lastDataAcqINTERVALS;
int currentTime;

// Sampling and FFT
#define SAMPLES 128 // 128 * 4ms = .512 s
#define SAMPLING_FREQUENCY 250 // 1/4ms = 250 Hz

int pressureSamples[SAMPLES];
float gyroscopeYSamples[SAMPLES];
float accelerometerZSamples[SAMPLES];

double vPressureReal[SAMPLES];
double vPressureImag[SAMPLES];

bool interpolatedSample = false;
int numSamplesCollected = 0;
double interpolatedSlope;

arduinoFFT PressureFFT = arduinoFFT(vPressureReal, vPressureImag, SAMPLES, SAMPLING_FREQUENCY);

/****** Walking Detection Tunes ********/
#define DOMINATING_FREQUENCY_FLOOR 40
#define DOMINATING_FREQUENCY_CEILING 300 
#define WALKING_BIAS .12

/******* Imbalance Detection Tunes ********/
#define STAND_COUNT 210 //200*4ms = 800 ms standing
#define PRESSURE_SAMPLE_CHECK 4 // How many pressure sensor samples do we check?
#define IMU_SAMPLE_CHECK 4 // How many IMU sensor samples do we check?

#define FORWARD_THRESHOLD 30
#define BACKWARD_THRESHOLD -50

#define ACCELEROMETER_Z_THRESHOLD 90 // m/s^2
#define GYROSCOPE_Y_THRESHOLD 12 // dps

#define LEFT_FOOT_BACKWARD -20
#define LEFT_FOOT_FORWARD 20
#define RIGHT_FOOT_BACKWARD -20
#define RIGHT_FOOT_FORWARD 20

// Tag Definitions
#define FORWARD_LEAN 1
#define BACKWARD_LEAN -1
#define NEUTRAL_LEAN 0

#define CURRENTLY_WALKING 0
#define CURRENTLY_NOT_WALKING 1

#define WALKING -1
#define STANDING 0
#define SHIFTING 1
#define SETTLING 2

// Global Counters
int standingCounter = STAND_COUNT;
int forwardCount;
int backwardCount;
int presentState = 0;
int previousState = 0;
bool firstStep = true;
int change; //Change from one state to another (-1 .. 2)
int lean; // Direction of lean (-1..1)
int imbalance; //Imbalance decision (0 or 1)
double domFrequency;


// Sensor Global Variables
sensors_event_t a, m, g, temp;
float ax, ay, az, gx, gy, gz;
int nodeA_diff, nodeB_diff;


// Payload structure
struct payload_t {
  float ax, ay, az, gx, gy, gz;
  int nodeA_diff, nodeB_diff;
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

  Wire.setClock(3400000);
}


void setup(void)
{
  Serial.begin(115200);

  lsm.begin();
  setupSensor();

  strip.begin();
  strip.setBrightness(64);
  strip.show();

  pinMode(BUZZER_PIN, OUTPUT); // Buzzer Pin
  pinMode(LED_PIN, OUTPUT);
  setupRadio(radio);
  radio.stopListening(); // Needed for a write only operation
}

void loop(void) {

currentTime = micros();

/******************************Data Acquisition INTERVALS*************************************/
  if ((currentTime - lastDataAcqINTERVALS) >= DATA_ACQ_INTERVALS_US)
  {
    if ((currentTime - lastDataAcqINTERVALS - DATA_ACQ_INTERVALS_US) < 500)
    {
      lastDataAcqINTERVALS = currentTime;
      
      // Read IMU data
      lsm.read();
      lsm.getEvent(&a, &m, &g, &temp);

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

      // Push into pressure sample array
      for (int i = 0; i < SAMPLES - 1; i++)
      {
        pressureSamples[i] = pressureSamples[i + 1];
      }
      pressureSamples[SAMPLES - 1] = (nodeA_diff + nodeB_diff);

      // Push into gyroscope Y sample array
      for (int i = 0; i < SAMPLES - 1; i++)
      {
        gyroscopeYSamples[i] = gyroscopeYSamples[i + 1];
      }
      gyroscopeYSamples[SAMPLES - 1] = g.gyro.y;

      // Push into accelerometer Z sample array
      for (int i = 0; i < SAMPLES - 1; i++)
      {
        accelerometerZSamples[i] = accelerometerZSamples[i + 1];
      }
      accelerometerZSamples[SAMPLES - 1] = a.acceleration.z;


      interpolatedSample = false;
      numSamplesCollected++;
    }
    else if (!interpolatedSample)
    {
      lastDataAcqINTERVALS = currentTime;
      
      // Push into sample array
      for (int i = 0; i < SAMPLES - 1; i++)
      {
        pressureSamples[i] = pressureSamples[i + 1];
      }

      // Interpolate next sample using the last 2 samples

      // Interpolate the pressure sample
      interpolatedSlope = (pressureSamples[SAMPLES - 2] - pressureSamples[SAMPLES - 4])/8;
      pressureSamples[SAMPLES - 1] = (int) pressureSamples[SAMPLES - 2] + interpolatedSlope * 4;

      // Interpolate the next gyroscope sample
      interpolatedSlope = (gyroscopeYSamples[SAMPLES - 2] - gyroscopeYSamples[SAMPLES - 4])/8;
      gyroscopeYSamples[SAMPLES - 1] = (float) gyroscopeYSamples[SAMPLES - 2] + interpolatedSlope * 4;

      // Interpolate the next accelerometer sample
      interpolatedSlope = (accelerometerZSamples[SAMPLES - 2] - accelerometerZSamples[SAMPLES - 4])/8;
      accelerometerZSamples[SAMPLES - 1] = (float) accelerometerZSamples[SAMPLES - 2] + interpolatedSlope * 4;

      interpolatedSample = true;

      Serial.println(pressureSamples[SAMPLES-1]);
      //Serial.print(nodeA_diff);
      //Serial.print(",");
      //Serial.println(nodeB_diff);
      /*Serial.print(",");
      Serial.print(gyroscopeYSamples[SAMPLES - 1]);
      Serial.print(",");
      Serial.println(accelerometerZSamples[SAMPLES - 1]);*/
      
    }
  }

  
/******************************Take the FFT every defined number samples*************************************/
  if (numSamplesCollected  >= 4)
  {
    numSamplesCollected  = 0;

    for (int i = 0; i <= SAMPLES - 1; i++)
    {
      vPressureReal[i] = pressureSamples[i];
      vPressureImag[i] = 0;
    }

    int t = micros();
    PressureFFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    PressureFFT.Compute(FFT_FORWARD);
    PressureFFT.ComplexToMagnitude();


    double previousFrequency = domFrequency; 
    domFrequency = (vPressureReal[4]+vPressureReal[5]+vPressureReal[6]+vPressureReal[7]+vPressureReal[8]+vPressureReal[9])/6;
    domFrequency = previousFrequency + WALKING_BIAS*(domFrequency-previousFrequency);

    //Serial.println(domFrequency);

    if (domFrequency > DOMINATING_FREQUENCY_FLOOR && domFrequency < DOMINATING_FREQUENCY_CEILING){
      presentState = CURRENTLY_WALKING; // User is walking
      sendMessage("Detected Walking");
      strip.setPixelColor(0, 0, 0, 255);
      strip.show();
      
    }
    else{
      presentState = CURRENTLY_NOT_WALKING; // User is not walking

    }

    detectImbalance();

    previousState = presentState;  
  }

}


void detectImbalance()
{

    /******************************Change Definition*************************************/
    if (presentState == CURRENTLY_WALKING){ // User is walking
      standingCounter = STAND_COUNT; // Reset standing timer
      change = WALKING; 
    }
    else{
      standingCounter--;
      change = STANDING;  
    }
    /******************************Lean Definition*************************************/  
      if (change == STANDING){ // If standing
        forwardCount = 0;
        backwardCount = 0;
        for (int i = SAMPLES; i >= SAMPLES - (PRESSURE_SAMPLE_CHECK + 1); i--){ // for last 4 samples
          if (pressureSamples[i] > FORWARD_THRESHOLD){ // if forward thresh crossed
            forwardCount++; // increment forward counter
          }
          else if (pressureSamples[i] < BACKWARD_THRESHOLD){ // if backward thresh crossed
            backwardCount++; // increment backward counter
          }
        }
        if (forwardCount > PRESSURE_SAMPLE_CHECK/2){ // if forward for more than half of samples
          lean = FORWARD_LEAN; // Lean defined as forward
          
          sendMessage("Forward lean");
          
          strip.setPixelColor(0, 255, 255, 0);
          strip.show();
        }
        else if (backwardCount > PRESSURE_SAMPLE_CHECK/2){ // if backward for more than half of samples
          lean = BACKWARD_LEAN;  // Lean defined as backward
          
          sendMessage("Backward lean");
          
          strip.setPixelColor(0, 255, 255, 0);
          strip.show();
        }
        else {
          lean = NEUTRAL_LEAN; // Lean defined as neutral

          sendMessage("Neutral lean");
          strip.setPixelColor(0, 0, 255, 0);
          strip.show();
        }
      }
    /******************************Imbalance Definition*************************************/ 

    
      if (change == STANDING && (lean == FORWARD_LEAN || lean == BACKWARD_LEAN)){ // If standing and leaning
        for (int i = SAMPLES; i >= SAMPLES - (IMU_SAMPLE_CHECK + 1); i--) {  
          if (abs((gyroscopeYSamples[i]) >= GYROSCOPE_Y_THRESHOLD || abs(accelerometerZSamples[i]) >= ACCELEROMETER_Z_THRESHOLD)){
            sendMessage("Detected Imbalance");

            digitalWrite(BUZZER_PIN, HIGH);
            
            strip.setPixelColor(0, 255, 0, 0);
            strip.show();
          }
        }    
      }
      else{
        digitalWrite(BUZZER_PIN, LOW);
      }    
}

void sendMessage(String message)
{
    // Send message to PC reciever
    openRadioToNode(3, radio); // Switch to transmit to the PC node

    Serial.println(message);
    char messageSend[32];

    int messageSize;

    while (message[messageSize] != 0x00)
    {
      messageSize++;
    }

    
    for (int i = 0; i<31; i++)
    {
      if (i > messageSize-1)
      {
        messageSend[i] = ' ';
      }
      else
      {
        messageSend[i] = message[i];
      }
    }
    
    if (radio.write(&messageSend, sizeof(messageSend)))
    {
      digitalWrite(13, HIGH);    // turn the LED on by making the voltage HIGH
    }
    else
    {
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    }
}
