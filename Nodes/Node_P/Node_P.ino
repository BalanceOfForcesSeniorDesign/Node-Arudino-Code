#include <RF24.h>
#include <RadioConfig.h>
#include <nRF24L01.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <arduinoFFT.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// NeoPixel
#define PIN 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);


// Radio Pins
#define CSN_PIN 11
#define CE_PIN 10
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN
byte sendByte;

// Timing
#define DATA_ACQ_INTERVAL_US 4000 // us = 4ms
#define PC_TX_INTERVAL_US 2000 // us = 2ms

int lastDataAcqInterval;
int currentTime;

// Sampling and FFT
#define SAMPLES 128 // 128 * 4ms = .512 s
#define SAMPLING_FREQUENCY 250 // 1/4ms = 250 Hz

int pressureSamples[SAMPLES];
double vPressureReal[SAMPLES];
double vPressureImag[SAMPLES];

bool interpolatedSample = false;
int numSamplesCollected = 0;
double interpolatedSlope;

arduinoFFT PressureFFT = arduinoFFT(vPressureReal, vPressureImag, SAMPLES, SAMPLING_FREQUENCY);


// Sensor global variables
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

  pinMode(13, OUTPUT);
  setupRadio(radio);
  radio.stopListening(); // Needed for a write only operation
}

void loop(void) {



  currentTime = micros();


  if ((currentTime - lastDataAcqInterval) >= DATA_ACQ_INTERVAL_US)
  {
    if ((currentTime - lastDataAcqInterval - DATA_ACQ_INTERVAL_US) < 500)
    {
      lastDataAcqInterval = currentTime;
      
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

      // Push into sample array
      for (int i = 0; i < SAMPLES - 1; i++)
      {
        pressureSamples[i] = pressureSamples[i + 1];
      }
      
      pressureSamples[SAMPLES - 1] = (nodeA_diff + nodeB_diff);
      interpolatedSample = false;
      numSamplesCollected++;
    }
    else if (!interpolatedSample)
    {
      lastDataAcqInterval = currentTime;
      
      // Push into sample array
      for (int i = 0; i < SAMPLES - 1; i++)
      {
        pressureSamples[i] = pressureSamples[i + 1];
      }

      // Interpolate next sample using the last 2 samples
      interpolatedSlope = (pressureSamples[SAMPLES - 2] - pressureSamples[SAMPLES - 4])/8;
      pressureSamples[SAMPLES - 1] = (int) pressureSamples[SAMPLES - 2] + interpolatedSlope * 4;
      interpolatedSample = true;
    }
  }

  

  if (numSamplesCollected  >= 4)
  {

    for (int i = 0; i <= SAMPLES - 1; i++)
    {
      vPressureReal[i] = pressureSamples[i];
      vPressureImag[i] = 0;
    }

    int t = micros();
    PressureFFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    PressureFFT.Compute(FFT_FORWARD);
    PressureFFT.ComplexToMagnitude();
    double domFrequency = PressureFFT.MajorPeak();
    Serial.println(domFrequency);
    numSamplesCollected  = 0;
  }




  // Send message to PC reciever
  /*Serial.println("Sending message....");
    openRadioToNode(3, radio); // Switch to transmit to the PC node
    payload_t packet = {ax, ay, az, gx, gy, gz, nodeA_diff, nodeB_diff};
    if (radio.write(&packet, sizeof(packet)))
    {
    digitalWrite(13, HIGH);    // turn the LED on by making the voltage HIGH
    }
    else
    {
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    }*/

  /*
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      gx = g.gyro.x;
      gy = g.gyro.y;
      gz = g.gyro.z;

      strip.setPixelColor(0, (int)abs(gx), (int)abs(gy), (int)abs(gz));
      strip.show();
  */




}
