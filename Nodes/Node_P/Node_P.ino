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

// Walking Detection
#define DOMINATING_FREQUENCY_FLOOR .27 // Hz
#define DOMINATING_FREQUENCY_CEILING 5 // Hz

// Imbalance Detection
#define FORWARD_THRESHOLD 50
#define BACKWARD_THRESHOLD -50
#define ACCELEROMETER_Z_THRESHOLD 4
#define GYROSCOPE_Y_THRESHOLD 12

int forwardCount;
int backwardCount;
int presentState = 0;
int previousState = 0;
int change; //Change from one state to another (-1 .. 2)
int lean; // Direction of lean (-1..1)
int imbalance; //Imbalance decision (0 or 1)

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
    }
  }

  
/******************************Take the FFT every defined number samples*************************************/
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

    // Computing the dominating frequency and if it is not defined as walking then detect imbalance
    double domFrequency = PressureFFT.MajorPeak();
    if (!(domFrequency > DOMINATING_FREQUENCY_FLOOR && domFrequency < DOMINATING_FREQUENCY_CEILING)){
      presentState = 1; // User is walking
      detectImbalance();
    }
    else{
      presentState = 0; // User is not walking
    }
    numSamplesCollected  = 0;
    /******************************Previous State Definition*************************************/
    previousState = presentState;  
  }

}


void detectImbalance()
{

    /******************************Change Definition*************************************/
      if (presentState + previousState == 0){ // If 'walking' to 'walking'
        change = -1; //walking
        lean = 0;
      }
      else if (presentState + previousState == 2){ // If 'standing' to 'standing'
        change = 0; //standing
      }
      else if (presentState - previousState == -1){ // If 'standing' to 'walking'
        change = 1; //shifting
      }
      else if (presentState - previousState == 1){ // If 'walking' to 'standing'
        change = 2; //settling
      } 
      else { // something went wrong
        change = 0; 
      }
    /******************************Lean Definition*************************************/  
      if (change >= 0){ // If standing, shifting, or settling
        forwardCount = 0;
        backwardCount = 0;
        for (int i = SAMPLES; i < SAMPLES - 4; i--){ // for all samples
          if (pressureSamples[i] > FORWARD_THRESHOLD){ // if forward thresh crossed
            forwardCount++; // increment forward counter
          }
          else if (pressureSamples[i] < BACKWARD_THRESHOLD){ // if backward thresh crossed
            backwardCount++; // increment backward counter
          }
        }
        if (forwardCount > SAMPLES/2){ // if forward for more than half of samples
          lean = 1; // Lean defined as forward
        }
        else if (backwardCount > SAMPLES/2){ // if backward for more than half of samples
          lean = -1;  // Lean defined as backward
        }
        else {
          lean = 0; // Lean defined as neutral
        }
      }
    /******************************Imbalance Definition*************************************/ 
      if (change >= 0 && abs(lean) == 1){ // If [standing, shifting, or settilng] AND [leaning]
        for (int i = SAMPLES; i < SAMPLES - 4; i--) {  
          if (abs(gyroscopeYSamples[i]) >= GYROSCOPE_Y_THRESHOLD || abs(accelerometerZSamples[i]) >= ACCELEROMETER_Z_THRESHOLD){
            imbalance = 1;
          }
          else{
            imbalance = 0;
          }
        }    
      }
      else{
        imbalance = 0;
      }    
    /******************************Display Results*************************************/ 
//    Serial.println(gyroscopeYSamples);
//    Serial.print(" ");
//    Serial.print(accelerometerZSamples);
//    Serial.print(" ");
//    Serial.print(imbalance);
}
