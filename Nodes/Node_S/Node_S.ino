#include <RF24.h>
#include <RadioConfig.h>

#define CSN_PIN 7
#define CE_PIN 8
RF24 radio(CE_PIN, CSN_PIN); // CE, CSN



unsigned long time;

struct payload_t {                 // Structure of our payload
  float ax,ay,az,gx,gy,gz,nodeA_diff,nodeB_diff;
};

float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,nodeA_diff,nodeB_diff;

unsigned long current_time;
void setup(void)
{
  Serial.begin(115200);
  setupAckRadio(3, radio);
}

void loop(void){
  
  time = millis();
  
  while ( radio.available() ) {     // Is there anything ready for us?
    payload_t payload;
    radio.read(&payload,sizeof(payload));
    
    acc_x = payload.ax;
    acc_y = payload.ay;
    acc_z = payload.az;
    gyro_x = payload.gx;
    gyro_y = payload.gy;
    gyro_z = payload.gz;
    nodeA_diff = payload.nodeA_diff;
    nodeB_diff = payload.nodeB_diff;


  }

    Serial.print(time);
    Serial.print(",");
    Serial.print(acc_x);
    Serial.print(",");
    Serial.print(acc_y);
    Serial.print(",");
    Serial.print(acc_z);
    Serial.print(",");
    Serial.print(gyro_x);
    Serial.print(",");
    Serial.print(gyro_y);
    Serial.print(",");
    Serial.print(gyro_z);
    Serial.print(",");
    Serial.print(nodeA_diff);
    Serial.print(",");
    Serial.print(nodeB_diff);
    Serial.println();
}
