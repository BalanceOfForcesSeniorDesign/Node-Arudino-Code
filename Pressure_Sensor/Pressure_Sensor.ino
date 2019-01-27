int flexiForcePin1 = A0;   //analog pin 0
int flexiForcePin2 = A1;   //analog pin 1

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
}
 
// the loop function runs over and over again forever
void loop() {
  int flexiForceReading1 = analogRead(flexiForcePin1); 
  int flexiForceReading2 = analogRead(flexiForcePin2); 
  
  String string1 = String("Ball: ") + (flexiForceReading1);
  Serial.println(string1);
  
  String string2 = String("Heel: ") + (flexiForceReading2);
  Serial.println(string2);
  
  int ratio = 100*(flexiForceReading1)/(flexiForceReading1 + flexiForceReading2);
  String string3 = String("RATIO: ") + ratio;
  Serial.println(string3);
  
  delay(250); //just here to slow down the output for easier reading
}
