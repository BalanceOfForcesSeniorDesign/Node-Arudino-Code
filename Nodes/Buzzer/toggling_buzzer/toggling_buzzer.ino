void setup() {
  // put your setup code here, to run once:
    pinMode(12, OUTPUT); //Buzzer pin
    int sound = 0 ; 
    Serial.begin(115200);

}
 int sound = 0 ;


 
void loop() {
 
 // If not in state imbalanced need to set digitalWrite(12,LOW)
delay(10);
if(sound == 0){
  digitalWrite(12,HIGH); 
  Serial.println("Toggling on");
  sound = 1;
}
else{
  digitalWrite(12,LOW);
  Serial.println("Toggling off");
  sound = 0;
  
}

}
