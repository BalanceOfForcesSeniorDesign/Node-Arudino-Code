void setup() {
  // put your setup code here, to run once:
    pinMode(12, OUTPUT); //Buzzer pin
    int sound = 0 ; 

}

void loop() {
 int sound = 0 ; 
 // If not in state imbalanced need to set digitalWrite(12,LOW)
if(sound = 0){
  digitalWrite(12,HIGH);
  sound = 1;
}
else{
  digitalWrite(12,LOW);
  sound = 0;
  
}
digitalWrite(12,HIGH);
}
