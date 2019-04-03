void setup() {
  // put your setup code here, to run once:
    pinMode(11, OUTPUT); //Buzzer pin
    int sound = 0 ; 

}

void loop() {
 int sound = 0 ; 
 // If not in state imbalanced need to set digitalWrite(11,LOW)
if(sound = 0){
  digitalWrite(11,HIGH);
  sound = 1;
}
else{
  digitalWrite(11,LOW);
  sound = 0;
  
}
digitalWrite(11,HIGH);
}
