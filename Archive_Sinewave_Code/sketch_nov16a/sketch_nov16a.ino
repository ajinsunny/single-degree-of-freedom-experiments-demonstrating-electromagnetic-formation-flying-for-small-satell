//Xbee receiver
//This example code is in the Public Domain

int sentDat;

void setup() {
  Serial.begin(9600);   
  pinMode(2, OUTPUT); 
}

void loop() {
  if (Serial.available() > 0) {
  sentDat = Serial.read(); 

  if(sentDat == 'h'){
          //activate the pumpkin for one second and then stop
      digitalWrite(2, HIGH);
          delay(1000);
          digitalWrite(2, LOW);
  }
  }
}
