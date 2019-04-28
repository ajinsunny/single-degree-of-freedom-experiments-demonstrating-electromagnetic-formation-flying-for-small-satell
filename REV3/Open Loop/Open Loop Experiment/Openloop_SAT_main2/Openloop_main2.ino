#include <DueTimer.h>
#include <SineWaveDue.h>

void setup()  { 
  analogReadResolution(10);
  analogWriteResolution(10);
  pinMode(9, OUTPUT);
  //delay(20000);

} 

void loop()  { 
  
    S.startSinusoid1(10);
    delay(1000);
    S.stopSinusoid();
  
    
//    sw.playTone2(1000, 1200);
//    delay(1000);
//    sw.stopTone();
//    sw.playTone(5000, 1000);
//    for( int i; i<10; i++){
//      digitalWrite(9, HIGH);
//      sw.playToneDecay(400, .1*i);
//      delay(1000);
//      digitalWrite(9, LOW);
//      delay(100);
//  }
    
} 


  
