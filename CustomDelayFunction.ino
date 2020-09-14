const unsigned long previousTime = 0;

void setup() {
  

}

void loop() {

    servo.write(angle);
    customDelay(200);
  }
}

void customDelay(int interval)
{
  while(1){
    unsigned currentTime = millis();
    if(currentTime-previousTime>=interval){
      previousTime = currentTime;
      return;}
    }
}
