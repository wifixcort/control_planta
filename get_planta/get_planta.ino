#define VIN 9 // PWM pin
#define V_2 A2 //Analog read pin
#define FLOW A0

unsigned long int counter = 0;
uint8_t a_out = 0;
unsigned long int time = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  pinMode(VIN, OUTPUT);
  pinMode(FLOW, INPUT_PULLUP);
  while(!Serial);
  analogWrite(VIN, 0);
  delay(100);
  time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(FLOW)){
    if(!counter){
      time = millis();
    }
    if(counter < 2500){
      a_out = 127; // PWM 50%
    }else if(counter < (2*2500)){
      a_out = 64; // PWM 25%
    }else{
      a_out = 255; // PWM 100%
    }
    analogWrite(VIN, a_out);
    Serial.print(millis()-time);
    Serial.print(",");
    Serial.print(analogRead(V_2)*(5.0 / 1023.0)); // Read in Volts
    Serial.print(",");
    Serial.println(a_out); // Step in volts
    counter++;
    //delay(10);    
  }
}
