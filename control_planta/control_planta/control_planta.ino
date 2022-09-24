//Simulador para Arduino
// https://www.simulide.com/p/downloads.html

#define V_OUT A0// Lectura tensión de salida
#define A_SP_IN A1 // Analog setpoint input
#define X_SIGNAL A2 // Entrada analógica en modo manual
#define M 8 // Input port M for manual/auto mode
#define V_IN 9 // Vin para planta(Salida PWM)

#define MANUAL 0 // Modo manual
#define AUTO 1 // PID
#define BUILDIN 13 // Led en pin 13

uint8_t setpoint; // Setpoint for PID(AUTO MODE)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400);
  pinMode(M, INPUT);
  pinMode(BUILDIN, OUTPUT);
  pinMode(V_IN, OUTPUT);
  analogWrite(V_IN, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(digitalRead(M)){
    case MANUAL:
      digitalWrite(BUILDIN, LOW);
      _manual();
      break;
    case AUTO:
      digitalWrite(BUILDIN, HIGH);
      _auto();
      break;
    default:
      Serial.println("UKNOWN MODE");
  }// end switch
}

void _manual(){
  // Mapear señal manual a salida para evitar saturación
  uint16_t manual_in = map(analogRead(X_SIGNAL), 0, 1024, 0 , 255);
  analogWrite(V_IN, manual_in);
  Serial.println(manual_in);
}// end manual

void _auto(){
  Serial.println("Auto Mode");
}// end auto
