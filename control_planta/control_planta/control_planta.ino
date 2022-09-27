// Controlador PID 

// Declaración de variables
#define V_OUT A0// Lectura tensión de salida
#define A_SP_IN A1 // Analog setpoint input
#define X_SIGNAL A2 // Entrada analógica en modo manual
#define M 8 // Input port M for manual/auto mode
#define V_IN 9 // Vin para planta(Salida PWM)

#define MANUAL 1 // Modo manual
#define AUTO 0 // PID
#define BUILDIN 13 // Led en pin 13

float kp = 2.2405;
float Ti = 0.08859;
float Td = 0;//0.008312;
float Ts = 0.01;

float E1 = 0;
float E2 = 0;
float U1 = 0;
float U2 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
  uint16_t manual_in = map(analogRead(X_SIGNAL), 0, 1023, 0 , 254);
  analogWrite(V_IN, manual_in);
  Serial.println(manual_in);
}// end manual

void _auto(){
  //uint16_t setpoint; 
  float setpoint = fmap(analogRead(A_SP_IN), 0, 1023, 0, 1000);// Setpoint for PID(AUTO MODE)
  float Y = fmap(analogRead(V_OUT), 0, 1023, 0, 1000);

  //Serial.print(setpoint);
  //Serial.print("  ");
  //Serial.print(Y);  
  //Serial.print("  ");

  // Calcular error 
  float E = setpoint - Y;
  //Serial.print(" E");
  //Serial.print(E);
  //Serial.print("  ");
  // Señal de salida del controlador PID estándar
  float U = U1 + kp*((1+ Ts/Ti + Td/Ts)*E - (1 + (2*Td)/Ts)*E1 + (Td/Ts)*E2);
  
  if(U > 1000){// Saturación
    U = 1000;
  }else if(U < 0){
    U = 0;
  }

  //Serial.print(U);
  //Serial.print("  ");
  // Para calcular errores y salidas del controlador pasadas  
  E2 = E1;
  E1 = E;
  U1 = U;

  //Salida del controlador
  unsigned int uk = fmap(U, 0, 1000, 0, 254);
  analogWrite(V_IN, uk);
  Serial.println(uk);
  delay(10);
}// end auto

//Funcion para cambiar de escala
float fmap(float x, float in_min, float in_max, float out_min, float out_max){
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
