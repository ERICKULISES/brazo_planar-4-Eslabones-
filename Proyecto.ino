int valor_potenciometro = 0;
int valor_potenciometro2= 0;
int valor_potenciometro3= 0;
int valor_potenciometro4= 0;
void setup(){
  Serial.begin(9600);}
void loop(){
  valor_potenciometro = analogRead(A0);
  valor_potenciometro2= analogRead(A1);
  valor_potenciometro3= analogRead(A2);
  valor_potenciometro4= analogRead(A3);
  Serial.print(valor_potenciometro);
  Serial.print(',');
  Serial.print(valor_potenciometro2);
  Serial.print(',');
  Serial.print(valor_potenciometro3);
  Serial.print(',');
  Serial.println(valor_potenciometro4);
  delay(300);}
