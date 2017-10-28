////Motor A
#define PIN_M1_EN              5
#define PIN_M1_IN1             A0
#define PIN_M1_IN2             A1


////Motor B
#define PIN_M2_EN              6
#define PIN_M2_IN1             A4
#define PIN_M2_IN2             4

char in;

void setup(){

  while (!Serial); 

  Serial.begin(115200);

  pinMode(PIN_M1_EN, OUTPUT);
  pinMode(PIN_M1_IN1, OUTPUT);
  pinMode(PIN_M1_IN2, OUTPUT);

  pinMode(PIN_M2_EN, OUTPUT);
  pinMode(PIN_M2_IN1, OUTPUT);
  pinMode(PIN_M2_IN2, OUTPUT);

}

void loop(){
  if(Serial.available()){
    in = Serial.read();     
    if(in == 'w'){
        digitalWrite(PIN_M1_IN1, HIGH);
        digitalWrite(PIN_M1_IN2, LOW);
        analogWrite(PIN_M1_EN, 120);
        digitalWrite(PIN_M2_IN1, HIGH);
        digitalWrite(PIN_M2_IN2, LOW);
        analogWrite(PIN_M2_EN, 120);
    }
    else if(in == 's'){
        digitalWrite(PIN_M1_IN1, LOW);
        digitalWrite(PIN_M1_IN2, HIGH);
        analogWrite(PIN_M1_EN, 120);
        digitalWrite(PIN_M2_IN1, LOW);
        digitalWrite(PIN_M2_IN2, HIGH);
        analogWrite(PIN_M2_EN, 120);
    }
    else if(in == 'b'){
        digitalWrite(PIN_M1_IN1, HIGH);
        digitalWrite(PIN_M1_IN2, HIGH);
        analogWrite(PIN_M1_EN, 0);
        digitalWrite(PIN_M2_IN1, HIGH);
        digitalWrite(PIN_M2_IN2, HIGH);
        analogWrite(PIN_M2_EN, 0);
    }
  }
}