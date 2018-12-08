////Motor A
#define PIN_M1_EN              5
#define PIN_M1_IN1             A1
#define PIN_M1_IN2             A0


////Motor B
#define PIN_M2_EN              6
#define PIN_M2_IN1             4
#define PIN_M2_IN2             10

#define BAT_R1                 22000
#define BAT_R2                 10000
#define BAT_DROP               0.26
#define VBAT_VOLTAGE(adc)      (adc / (1023 / 5.0)) * (BAT_R1 + BAT_R2) * (1.0 / BAT_R2) + BAT_DROP

char in;

void setup(){

  while (!Serial); 

  Serial.begin(115200);

  pinMode(A3, INPUT);

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
        analogWrite(PIN_M1_EN, 51);
        digitalWrite(PIN_M2_IN1, HIGH);
        digitalWrite(PIN_M2_IN2, LOW);
        analogWrite(PIN_M2_EN, 51);
    }
    else if(in == 's'){
        digitalWrite(PIN_M1_IN1, LOW);
        digitalWrite(PIN_M1_IN2, HIGH);
        analogWrite(PIN_M1_EN, 51);
        digitalWrite(PIN_M2_IN1, LOW);
        digitalWrite(PIN_M2_IN2, HIGH);
        analogWrite(PIN_M2_EN, 51);
    }
    else if(in == 'f'){
        digitalWrite(PIN_M1_IN1, HIGH);
        digitalWrite(PIN_M1_IN2, HIGH);
        analogWrite(PIN_M1_EN, 0);
        digitalWrite(PIN_M2_IN1, HIGH);
        digitalWrite(PIN_M2_IN2, HIGH);
        analogWrite(PIN_M2_EN, 0);
    }
    else if(in == 'a'){
        digitalWrite(PIN_M1_IN1, LOW);
        digitalWrite(PIN_M1_IN2, HIGH);
        analogWrite(PIN_M1_EN, 51);
        digitalWrite(PIN_M2_IN1, HIGH);
        digitalWrite(PIN_M2_IN2, LOW);
        analogWrite(PIN_M2_EN, 51);
    }
    else if(in == 'd'){
        digitalWrite(PIN_M1_IN1, HIGH);
        digitalWrite(PIN_M1_IN2, LOW);
        analogWrite(PIN_M1_EN, 51);
        digitalWrite(PIN_M2_IN1, LOW);
        digitalWrite(PIN_M2_IN2, HIGH);
        analogWrite(PIN_M2_EN, 51);
    }
    else if(in == 'b'){
        float bat = VBAT_VOLTAGE(analogRead(A3));
        Serial.print("Bat: ");
        Serial.println(bat);
    }
  }
}