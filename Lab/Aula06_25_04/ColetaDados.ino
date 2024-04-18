#include <PololuMagneticEncoder.h>

int PWMA = 33; 
int AIN2 = 25;
int AIN1 = 26;
int STBY = 27;
int BIN1 = 14;
int BIN2 = 12;
int PWMB = 13;

int MTA = 0;
int MTB = 0;

long tempo = 0;
int dado = 0;
int aux = 0;
int dt = 100;

PololuMagneticEncoder encoders;
int esquerdo,direito;
float esquerdoRPM, direitoRPM;
//COM ROBO MIRADO PARA FRENTE
//COMANDO A -> RODA DA DIREITA (AIN1, AIN2)
    //roda indo pra frente -> AIN1 = LOW, AIN2 = HIGH
    //roda indo pra tras -> AIN1 = HIGH, AIN2 = LOW
    //roda freiando -> AIN1 = HIGH, AIN2 = HIGH
    //roda ponto morto -> AIN1 = LOW, AIN2 = LOW
//COMANDO B -> RODA DA ESQUERDA (BIN1, BIN2)
    //roda indo pra frente -> BIN1 = LOW, BIN2 = HIGH
    //roda indo pra tras -> BIN1 = HIGH, BIN2 = LOW
    //roda freiando -> BIN1 = HIGH, BIN2 = HIGH
    //roda ponto morto -> BIN1 = LOW, BIN2 = LOW

void setup(){
    encoders.setupEncoders(34,39,35,32);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    digitalWrite(STBY,HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMA,MTA);
    analogWrite(PWMB,MTB);
    Serial.begin(115200);
}

void loop(){
    if(Serial.available() > 0){
        dado = Serial.read();
        aux = 1;
        if(millis() - tempo >= dt){
            tempo = millis();
            if(aux == 1){
                esquerdo = encoders.getCountsAndResetEncoderLeft();
                direito = encoders.getCountsAndResetEncoderRight();
                Serial.println(dado);
                Serial.println(esquerdo);
                Serial.println(direito);
                MTA = dado;
                MTB = dado;
                analogWrite(PWMA, MTA);
                analogWrite(PWMB, MTB);
            }
        }
    }
}