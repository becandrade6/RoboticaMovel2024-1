#include <PololuMagneticEncoder.h>

// No encoder, 900 rotações equivalem a 1 volta completa da roda

PololuMagneticEncoder encoders;
long tempo = 0;
int dt = 100;
int esquerdo,direito;
float esquerdoRPM, direitoRPM;

int PWMA = 33; 
int AIN2 = 25;
int AIN1 = 26;
int STBY = 27;
int BIN1 = 14;
int BIN2 = 12;
int PWMB = 13;

int MTA = 0;
int MTB = 0;

void setup(){
    encoders.setupEncoders(34,39,35,32);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    digitalWrite(STBY, HIGH);
    Serial.begin(9600);
}

void loop(){
    if(millis() - tempo >= dt){
        tempo = millis();
        esquerdo = encoders.getCountsAndResetEncoderLeft();
        direito = encoders.getCountsAndResetEncoderRight();
        esquerdoRPM = (esquerdo*2/3);
        direitoRPM = (direito*2/3);
        Serial.print("Esquerdo: ");
        Serial.print(esquerdo);
        Serial.print(' ');
        Serial.println(esquerdoRPM);
        Serial.print("Direito: ");
        Serial.print(direito);
        Serial.print(' ');
        Serial.println(direitoRPM);
        Serial.println(' ');
    }
    if(Serial.available() > 0){
        int leitura = Serial.read();
        executaMovimento(leitura);
    }
}

void executaMovimento(int dado) {
    if(dado == 'a'){
        //roda da direita indo pra frente
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        //roda da esquerda indo pra frente
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        MTA = 255;
        MTB = 255;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
    }else if(dado == 'b'){
        //roda da direita indo pra tras
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        //roda da esquerda indo pra tras
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        MTA = 255;
        MTB = 255;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
    }else if(dado == 'c'){
        //roda da direita parada
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        //roda da esquerda parada
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        MTA = 0;
        MTB = 0;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
    }else if(dado == 'd'){
        //roda da direita indo pra frente
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        //roda da esquerda indo pra tras
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        MTA = 120;
        MTB = 120;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
    }else if(dado == 'e'){
        //roda da direita indo pra tras
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        //roda da esquerda indo pra frente
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        MTA = 120;
        MTB = 120;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
    }
}