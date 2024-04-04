int PWMA = 33; 
int AIN2 = 25;
int AIN1 = 26;
int STBY = 27;
int BIN1 = 14;
int BIN2 = 12;
int PWMB = 13;

int MTA = 0;
int MTB = 0;

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