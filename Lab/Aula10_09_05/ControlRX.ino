#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include "PololuMagneticEncoder.h"

// Configura pinos
#define LED 2 // LED ESP32

PololuMagneticEncoder encoders;

// Definindo macAddress do ESP
#define MAC {0xD4 , 0x8A , 0xFC , 0xAA , 0x2C , 0xB4}

int PWMA = 33; 
int AIN2 = 25;
int AIN1 = 26;
int STBY = 27;
int BIN1 = 14;
int BIN2 = 12;
int PWMB = 13;

int MTA = 0;
int MTB = 0;

float Kd = 27.0403;
float Ki = 540.8069;
float erroEsquerda;
float erroDireita;
int numeroPulsosPorVolta = 900;
float diametroRodaEmMetros = 0.042;
float dS = 0.001466077;
float Idireita = 0;
float Iesquerda = 0;
float Pdireita = 0;
float Pesquerda = 0;

// macAddress do ESP32 TX escolhido
uint8_t MAC_TX[] = MAC;

int dt = 100; // Tempo de amostragem
long tempo = 0;
bool aux = false; // Variável auxiliar

int esquerdo = 0, direito = 0; // Variáveis para armazenar leitura do encoder
float esquerdoMs = 0, direitoMs = 0; // Variáveis para armazenar velocidade do encoder em m/s

bool auxD = false, auxE = false;
bool estado = false;

typedef struct receive_message {
    float left;
    float right;
} receive_message;

// Create a struct_message called BME280Readings to hold sensor readings
receive_message referenceSpeed;

typedef struct send_message {
    int esquerdo;
    int direito;
} send_message;

// Create a struct_message called BME280Readings to hold sensor readings
send_message encoderRead;

bool available = false;

// Função callback
void OnDataRecv(const uint8_t *mac , const uint8_t *incomingData , int len)
{
  memcpy(&referenceSpeed, incomingData, sizeof(referenceSpeed));
  available = true;
  aux = true;
}

void setup()
{
  // Configura pinos como saída
  pinMode(LED,OUTPUT);
  encoders.setupEncoders(34,39,35,32);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);

  // Configurando dispositivo como WiFi Station
  WiFi.mode(WIFI_STA);
  
  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK ) 
  {
    return ;
  }

  esp_now_peer_info_t peerInfo;

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer 
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, MAC_TX, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    return;
  }

  // Configurando função de CallBack
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  if (available) 
  {
    available = false;

    if(referenceSpeed.left > 0){
      //seta roda esquerda para frente
      auxE = true;
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }else if(referenceSpeed.left < 0){
      //seta roda esquerda para trás
      auxE = true;
      referenceSpeed.left = abs(referenceSpeed.left);
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }else if(referenceSpeed.left === 0){
      //seta roda esquerda para parado
      auxE = false;
      MTB = 0;
      Iesquerda = 0;
    }


    if(referenceSpeed.right > 0){
      //seta roda direita para frente
      auxD = true;
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }else if(referenceSpeed.right < 0){
      //seta roda direita para trás
      auxD = true;
      referenceSpeed.right = abs(referenceSpeed.right);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }else if (referenceSpeed.right === 0){
      //seta roda direita para parado
      auxD = false;
      MTA = 0;
      Idireita = 0;
    }

    encoderRead.esquerdo = esquerdo;
    encoderRead.direito = direito;

    esp_now_send(MAC_TX, (uint8_t *) &encoderRead, sizeof(encoderRead));
  }
  // Leitura do encoder a cada dt milissegundos
  if (millis() - tempo >= dt) 
  {
    tempo = millis();

    if(aux)
    {
      // Faça a leitura dos encoders aqui
      direito = encoders.getCountsAndResetEncoderRight()
      direitoMs = direito*dS;
      esquerdo = encoders.getCountsAndResetEncoderLeft();
      esquerdoMs = esquerdo*dS;

      erroEsquerda = referenceSpeed.left - esquerdoMs;
      erroDireita = referenceSpeed.right - direitoMs;
      if(auxD)
      {
        // Implemente o controle da roda direita aqui
        Pdireita = erroDireita * Kd;
        Idireita = Idireita + erroDireita * Ki * dt;
        MTA = Pdireita + Idireita;
        if(MTA > 255)
        {
          MTA = 255;
        }else if(MTA < 0)
        {
          MTA = 0;
        }
      }

      if(auxE)
      {
        // Implemente o controle da roda esquerda aqui
        Pesquerda = erroEsquerda * Kd;
        Iesquerda = Iesquerda + erroEsquerda * Ki * dt;
        MTB = Pesquerda + Iesquerda;
        if(MTB > 255)
        {
          MTB = 255;
        }else if(MTB < 0)
        {
          MTB = 0;
        }
      }
      // Atualize as velocidades dos motores aqui
      analogWrite(PWMA, MTA);
      analogWrite(PWMB, MTB);
    }
  }     
}