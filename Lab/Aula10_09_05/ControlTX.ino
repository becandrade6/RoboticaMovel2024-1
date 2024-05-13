#include <esp_now.h>
#include <WiFi.h>

#define LED 2

// Definindo macAddress do ESP
#define MAC {0x24 , 0xDC , 0xC3 , 0x45 , 0xD0 , 0x60}

// macAddress do ESP32 RX escolhido
uint8_t MAC_RX[] = MAC;

typedef struct message {
    int left;
    int right;
} message;

// Create a struct_message to hold incoming sensor readings
message encodersRead;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&encodersRead, incomingData, sizeof(encodersRead));
  Serial.println(encodersRead.left);
  Serial.println(encodersRead.right);
}

byte msg[9];

void setup() 
{
  pinMode(LED, OUTPUT);
  
  // Iniciando comunica ção serial
  Serial.begin(115200);

  // Colocando ESP em modo WiFi station
  WiFi.mode(WIFI_STA);

  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK)
  {
    return;
  }

  esp_now_peer_info_t peerInfo;

  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, MAC_RX, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    return;

  esp_now_register_recv_cb(OnDataRecv);

  delay(2000);
}

// [bit 1 = cte de comunicação, bits 2,3,4 e 5 = v roda direita, bits 6, 7, 8 e 9 = v roda esqueda]
void loop() 
{
  if (Serial.available() > 8) // Observe que o código só entrará no if depois de ler 9 bytes
  {
    for (int i =0; i < 9; i++){
      msg[i]= Serial.read();
    }
      if(msg[0] == 1){
        digitalWrite(LED, HIGH);
        // Em caso afirmativo, utilizar o comando abaixo para enviar a mensagem para o robô
        esp_now_send(MAC_RX, (uint8_t*) &msg[1] , sizeof(float)*2);
      }
     

    }
}

