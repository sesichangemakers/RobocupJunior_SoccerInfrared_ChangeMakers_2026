// Teste ESP-NOW bidirecional.
// Funcao: enviar "OI" para um MAC alvo, responder "OI: RESPONDI" e sinalizar falha de comunicacao.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Troque este MAC pelo endereco do outro ESP.
// Exemplo: "24:6F:28:AA:BB:CC"

// mac cronos AC:A7:04:2B:9B:60
//mac nexus 1C:DB:D4:46:CB:FC
const char* TARGET_MAC_STR = "AC:A7:04:2B:9B:60";
uint8_t TARGET_MAC[6] = {0};

static const unsigned long SEND_INTERVAL_MS = 2000;
static const unsigned long COMM_TIMEOUT_MS = 6000;

struct EspNowMessage {
  uint32_t seq;
  char text[24];
};

unsigned long lastSendMs = 0;
unsigned long lastRxMs = 0;
unsigned long lastSendOkMs = 0;
uint32_t seqCounter = 0;
bool falhaComunicacaoAvisada = false;

bool parseMacString(const char* macStr, uint8_t* outMac) {
  unsigned int b0, b1, b2, b3, b4, b5;
  int parsed = sscanf(macStr, "%2x:%2x:%2x:%2x:%2x:%2x", &b0, &b1, &b2, &b3, &b4, &b5);
  if (parsed != 6) {
    return false;
  }

  outMac[0] = static_cast<uint8_t>(b0);
  outMac[1] = static_cast<uint8_t>(b1);
  outMac[2] = static_cast<uint8_t>(b2);
  outMac[3] = static_cast<uint8_t>(b3);
  outMac[4] = static_cast<uint8_t>(b4);
  outMac[5] = static_cast<uint8_t>(b5);
  return true;
}

bool macEquals(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

void printMac(const uint8_t* mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void enviarMensagem(const char* texto) {
  EspNowMessage msg;
  msg.seq = ++seqCounter;
  memset(msg.text, 0, sizeof(msg.text));
  strncpy(msg.text, texto, sizeof(msg.text) - 1);

  esp_err_t result = esp_now_send(TARGET_MAC, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
  if (result != ESP_OK) {
    Serial.printf("[ERRO] Falha ao enfileirar envio. Codigo: %d\n", result);
  }
}

void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (!macEquals(mac_addr, TARGET_MAC)) {
    return;
  }

  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSendOkMs = millis();
    falhaComunicacaoAvisada = false;
    Serial.println("[TX] OI enviado com sucesso");
  } else {
    Serial.println("[FALHA] Falhou a comunicacao com o alvo");
  }
}

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (!macEquals(mac, TARGET_MAC)) {
    return;
  }

  EspNowMessage msg;
  memset(&msg, 0, sizeof(msg));
  int copyLen = len;
  if (copyLen > (int)sizeof(msg)) {
    copyLen = sizeof(msg);
  }
  memcpy(&msg, incomingData, copyLen);

  lastRxMs = millis();
  falhaComunicacaoAvisada = false;

  Serial.print("[RX] De ");
  printMac(mac);
  Serial.print(" -> ");
  Serial.println(msg.text);

  if (strcmp(msg.text, "OI") == 0) {
    enviarMensagem("OI: RESPONDI");
    Serial.println("[TX] Resposta enviada: OI: RESPONDI");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!parseMacString(TARGET_MAC_STR, TARGET_MAC)) {
    Serial.println("[ERRO] MAC invalido. Use o formato AA:BB:CC:DD:EE:FF");
    while (true) {
      delay(1000);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("=== TESTE ESPNOW ===");
  Serial.print("MAC local: ");
  Serial.println(WiFi.macAddress());
  Serial.print("MAC alvo : ");
  printMac(TARGET_MAC);
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERRO] Falha ao inicializar ESP-NOW");
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, TARGET_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(TARGET_MAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ERRO] Nao foi possivel adicionar peer ESP-NOW");
      while (true) {
        delay(1000);
      }
    }
  }

  lastRxMs = millis();
  lastSendOkMs = millis();
  enviarMensagem("OI");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    enviarMensagem("OI");
  }

  if (!falhaComunicacaoAvisada) {
    bool semRx = (now - lastRxMs) > COMM_TIMEOUT_MS;
    bool semAck = (now - lastSendOkMs) > COMM_TIMEOUT_MS;
    if (semRx && semAck) {
      falhaComunicacaoAvisada = true;
      Serial.println("[FALHA] Falhou a comunicacao: alvo sem resposta");
    }
  }

  delay(20);
}
