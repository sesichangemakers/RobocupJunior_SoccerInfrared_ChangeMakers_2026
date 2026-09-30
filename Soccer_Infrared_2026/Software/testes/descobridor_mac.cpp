// Descobridor de MAC.
// Funcao: imprimir o endereco MAC da placa ESP32 no monitor serial.
// Resultado esperado: endereco MAC impresso a cada 3 segundos.
#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inicializa WiFi apenas para poder ler o MAC (sem conectar a nenhuma rede)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("=== DESCOBRIDOR DE MAC ===");
}

void loop() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  delay(3000);
}