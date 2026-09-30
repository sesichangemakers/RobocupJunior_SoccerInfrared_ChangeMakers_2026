// Teste da entrada da chave do kicker.
// Funcao: ler o estado do pino do kicker e imprimir no monitor serial.
// Resultado esperado: alternancia entre 0/1 conforme a chave fisica.
#include <arduino.h>

constexpr uint8_t KICKER_PIN = 45;
constexpr unsigned long KICK_PULSE_MS = 100;
constexpr unsigned long KICK_INTERVAL_MS = 1000;

void setup() {
  Serial.begin(115200);
  pinMode(KICKER_PIN, INPUT_PULLUP); // chave fim de curso
}

void loop() {
Serial.println(digitalRead(KICKER_PIN));
delay(50);
}