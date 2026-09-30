// Teste minimo de resposta da placa Olho para handshake.
// Funcao: quando recebe "oi" da Cabeca, responde "OI".
// Uso: validar serial e protocolo de vida entre placas.
#include <Arduino.h>

#define RX_CABECA 17
#define TX_CABECA 18

String bufferEntrada = "";
unsigned long ultimoByteMs = 0;

void processarMensagem(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "oi") {
    Serial1.println("OI");
    Serial.println("Recebi OI da Cabeca e respondi OI");
  }
}

void lerSerialCabeca() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    ultimoByteMs = millis();

    if (c == '\n' || c == '\r') {
      if (bufferEntrada.length() > 0) {
        processarMensagem(bufferEntrada);
        bufferEntrada = "";
      }
      continue;
    }

    if (isPrintable(c) && bufferEntrada.length() < 32) {
      bufferEntrada += c;
    } else {
      bufferEntrada = "";
    }
  }

  if (bufferEntrada.length() > 0 && (millis() - ultimoByteMs) > 80) {
    processarMensagem(bufferEntrada);
    bufferEntrada = "";
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_CABECA, TX_CABECA);
  Serial.println("Teste Olho pronto para responder Cabeca");
}

void loop() {
  lerSerialCabeca();
  delay(5);
}
