// Teste de comunicacao da Cabeca com Musculo e leitura de botoes.
// Funcao: handshake "oi" e envio de eventos BTN:1/2/3 quando ha clique.
// Saida: mensagens seriais para validar protocolo e debounce.
#include <Arduino.h>

#define RX_MUSCULO 44
#define TX_MUSCULO 43

// Botões (mesmos pinos da programação da cabeça)
#define BOTAO_1 3
#define BOTAO_2 37
#define BOTAO_3 46
#define moduloRobocup 10
const unsigned long DEBOUNCE_BOTAO_MS = 180;

HardwareSerial SerialMusculo(0);
String bufferEntrada = "";
unsigned long ultimoByteMs = 0;
unsigned long ultimoEventoBotaoMs = 0;

bool botao1Anterior = HIGH;
bool botao2Anterior = HIGH;
bool botao3Anterior = HIGH;

void processarMensagem(String msg) {
  msg.trim();
  msg.toLowerCase();

  if (msg == "oi") {
    SerialMusculo.println("oi");
    Serial.println("Recebi 'oi' do musculo e respondi 'oi'");
  } else if (msg.length() > 0) {
    Serial.print("Recebido (ignorado): ");
    Serial.println(msg);
  }
}

void enviarEventoBotao(uint8_t botao) {
  SerialMusculo.print("BTN:");
  SerialMusculo.println(botao);

  Serial.print("Botao pressionado -> BTN:");
  Serial.println(botao);
}

void verificarBotoes() {
  bool botao1Atual = digitalRead(BOTAO_1);
  bool botao2Atual = digitalRead(BOTAO_2);
  bool botao3Atual = digitalRead(BOTAO_3);
  unsigned long agora = millis();

  if (agora - ultimoEventoBotaoMs >= DEBOUNCE_BOTAO_MS) {
    if (botao1Anterior == HIGH && botao1Atual == LOW) {
      ultimoEventoBotaoMs = agora;
      enviarEventoBotao(1);
    } else if (botao2Anterior == HIGH && botao2Atual == LOW) {
      ultimoEventoBotaoMs = agora;
      enviarEventoBotao(2);
    } else if (botao3Anterior == HIGH && botao3Atual == LOW) {
      ultimoEventoBotaoMs = agora;
      enviarEventoBotao(3);
    }
  }

  botao1Anterior = botao1Atual;
  botao2Anterior = botao2Atual;
  botao3Anterior = botao3Atual;
}

void setup() {
  Serial.begin(115200);
  SerialMusculo.begin(9600, SERIAL_8N1, RX_MUSCULO, TX_MUSCULO);
  pinMode(moduloRobocup, INPUT);
  pinMode(BOTAO_1, INPUT_PULLUP);
  pinMode(BOTAO_2, INPUT_PULLUP);
  pinMode(BOTAO_3, INPUT_PULLUP);

  Serial.println("Teste comunicacao cabeca: pronto");
  Serial.println("Se receber 'oi' no RX, responde 'oi' no TX");
  Serial.println("Botões ativos: envia BTN:1 / BTN:2 / BTN:3");
}

void loop() {
  verificarBotoes();

  while (SerialMusculo.available() > 0) {
    char c = (char)SerialMusculo.read();
    ultimoByteMs = millis();

    if (c == '\n' || c == '\r') {
      processarMensagem(bufferEntrada);
      bufferEntrada = "";
      continue;
    }

    if (bufferEntrada.length() < 32) {
      bufferEntrada += c;
    }
  }

  if (bufferEntrada.length() > 0 && (millis() - ultimoByteMs) > 80) {
    processarMensagem(bufferEntrada);
    bufferEntrada = "";
  }
  Serial.println(digitalRead(moduloRobocup));
  delay(5);
}
