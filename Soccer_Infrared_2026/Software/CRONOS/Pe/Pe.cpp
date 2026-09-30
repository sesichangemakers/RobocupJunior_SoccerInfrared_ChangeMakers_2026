// Arquivo principal da placa Pe.
// Funcao: ler 32 sensores de linha via dois multiplexadores,
// calcular o angulo da linha e enviar esse angulo para a Cabeca.
// Entrada: LDRs da linha e estado atacante/defensor vindo da Cabeca.
// Saida: pacote serial com angulo da linha (em decimos de grau).
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdint.h>

#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55

#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02

#define RX_CABECA 17
#define TX_CABECA 18

#define NUM_SENSORES 32
#define LIMIAR_LINHA_PADRAO 2700
#define INTERVALO_DEBUG_MS 250
#define BAUD_PE_CABECA 115200
#define DEBUG_LINHA 0
#define DELAY_LOOP_MS 2

uint8_t mapaSensores[NUM_SENSORES] = {
  0,  1,  2,  3,
  4,  5,  6,  7,
  8,  9, 10, 11,
  12, 13, 14, 15,
  16, 17, 18, 19,
  20, 21, 22, 23,
  24, 25, 26, 27,
  28, 29, 30, 31
};

const int MUX1_SIG = 3;
const int MUX1_S0 = 21;
const int MUX1_S1 = 47;
const int MUX1_S2 = 48;
const int MUX1_S3 = 45;

const int MUX2_SIG = 8;
const int MUX2_S0 = 4;
const int MUX2_S1 = 5;
const int MUX2_S2 = 6;
const int MUX2_S3 = 7;

int ldr[NUM_SENSORES];
float sensorX[NUM_SENSORES];
float sensorY[NUM_SENSORES];
bool atacante = true;  // Inicia como ATACANTE para sincronizar com Cabeca
unsigned long ultimoDebugMs = 0;
String bufferHandshakeCabeca = "";
unsigned long ultimoByteHandshakeCabeca = 0;
bool enviarSensoresBrutosPendentes = false;
int limiarLinha = LIMIAR_LINHA_PADRAO;

const int EEPROM_SIZE = 64;
const int EEPROM_ADDR_LIMIAR_LINHA = 0;
const int LIMIAR_LINHA_MIN = 100;
const int LIMIAR_LINHA_MAX = 4000;

struct Pacote {
  int16_t angulo;
};

struct PacoteDefensor {
  int16_t anguloZonaA;
  int16_t anguloZonaB;
  uint8_t temLinhaZonaA;
  uint8_t temLinhaZonaB;
};

struct PacoteEstado {
  bool sozinho;
  bool atacante;
  bool corGolAzul;
};

void processarComandoCabeca(String comando) {
  comando.trim();
  comando.toLowerCase();

  if (comando == "oi") {
    Serial1.println("OI");
  } else if (comando == "req:sens") {
    enviarSensoresBrutosPendentes = true;
  } else if (comando == "req:lim") {
    Serial1.print("LIM:");
    Serial1.println(limiarLinha);
  } else if (comando.startsWith("setlim:")) {
    int novoLimiar = comando.substring(7).toInt();
    if (novoLimiar < LIMIAR_LINHA_MIN) novoLimiar = LIMIAR_LINHA_MIN;
    if (novoLimiar > LIMIAR_LINHA_MAX) novoLimiar = LIMIAR_LINHA_MAX;
    limiarLinha = novoLimiar;
    EEPROM.put(EEPROM_ADDR_LIMIAR_LINHA, limiarLinha);
    EEPROM.commit();
    Serial1.print("LIM:");
    Serial1.println(limiarLinha);
  }
}

// Seleciona um canal em um multiplexador 16:1 pelos pinos de endereco.
void selecionarCanalMUX(int s0, int s1, int s2, int s3, int canal) {
  digitalWrite(s0, bitRead(canal, 0));
  digitalWrite(s1, bitRead(canal, 1));
  digitalWrite(s2, bitRead(canal, 2));
  digitalWrite(s3, bitRead(canal, 3));
}

// Responde handshake textual da Cabeca sem consumir pacotes binarios.
void ProcessarPingCabeca() {
  while (Serial1.available() > 0) {
    if (Serial1.peek() == BYTE_INICIA) {
      return;
    }

    char c = (char)Serial1.read();
    ultimoByteHandshakeCabeca = millis();

    if (c == '\n' || c == '\r') {
      processarComandoCabeca(bufferHandshakeCabeca);
      bufferHandshakeCabeca = "";
      continue;
    }

    if (isPrintable(c) && bufferHandshakeCabeca.length() < 16) {
      bufferHandshakeCabeca += c;
    } else {
      bufferHandshakeCabeca = "";
    }
  }

  if (bufferHandshakeCabeca.length() > 0 && (millis() - ultimoByteHandshakeCabeca) > 80) {
    processarComandoCabeca(bufferHandshakeCabeca);
    bufferHandshakeCabeca = "";
  }
}

// Le pacote de estado da Cabeca e atualiza papel atacante/defensor.
void LeituraSerial() {
  while (Serial1.available() > 0) {
    if (Serial1.peek() != BYTE_INICIA) {
      return;
    }

    if (Serial1.available() < 2) {
      return;
    }

    Serial1.read();
    byte id = Serial1.read();
    if (id == ID_PLACA_OLHO || id == ID_PLACA_PE) {
      if (Serial1.available() < (int)(sizeof(PacoteEstado) + 1)) {
        return;
      }

      PacoteEstado temp;
      Serial1.readBytes((uint8_t*)&temp, sizeof(PacoteEstado));
      byte stop = Serial1.read();
      if (stop == BYTE_PARA) {
        bool mudouPapel = (temp.atacante != atacante);
        atacante = temp.atacante;

        Serial.print("Recebido estado da Cabeca: atacante=");
        Serial.println(atacante ? "1" : "0");

        // Se papel mudou, resincroniza
        if (mudouPapel) {
          Serial.println("Papel mudou! Resincronizando...");
          // Pequeno delay para estabilizar buffer
          delay(5);
        }
      }
    }
  }
}

void enviarSensoresBrutosSolicitados() {
  if (!enviarSensoresBrutosPendentes) {
    return;
  }

  enviarSensoresBrutosPendentes = false;
  Serial1.print("SENS:");
  Serial1.print(ldr[0]);
  Serial1.print(",");
  Serial1.print(ldr[8]);
  Serial1.print(",");
  Serial1.print(ldr[16]);
  Serial1.print(",");
  Serial1.println(ldr[24]);
}

// Calcula o angulo da linha por centroide ponderado dos sensores ativos.
int16_t calcularAngulo(bool repulsao) {
  float centroX = 0.0;
  float centroY = 0.0;
  float soma = 0.0;

  for (int i = 0; i < NUM_SENSORES; i++) {
    int idxFisico = mapaSensores[i];
    float peso = ldr[idxFisico];
    if (peso >= limiarLinha) {
      centroX += peso * sensorX[i];
      centroY += peso * sensorY[i];
      soma += peso;
    }
  }

  if (soma == 0) {
    return -1;
  }

  centroX /= soma;
  centroY /= soma;

  float vetorX = repulsao ? -centroX : centroX;
  float vetorY = repulsao ? -centroY : centroY;

  float anguloRad = atan2(vetorY, vetorX);
  float anguloGraus = anguloRad * 180.0 / PI;
  if (anguloGraus < 0) {
    anguloGraus += 360.0;
  }

  return (int16_t)round(anguloGraus * 10.0);
}

// Calcula o angulo da linha em uma faixa de sensores [inicio, fim].
int16_t calcularAnguloZona(int inicio, int fim, bool &temLinha) {
  float centroX = 0.0f;
  float centroY = 0.0f;
  float somaPesos = 0.0f;

  for (int i = inicio; i <= fim; i++) {
    int idxFisico = mapaSensores[i];
    int leitura = ldr[idxFisico];
    float peso = (float)(leitura - limiarLinha);
    if (peso <= 0.0f) {
      continue;
    }

    centroX += peso * sensorX[i];
    centroY += peso * sensorY[i];
    somaPesos += peso;
  }

  if (somaPesos <= 0.0f) {
    temLinha = false;
    return -1;
  }

  temLinha = true;
  centroX /= somaPesos;
  centroY /= somaPesos;

  float anguloRad = atan2(centroY, centroX);
  float anguloGraus = anguloRad * 180.0f / PI;
  if (anguloGraus < 0.0f) {
    anguloGraus += 360.0f;
  }

  return (int16_t)round(anguloGraus * 10.0f);
}

// Deteccao de linha para atacante (modo repulsao da linha).
int16_t detectarLinhaAtacante() {
  return calcularAngulo(true);
}

// Deteccao de linha para defensor (modo atracao da linha).
int16_t detectarLinhaDefensor() {
  return calcularAngulo(false);
}

// Deteccao do defensor separada por zonas: A (0-180) e B (180-360).
PacoteDefensor detectarLinhaDefensorPorZonas() {
  PacoteDefensor pacote;
  bool temZonaA = false;
  bool temZonaB = false;

  pacote.anguloZonaA = calcularAnguloZona(0, 15, temZonaA);
  pacote.anguloZonaB = calcularAnguloZona(16, 31, temZonaB);

  // Mantem os angulos absolutos originais de cada zona.
  // Isso preserva compatibilidade com a logica historica do defensor no Musculo.

  pacote.temLinhaZonaA = temZonaA ? 1 : 0;
  pacote.temLinhaZonaB = temZonaB ? 1 : 0;
  return pacote;
}

// Imprime leitura bruta de todos os sensores para depuracao.
void imprimirLeituraSensores() {
  Serial.print("Sensores: ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print("=");
    Serial.print(ldr[i]);
    if (ldr[i] >= limiarLinha) {
      Serial.print("*");
    }
    if (i < NUM_SENSORES - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println();
}

// Inicializa serial, MUX e tabela angular dos 32 sensores.
void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD_PE_CABECA, SERIAL_8N1, RX_CABECA, TX_CABECA);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADDR_LIMIAR_LINHA, limiarLinha);
  if (limiarLinha < LIMIAR_LINHA_MIN || limiarLinha > LIMIAR_LINHA_MAX) {
    limiarLinha = LIMIAR_LINHA_PADRAO;
    EEPROM.put(EEPROM_ADDR_LIMIAR_LINHA, limiarLinha);
    EEPROM.commit();
  }

  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  pinMode(MUX1_S3, OUTPUT);
  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);
  pinMode(MUX2_S3, OUTPUT);

  pinMode(MUX1_SIG, INPUT);
  pinMode(MUX2_SIG, INPUT);

  for (int i = 0; i < NUM_SENSORES; i++) {
    float angulo = (2.0 * PI / NUM_SENSORES) * i;
    sensorX[i] = cos(angulo);
    sensorY[i] = sin(angulo);
  }

  Serial.println("Placa PE inicializada com leitura de sensores");
}

// Laco principal: le sensores, calcula angulo e envia pacote para Cabeca.
void loop() {
  ProcessarPingCabeca();

  for (int canal = 0; canal < 16; canal++) {
    selecionarCanalMUX(MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3, canal);
    selecionarCanalMUX(MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3, canal);
    delayMicroseconds(10);
    ldr[canal] = analogRead(MUX1_SIG);
    ldr[canal + 16] = analogRead(MUX2_SIG);
  }

  LeituraSerial();

  if (atacante) {
    Pacote pacote;
    pacote.angulo = detectarLinhaAtacante();

    Serial1.write(BYTE_INICIA);
    Serial1.write(ID_PLACA_PE);
    Serial1.write((uint8_t*)&pacote, sizeof(Pacote));
    Serial1.write(BYTE_PARA);
  } else {
    PacoteDefensor pacoteDef;
    pacoteDef = detectarLinhaDefensorPorZonas();

    Serial1.write(BYTE_INICIA);
    Serial1.write(ID_PLACA_PE);
    Serial1.write((uint8_t*)&pacoteDef, sizeof(PacoteDefensor));
    Serial1.write(BYTE_PARA);
  }

  enviarSensoresBrutosSolicitados();

  unsigned long agora = millis();
  if (DEBUG_LINHA && (agora - ultimoDebugMs >= INTERVALO_DEBUG_MS)) {
    ultimoDebugMs = agora;
    imprimirLeituraSensores();
    int16_t anguloDebug = -1;
    if (atacante) {
      anguloDebug = detectarLinhaAtacante();
    } else {
      PacoteDefensor dbg = detectarLinhaDefensorPorZonas();
      anguloDebug = (dbg.temLinhaZonaA == 1) ? dbg.anguloZonaA : dbg.anguloZonaB;
    }

    if (anguloDebug == -1) {
      Serial.println("Linha: NAO detectada");
    } else {
      Serial.print("Linha angulo (deg): ");
      Serial.println(anguloDebug / 10.0, 1);
    }
  }

  delay(DELAY_LOOP_MS);
}

