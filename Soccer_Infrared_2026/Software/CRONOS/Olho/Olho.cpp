// Arquivo principal da placa Olho.
// Funcao: ler sensores IR (direcao da bola), ultrassonicos e camera,
// montar pacote com deteccoes e enviar para a Cabeca.
// Entrada: TSOPs, ultrassonicos, camera UART e estado recebido da Cabeca.
// Saida: pacote serial com angulo/intensidade IR, dados do gol e cameraOK.

#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>
#include "comunicacao_unificada.hpp"
#define BYTE_INICIA 0xAA
#define BYTE_PARA 0x55
#include <HCSR04.h>

//------------------------------------ ESTADO -----------------------------//

bool atacante = false;          // true = atacante, false = defensor
bool jaSaudouCabeca = false;    // Flag para enviar "OK" no startup
bool corGolAzul = false;        // false = amarelo, true = azul

#define ID_PLACA_OLHO 0x01
#define ID_PLACA_PE 0x02
#define ID_PLACA_CAMERA 0x03

// Tempos ajustados para acomodar a inicializacao da OpenMV RT1062
#define CAMERA_TIMEOUT_MS 3000
#define CAMERA_RESET_INTERVAL_MS 8000
#define RESET_PIN 40

#define EEPROM_SIZE 16
#define EEPROM_ADDR_COR_GOL 0

//------------------------------------ ULTRASSONICOS -----------------------------//

HCSR04 hcF(4, 5);     // Ultrassonico 1 - TRIGG, ECHO
HCSR04 hcD(10, 9);    // Ultrassonico 2 - TRIGG, ECHO
HCSR04 hcT(21, 47);   // Ultrassonico 3 - TRIGG, ECHO
HCSR04 hcE(37, 36);   // Ultrassonico 4 - TRIGG, ECHO

float ultraT = 0;
float ultraF = 0;
float ultraE = 0;
float ultraD = 0;

// Le os quatro ultrassonicos e atualiza distancias globais em cm.
void L_Ultra() {
  ultraD = hcD.dist();
  ultraE = hcE.dist();
  ultraF = hcF.dist();
  ultraT = hcT.dist();
}

//------------------------------------ COMUNICACAO -----------------------------//

#define RX_CABECA 17
#define TX_CABECA 18
#define RX_CAMERA 8
#define TX_CAMERA 3

//------------------------------------ IR SEEKER -----------------------------//

const int NUM_SENSORES = 12;

const int sensoresTSOP[NUM_SENSORES] = {
  6, 7, 46, 11, 12, 13, 14, 48,
  45, 35, 38, 39
};

float angulos[NUM_SENSORES] = {
  0, 30, 60, 90, 120, 150, 180, 210,
  240, 270, 300, 330
};

const unsigned long JANELA_TEMPO = 15;    // Janela de tempo para contagens de pulso de IR
const int LIMIAR_PULSOS = 2;              // Limiar de pulsos para identificar que é a bola
const int NUM_AMOSTRAS_VOTO = 5;          // Quantidade de amostras para votacao
const float TOLERANCIA_VOTO_GRAUS = 45.0f;

unsigned int pulsos[NUM_SENSORES];
unsigned int nivelBaixo[NUM_SENSORES];
float pesosIr[NUM_SENSORES];
unsigned long amostrasJanela = 1;
float intensidade = 0;

//--------------------------------------------------------------------//
// LEITURA DOS SENSORES
//
// Sensor 0 = pino 6 = angulo 0 graus
//
// Para esse sensor:
//   analogRead(6) < 950  -> LOW  -> detectou
//   analogRead(6) >= 950 -> HIGH -> nao detectou
//
// Todos os demais sensores continuam usando digitalRead().
//--------------------------------------------------------------------//

int lerSensorTSOP(int indice) {

  // Sensor do angulo 0 graus / pino 6
  if (indice == 0) {

    int valorAnalogico = analogRead(6);

    if (valorAnalogico < 950) {
      return LOW;
    } else {
      return HIGH;
    }
  }

  // Demais sensores continuam digitais
  return digitalRead(sensoresTSOP[indice]);
}

// Conta pulsos IR por sensor em uma janela curta e calcula pesos por deteccao.
void contarPulsosSensores() {

  for (int i = 0; i < NUM_SENSORES; i++) {
    pulsos[i] = 0;
    nivelBaixo[i] = 0;
    pesosIr[i] = 0;
  }

  unsigned long t0 = millis();
  amostrasJanela = 0;

  int oldState[NUM_SENSORES];

  // Leitura inicial dos estados
  for (int i = 0; i < NUM_SENSORES; i++) {
    oldState[i] = lerSensorTSOP(i);
  }

  while (millis() - t0 < JANELA_TEMPO) {

    amostrasJanela++;

    for (int i = 0; i < NUM_SENSORES; i++) {

      // Para o sensor 0, essa funcao usa analogRead().
      // Para os demais, usa digitalRead().
      int s = lerSensorTSOP(i);

      // Conta quanto tempo/amostras o sensor ficou LOW
      if (s == LOW) {
        nivelBaixo[i]++;
      }

      // Detecta transicao HIGH -> LOW
      if (oldState[i] == HIGH && s == LOW) {
        pulsos[i]++;
      }

      oldState[i] = s;
    }
  }

  if (amostrasJanela == 0) {
    amostrasJanela = 1;
  }

  // Atualizar intensidade global somente por contagem de pulsos validos.
  intensidade = 0;

  for (int i = 0; i < NUM_SENSORES; i++) {

    bool detectou = (pulsos[i] >= LIMIAR_PULSOS);

    if (detectou) {
      pesosIr[i] = (float)pulsos[i];
      intensidade += pesosIr[i];
    }
  }
}

// Calcula o angulo da bola por media vetorial dos sensores IR ativos.
float calculaAnguloBola() {

  float x = 0;
  float y = 0;
  float soma_pesos = 0;

  for (int i = 0; i < NUM_SENSORES; i++) {

    if (pesosIr[i] > 0.0f) {

      float rad = angulos[i] * PI / 180.0;

      x += pesosIr[i] * cos(rad);
      y += pesosIr[i] * sin(rad);

      soma_pesos += pesosIr[i];
    }
  }

  if (soma_pesos == 0) {
    return -1.0;
  }

  float angulo_bola = atan2(y, x) * 180.0 / PI;

  if (angulo_bola < 0) {
    angulo_bola += 360.0;
  }

  return angulo_bola;
}

// Retorna a menor diferenca angular absoluta entre dois angulos.
float diferencaAngularAbsoluta(float a, float b) {

  float d = fabs(a - b);

  if (d > 180.0f) {
    d = 360.0f - d;
  }

  return d;
}

// Filtra o angulo da bola por votacao entre amostras para reduzir ruido.
float filtrarAnguloBola() {

  float amostras[NUM_AMOSTRAS_VOTO];
  int validas = 0;

  for (int i = 0; i < NUM_AMOSTRAS_VOTO; i++) {

    contarPulsosSensores();

    float a = calculaAnguloBola();

    if (a >= 0.0f) {
      amostras[validas++] = a;
    }
  }

  if (validas == 0) {
    return -1.0f;
  }

  if (validas == 1) {
    return amostras[0];
  }

  int melhorVotos = 0;
  int melhorIdx = 0;

  for (int i = 0; i < validas; i++) {

    int votos = 0;

    for (int j = 0; j < validas; j++) {

      if (diferencaAngularAbsoluta(
            amostras[i],
            amostras[j]
          ) <= TOLERANCIA_VOTO_GRAUS) {

        votos++;
      }
    }

    if (votos > melhorVotos) {
      melhorVotos = votos;
      melhorIdx = i;
    }
  }

  float sx = 0;
  float sy = 0;

  for (int i = 0; i < validas; i++) {

    if (diferencaAngularAbsoluta(
          amostras[i],
          amostras[melhorIdx]
        ) <= TOLERANCIA_VOTO_GRAUS) {

      float rad = amostras[i] * PI / 180.0f;

      sx += cosf(rad);
      sy += sinf(rad);
    }
  }

  float resultado = atan2f(sy, sx) * 180.0f / PI;

  if (resultado < 0) {
    resultado += 360.0f;
  }

  return resultado;
}

void imprimirDebugSensoresIR(float angulo) {

  Serial.println("===== TESTE PLACA OLHO (IR) =====");

  for (int i = 0; i < NUM_SENSORES; i++) {

    Serial.print("IR[");

    Serial.print(i);

    Serial.print("] = ");

    Serial.println(pulsos[i]);
  }

  if (angulo < 0.0f) {

    Serial.println("Angulo: sem deteccao");

  } else {

    Serial.print("Angulo: ");

    Serial.print(angulo, 1);

    Serial.println(" graus");
  }

  Serial.println();
}

//------------------------------------------------//

#include <stdint.h>

struct Pacote {

  int16_t uD;
  int16_t uE;
  int16_t uF;
  int16_t uT;

  int16_t angulo;
  int16_t intensidade;

  int16_t ballAngle;
  uint16_t ballDist;

  int16_t blueAngle;
  uint16_t blueDist;

  int16_t yellowAngle;
  uint16_t yellowDist;

  uint8_t cameraOK;
};

struct PacoteEstado {

  bool sozinho;
  bool atacante;
  bool corGolAzul;
};

using namespace ComunicacaoUnificada;

Receptor receptorCabeca;

int16_t ballCameraAngle = 0;
uint16_t ballCameraDist = 0;

int16_t blueCameraAngle = -999;
uint16_t blueCameraDist = 0;

int16_t yellowCameraAngle = -999;
uint16_t yellowCameraDist = 0;

unsigned long ultimoRxCameraMs = 0;
unsigned long ultimaAtividadeCameraMs = 0;
unsigned long ultimoResetCameraMs = 0;

bool cameraOffline = false;

uint16_t falhasCamera = 0;

const uint8_t CAMERA_PAYLOAD_BYTES = 12;
const uint16_t CAMERA_INVALID_BYTES_FLUSH_LIMIT = 32;
const unsigned long CAMERA_INTERBYTE_TIMEOUT_MS = 20;

uint16_t bytesInvalidosCamera = 0;

String bufferHandshakeCabeca = "";

unsigned long ultimoByteHandshakeCabeca = 0;

//------------------------------------------------//
// CAMERA RX
//------------------------------------------------//

enum EstadoRxCamera {

  CAMERA_RX_AGUARDANDO_START = 0,
  CAMERA_RX_AGUARDANDO_ID,
  CAMERA_RX_LENDO_PAYLOAD,
  CAMERA_RX_AGUARDANDO_STOP
};

void RegistrarFalhaCamera(const char *mensagem) {

  falhasCamera++;

  Serial.println(mensagem);
}

void LimparBufferCameraCorrompido() {

  while (Serial2.available() > 0) {
    Serial2.read();
  }

  bytesInvalidosCamera = 0;

  Serial.println("CAMERA BUFFER FLUSH");
}

void ResetCamera() {

  digitalWrite(RESET_PIN, LOW);

  ultimoResetCameraMs = millis();

  Serial.println("CAMERA RESET SENT");
  Serial.println("WATCHDOG: RESET CAMERA");

  delay(100);

  digitalWrite(RESET_PIN, HIGH);
}

// Verifica se o angulo eh valido (-180 a 360)
// ou se eh o codigo de nao-detectado (-999)
bool anguloValido(int16_t angulo) {

  if (angulo == -999) {
    return true;
  }

  if (angulo >= -180 && angulo <= 360) {
    return true;
  }

  return false;
}

void AplicarPayloadCamera(const uint8_t *payload) {

  int16_t bAngle =
      (int16_t)((payload[0] << 8) | payload[1]);

  uint16_t bDist =
      (uint16_t)((payload[2] << 8) | payload[3]);

  int16_t blueA =
      (int16_t)((payload[4] << 8) | payload[5]);

  uint16_t blueD =
      (uint16_t)((payload[6] << 8) | payload[7]);

  int16_t yellowA =
      (int16_t)((payload[8] << 8) | payload[9]);

  uint16_t yellowD =
      (uint16_t)((payload[10] << 8) | payload[11]);

  // Se algum angulo for ruido/corrompido, descarta o pacote
  if (!anguloValido(bAngle) ||
      !anguloValido(blueA) ||
      !anguloValido(yellowA)) {

    RegistrarFalhaCamera("CAMERA ANGLE CORRUPTED");

    return;
  }

  ballCameraAngle = bAngle;
  ballCameraDist = bDist;

  blueCameraAngle = blueA;
  blueCameraDist = blueD;

  yellowCameraAngle = yellowA;
  yellowCameraDist = yellowD;

  ultimoRxCameraMs = millis();
  ultimaAtividadeCameraMs = ultimoRxCameraMs;

  bytesInvalidosCamera = 0;

  if (cameraOffline) {
    Serial.println("CAMERA RECOVERED");
  }

  cameraOffline = false;
}

void ProcessarPingCabeca() {

  while (Serial1.available() > 0) {

    if (Serial1.peek() == BYTE_INICIA) {
      return;
    }

    char c = (char)Serial1.read();

    ultimoByteHandshakeCabeca = millis();

    if (c == '\n' || c == '\r') {

      bufferHandshakeCabeca.trim();
      bufferHandshakeCabeca.toLowerCase();

      if (bufferHandshakeCabeca == "oi") {
        Serial1.println("OI");
      }

      bufferHandshakeCabeca = "";

      continue;
    }

    if (isPrintable(c) &&
        bufferHandshakeCabeca.length() < 16) {

      bufferHandshakeCabeca += c;

    } else {

      bufferHandshakeCabeca = "";
    }
  }

  if (bufferHandshakeCabeca.length() > 0 &&
      (millis() - ultimoByteHandshakeCabeca) > 80) {

    bufferHandshakeCabeca.trim();
    bufferHandshakeCabeca.toLowerCase();

    if (bufferHandshakeCabeca == "oi") {
      Serial1.println("OI");
    }

    bufferHandshakeCabeca = "";
  }
}

//------------------------------------------------//
// EEPROM
//------------------------------------------------//

void SalvarCorGolEEPROM() {

  EEPROM.writeByte(
      EEPROM_ADDR_COR_GOL,
      corGolAzul ? 1 : 0
  );

  EEPROM.commit();
}

void CarregarCorGolEEPROM() {

  uint8_t val =
      EEPROM.readByte(EEPROM_ADDR_COR_GOL);

  corGolAzul = (val == 1);
}

//------------------------------------------------//
// LEITURA SERIAL CABECA
//------------------------------------------------//

void LeituraSerial() {

  Frame frame;

  while (receptorCabeca.poll(Serial1, frame)) {

    if (frame.rota != Rota::CABECA_PARA_OLHO &&
        frame.rota != Rota::CABECA_PARA_TODOS) {

      continue;
    }

    PacoteEstado temp;

    if (!lerStruct(frame, temp)) {
      continue;
    }

    atacante = temp.atacante;

    bool novaCorGol = temp.corGolAzul;

    if (novaCorGol != corGolAzul) {

      corGolAzul = novaCorGol;

      SalvarCorGolEEPROM();
    }
  }

  while (Serial1.available() >= 2) {

    if (Serial1.read() == BYTE_INICIA) {

      byte id = Serial1.read();

      if (id == ID_PLACA_OLHO ||
          id == ID_PLACA_PE) {

        if (Serial1.available() >=
            sizeof(PacoteEstado) + 1) {

          PacoteEstado temp;

          Serial1.readBytes(
              (uint8_t*)&temp,
              sizeof(PacoteEstado)
          );

          byte stop = Serial1.read();

          if (stop == BYTE_PARA) {

            atacante = temp.atacante;

            bool novaCorGol =
                temp.corGolAzul;

            if (novaCorGol != corGolAzul) {

              corGolAzul = novaCorGol;

              SalvarCorGolEEPROM();
            }
          }
        }
      }
    }
  }
}

//------------------------------------------------//
// LEITURA CAMERA
//------------------------------------------------//

void LeituraCamera() {

  static EstadoRxCamera estadoRx =
      CAMERA_RX_AGUARDANDO_START;

  static uint8_t payload[CAMERA_PAYLOAD_BYTES];

  static uint8_t indicePayload = 0;

  static unsigned long ultimoByteCameraParserMs = 0;

  unsigned long agora = millis();

  while (Serial2.available() > 0) {

    uint8_t dado =
        (uint8_t)Serial2.read();

    agora = millis();

    if ((estadoRx != CAMERA_RX_AGUARDANDO_START) &&
        ((agora - ultimoByteCameraParserMs) >
         CAMERA_INTERBYTE_TIMEOUT_MS)) {

      estadoRx = CAMERA_RX_AGUARDANDO_START;

      indicePayload = 0;

      RegistrarFalhaCamera(
          "CAMERA PACKET INVALID"
      );
    }

    ultimoByteCameraParserMs = agora;

    switch (estadoRx) {

      case CAMERA_RX_AGUARDANDO_START:

        if (dado == BYTE_INICIA) {

          estadoRx = CAMERA_RX_AGUARDANDO_ID;

          indicePayload = 0;

        } else {

          bytesInvalidosCamera++;
        }

        break;

      case CAMERA_RX_AGUARDANDO_ID:

        if (dado == ID_PLACA_CAMERA) {

          estadoRx = CAMERA_RX_LENDO_PAYLOAD;

          indicePayload = 0;

        } else if (dado == BYTE_INICIA) {

          estadoRx = CAMERA_RX_AGUARDANDO_ID;

          indicePayload = 0;

          bytesInvalidosCamera++;

        } else {

          estadoRx = CAMERA_RX_AGUARDANDO_START;

          indicePayload = 0;

          bytesInvalidosCamera++;

          RegistrarFalhaCamera(
              "CAMERA PACKET INVALID"
          );
        }

        break;

      case CAMERA_RX_LENDO_PAYLOAD:

        payload[indicePayload++] = dado;

        if (indicePayload >= CAMERA_PAYLOAD_BYTES) {

          estadoRx = CAMERA_RX_AGUARDANDO_STOP;
        }

        break;

      case CAMERA_RX_AGUARDANDO_STOP:

        if (dado == BYTE_PARA) {

          AplicarPayloadCamera(payload);

          estadoRx = CAMERA_RX_AGUARDANDO_START;

          indicePayload = 0;

        } else {

          bytesInvalidosCamera++;

          RegistrarFalhaCamera(
              "CAMERA PACKET INVALID"
          );

          estadoRx =
              (dado == BYTE_INICIA)
              ? CAMERA_RX_AGUARDANDO_ID
              : CAMERA_RX_AGUARDANDO_START;

          indicePayload = 0;
        }

        break;
    }

    if (bytesInvalidosCamera >=
        CAMERA_INVALID_BYTES_FLUSH_LIMIT) {

      LimparBufferCameraCorrompido();

      estadoRx = CAMERA_RX_AGUARDANDO_START;

      indicePayload = 0;

      RegistrarFalhaCamera(
          "CAMERA PACKET INVALID"
      );

      break;
    }
  }

  unsigned long referenciaAtividadeCameraMs =
      (ultimaAtividadeCameraMs > 0)
      ? ultimaAtividadeCameraMs
      : ultimoRxCameraMs;

  bool cameraSemAtividade =
      ((referenciaAtividadeCameraMs > 0) &&
       ((agora - referenciaAtividadeCameraMs) >
        CAMERA_TIMEOUT_MS)) ||
      ((referenciaAtividadeCameraMs == 0) &&
       (agora > CAMERA_TIMEOUT_MS));

  if (cameraSemAtividade) {

    if (!cameraOffline) {

      cameraOffline = true;

      RegistrarFalhaCamera(
          "CAMERA TIMEOUT"
      );
    }

    if ((agora - ultimoResetCameraMs) >
        CAMERA_RESET_INTERVAL_MS) {

      ResetCamera();
    }
  }
}

//------------------------------------------------//
// ENVIO DOS DADOS
//------------------------------------------------//

void enviarDados() {

  Pacote p;

  p.uD = (int16_t)round(ultraD * 10.0);
  p.uE = (int16_t)round(ultraE * 10.0);
  p.uF = (int16_t)round(ultraF * 10.0);
  p.uT = (int16_t)round(ultraT * 10.0);

  p.angulo =
      (int16_t)round(
          filtrarAnguloBola() * 10.0
      );

  p.intensidade =
      (int16_t)round(
          intensidade * 10.0
      );

  p.ballAngle = ballCameraAngle;
  p.ballDist = ballCameraDist;

  p.blueAngle = blueCameraAngle;
  p.blueDist = blueCameraDist;

  p.yellowAngle = yellowCameraAngle;
  p.yellowDist = yellowCameraDist;

  p.cameraOK =
      (!cameraOffline &&
       (ultimaAtividadeCameraMs > 0) &&
       ((millis() - ultimaAtividadeCameraMs) <=
        CAMERA_TIMEOUT_MS))
      ? 1
      : 0;

  enviarStruct(
      Serial1,
      Rota::OLHO_PARA_CABECA,
      p
  );
}

//------------------------------------------------//
// SETUP
//------------------------------------------------//

void setup() {

  pinMode(RESET_PIN, OUTPUT);

  digitalWrite(RESET_PIN, HIGH);

  ultimoResetCameraMs = millis();

  // Inicializa os TSOPs
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(sensoresTSOP[i], INPUT);
  }

  Serial.begin(115200);

  unsigned long tSerial = millis();

  while (!Serial &&
         (millis() - tSerial) < 2000) {

    delay(10);
  }

  Serial.println("OLHO boot");

  EEPROM.begin(EEPROM_SIZE);

  CarregarCorGolEEPROM();

  Serial1.begin(
      115200,
      SERIAL_8N1,
      RX_CABECA,
      TX_CABECA
  );

  Serial2.begin(
      115200,
      SERIAL_8N1,
      RX_CAMERA,
      TX_CAMERA
  );

  Serial2.setTimeout(5);
}

//------------------------------------------------//
// LOOP
//------------------------------------------------//

void loop() {

  ProcessarPingCabeca();

  contarPulsosSensores();

  float angulo =
      calculaAnguloBola();

  imprimirDebugSensoresIR(angulo);

  L_Ultra();

  LeituraSerial();

  LeituraCamera();

  enviarDados();

  delay(50);
}