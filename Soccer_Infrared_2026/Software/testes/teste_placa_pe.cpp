// Teste da placa Pe focado em sensores de linha.
// Funcao: ler 32 sensores via MUX, detectar linha e calcular angulo.
// Saida: resumo serial com linha detectada, angulo e sensores ativos.
#include <Arduino.h>
#include <math.h>

#define NUM_SENSORES 32
#define LIMIAR_LINHA 2500
#define INTERVALO_DEBUG_MS 200

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

uint8_t mapaSensores[NUM_SENSORES] = {
  0, 1, 2, 3,
  4, 5, 6, 7,
  8, 9, 10, 11,
  12, 13, 14, 15,
  16, 17, 18, 19,
  20, 21, 22, 23,
  24, 25, 26, 27,
  28, 29, 30, 31
};

int ldr[NUM_SENSORES];
float sensorX[NUM_SENSORES];
float sensorY[NUM_SENSORES];
unsigned long ultimoDebugMs = 0;

void selecionarCanalMUX(int s0, int s1, int s2, int s3, int canal) {
  digitalWrite(s0, bitRead(canal, 0));
  digitalWrite(s1, bitRead(canal, 1));
  digitalWrite(s2, bitRead(canal, 2));
  digitalWrite(s3, bitRead(canal, 3));
}

void lerSensores() {
  for (int canal = 0; canal < 16; canal++) {
    selecionarCanalMUX(MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3, canal);
    selecionarCanalMUX(MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3, canal);

    delayMicroseconds(10);

    ldr[canal] = analogRead(MUX1_SIG);
    ldr[canal + 16] = analogRead(MUX2_SIG);
  }
}

bool detectarLinha() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (ldr[mapaSensores[i]] >= LIMIAR_LINHA) {
      return true;
    }
  }
  return false;
}

int16_t calcularAnguloLinha() {
  float centroX = 0.0;
  float centroY = 0.0;
  float soma = 0.0;

  for (int i = 0; i < NUM_SENSORES; i++) {
    int indiceFisico = mapaSensores[i];
    float peso = ldr[indiceFisico];
    if (peso >= LIMIAR_LINHA) {
      centroX += peso * sensorX[i];
      centroY += peso * sensorY[i];
      soma += peso;
    }
  }

  if (soma == 0.0) {
    return -1;
  }

  centroX /= soma;
  centroY /= soma;

  float anguloRad = atan2(centroY, centroX);
  float anguloGraus = anguloRad * 180.0 / PI;
  if (anguloGraus < 0.0) {
    anguloGraus += 360.0;
  }

  return (int16_t)round(anguloGraus * 10.0);
}

void imprimirSensoresAtivos() {
  bool encontrou = false;

  Serial.print("Sensores na linha: ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    int indiceFisico = mapaSensores[i];
    if (ldr[indiceFisico] >= LIMIAR_LINHA) {
      if (encontrou) {
        Serial.print(" | ");
      }
      Serial.print("S");
      Serial.print(i);
      Serial.print("=");
      Serial.print(ldr[indiceFisico]);
      encontrou = true;
    }
  }

  if (!encontrou) {
    Serial.print("nenhum");
  }
  Serial.println();
}

void imprimirResumo(int16_t anguloLinha, bool linhaDetectada) {
  Serial.println("===== TESTE PLACA PE =====");
  Serial.print("Linha detectada: ");
  Serial.println(linhaDetectada ? "SIM" : "NAO");

  Serial.print("Angulo da linha: ");
  if (anguloLinha < 0) {
    Serial.println("sem linha");
  } else {
    Serial.print(anguloLinha / 10.0, 1);
    Serial.println(" graus");
  }

  imprimirSensoresAtivos();
  Serial.println();
}

void setup() {
  Serial.begin(115200);

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

  Serial.println("Teste da placa PE iniciado");
  Serial.print("Limiar de linha: ");
  Serial.println(LIMIAR_LINHA);
}

void loop() {
  lerSensores();

  bool linhaDetectada = detectarLinha();
  int16_t anguloLinha = calcularAnguloLinha();

  unsigned long agora = millis();
  if (agora - ultimoDebugMs >= INTERVALO_DEBUG_MS) {
    ultimoDebugMs = agora;
    imprimirResumo(anguloLinha, linhaDetectada);
  }

  delay(30);
}
