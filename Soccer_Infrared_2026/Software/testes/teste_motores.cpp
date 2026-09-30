// Teste direto da cinematica dos quatro motores.
// Funcao: validar sentido, PWM e mapeamento vetorial por angulo.
// Entrada: angulo e velocidade definidos no loop de teste.
// Saida: comando para cada motor (M1..M4) para conferir movimentacao.
#include <Arduino.h>

// MOTOR A
#define IN1_1_A 5
#define IN2_1_A 6
#define PWM_1_A 4

#define IN1_2_A 3
#define IN2_2_A 46
#define PWM_2_A 7

// MOTOR B
#define IN1_1_B 11
#define IN2_1_B 12
#define PWM_1_B 10

#define IN1_2_B 13
#define IN2_2_B 14
#define PWM_2_B 47

// CANAIS PWM
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

#define PWM_FREQ 20000
#define PWM_RES 8

void Motor_1(int vel1) {
  int pwm1 = constrain(abs(vel1), 0, 255);
  ledcWrite(PWM_CH1, pwm1);
  if (vel1 >= 0) {
    digitalWrite(IN1_1_A, HIGH);
    digitalWrite(IN2_1_A, LOW);
  } else {
    digitalWrite(IN1_1_A, LOW);
    digitalWrite(IN2_1_A, HIGH);
  }
}

void Motor_2(int vel2) {
  int pwm2 = constrain(abs(vel2), 0, 255);
  ledcWrite(PWM_CH2, pwm2);
  if (vel2 >= 0) {
    digitalWrite(IN1_2_A, HIGH);
    digitalWrite(IN2_2_A, LOW);
  } else {
    digitalWrite(IN1_2_A, LOW);
    digitalWrite(IN2_2_A, HIGH);
  }
}

void Motor_3(int vel3) {
  int pwm3 = constrain(abs(vel3), 0, 255);
  ledcWrite(PWM_CH3, pwm3);
  if (vel3 >= 0) {
    digitalWrite(IN1_1_B, HIGH);
    digitalWrite(IN2_1_B, LOW);
  } else {
    digitalWrite(IN1_1_B, LOW);
    digitalWrite(IN2_1_B, HIGH);
  }
}

void Motor_4(int vel4) {
  int pwm4 = constrain(abs(vel4), 0, 255);
  ledcWrite(PWM_CH4, pwm4);
  if (vel4 >= 0) {
    digitalWrite(IN1_2_B, HIGH);
    digitalWrite(IN2_2_B, LOW);
  } else {
    digitalWrite(IN1_2_B, LOW);
    digitalWrite(IN2_2_B, HIGH);
  }
}

void moverRobo(int vel1, int vel2, int vel3, int vel4) {
  Motor_1(vel1);
  Motor_2(vel2);
  Motor_3(vel3);
  Motor_4(vel4);
}

void moverPorAnguloCardinal(int anguloGraus, int velocidadeBase) {
  int v = constrain(velocidadeBase, 0, 255);

  switch (anguloGraus) {
    case 0:
      // 0: -vel1, -vel2, vel3, vel4
      moverRobo(-v, -v, v, v);
      break;
    case 90:
      // 90: -vel1, vel2, -vel3, vel4
      moverRobo(-v, v, -v, v);
      break;
    case 180:
      // 180: vel1, vel2, -vel3, -vel4
      moverRobo(v, v, -v, -v);
      break;
    case 270:
      // 270: vel1, -vel2, vel3, -vel4
      moverRobo(v, -v, v, -v);
      break;
    default:
      moverRobo(0, 0, 0, 0);
      break;
  }
}

float normalizarAngulo360(float ang) {
  while (ang >= 360.0f) ang -= 360.0f;
  while (ang < 0.0f) ang += 360.0f;
  return ang;
}

float lerpFloat(float a, float b, float t) {
  return a + (b - a) * t;
}

void moverPorAnguloVetorialPlaca(float anguloGraus, int velocidadeBase) {
  float ang = normalizarAngulo360(anguloGraus);
  float v = (float)constrain(velocidadeBase, 0, 255);
  float t = 0.0f;

  float m1 = 0.0f;
  float m2 = 0.0f;
  float m3 = 0.0f;
  float m4 = 0.0f;

  if (ang < 90.0f) {
    // Interpola entre 0 e 90:
    // 0   -> [-v, -v, +v, +v]
    // 90  -> [-v, +v, -v, +v]
    t = ang / 90.0f;
    m1 = -v;
    m2 = lerpFloat(-v, v, t);
    m3 = lerpFloat(v, -v, t);
    m4 = v;
  } else if (ang < 180.0f) {
    // Interpola entre 90 e 180:
    // 90  -> [-v, +v, -v, +v]
    // 180 -> [+v, +v, -v, -v]
    t = (ang - 90.0f) / 90.0f;
    m1 = lerpFloat(-v, v, t);
    m2 = v;
    m3 = -v;
    m4 = lerpFloat(v, -v, t);
  } else if (ang < 270.0f) {
    // Interpola entre 180 e 270:
    // 180 -> [+v, +v, -v, -v]
    // 270 -> [+v, -v, +v, -v]
    t = (ang - 180.0f) / 90.0f;
    m1 = v;
    m2 = lerpFloat(v, -v, t);
    m3 = lerpFloat(-v, v, t);
    m4 = -v;
  } else {
    // Interpola entre 270 e 360(0):
    // 270 -> [+v, -v, +v, -v]
    // 360 -> [-v, -v, +v, +v]
    t = (ang - 270.0f) / 90.0f;
    m1 = lerpFloat(v, -v, t);
    m2 = -v;
    m3 = v;
    m4 = lerpFloat(-v, v, t);
  }

  moverRobo((int)m1, (int)m2, (int)m3, (int)m4);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1_1_A, OUTPUT);
  pinMode(IN2_1_A, OUTPUT);
  pinMode(IN1_2_A, OUTPUT);
  pinMode(IN2_2_A, OUTPUT);

  pinMode(IN1_1_B, OUTPUT);
  pinMode(IN2_1_B, OUTPUT);
  pinMode(IN1_2_B, OUTPUT);
  pinMode(IN2_2_B, OUTPUT);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_A, PWM_CH1);

  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_A, PWM_CH2);

  ledcSetup(PWM_CH3, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_1_B, PWM_CH3);

  ledcSetup(PWM_CH4, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_2_B, PWM_CH4);

  Serial.println("Teste motores iniciado");
}

void loop() {
  float angulo = 10.0f;  // Ex.: bola em 10 graus (setor 0-90)
  int velocidade = 200;

  moverPorAnguloVetorialPlaca(angulo, velocidade);
  delay(1000);
  moverRobo(255, 0, 0, 0);
  delay(2000);

}
