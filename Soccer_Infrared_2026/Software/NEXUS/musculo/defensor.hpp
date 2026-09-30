#ifndef DEFENSOR_HPP
#define DEFENSOR_HPP

#include <Arduino.h>

// ============================================================
// FUNÇÃO PRINCIPAL
// ============================================================

void defensor();

// ============================================================
// CONTROLE PRINCIPAL DO DEFENSOR
// ============================================================

bool executarAvancoFrontalTemporizadoDefensor(
    unsigned long agora,
    float &vetorXSuave,
    float &vetorYSuave,
    float &cmdGiroSuave
);

// ============================================================
// PID DE MAGNITUDE DA LINHA DO GOLEIRO
// ============================================================

void resetPidLinha();

float calcularSaidaPidLinha(
    float erroMagnitude,
    unsigned long agora
);

// ============================================================
// PID DA BÚSSOLA DO DEFENSOR
// ============================================================

float PIDZIMBUSSOLANOVINHA_DEFENSOR(
    float erro
);

void resetPidZimBussola();

// ============================================================
// FUNÇÕES AUXILIARES DE MOVIMENTO DO DEFENSOR
// ============================================================

float suavizarDefensor(float atual, float alvo, float fator);
float aplicarDeadzoneDefensor(float valor, float deadzone);
float calcularMagnitudeVetorDefensor(float vetorX, float vetorY);
float calcularAnguloVetorDefensor(float vetorX, float vetorY);

void calcularVetorPontoMedioLinha(
    float anguloA,
    float anguloB,
    float &anguloResultante,
    float &magnitudeResultante
);

// ============================================================
// VARIÁVEIS GLOBAIS EXTERNAS
// ============================================================

extern bool alinhandoAgora;
extern bool fugindoLinhaAgora;
extern float erroAlinhamentoGraus;

extern bool linhaZonaAValida;
extern float anguloLinhaZonaA;
extern float ultimoAnguloLinhaZonaAValido;
extern unsigned long ultimoRxLinhaZonaAMs;

extern bool linhaZonaBValida;
extern float anguloLinhaZonaB;
extern float ultimoAnguloLinhaZonaBValido;
extern unsigned long ultimoRxLinhaZonaBMs;

extern bool bussolaTemReferenciaValida();
extern float calcularAnguloRetornoGolPorBussola();
extern float calcularErroReferenciaBussola();
extern void resetPidBussola();
extern int calcularSaidaPidBussola(float erroGraus);

extern bool irDetectado;
extern float anguloIr;
extern bool obterAnguloIrDisponivel(float &anguloBolaGraus);

extern bool cameraLerGolSelecionadoMenu(int16_t &anguloGol, uint16_t &distanciaGol);
extern bool cameraTemGolSelecionadoValido(int16_t &anguloGol);
extern bool cameraTemBolaValida();

extern bool ultrasValidos;
extern unsigned long ultimoRxUltraMs;
extern float ultraDcm;
extern float ultraEcm;
extern float ultraFcm;
extern float ultraTcm;

extern bool corGolAzul;

extern const int VELOCIDADE_GIRO_ALINHAMENTO;
extern const int SINAL_GIRO_PID;
extern const float TOLERANCIA_ALINHAMENTO_GRAUS;
extern const unsigned long TIMEOUT_ULTRA_MS;
extern const unsigned long RETENCAO_ZONA_LINHA_DEFENSOR_MS;
extern int velocidade_maxima;

extern float normalizarAngulo360(float angulo);
extern float normalizarErro180(float erro);
extern float mapearFaixaClamped(float valor, float entradaMin, float entradaMax, float saidaMin, float saidaMax);

// ============================================================
// FUNÇÕES DE MOVIMENTAÇÃO (MOTORES)
// ============================================================

void girarNoEixo(int velocidade);
void seguirDirecaoPorAngulo(float angulo, int velocidade);
void seguirDirecaoComGiro(float angulo, int velocidade, int cmdGiro);
void seguirDirecaoComGiroLaterais(float anguloMovimento, int velocidade, int cmdGiro);

#endif