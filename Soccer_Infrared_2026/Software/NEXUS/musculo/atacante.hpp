#ifndef ATACANTE_HPP
#define ATACANTE_HPP

#include <Arduino.h>

// ============================================================
// FUNÇÃO PRINCIPAL
// ============================================================

void atacante();


// ============================================================
// CONTROLE PRINCIPAL DO ATACANTE
// ============================================================

void resetControleMovimentoAtacante();

float calcularPidMovimento(float erro);

float suavizarAnguloMovimentoAtacante(
    float anguloMovimentoDesejado
);

extern float obterAnguloIrSuavizado(
    float anguloAlvoGraus
);

float suavizadorMegaAnguloMovimento(
    float anguloNovo
);


// ============================================================
// CONTROLE DA BOLA / IR
// ============================================================

bool obterAnguloIrDisponivel(
    float &anguloBolaGraus
);

bool irNaFaixaFrontal(
    float anguloBolaGraus
);

float mapearAnguloBolaParaMovimento(
    float anguloBolaGraus
);

int calcularVelocidadeIrPorAngulo(
    float anguloBolaGraus
);

float calcularAnguloBuscaSemBolaCameraAtacante();


// ============================================================
// CONTROLE ULTRASSÔNICO DO ATACANTE
// ============================================================

bool ultraLateralCriticoAtacante();

int aplicarFreioUltrassonicoAtacante(
    int velocidadeDesejada
);

int aplicarFreioUltrassonicoAtacanteFrente(
    int velocidadeDesejada
);


// ============================================================
// CONTROLE DA CÂMERA / GOL
// ============================================================

void resetPidGolCamera();

void resetControleGolCamera();

bool obterAnguloGolCameraAtaque(
    float &anguloGol
);

int calcularCmdGiroGolCamera(
    float anguloGol
);

void moverFrenteComGiroParaGol(
    int velocidade
);


// ============================================================
// PID DA BÚSSOLA UTILIZADO PELO ATACANTE
// ============================================================

float calcularErroReferenciaBussola();

float normalizarErro180(
    float erro
);

float PIDZIMBUSSOLANOVINHA(
    float erro
);


// ============================================================
// VARIÁVEIS GLOBAIS DO ATACANTE
// DEFINIDAS NO atacante.cpp
// ============================================================

// ------------------------------------------------------------
// Velocidades
// ------------------------------------------------------------

extern const int VELOCIDADE_IR_FRONTAL_PWM;

extern const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;


// ------------------------------------------------------------
// Freio ultrassônico
// ------------------------------------------------------------

extern const float ATACANTE_ULTRA_FREIO_INICIO_CM;

extern const float ATACANTE_ULTRA_FREIO_CRITICO_CM;

extern const int ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;

extern const int ATACANTE_ULTRA_FREIO_PWM_POR_CM;


// ------------------------------------------------------------
// Confirmação / temporização
// ------------------------------------------------------------

extern const uint8_t ATACANTE_LINHA_PAREDE_CONFIRMACAO;

extern const unsigned long ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS;


// ------------------------------------------------------------
// PID de movimento
// ------------------------------------------------------------

extern const float PID_MOVIMENTO_KP;

extern const float PID_MOVIMENTO_KI;

extern const float PID_MOVIMENTO_KD;

extern const float PID_MOVIMENTO_INTEGRAL_MAX;

extern const float PID_MOVIMENTO_SAIDA_MAX;

extern const float ALPHA_MOVIMENTO;

extern const float ALPHA_MOVIMENTO_ALVO;

extern const float PASSO_MAX_MOVIMENTO_ALVO_GRAUS;

extern uint16_t cameraGolSelecionadoDist;
// ------------------------------------------------------------
// Estado interno do PID de movimento
// ------------------------------------------------------------

extern float pidMovimentoIntegral;

extern float pidMovimento;

extern float erroMovimento;

extern float erroAnteriorMovimento;

extern float anguloMovimentoAtual;

extern float anguloMovimentoSuavizado;

extern float anguloMovimentoDesejadoFiltrado;

extern unsigned long ultimoTempoPidMovimento;

extern unsigned long inicioCameraSemIrMs;


// ------------------------------------------------------------
// PID do gol pela câmera
// ------------------------------------------------------------

extern const float PID_GOL_CAMERA_KP;

extern const float PID_GOL_CAMERA_KI;

extern const float PID_GOL_CAMERA_KD;

extern const float PID_GOL_CAMERA_INTEGRAL_MAX;

extern const int PID_GOL_CAMERA_SAIDA_MIN;

extern const int PID_GOL_CAMERA_SAIDA_MAX;

extern const float TOLERANCIA_GOL_CAMERA_GRAUS;

extern const unsigned long RETENCAO_GOL_CAMERA_ATAQUE_MS;

extern const float SALTO_MAX_GOL_CAMERA_GRAUS;


// ------------------------------------------------------------
// Estado interno do PID do gol
// ------------------------------------------------------------

extern float pidGolCameraIntegral;

extern float pidGolCameraErroAnterior;

extern unsigned long pidGolCameraUltimoMs;


// ------------------------------------------------------------
// Estado do filtro do ângulo do gol
// ------------------------------------------------------------

extern float cameraGolAnguloFiltrado;

extern bool cameraGolFiltroInicializado;

extern unsigned long cameraGolUltimaLeituraValidaMs;


// ============================================================
// VARIÁVEIS COMPARTILHADAS
// DEFINIDAS NO musculo.cpp
// ============================================================

// ------------------------------------------------------------
// Linha
// ------------------------------------------------------------

extern bool linhaDetectada;

extern float anguloLinhaPe;

extern bool fugindoLinhaAgora;

extern float anguloFugaLinhaCmd;

extern const int VELOCIDADE_FUGA_LINHA;


// ------------------------------------------------------------
// IR
// ------------------------------------------------------------

extern bool irDetectado;

extern float anguloIr;

extern float ultimoAnguloIrValido;

extern unsigned long ultimoRxIrValidoMs;

extern const unsigned long RETENCAO_IR_VALIDO_MS;


// ------------------------------------------------------------
// Ultrassônicos
// ------------------------------------------------------------

extern bool ultrasValidos;

extern unsigned long ultimoRxUltraMs;

extern float ultraDcm;

extern float ultraEcm;

extern float ultraFcm;

extern float ultraTcm;

extern const unsigned long TIMEOUT_ULTRA_MS;


// ------------------------------------------------------------
// Controle geral de velocidade / giro
// ------------------------------------------------------------

extern int velocidade_maxima;

extern const int VELOCIDADE_GIRO_ALINHAMENTO;

extern const int SINAL_GIRO_PID;


// ------------------------------------------------------------
// Zona da bola recebida do parceiro (papel defensor) via ESP-NOW
// ------------------------------------------------------------

extern char zonaDefensorRecebida;

extern unsigned long ultimoRxZonaDefensorMs;

extern const unsigned long TIMEOUT_ZONA_DEFENSOR_MS;


// ------------------------------------------------------------
// Dimensoes do campo (cm)
// ------------------------------------------------------------

extern const float CAMPO_LARGURA_CM;

extern const float CAMPO_ALTURA_CM;


// ============================================================
// FUNÇÕES COMPARTILHADAS
// DEFINIDAS EM OUTROS MÓDULOS / musculo.cpp
// ============================================================

// ------------------------------------------------------------
// Ângulos
// ------------------------------------------------------------

float normalizarAngulo360(
    float angulo
);


// ------------------------------------------------------------
// Linha
// ------------------------------------------------------------

bool sairDaLinha(
    bool linhaDetectada,
    float anguloLinhaPe,
    int velocidadeFuga,
    float *anguloFuga
);


// ------------------------------------------------------------
// Câmera
// ------------------------------------------------------------

bool cameraTemGolSelecionadoValido(
    int16_t &anguloGol
);


// ============================================================
// FUNÇÕES DE MOVIMENTAÇÃO
// DEFINIDAS EM motores_movimentacao.cpp / .hpp
// ============================================================

void seguirDirecaoComGiroLaterais(
    float anguloMovimento,
    int velocidade,
    int cmdGiro
);

void girarNoEixo(
    int velocidade
);

void moverFrenteComGiro(
    int velocidade,
    int cmdGiro
);

bool moverParaComGiro(
    float xCm,
    float yCm
);

void atualizarLeiturasPosicionamento(
    float ultraEsquerdaCm,
    float ultraDireitaCm,
    float ultraFrenteCm,
    float ultraTrasCm,
    bool leiturasValidas
);


#endif