#include <Arduino.h>
#include "defensor.hpp"
#include "motores_movimentacao.hpp"

// =============================================================================
// PARÂMETROS E CONSTANTES DO DEFENSOR
// =============================================================================

// Referências de zona da linha
constexpr float DEFENSOR_REFERENCIA_ZONA_A = 90.0f;
constexpr float DEFENSOR_REFERENCIA_ZONA_B = 270.0f;

// Tolerâncias
constexpr float DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS = 3.0f;
constexpr float DEFENSOR_TOLERANCIA_GIRO_GRAUS             = 5.0f;

// Deadzones / suavização
constexpr float DEFENSOR_DEADZONE_VETOR = 6.0f;
constexpr float DEFENSOR_DEADZONE_GIRO  = 5.0f;
constexpr float DEFENSOR_SUAVIZACAO_VETOR = 0.45f;
constexpr float DEFENSOR_SUAVIZACAO_GIRO  = 0.35f;
// -----------------------------------------------------------------------------
// GERENCIADOR DE ACAO PROGRESSIVA
// -----------------------------------------------------------------------------
// Toda translacao do defensor passa por este filtro antes de chegar aos motores.
// A estrategia continua podendo mudar o alvo instantaneamente, mas a acao fisica
// muda de forma progressiva em X/Y e no giro. As taxas sao por segundo, portanto
// o comportamento fica praticamente independente da frequencia do loop.
constexpr float DEFENSOR_FILTRO_ALVO_TAU_MS          = 45.0f;
constexpr float DEFENSOR_FILTRO_GIRO_TAU_MS          = 35.0f;
constexpr float DEFENSOR_RAMPA_ACEL_PWM_S            = 1800.0f;
constexpr float DEFENSOR_RAMPA_FREIO_PWM_S           = 3000.0f;
constexpr float DEFENSOR_RAMPA_INVERSAO_PWM_S        = 2400.0f;
constexpr float DEFENSOR_RAMPA_GIRO_PWM_S            = 3600.0f;
constexpr float DEFENSOR_RAMPA_RAPIDA_PWM_S          = 7600.0f;
constexpr float DEFENSOR_DT_MIN_S                     = 0.001f;
constexpr float DEFENSOR_DT_MAX_S                     = 0.050f;
constexpr float DEFENSOR_SAIDA_ZERO_PWM               = 4.0f;

// Velocidades base
constexpr float DEFENSOR_VELOCIDADE_MIN_PWM            = 180.0f;
constexpr float DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM    = 220.0f;
constexpr int   DEFENSOR_VELOCIDADE_MAX_BOLA_PWM       = 255;
constexpr int   DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM   = 100;
constexpr int   DEFENSOR_VELOCIDADE_MAX_CORRECAO_PWM   = 255;
constexpr float DEFENSOR_MAGNITUDE_MINIMA_PARAR        = 0.2f;

// Avanço frontal
const float         DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS      = 45.0f;
const int           DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM = 230;
const unsigned long DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS      = 3000;
const unsigned long DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS       = 1800;

// PID linha (magnitude)
constexpr float KP_DEFENSOR_LINHA = 200.0f;
constexpr float KI_DEFENSOR_LINHA = 0.1f;
constexpr float KD_DEFENSOR_LINHA = 20.0f;

// PID bússola
constexpr float KP_BUSSOLA_NOVINHA = 2.5f;
constexpr float KI_BUSSOLA_NOVINHA = 0.0f;
constexpr float KD_BUSSOLA_NOVINHA = 0.8f;

// Pesos de bola / ultra
constexpr float DEFENSOR_PESO_MIN_BOLA = 75.0f;
constexpr float DEFENSOR_PESO_MAX_BOLA = 200.0f;
constexpr float DEFENSOR_PESO_MAX_ULTRA = 100.0f;
constexpr float DEFENSOR_PESO_MAX_ULTRA_PROFUNDIDADE = 38.0f;

// Ultras (repulsão/ajuste)
constexpr float DEFENSOR_ULTRA_LATERAL_ATIVO_CM       = 45.0f;
constexpr float DEFENSOR_ULTRA_LATERAL_CRITICO_CM     = 30.0f;
constexpr float DEFENSOR_ULTRA_LATERAL_CONFIRMADOR_CM = 120.0f;
constexpr float DEFENSOR_ULTRA_FRENTE_LIMITE_CM       = 40.0f;
constexpr float DEFENSOR_ULTRA_TRAS_LIMITE_CM         = 25.0f;
// O ultrassonico traseiro so pode atuar quando o robo estiver proximo
// de pelo menos uma das laterais. Se D e E forem >= 55 cm, o T e ignorado.
constexpr float DEFENSOR_ULTRA_TRAS_HABILITA_LATERAL_CM = 55.0f;
constexpr float DEFENSOR_ULTRA_FRONTAL_CONFIRMADOR_CM = 100.0f;
constexpr float DEFENSOR_ULTRA_PROFUNDIDADE_MAX_CM    = 60.0f;
constexpr float DEFENSOR_ULTRA_PROFUNDIDADE_MIN_CM    = 10.0f;

// Velocidade lateral por IR
constexpr int   DEFENSOR_VEL_BOLA_LATERAL_MIN_PWM = 220;
constexpr int   DEFENSOR_VEL_BOLA_LATERAL_MAX_PWM = 255;
constexpr float DEFENSOR_ANGULO_LATERAL_MIN_GRAUS = 0.0f;
constexpr float DEFENSOR_ANGULO_LATERAL_MAX_GRAUS = 135.0f;

// Limites físicos da área de atuação por ultrassom.
//
// IMPORTANTE:
// Não há estado persistente / latch / histerese.
// A proteção é recalculada em TODOS os ciclos usando somente a leitura atual.
//
// Laterais:
//   > 45 cm          -> movimento normal
//   30 < d <= 45 cm  -> bloqueia o sentido para fora e corrige para dentro
//   d <= 30 cm       -> correção forte para dentro
//
// Traseiro:
//   limite NÃO é fixo.
//   Na borda lateral: proteção ~20 cm / crítico ~15 cm.
//   No centro do gol: proteção ~32 cm / crítico ~28 cm.
//   Entre esses pontos o limite varia continuamente conforme os ultras laterais.
constexpr float ULTRA_LIMITE_LATERAL_DIR_CM        = 50.0f;
constexpr float ULTRA_LIMITE_LATERAL_ESQ_CM        = 50.0f;

// Limite traseiro DINÂMICO conforme a posição lateral do defensor.
//
// Na borda lateral da área:
//   proteção traseira ~25 cm
//   crítico traseiro  ~20 cm
//
// No centro do gol:
//   proteção traseira ~32 cm
//   crítico traseiro  ~30 cm
//
// A transição é contínua usando a MENOR leitura lateral.
// Quanto mais longe das paredes laterais, mais o robô é considerado centralizado.
constexpr float DEFENSOR_TRAS_LIMITE_BORDA_CM      = 25.0f;
constexpr float DEFENSOR_TRAS_LIMITE_CENTRO_CM     = 40.0f;
constexpr float DEFENSOR_TRAS_CRITICO_BORDA_CM     = 20.0f;
constexpr float DEFENSOR_TRAS_CRITICO_CENTRO_CM    = 35.0f;

// Faixa usada para estimar posição lateral.
// <=45 cm  -> região de borda
// >=80 cm  -> região central
constexpr float DEFENSOR_LATERAL_REF_BORDA_CM      = 50.0f;
constexpr float DEFENSOR_LATERAL_REF_CENTRO_CM     = 80.0f;

// Intensidade mínima ao entrar na faixa e máxima na zona crítica.
// Esses valores são componentes vetoriais em PWM.
constexpr float DEFENSOR_CORRECAO_LIMITE_MIN_PWM   = 100.0f;
constexpr float DEFENSOR_CORRECAO_LIMITE_MAX_PWM   = 220.0f;

// Prioridade dinâmica: bola domina enquanto a correção de linha é pequena.
// A linha só ganha força quando o erro cresce e há risco real de sair da região.
constexpr float DEFENSOR_LINHA_ERRO_INICIO_CONTENCAO = 0.25f;
constexpr float DEFENSOR_LINHA_ERRO_FORTE_CONTENCAO  = 0.70f;
constexpr float DEFENSOR_PESO_LINHA_MIN_COM_BOLA     = 0.08f;
constexpr float DEFENSOR_PESO_LINHA_MAX_COM_BOLA     = 0.2f;

// Robustez da leitura da linha
// Retém a última leitura válida por um intervalo curto para absorver perdas pontuais.
constexpr unsigned long DEFENSOR_RETENCAO_LEITURA_LINHA_MS = 50;
// Se uma zona continua marcada como válida, mas seu timestamp deixa de atualizar,
// corta a ação atual. Valor deliberadamente maior que a retenção curta acima.
constexpr unsigned long DEFENSOR_TIMEOUT_ATUALIZACAO_LINHA_MS = 600;

// NOVO: faixa traseira da bola (não perseguir)
constexpr float DEFENSOR_ANGULO_TRASEIRO_MIN = 150.0f;
constexpr float DEFENSOR_ANGULO_TRASEIRO_MAX = 210.0f;

// NOVO: centralização lateral sem IR
constexpr float DEFENSOR_CENTRO_GOL_LATERAL_CM      = 85.0f;
constexpr float DEFENSOR_CENTRO_GOL_TOLERANCIA_CM   = 8.0f;
constexpr float DEFENSOR_CENTRO_GOL_ERRO_MAX_REF_CM = 35.0f;
constexpr float DEFENSOR_CENTRO_GOL_PESO_MAX        = 140.0f;

constexpr unsigned long DEFENSOR_TEMPO_SEM_BOLA_CENTRO_MS = 500;
constexpr float DEFENSOR_ULTRA_CENTRO_GATILHO_CM = 75.0f;
constexpr int DEFENSOR_ULTRA_CENTRO_VELOCIDADE_PWM = 220;
constexpr float DEFENSOR_TOL_LINHA_PARA_CENTRO_ULTRA_GRAUS = 25.0f;

// =============================================================================
// PID LINHA
// =============================================================================
static float erroAnteriorLinha = 0.0f;
static float integralLinha = 0.0f;
static unsigned long tempoAnteriorLinhaMs = 0;

void resetPidLinha() {
  integralLinha = 0.0f;
  erroAnteriorLinha = 0.0f;
  tempoAnteriorLinhaMs = 0;
}

float calcularSaidaPidLinha(float erroMagnitude, unsigned long agora) {
  float dt = (tempoAnteriorLinhaMs > 0) ? (agora - tempoAnteriorLinhaMs) / 1000.0f : 0.02f;
  if (dt <= 0.0f) dt = 0.02f;

  float pTerm = KP_DEFENSOR_LINHA * erroMagnitude;
  integralLinha += erroMagnitude * dt;
  float iTerm = KI_DEFENSOR_LINHA * integralLinha;
  float dTerm = KD_DEFENSOR_LINHA * ((erroMagnitude - erroAnteriorLinha) / dt);

  erroAnteriorLinha = erroMagnitude;
  tempoAnteriorLinhaMs = agora;

  return pTerm + iTerm + dTerm;
}

// =============================================================================
// PID BÚSSOLA
// =============================================================================
static float erroAnteriorBussola = 0.0f;
static float integralBussola = 0.0f;
static unsigned long tempoAnteriorBussolaMs = 0;

void resetPidZimBussola() {
  integralBussola = 0.0f;
  erroAnteriorBussola = 0.0f;
  tempoAnteriorBussolaMs = 0;
}

float PIDZIMBUSSOLANOVINHA_DEFENSOR(float erroBussola) {
  unsigned long agora = millis();
  float dt = (tempoAnteriorBussolaMs > 0) ? (agora - tempoAnteriorBussolaMs) / 1000.0f : 0.02f;
  if (dt <= 0.0f) dt = 0.02f;

  float pTerm = KP_BUSSOLA_NOVINHA * erroBussola;
  integralBussola += erroBussola * dt;
  float iTerm = KI_BUSSOLA_NOVINHA * integralBussola;
  float dTerm = KD_BUSSOLA_NOVINHA * ((erroBussola - erroAnteriorBussola) / dt);

  erroAnteriorBussola = erroBussola;
  tempoAnteriorBussolaMs = agora;

  return pTerm + iTerm + dTerm;
}

// =============================================================================
// AUXILIARES
// =============================================================================
float suavizarDefensor(float atual, float alvo, float fator) {
  float fatorClamped = constrain(fator, 0.0f, 1.0f);
  return atual + ((alvo - atual) * fatorClamped);
}

float aplicarDeadzoneDefensor(float valor, float deadzone) {
  return (fabsf(valor) < deadzone) ? 0.0f : valor;
}

float calcularMagnitudeVetorDefensor(float vetorX, float vetorY) {
  return sqrtf((vetorX * vetorX) + (vetorY * vetorY));
}

float calcularAnguloVetorDefensor(float vetorX, float vetorY) {
  return normalizarAngulo360(atan2f(vetorX, vetorY) * 180.0f / PI);
}

void calcularVetorPontoMedioLinha(float anguloA, float anguloB, float &anguloResultante, float &magnitudeResultante) {
  float radA = anguloA * PI / 180.0f;
  float radB = anguloB * PI / 180.0f;

  float xA = sinf(radA);
  float yA = cosf(radA);
  float xB = sinf(radB);
  float yB = cosf(radB);

  float xM = (xA + xB) / 2.0f;
  float yM = (yA + yB) / 2.0f;

  anguloResultante = calcularAnguloVetorDefensor(xM, yM);
  magnitudeResultante = calcularMagnitudeVetorDefensor(xM, yM);
}

float calcularAnguloMistoDefesa(float anguloBola, float anguloLinha, float magnitudeLinha) {
  // BOLA É A PRIORIDADE. A correção da linha começa fraca e cresce apenas
  // quando o erro de linha indica risco de o robô abandonar a região de defesa.
  float pesoLinha = DEFENSOR_PESO_LINHA_MIN_COM_BOLA;

  if (magnitudeLinha > DEFENSOR_LINHA_ERRO_INICIO_CONTENCAO) {
    float t = constrain(
      (magnitudeLinha - DEFENSOR_LINHA_ERRO_INICIO_CONTENCAO) /
      (DEFENSOR_LINHA_ERRO_FORTE_CONTENCAO - DEFENSOR_LINHA_ERRO_INICIO_CONTENCAO),
      0.0f, 1.0f
    );
    pesoLinha = DEFENSOR_PESO_LINHA_MIN_COM_BOLA +
                t * (DEFENSOR_PESO_LINHA_MAX_COM_BOLA - DEFENSOR_PESO_LINHA_MIN_COM_BOLA);
  }

  float pesoBola = 1.0f - pesoLinha;
  float radBola  = normalizarAngulo360(anguloBola)  * PI / 180.0f;
  float radLinha = normalizarAngulo360(anguloLinha) * PI / 180.0f;

  float x = sinf(radBola) * pesoBola + sinf(radLinha) * pesoLinha;
  float y = cosf(radBola) * pesoBola + cosf(radLinha) * pesoLinha;

  return calcularAnguloVetorDefensor(x, y);
}

float calcularDirecaoObrigatoriaBolaDefensor(float anguloBola) {
  if (anguloBola >= 210.0f && anguloBola <= 350.0f) return 270.0f;
  if (anguloBola >= 10.0f && anguloBola <= 150.0f) return 90.0f;

  anguloBola = normalizarAngulo360(anguloBola);
  if (anguloBola >= 210.0f) return 270.0f;
  if (anguloBola <= 150.0f) return 90.0f;

  return anguloBola;
}

bool bolaEmSetorTraseiro(float anguloBola) {
  float a = normalizarAngulo360(anguloBola);
  return (a >= DEFENSOR_ANGULO_TRASEIRO_MIN && a <= DEFENSOR_ANGULO_TRASEIRO_MAX);
}

int calcularVelocidadeLateralPorIr(float anguloIrBruto) {
  float a = normalizarAngulo360(anguloIrBruto);
  float distanciaFrontal = 0.0f;

  if (a >= 0.0f && a <= 150.0f) {
    distanciaFrontal = a;
  } else if (a >= 210.0f && a <= 360.0f) {
    distanciaFrontal = 360.0f - a;
  } else {
    return DEFENSOR_VEL_BOLA_LATERAL_MAX_PWM;
  }

  float ganho = mapearFaixaClamped(
    distanciaFrontal,
    DEFENSOR_ANGULO_LATERAL_MIN_GRAUS,
    DEFENSOR_ANGULO_LATERAL_MAX_GRAUS,
    0.0f,
    1.0f
  );

  return (int)roundf(
    DEFENSOR_VEL_BOLA_LATERAL_MIN_PWM +
    ganho * (DEFENSOR_VEL_BOLA_LATERAL_MAX_PWM - DEFENSOR_VEL_BOLA_LATERAL_MIN_PWM)
  );
}

// =============================================================================
// LIMITES FÍSICOS DA ÁREA DE ATUAÇÃO POR ULTRASSOM
// =============================================================================
//
// Esta lógica NÃO guarda estado entre ciclos.
// Ela usa somente a leitura atual dos ultras.
//
// Convenção do movimento:
//   0°   = frente       -> Y+
//   90°  = direita      -> X+
//   180° = trás         -> Y-
//   270° = esquerda     -> X-
//
// Regras:
// - Entrou na faixa de proteção: nunca permite continuar para fora.
// - Ao mesmo tempo, garante uma componente mínima no sentido de recuperação.
// - Quanto mais próximo da zona crítica, maior a correção.
// - Saiu da faixa no ciclo seguinte: a correção deixa de existir imediatamente.

// Calcula o limite traseiro permitido conforme a posição lateral estimada.
// Usa a menor distância entre os ultras laterais:
// - perto de uma lateral -> limite traseiro menor
// - perto do centro      -> limite traseiro maior
//
// Não usa estado persistente. O valor é recalculado a cada ciclo.
bool ultraTrasHabilitadoPelasLaterais(bool ultrasRecentes) {
  if (!ultrasRecentes) return false;

  const bool direitaHabilita =
    (ultraDcm >= 0.0f) &&
    (ultraDcm < DEFENSOR_ULTRA_TRAS_HABILITA_LATERAL_CM);

  const bool esquerdaHabilita =
    (ultraEcm >= 0.0f) &&
    (ultraEcm < DEFENSOR_ULTRA_TRAS_HABILITA_LATERAL_CM);

  // Nova condicao: se faltar a Zona A OU a Zona B, o ultra traseiro
  // tambem fica autorizado a atuar, mesmo com D/E acima de 55 cm.
  const bool zonaAAusente =
    (!linhaZonaAValida) || (anguloLinhaZonaA < 0.0f);

  const bool zonaBAusente =
    (!linhaZonaBValida) || (anguloLinhaZonaB < 0.0f);

  const bool linhaIncompleta = zonaAAusente || zonaBAusente;

  return direitaHabilita || esquerdaHabilita || linhaIncompleta;
}
void calcularLimitesTraseirosDinamicos(bool ultrasRecentes,
                                       float &limiteTrasCm,
                                       float &criticoTrasCm) {
  // Fallback conservador caso os ultras laterais não estejam confiáveis.
  limiteTrasCm  = DEFENSOR_ULTRA_TRAS_LIMITE_CM;
  criticoTrasCm = DEFENSOR_TRAS_CRITICO_BORDA_CM;

  if (!ultrasRecentes) return;
  if (ultraDcm < 0.0f || ultraEcm < 0.0f) return;

  float menorLateral = (ultraDcm < ultraEcm) ? ultraDcm : ultraEcm;

  float fatorCentro = mapearFaixaClamped(
    menorLateral,
    DEFENSOR_LATERAL_REF_BORDA_CM,
    DEFENSOR_LATERAL_REF_CENTRO_CM,
    0.0f,
    1.0f
  );

  limiteTrasCm =
    DEFENSOR_TRAS_LIMITE_BORDA_CM +
    fatorCentro *
    (DEFENSOR_TRAS_LIMITE_CENTRO_CM -
     DEFENSOR_TRAS_LIMITE_BORDA_CM);

  criticoTrasCm =
    DEFENSOR_TRAS_CRITICO_BORDA_CM +
    fatorCentro *
    (DEFENSOR_TRAS_CRITICO_CENTRO_CM -
     DEFENSOR_TRAS_CRITICO_BORDA_CM);

  if (criticoTrasCm >= limiteTrasCm) {
    criticoTrasCm = limiteTrasCm - 1.0f;
  }
}

float calcularForcaCorrecaoLimiteUltra(float distanciaCm,
                                       float limiteCm,
                                       float criticoCm) {
  if (distanciaCm < 0.0f || distanciaCm > limiteCm) {
    return 0.0f;
  }

  if (distanciaCm <= criticoCm) {
    return DEFENSOR_CORRECAO_LIMITE_MAX_PWM;
  }

  float faixa = limiteCm - criticoCm;
  if (faixa <= 0.001f) {
    return DEFENSOR_CORRECAO_LIMITE_MAX_PWM;
  }

  // 0 no crítico -> 1 no limite.
  // Queremos MAX no crítico e MIN no limite.
  float t = (distanciaCm - criticoCm) / faixa;
  t = constrain(t, 0.0f, 1.0f);

  return DEFENSOR_CORRECAO_LIMITE_MAX_PWM -
         t * (DEFENSOR_CORRECAO_LIMITE_MAX_PWM -
              DEFENSOR_CORRECAO_LIMITE_MIN_PWM);
}

// Aplica os limites diretamente nas componentes X/Y do comando.
//
// Em vez de criar um novo "modo de recuperação", modifica apenas o comando
// deste ciclo. Isso evita o problema de continuar andando mesmo depois que
// o ultra já recuperou a distância.
void aplicarLimitesUltraAoVetor(float &vetorX,
                                float &vetorY,
                                bool ultrasRecentes) {
  if (!ultrasRecentes) {
    return;
  }

  bool limiteDirAtivo =
    (ultraDcm >= 0.0f) &&
    (ultraDcm <= ULTRA_LIMITE_LATERAL_DIR_CM);

  bool limiteEsqAtivo =
    (ultraEcm >= 0.0f) &&
    (ultraEcm <= ULTRA_LIMITE_LATERAL_ESQ_CM);

  float limiteTrasAtualCm = DEFENSOR_ULTRA_TRAS_LIMITE_CM;
  float criticoTrasAtualCm = DEFENSOR_TRAS_CRITICO_BORDA_CM;

  calcularLimitesTraseirosDinamicos(
    ultrasRecentes,
    limiteTrasAtualCm,
    criticoTrasAtualCm
  );

  // O traseiro so participa quando D OU E estiver abaixo de 55 cm.
  // Se os dois laterais estiverem >= 55 cm, o T nao altera o vetor.
  const bool ultraTrasPermitido =
    ultraTrasHabilitadoPelasLaterais(ultrasRecentes);
  bool limiteTrasAtivo =
    ultraTrasPermitido &&
    (ultraTcm >= 0.0f) &&
    (ultraTcm <= limiteTrasAtualCm);

  // -------------------------------------------------------------------
  // LATERAIS
  // -------------------------------------------------------------------
  //
  // Se por alguma anomalia os dois limites laterais estiverem ativos ao
  // mesmo tempo, não força nenhum lado: apenas corta a componente lateral.
  // Isso evita escolher arbitrariamente direita ou esquerda.
  if (limiteDirAtivo && limiteEsqAtivo) {
    vetorX = 0.0f;
  }
  else if (limiteDirAtivo) {
    // Perto demais da direita:
    // 1) jamais deixa continuar para direita;
    // 2) garante movimento mínimo para esquerda até sair da faixa.
    if (vetorX > 0.0f) {
      vetorX = 0.0f;
    }

    float correcao = calcularForcaCorrecaoLimiteUltra(
      ultraDcm,
      ULTRA_LIMITE_LATERAL_DIR_CM,
      DEFENSOR_ULTRA_LATERAL_CRITICO_CM
    );

    // Se o comando original já está indo para esquerda mais forte do que
    // a correção necessária, preserva esse comando.
    if (vetorX > -correcao) {
      vetorX = -correcao;
    }
  }
  else if (limiteEsqAtivo) {
    // Perto demais da esquerda:
    // 1) jamais deixa continuar para esquerda;
    // 2) garante movimento mínimo para direita até sair da faixa.
    if (vetorX < 0.0f) {
      vetorX = 0.0f;
    }

    float correcao = calcularForcaCorrecaoLimiteUltra(
      ultraEcm,
      ULTRA_LIMITE_LATERAL_ESQ_CM,
      DEFENSOR_ULTRA_LATERAL_CRITICO_CM
    );

    if (vetorX < correcao) {
      vetorX = correcao;
    }
  }

  // -------------------------------------------------------------------
  // TRASEIRO
  // -------------------------------------------------------------------
  if (limiteTrasAtivo) {
    // Limite traseiro variável conforme a posição lateral.
    // Na borda fica próximo de 20 cm; no centro, próximo de 32 cm.
    if (vetorY < 0.0f) {
      vetorY = 0.0f;
    }

    float correcao = calcularForcaCorrecaoLimiteUltra(
      ultraTcm,
      limiteTrasAtualCm,
      criticoTrasAtualCm
    );

    // Se a estratégia já manda para frente com força maior, preserva.
    if (vetorY < correcao) {
      vetorY = correcao;
    }
  }
}

// =============================================================================
// GERENCIADOR CENTRAL DE MOVIMENTO PROGRESSIVO
// =============================================================================
//
// Fluxo:
//   estrategia -> vetor alvo -> limites dos ultras -> filtro -> rampa -> motores
//
// O filtro remove mudancas muito curtas do alvo. A rampa impede saltos bruscos de
// velocidade e, principalmente, inversoes instantaneas (+PWM -> -PWM).
//
// Em situacao critica de ultrassom a resposta fica bem mais rapida, mas ainda
// passa pela mesma saida centralizada.
struct EstadoMovimentoProgressivoDefensor {
  float alvoFiltradoX = 0.0f;
  float alvoFiltradoY = 0.0f;
  float saidaX = 0.0f;
  float saidaY = 0.0f;
  float giroFiltrado = 0.0f;
  float giroSaida = 0.0f;
  unsigned long ultimoMs = 0;
  bool inicializado = false;
};
static EstadoMovimentoProgressivoDefensor movimentoProgressivoDefensor;
void resetMovimentoProgressivoDefensor() {
  movimentoProgressivoDefensor.alvoFiltradoX = 0.0f;
  movimentoProgressivoDefensor.alvoFiltradoY = 0.0f;
  movimentoProgressivoDefensor.saidaX = 0.0f;
  movimentoProgressivoDefensor.saidaY = 0.0f;
  movimentoProgressivoDefensor.giroFiltrado = 0.0f;
  movimentoProgressivoDefensor.giroSaida = 0.0f;
  movimentoProgressivoDefensor.ultimoMs = 0;
  movimentoProgressivoDefensor.inicializado = false;
}
float calcularAlphaFiltroDefensor(float dt, float tauMs) {
  float tauS = tauMs / 1000.0f;
  if (tauS <= 0.0001f) return 1.0f;
  return constrain(dt / (tauS + dt), 0.0f, 1.0f);
}
float aplicarRampaProgressivaDefensor(float atual,
                                      float alvo,
                                      float dt,
                                      bool respostaRapida) {
  float erro = alvo - atual;
  if (fabsf(erro) <= 0.001f) return alvo;
  float taxa = DEFENSOR_RAMPA_ACEL_PWM_S;
  if (respostaRapida) {
    taxa = DEFENSOR_RAMPA_RAPIDA_PWM_S;
  } else {
    bool invertendo =
      (fabsf(atual) > DEFENSOR_SAIDA_ZERO_PWM) &&
      (fabsf(alvo)  > DEFENSOR_SAIDA_ZERO_PWM) &&
      ((atual > 0.0f && alvo < 0.0f) ||
       (atual < 0.0f && alvo > 0.0f));
    if (invertendo) {
      taxa = DEFENSOR_RAMPA_INVERSAO_PWM_S;
    } else if (fabsf(alvo) < fabsf(atual)) {
      taxa = DEFENSOR_RAMPA_FREIO_PWM_S;
    }
  }
  float deltaMax = taxa * dt;
  erro = constrain(erro, -deltaMax, deltaMax);
  return atual + erro;
}
bool ultraEmCondicaoCriticaDefensor(bool ultrasRecentes) {
  if (!ultrasRecentes) return false;
  if ((ultraDcm >= 0.0f && ultraDcm <= DEFENSOR_ULTRA_LATERAL_CRITICO_CM) ||
      (ultraEcm >= 0.0f && ultraEcm <= DEFENSOR_ULTRA_LATERAL_CRITICO_CM)) {
    return true;
  }
  // Mesmo a resposta rapida do traseiro respeita a mesma condicao:
  // T so existe para a estrategia se D OU E estiver abaixo de 55 cm.
  if (!ultraTrasHabilitadoPelasLaterais(ultrasRecentes)) {
    return false;
  }
  float limiteTras = DEFENSOR_ULTRA_TRAS_LIMITE_CM;
  float criticoTras = DEFENSOR_TRAS_CRITICO_BORDA_CM;
  calcularLimitesTraseirosDinamicos(ultrasRecentes, limiteTras, criticoTras);
  return (ultraTcm >= 0.0f && ultraTcm <= criticoTras);
}
void executarMovimentoProgressivoDefensor(float alvoX,
                                          float alvoY,
                                          int cmdGiro,
                                          bool respostaRapida = false) {
  unsigned long agora = millis();
  float dt = 0.02f;
  if (movimentoProgressivoDefensor.inicializado &&
      movimentoProgressivoDefensor.ultimoMs > 0) {
    dt = (agora - movimentoProgressivoDefensor.ultimoMs) / 1000.0f;
  }
  dt = constrain(dt, DEFENSOR_DT_MIN_S, DEFENSOR_DT_MAX_S);
  movimentoProgressivoDefensor.ultimoMs = agora;
  if (!movimentoProgressivoDefensor.inicializado) {
    movimentoProgressivoDefensor.inicializado = true;
  }
  alvoX = constrain(alvoX, -255.0f, 255.0f);
  alvoY = constrain(alvoY, -255.0f, 255.0f);
  float giroAlvo = (float)constrain(cmdGiro, -255, 255);
  // Deadzone somente no alvo. Evita que pequenos residuos mantenham o robo vibrando.
  alvoX = aplicarDeadzoneDefensor(alvoX, DEFENSOR_DEADZONE_VETOR);
  alvoY = aplicarDeadzoneDefensor(alvoY, DEFENSOR_DEADZONE_VETOR);
  giroAlvo = aplicarDeadzoneDefensor(giroAlvo, DEFENSOR_DEADZONE_GIRO);
  // 1) Filtro de primeira ordem no comando desejado.
  // Em emergencia a filtragem fica praticamente transparente.
  float alphaVetor = respostaRapida
    ? 0.85f
    : calcularAlphaFiltroDefensor(dt, DEFENSOR_FILTRO_ALVO_TAU_MS);
  float alphaGiro = respostaRapida
    ? 0.90f
    : calcularAlphaFiltroDefensor(dt, DEFENSOR_FILTRO_GIRO_TAU_MS);
  movimentoProgressivoDefensor.alvoFiltradoX = suavizarDefensor(
    movimentoProgressivoDefensor.alvoFiltradoX, alvoX, alphaVetor);
  movimentoProgressivoDefensor.alvoFiltradoY = suavizarDefensor(
    movimentoProgressivoDefensor.alvoFiltradoY, alvoY, alphaVetor);
  movimentoProgressivoDefensor.giroFiltrado = suavizarDefensor(
    movimentoProgressivoDefensor.giroFiltrado, giroAlvo, alphaGiro);
  // 2) Rampa de aceleracao/frenagem/inversao.
  movimentoProgressivoDefensor.saidaX = aplicarRampaProgressivaDefensor(
    movimentoProgressivoDefensor.saidaX,
    movimentoProgressivoDefensor.alvoFiltradoX,
    dt,
    respostaRapida);
  movimentoProgressivoDefensor.saidaY = aplicarRampaProgressivaDefensor(
    movimentoProgressivoDefensor.saidaY,
    movimentoProgressivoDefensor.alvoFiltradoY,
    dt,
    respostaRapida);
  float taxaGiro = respostaRapida
    ? DEFENSOR_RAMPA_RAPIDA_PWM_S
    : DEFENSOR_RAMPA_GIRO_PWM_S;
  float erroGiro = movimentoProgressivoDefensor.giroFiltrado -
                   movimentoProgressivoDefensor.giroSaida;
  float deltaGiroMax = taxaGiro * dt;
  erroGiro = constrain(erroGiro, -deltaGiroMax, deltaGiroMax);
  movimentoProgressivoDefensor.giroSaida += erroGiro;
  // Zera residuos quando tanto alvo quanto saida ja estao praticamente parados.
  if (fabsf(alvoX) <= DEFENSOR_SAIDA_ZERO_PWM &&
      fabsf(movimentoProgressivoDefensor.saidaX) <= DEFENSOR_SAIDA_ZERO_PWM) {
    movimentoProgressivoDefensor.alvoFiltradoX = 0.0f;
    movimentoProgressivoDefensor.saidaX = 0.0f;
  }
  if (fabsf(alvoY) <= DEFENSOR_SAIDA_ZERO_PWM &&
      fabsf(movimentoProgressivoDefensor.saidaY) <= DEFENSOR_SAIDA_ZERO_PWM) {
    movimentoProgressivoDefensor.alvoFiltradoY = 0.0f;
    movimentoProgressivoDefensor.saidaY = 0.0f;
  }
  if (fabsf(giroAlvo) <= DEFENSOR_SAIDA_ZERO_PWM &&
      fabsf(movimentoProgressivoDefensor.giroSaida) <= DEFENSOR_SAIDA_ZERO_PWM) {
    movimentoProgressivoDefensor.giroFiltrado = 0.0f;
    movimentoProgressivoDefensor.giroSaida = 0.0f;
  }
  float magnitude = calcularMagnitudeVetorDefensor(
    movimentoProgressivoDefensor.saidaX,
    movimentoProgressivoDefensor.saidaY);
  int giroFinal = constrain(
    (int)roundf(movimentoProgressivoDefensor.giroSaida),
    -255,
    255);
  if (magnitude < 0.5f) {
    girarNoEixo(giroFinal);
    return;
  }
  float direcaoFinal = calcularAnguloVetorDefensor(
    movimentoProgressivoDefensor.saidaX,
    movimentoProgressivoDefensor.saidaY);
  int velocidadeFinal = (int)roundf(constrain(magnitude, 0.0f, 255.0f));
  seguirDirecaoComGiroLaterais(
    direcaoFinal,
    velocidadeFinal,
    giroFinal
  );
}
void girarDefensorProgressivo(int cmdGiro, bool respostaRapida = false) {
  executarMovimentoProgressivoDefensor(0.0f, 0.0f, cmdGiro, respostaRapida);
}
// ÚNICA saída de translação do defensor com proteção dos ultras.
//
// A direcao/velocidade solicitada pela estrategia e convertida para X/Y.
// Os ultras modificam somente as componentes necessarias.
// Depois o resultado passa pelo gerenciador progressivo antes dos motores.
void seguirDirecaoDefensorComLimites(float direcaoCmd,
                                     int velocidadePwm,
                                     int cmdGiro,
                                     bool ultrasRecentes) {
  float anguloRad = normalizarAngulo360(direcaoCmd) * PI / 180.0f;
  float velocidade = (float)constrain(velocidadePwm, 0, 255);
  float vetorX = sinf(anguloRad) * velocidade;
  float vetorY = cosf(anguloRad) * velocidade;
  aplicarLimitesUltraAoVetor(
    vetorX,
    vetorY,
    ultrasRecentes
  );
  bool respostaRapida = ultraEmCondicaoCriticaDefensor(ultrasRecentes);
  executarMovimentoProgressivoDefensor(
    vetorX,
    vetorY,
    cmdGiro,
    respostaRapida
  );
}
// NOVO: centraliza no gol por ultras laterais (sem IR)
float calcularCorrecaoCentroGolPorUltraX(bool ultrasRecentes) {
  if (!ultrasRecentes) return 0.0f;
  if (ultraDcm < 0.0f || ultraEcm < 0.0f) return 0.0f;

  float erroDir = ultraDcm - DEFENSOR_CENTRO_GOL_LATERAL_CM;
  float erroEsq = ultraEcm - DEFENSOR_CENTRO_GOL_LATERAL_CM;

  float mediaAbsErro = (fabsf(erroDir) + fabsf(erroEsq)) * 0.5f;
  if (mediaAbsErro <= DEFENSOR_CENTRO_GOL_TOLERANCIA_CM) return 0.0f;

  // Se direita > esquerda, robô está deslocado para a esquerda -> mover para direita (X+)
  float erroCentro = (ultraDcm - ultraEcm) * 0.5f;

  return mapearFaixaClamped(
    erroCentro,
    -DEFENSOR_CENTRO_GOL_ERRO_MAX_REF_CM,
     DEFENSOR_CENTRO_GOL_ERRO_MAX_REF_CM,
    -DEFENSOR_CENTRO_GOL_PESO_MAX,
     DEFENSOR_CENTRO_GOL_PESO_MAX
  );
}

float comporAnguloRetornoBussolaComUltraLaterais(float anguloRetornoBase, bool ultrasRecentes) {
  float anguloBaseRad = anguloRetornoBase * PI / 180.0f;

  // A bússola continua definindo o avanço para o gol.
  float vetorX = sinf(anguloBaseRad) * DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;
  float vetorY = cosf(anguloBaseRad) * DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM;

  // Durante o retorno, usa exatamente a mesma lógica de centralização lateral
  // já usada quando o defensor está sobre a linha e perde o IR.
  // Alvo: ultraD ~= 85 cm e ultraE ~= 85 cm, com tolerância de +/- 8 cm.
  float correcaoCentroX = calcularCorrecaoCentroGolPorUltraX(ultrasRecentes);
  vetorX += correcaoCentroX;

  return calcularAnguloVetorDefensor(vetorX, vetorY);
}

// =============================================================================
// AVANÇO FRONTAL TEMPORIZADO
// =============================================================================
// Sinalizado pelo watchdog da linha para que um avanço/alinhamento temporizado
// antigo não seja retomado depois que a leitura voltar a atualizar.
static bool cancelarAvancoFrontalPorWatchdogLinha = false;

// Indica SOMENTE o período em que o robô está realmente executando o avanço
// frontal temporizado. Enquanto true:
// - perda/retenção da linha NÃO pode ativar retorno ao gol;
// - watchdog da linha NÃO pode cortar o avanço;
// - ao terminar o avanço, a lógica normal da linha volta automaticamente.
static bool avancoFrontalDefensorAtivo = false;

bool executarAvancoFrontalTemporizadoDefensor(unsigned long agora,
                                              float &vetorXSuave,
                                              float &vetorYSuave,
                                              float &cmdGiroSuave,
                                              bool ultrasRecentes) {
  static unsigned long inicioDeteccaoIrFrontalMs = 0;
  static unsigned long inicioAvancoIrFrontalMs = 0;
  static bool avancoIrFrontalAtivo = false;
  static bool alinhamentoIrFrontalAtivo = false;
  static bool aguardarSaidaJanelaIrFrontal = false;
  static float ultimoAnguloAvancoIrFrontal = 0.0f;

  if (cancelarAvancoFrontalPorWatchdogLinha) {
    inicioDeteccaoIrFrontalMs = 0;
    inicioAvancoIrFrontalMs = 0;
    avancoIrFrontalAtivo = false;
    avancoFrontalDefensorAtivo = false;
    alinhamentoIrFrontalAtivo = false;
    aguardarSaidaJanelaIrFrontal = false;
    ultimoAnguloAvancoIrFrontal = 0.0f;
    cancelarAvancoFrontalPorWatchdogLinha = false;
    return false;
  }

  bool irFrontalAtivo = irDetectado &&
                        (fabsf(normalizarErro180(anguloIr)) <= DEFENSOR_TOLERANCIA_IR_FRONTAL_GRAUS);

  if (avancoIrFrontalAtivo) {
    if ((agora - inicioAvancoIrFrontalMs) < DEFENSOR_TEMPO_AVANCO_IR_FRONTAL_MS) {
      avancoFrontalDefensorAtivo = true;
      if (irDetectado) ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
      alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
      resetPidBussola();
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      seguirDirecaoDefensorComLimites(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM, 0, ultrasRecentes);
      return true;
    }
    avancoIrFrontalAtivo = false;
    avancoFrontalDefensorAtivo = false;
    inicioAvancoIrFrontalMs = inicioDeteccaoIrFrontalMs = 0;
  }

  if (alinhamentoIrFrontalAtivo) {
    if (!irDetectado) {
      alinhamentoIrFrontalAtivo = false;
      avancoFrontalDefensorAtivo = false;
      inicioDeteccaoIrFrontalMs = 0;
      aguardarSaidaJanelaIrFrontal = false;
      resetPidBussola();
      return false;
    }
    float erroAlinhamentoBola = normalizarErro180(anguloIr);
    ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
    erroAlinhamentoGraus = erroAlinhamentoBola;
    if (fabsf(erroAlinhamentoBola) > DEFENSOR_TOLERANCIA_ALINHAMENTO_BOLA_GRAUS) {
      alinhandoAgora = true;
      vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
      int cmdPidBola = calcularSaidaPidBussola(erroAlinhamentoBola);
      int cmdGiroBola = -SINAL_GIRO_PID * cmdPidBola;
      girarDefensorProgressivo(-cmdGiroBola, true); return true;
    }
    alinhamentoIrFrontalAtivo = false;
    avancoIrFrontalAtivo = true;
    avancoFrontalDefensorAtivo = true;
    inicioAvancoIrFrontalMs = agora;
    alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
    resetPidBussola();
    vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
    seguirDirecaoDefensorComLimites(ultimoAnguloAvancoIrFrontal, DEFENSOR_VELOCIDADE_AVANCO_IR_FRONTAL_PWM, 0, ultrasRecentes);
    return true;
  }

  if (!irFrontalAtivo) { inicioDeteccaoIrFrontalMs = 0; aguardarSaidaJanelaIrFrontal = false; return false; }
  if (aguardarSaidaJanelaIrFrontal) return false;

  if (inicioDeteccaoIrFrontalMs == 0) { inicioDeteccaoIrFrontalMs = agora; return false; }
  if ((agora - inicioDeteccaoIrFrontalMs) < DEFENSOR_TEMPO_GATILHO_IR_FRONTAL_MS) return false;

  alinhamentoIrFrontalAtivo = true;
  aguardarSaidaJanelaIrFrontal = true;
  ultimoAnguloAvancoIrFrontal = normalizarAngulo360(anguloIr);
  alinhandoAgora = false; erroAlinhamentoGraus = 0.0f;
  resetPidBussola();
  vetorXSuave = vetorYSuave = cmdGiroSuave = 0.0f;
  return false;
}

// =============================================================================
// ESTRATÉGIA DO DEFENSOR
// =============================================================================
void defensor() {
  float anguloBola = -1.0f;
  bool bolaDisponivel = obterAnguloIrDisponivel(anguloBola);

  static float vetorXSuave = 0.0f;
  static float vetorYSuave = 0.0f;
  static float cmdGiroSuave = 0.0f;
  static bool retornoGolAtivo = false;
  static unsigned long inicioSemBolaMs = 0;

  bool temZonaAAtual = linhaZonaAValida && (anguloLinhaZonaA >= 0.0f);
  bool temZonaBAtual = linhaZonaBValida && (anguloLinhaZonaB >= 0.0f);
  float anguloZonaAUsado = temZonaAAtual ? anguloLinhaZonaA : -1.0f;
  float anguloZonaBUsado = temZonaBAtual ? anguloLinhaZonaB : -1.0f;

  unsigned long agora = millis();

  bool temZonaARetida =
    (!temZonaAAtual) &&
    (ultimoAnguloLinhaZonaAValido >= 0.0f) &&
    ((agora - ultimoRxLinhaZonaAMs) <= DEFENSOR_RETENCAO_LEITURA_LINHA_MS);

  bool temZonaBRetida =
    (!temZonaBAtual) &&
    (ultimoAnguloLinhaZonaBValido >= 0.0f) &&
    ((agora - ultimoRxLinhaZonaBMs) <= DEFENSOR_RETENCAO_LEITURA_LINHA_MS);

  if (temZonaARetida) anguloZonaAUsado = ultimoAnguloLinhaZonaAValido;
  if (temZonaBRetida) anguloZonaBUsado = ultimoAnguloLinhaZonaBValido;

  bool temZonaA = temZonaAAtual || temZonaARetida;
  bool temZonaB = temZonaBAtual || temZonaBRetida;

  alinhandoAgora = false;
  fugindoLinhaAgora = false;
  erroAlinhamentoGraus = 0.0f;

  const bool centroLinhaValido = temZonaA && temZonaB;
  const bool algumaLinhaValida = temZonaA || temZonaB;
  const bool ultrasRecentes =
    ultrasValidos &&
    (ultimoRxUltraMs > 0) &&
    ((agora - ultimoRxUltraMs) <= TIMEOUT_ULTRA_MS);

  if (bolaDisponivel) {
    inicioSemBolaMs = 0;
  } else if (inicioSemBolaMs == 0) {
    inicioSemBolaMs = agora;
  }

  bool semBolaTempoSuficiente =
    !bolaDisponivel &&
    (inicioSemBolaMs > 0) &&
    ((agora - inicioSemBolaMs) >= DEFENSOR_TEMPO_SEM_BOLA_CENTRO_MS);

  bool linhaAlinhadaParaCentroUltra = false;

  if (temZonaA && temZonaB) {
    float ang = 0.0f;
    float mag = 0.0f;
    calcularVetorPontoMedioLinha(
      anguloZonaAUsado,
      anguloZonaBUsado,
      ang,
      mag
    );
    linhaAlinhadaParaCentroUltra =
      (mag <= DEFENSOR_MAGNITUDE_MINIMA_PARAR);
  }
  else if (temZonaA) {
    float erroA = fabsf(
      normalizarErro180(
        normalizarAngulo360(anguloZonaAUsado) -
        DEFENSOR_REFERENCIA_ZONA_A
      )
    );
    linhaAlinhadaParaCentroUltra =
      (erroA <= DEFENSOR_TOL_LINHA_PARA_CENTRO_ULTRA_GRAUS);
  }
  else if (temZonaB) {
    float erroB = fabsf(
      normalizarErro180(
        normalizarAngulo360(anguloZonaBUsado) -
        DEFENSOR_REFERENCIA_ZONA_B
      )
    );
    linhaAlinhadaParaCentroUltra =
      (erroB <= DEFENSOR_TOL_LINHA_PARA_CENTRO_ULTRA_GRAUS);
  }

  const bool suspenderLogicaLinhaPorAvanco =
    avancoFrontalDefensorAtivo;

  bool zonaASemAtualizacao =
    temZonaAAtual &&
    (ultimoRxLinhaZonaAMs > 0) &&
    ((agora - ultimoRxLinhaZonaAMs) >
      DEFENSOR_TIMEOUT_ATUALIZACAO_LINHA_MS);

  bool zonaBSemAtualizacao =
    temZonaBAtual &&
    (ultimoRxLinhaZonaBMs > 0) &&
    ((agora - ultimoRxLinhaZonaBMs) >
      DEFENSOR_TIMEOUT_ATUALIZACAO_LINHA_MS);

  bool timeoutAtualizacaoLinha =
    zonaASemAtualizacao || zonaBSemAtualizacao;

  bool exatamenteCentroLinha = false;

  if (centroLinhaValido) {
    float a = 0.0f;
    float m = 0.0f;
    calcularVetorPontoMedioLinha(
      anguloZonaAUsado,
      anguloZonaBUsado,
      a,
      m
    );
    exatamenteCentroLinha =
      (m <= DEFENSOR_MAGNITUDE_MINIMA_PARAR);
  }

  bool exatamenteCentroGol =
    ultrasRecentes &&
    (ultraDcm >= 0.0f) &&
    (ultraEcm >= 0.0f) &&
    (fabsf(ultraDcm - DEFENSOR_CENTRO_GOL_LATERAL_CM)
      <= DEFENSOR_CENTRO_GOL_TOLERANCIA_CM) &&
    (fabsf(ultraEcm - DEFENSOR_CENTRO_GOL_LATERAL_CM)
      <= DEFENSOR_CENTRO_GOL_TOLERANCIA_CM);

  bool ultraPedeCentro =
    ultrasRecentes &&
    (
      ((ultraDcm >= 0.0f) &&
       (ultraDcm < DEFENSOR_ULTRA_CENTRO_GATILHO_CM))
      ||
      ((ultraEcm >= 0.0f) &&
       (ultraEcm < DEFENSOR_ULTRA_CENTRO_GATILHO_CM))
    );

  bool podeIgnorarWatchdogParaUltra =
    semBolaTempoSuficiente &&
    algumaLinhaValida &&
    linhaAlinhadaParaCentroUltra &&
    ultraPedeCentro &&
    !retornoGolAtivo;

  if (!suspenderLogicaLinhaPorAvanco &&
      timeoutAtualizacaoLinha &&
      !podeIgnorarWatchdogParaUltra &&
      !(exatamenteCentroLinha && exatamenteCentroGol)) {

    cancelarAvancoFrontalPorWatchdogLinha = true;
    vetorXSuave = 0.0f;
    vetorYSuave = 0.0f;
    cmdGiroSuave = 0.0f;
    resetPidLinha();
    resetPidZimBussola();
    resetMovimentoProgressivoDefensor();
    girarNoEixo(0);
    return;
  }

  if (suspenderLogicaLinhaPorAvanco) {
    retornoGolAtivo = false;
  } else {
    if (!algumaLinhaValida) {
      retornoGolAtivo = true;
    }

    if (retornoGolAtivo && algumaLinhaValida) {
      retornoGolAtivo = false;
      resetPidLinha();
    }
  }

  float erroBussola = calcularErroReferenciaBussola();

  int cmdGiroBussola =
    constrain(
      (int)roundf(
        -PIDZIMBUSSOLANOVINHA_DEFENSOR(erroBussola)
      ),
      -255,
      255
    );

  // 1) RETORNO TEM PRIORIDADE TOTAL
  if (retornoGolAtivo) {
    resetPidLinha();

    if (bussolaTemReferenciaValida()) {
      float anguloRetornoBase =
        calcularAnguloRetornoGolPorBussola();

      float anguloRetornoFinal =
        comporAnguloRetornoBussolaComUltraLaterais(
          anguloRetornoBase,
          ultrasRecentes
        );

      seguirDirecaoDefensorComLimites(
        anguloRetornoFinal,
        (int)DEFENSOR_VELOCIDADE_RETORNO_GOL_PWM,
        cmdGiroBussola,
        ultrasRecentes
      );
    } else {
      girarDefensorProgressivo(cmdGiroBussola);
    }

    return;
  }

  // 2) AVANÇO FRONTAL
  if (executarAvancoFrontalTemporizadoDefensor(
        agora,
        vetorXSuave,
        vetorYSuave,
        cmdGiroSuave,
        ultrasRecentes)) {
    return;
  }

  bool bolaTraseira =
    bolaDisponivel &&
    bolaEmSetorTraseiro(anguloBola);

  // 3) BOLA TEM PRIORIDADE SOBRE CENTRALIZAÇÃO ULTRA
  if (bolaDisponivel && !bolaTraseira) {

    if (centroLinhaValido) {
      float anguloCorrecaoLinha = 0.0f;
      float magnitudeErroLinha = 0.0f;

      calcularVetorPontoMedioLinha(
        anguloZonaAUsado,
        anguloZonaBUsado,
        anguloCorrecaoLinha,
        magnitudeErroLinha
      );

      float direcaoObrigatoria =
        calcularDirecaoObrigatoriaBolaDefensor(anguloBola);

      float anguloResultante =
        calcularAnguloMistoDefesa(
          direcaoObrigatoria,
          anguloCorrecaoLinha,
          magnitudeErroLinha
        );

      int velocidadeBola =
        calcularVelocidadeLateralPorIr(anguloBola);

      int velocidadePwm =
        (magnitudeErroLinha > DEFENSOR_MAGNITUDE_MINIMA_PARAR)
        ? (int)constrain(
            DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM +
              calcularSaidaPidLinha(
                magnitudeErroLinha,
                agora
              ),
            DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM,
            velocidadeBola
          )
        : velocidadeBola;

      seguirDirecaoDefensorComLimites(
        anguloResultante,
        velocidadePwm,
        cmdGiroBussola,
        ultrasRecentes
      );
      return;
    }

    if (algumaLinhaValida) {
      resetPidLinha();

      float direcaoObrigatoria =
        calcularDirecaoObrigatoriaBolaDefensor(anguloBola);

      int velocidadeBola =
        calcularVelocidadeLateralPorIr(anguloBola);

      seguirDirecaoDefensorComLimites(
        direcaoObrigatoria,
        velocidadeBola,
        cmdGiroBussola,
        ultrasRecentes
      );
      return;
    }
  }

  // 4) SE A LINHA PRECISA CORRIGIR, IGNORA ULTRA
  if (centroLinhaValido) {
    float anguloCorrecaoLinha = 0.0f;
    float magnitudeErroLinha = 0.0f;

    calcularVetorPontoMedioLinha(
      anguloZonaAUsado,
      anguloZonaBUsado,
      anguloCorrecaoLinha,
      magnitudeErroLinha
    );

    if (magnitudeErroLinha > DEFENSOR_MAGNITUDE_MINIMA_PARAR) {
      float saidaPidLinha =
        calcularSaidaPidLinha(
          magnitudeErroLinha,
          agora
        );

      int velocidadeLinha =
        (int)constrain(
          DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM +
            saidaPidLinha,
          DEFENSOR_VELOCIDADE_MIN_CORRECAO_PWM,
          DEFENSOR_VELOCIDADE_MAX_CORRECAO_PWM
        );

      seguirDirecaoDefensorComLimites(
        anguloCorrecaoLinha,
        velocidadeLinha,
        cmdGiroBussola,
        ultrasRecentes
      );
      return;
    }
  }

  // 5) CENTRALIZAÇÃO ULTRA SOMENTE EM REPOUSO DEFENSIVO
  bool podeCentralizarPorUltra =
    !bolaDisponivel &&
    semBolaTempoSuficiente &&
    algumaLinhaValida &&
    linhaAlinhadaParaCentroUltra &&
    ultrasRecentes;

  if (podeCentralizarPorUltra) {

    if ((ultraDcm >= 0.0f) &&
        (ultraDcm < DEFENSOR_ULTRA_CENTRO_GATILHO_CM)) {

      resetPidLinha();

      seguirDirecaoDefensorComLimites(
        270.0f,
        DEFENSOR_ULTRA_CENTRO_VELOCIDADE_PWM,
        cmdGiroBussola,
        ultrasRecentes
      );
      return;
    }

    if ((ultraEcm >= 0.0f) &&
        (ultraEcm < DEFENSOR_ULTRA_CENTRO_GATILHO_CM)) {

      resetPidLinha();

      seguirDirecaoDefensorComLimites(
        90.0f,
        DEFENSOR_ULTRA_CENTRO_VELOCIDADE_PWM,
        cmdGiroBussola,
        ultrasRecentes
      );
      return;
    }
  }

  resetPidLinha();
  girarDefensorProgressivo(cmdGiroBussola);
}
