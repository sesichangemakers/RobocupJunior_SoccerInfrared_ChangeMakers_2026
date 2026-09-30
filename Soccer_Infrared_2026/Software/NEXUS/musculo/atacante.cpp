#include <Arduino.h>
#include "atacante.hpp"
#include "motores_movimentacao.hpp"

// O ESP32-S3 DevKitC-1 usa o NeoPixel embutido no GPIO 48. O framework
// normalmente fornece RGB_BUILTIN; o fallback mantém o código explícito.
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

void atualizarLedLinhaAtacante(bool linhaAtiva) {
    static int8_t ultimoEstadoLinha = -1;
    const int8_t estadoAtualLinha = linhaAtiva ? 1 : 0;

    if (estadoAtualLinha == ultimoEstadoLinha) {
        return;
    }

    if (linhaAtiva) {
        neopixelWrite(RGB_BUILTIN, 25, 25, 25);  // Branco durante a linha.
    } else {
        neopixelWrite(RGB_BUILTIN, 0, 25, 0);    // Verde fora da linha.
    }

    ultimoEstadoLinha = estadoAtualLinha;
}

// =============================================================================
// REPOSICIONAMENTO POR ZONA (A/B/C) RECEBIDA DO DEFENSOR VIA ESP-NOW
// =============================================================================
//
// A referencia bussola (headingReferenciaBussola) e sempre calibrada apontando
// para o gol adversario, e o controlador de giro do posicionamento mantem o
// robo alinhado a ela. Por isso, no referencial X/Y do campo (ultrassonicos
// F/T = eixo Y, E/D = eixo X), Y pequeno = lado de ataque (gol adversario) e
// Y grande = lado de tras/defesa — sempre, independente de qual lado fisico
// da sala esta sendo defendido no jogo atual.
//
// Zona A (direita, tras) e Zona B (esquerda, tras) ficam a 1/4 do campo de
// distancia da parede de tras, e a 1/4 do campo de distancia da lateral.
// =============================================================================

bool obterAlvoReposicionamentoPorZona(float &xCm, float &yCm) {
    const bool zonaRecente =
        (ultimoRxZonaDefensorMs > 0) &&
        ((millis() - ultimoRxZonaDefensorMs) <= TIMEOUT_ZONA_DEFENSOR_MS);

    if (!zonaRecente) {
        return false;
    }

    // Zona C = meio do campo (sem lado preferencial informado pelo defensor).
    if (zonaDefensorRecebida == 'C') {
        xCm = 89.0f;
        yCm = 120.0f;
        return true;
    }

    // Zona A = canto esquerdo inferior
    else if (zonaDefensorRecebida == 'A') {
        xCm = 40.0f;
        yCm = 181.0f;
        return true;
    }

    // Zona B = canto direito inferior
    else if (zonaDefensorRecebida == 'B') {
      xCm = 140.0f;
      yCm = 181.0f;
      return true;
    }

    yCm = CAMPO_ALTURA_CM - (CAMPO_ALTURA_CM * 0.25f);
    xCm = (zonaDefensorRecebida == 'A')
              ? (CAMPO_LARGURA_CM - (CAMPO_LARGURA_CM * 0.25f))
              : (CAMPO_LARGURA_CM * 0.25f);
    return true;
}

// =============================================================================
// ESTRATEGIA DO ATACANTE
// =============================================================================
//
// Responsabilidades:
//   • Alinhar o robô ao gol via câmera (PID de bússola)
//   • Seguir a bola por IR com controle progressivo de movimento
//   • Usar câmera como fallback quando IR está ausente
//   • Fugir da linha branca com prioridade máxima
//   • Frear ao se aproximar de paredes (ultrassônicos)
//   • Chutar ao se alinhar (kicker automático via atualizarKicker)
//

void atacante() {
    atualizarLedLinhaAtacante(linhaDetectada);

    int velo = 220;
    int veloFrente = 220;

    float anguloIrAtual = -1.0f;

    float anguloBussolaAlvo =
        calcularErroReferenciaBussola();

    float erroBussola =
        normalizarErro180(-anguloBussolaAlvo);

    int cmdGiro =
        constrain(
            (int)roundf(
                -PIDZIMBUSSOLANOVINHA(erroBussola)
            ),
            -255,
            255
        );

    float anguloFuga = 0.0f;

    // =========================================================================
    // LINHA — TRAVA DE FUGA PARA TRÁS
    // =========================================================================
    //
    // Sequência problemática observada:
    //
    //      linha 180° -> pequeno trecho sem linha -> linha 0°
    //
    // O 180° é a fuga para trás gerada quando o robô entra na linha pela
    // frente. Depois, por causa da região morta/velocidade, os sensores de trás
    // podem devolver 0°. Esse 0° NÃO deve inverter imediatamente a fuga.
    //
    // IMPORTANTE:
    // Um 0° isolado NÃO cria mais a trava, porque uma linha fisicamente atrás
    // do robô também pode devolver 0°. Nesse caso sairDaLinha() decide
    // normalmente e o robô pode fugir para frente.
    //
    // A trava só pode NASCER quando:
    //   1) o último movimento normal tinha componente frontal; E
    //   2) a linha está pedindo fuga aproximadamente para 180°.
    //
    // Depois de criada, ela tolera:
    //   • pequenos intervalos sem linha;
    //   • leitura posterior próxima de 0° da mesma passagem.
    // =========================================================================

    const unsigned long agoraLinhaMs = millis();

    static constexpr unsigned long TEMPO_SEM_LINHA_LIBERAR_TRAVA_MS = 200UL;
    static constexpr unsigned long TEMPO_MAX_FUGA_TRASEIRA_MS = 1500UL;

    // Faixa que realmente INICIA uma fuga para trás.
    static constexpr float ANGULO_FUGA_TRAS_MIN = 150.0f;
    static constexpr float ANGULO_FUGA_TRAS_MAX = 210.0f;

    // Faixa de 0° que pode aparecer DEPOIS que a trava já começou.
    static constexpr float FAIXA_ZERO_TRAVA_GRAUS = 60.0f;

    static bool fugaTraseiraTravada = false;
    static unsigned long inicioFugaTraseiraMs = 0UL;
    static unsigned long inicioSemLinhaTravaMs = 0UL;

    // Guarda se o último comando NORMAL do atacante estava indo
    // majoritariamente para frente.
    static bool ultimoMovimentoEraFrente = false;

    // Mantidos porque a estratégia normal abaixo usa essas variáveis.
    static unsigned long inicioMovimentoFrenteGolMs = 0UL;
    static unsigned long tempoContinuoFrenteGolMs = 0UL;
    static bool indoDeFrenteParaGolAgora = false;

    // Tempo seguido sem enxergar a bola antes de reposicionar por zona.
    static constexpr unsigned long TEMPO_SEM_BOLA_PARA_REPOSICIONAR_MS = 1500UL;
    static unsigned long inicioSemBolaMs = 0UL;

    bool linhaPodeIniciarTrava180 = false;
    bool linhaEh180DuranteTrava = false;
    bool linhaEhZeroDuranteTrava = false;
    bool linhaIncompativelComTrava = false;

    if (linhaDetectada && anguloLinhaPe >= 0.0f) {
        const float anguloLinhaNormalizado =
            normalizarAngulo360(anguloLinhaPe);

        linhaEh180DuranteTrava =
            (anguloLinhaNormalizado >= ANGULO_FUGA_TRAS_MIN &&
             anguloLinhaNormalizado <= ANGULO_FUGA_TRAS_MAX);

        linhaEhZeroDuranteTrava =
            (anguloLinhaNormalizado <= FAIXA_ZERO_TRAVA_GRAUS ||
             anguloLinhaNormalizado >=
                 (360.0f - FAIXA_ZERO_TRAVA_GRAUS));

        // NOVO:
        // 0° sozinho NUNCA cria a trava.
        // Só uma fuga aproximadamente 180° e vindo de movimento frontal.
        linhaPodeIniciarTrava180 =
            ultimoMovimentoEraFrente &&
            linhaEh180DuranteTrava;

        // Durante uma trava já existente, só aceitamos:
        //   150°..210° = fuga traseira real
        //   300°..360°/0°..60° = leitura 0° posterior da mesma passagem
        linhaIncompativelComTrava =
            !linhaEh180DuranteTrava &&
            !linhaEhZeroDuranteTrava;
    }

    // -------------------------------------------------------------------------
    // 1) SE A TRAVA JÁ EXISTE, DECIDE SE MANTÉM OU LIBERA
    // -------------------------------------------------------------------------
    if (fugaTraseiraTravada) {

        // Se apareceu uma direção que não combina nem com 180° nem com 0°,
        // cancela a trava e deixa sairDaLinha() decidir ainda neste ciclo.
        if (linhaDetectada && linhaIncompativelComTrava) {
            fugaTraseiraTravada = false;
            inicioFugaTraseiraMs = 0UL;
            inicioSemLinhaTravaMs = 0UL;
            ultimoMovimentoEraFrente = false;
        }
        else {
            // Failsafe: evita recuo infinito caso algum sensor fique travado.
            if ((agoraLinhaMs - inicioFugaTraseiraMs) >=
                TEMPO_MAX_FUGA_TRASEIRA_MS) {

                fugaTraseiraTravada = false;
                inicioFugaTraseiraMs = 0UL;
                inicioSemLinhaTravaMs = 0UL;

                fugindoLinhaAgora = false;
                anguloFugaLinhaCmd = 0.0f;
                ultimoMovimentoEraFrente = false;

                girarNoEixo(0);
                return;
            }

            if (linhaDetectada) {
                // Tanto 180° quanto 0° continuam pertencendo à mesma passagem.
                if (linhaEh180DuranteTrava ||
                    linhaEhZeroDuranteTrava) {

                    inicioSemLinhaTravaMs = 0UL;
                }
            }
            else {
                // Mantém 180° durante um intervalo curto sem linha.
                if (inicioSemLinhaTravaMs == 0UL) {
                    inicioSemLinhaTravaMs = agoraLinhaMs;
                }

                // Só libera quando ficar continuamente sem linha pelo tempo definido.
                if ((agoraLinhaMs - inicioSemLinhaTravaMs) >=
                    TEMPO_SEM_LINHA_LIBERAR_TRAVA_MS) {

                    fugaTraseiraTravada = false;
                    inicioFugaTraseiraMs = 0UL;
                    inicioSemLinhaTravaMs = 0UL;

                    fugindoLinhaAgora = false;
                    anguloFugaLinhaCmd = 0.0f;
                    ultimoMovimentoEraFrente = false;
                }
            }

            // Enquanto a trava existir, o comando continua sendo 180°.
            if (fugaTraseiraTravada) {
                fugindoLinhaAgora = true;
                anguloFugaLinhaCmd = 180.0f;

                // O movimento atual é para trás, então não carregamos
                // um estado antigo de "movimento frontal".
                ultimoMovimentoEraFrente = false;

                seguirDirecaoPorAngulo(
                    180.0f,
                    VELOCIDADE_FUGA_LINHA
                );

                return;
            }
        }
    }

    // -------------------------------------------------------------------------
    // 2) CRIA A TRAVA SOMENTE EM UMA FUGA REAL PARA 180°
    // -------------------------------------------------------------------------
    if (linhaPodeIniciarTrava180) {
        fugaTraseiraTravada = true;
        inicioFugaTraseiraMs = agoraLinhaMs;
        inicioSemLinhaTravaMs = 0UL;

        fugindoLinhaAgora = true;
        anguloFugaLinhaCmd = 180.0f;

        ultimoMovimentoEraFrente = false;

        seguirDirecaoPorAngulo(
            180.0f,
            VELOCIDADE_FUGA_LINHA
        );

        return;
    }

    // -------------------------------------------------------------------------
    // 3) TODAS AS OUTRAS LINHAS VÃO PARA A LÓGICA NORMAL
    // -------------------------------------------------------------------------
    //
    // Isto inclui principalmente:
    //      linha traseira isolada -> anguloLinhaPe = 0°
    //
    // Como 0° não cria mais a trava, sairDaLinha() volta a responder
    // normalmente a essa leitura.
    // -------------------------------------------------------------------------
    if (sairDaLinha(
            linhaDetectada,
            anguloLinhaPe,
            VELOCIDADE_FUGA_LINHA,
            &anguloFuga)) {

        fugindoLinhaAgora = true;
        anguloFugaLinhaCmd = normalizarAngulo360(anguloFuga);

        // O comando atual é uma fuga da linha, e não um avanço normal.
        ultimoMovimentoEraFrente = false;
        return;
    }

    fugindoLinhaAgora = false;
    anguloFugaLinhaCmd = 0.0f;

    // -------------------------------------------------------------------------
    // ESTRATÉGIA NORMAL DO ATACANTE
    // -------------------------------------------------------------------------
    if (obterAnguloIrDisponivel(anguloIrAtual)) {

        inicioSemBolaMs = 0UL;

        if (irNaFaixaFrontal(anguloIrAtual)) {

            // Movimento frontal real: permite que uma futura fuga 180°
            // crie a trava especial.
            ultimoMovimentoEraFrente = true;

            if (!indoDeFrenteParaGolAgora) {
                indoDeFrenteParaGolAgora = true;
                inicioMovimentoFrenteGolMs = agoraLinhaMs;
            }

            tempoContinuoFrenteGolMs =
                agoraLinhaMs - inicioMovimentoFrenteGolMs;

            // Velocidade frontal fixa.
            veloFrente = 220;

            moverFrenteComGiroParaGol(veloFrente);

        } else {

            indoDeFrenteParaGolAgora = false;
            tempoContinuoFrenteGolMs = 0UL;

            resetControleGolCamera();

            float anguloMovimento =
                mapearAnguloBolaParaMovimento(anguloIrAtual);

            const float anguloMovimentoNormalizado =
                normalizarAngulo360(anguloMovimento);

            // Se o comando ainda aponta majoritariamente para a frente,
            // também consideramos que existe componente frontal.
            ultimoMovimentoEraFrente =
                (anguloMovimentoNormalizado <= 60.0f ||
                 anguloMovimentoNormalizado >= 300.0f);

            seguirDirecaoComGiro(
                anguloMovimento,
                velo,
                cmdGiro
            );
        }

    }

    // Se IR não estiver disponível (SEM BOLA!!)
    else {
        indoDeFrenteParaGolAgora = false;
        tempoContinuoFrenteGolMs = 0UL;

        // Sem bola, não deixa um estado antigo ativar a trava especial.
        ultimoMovimentoEraFrente = false;

        if (inicioSemBolaMs == 0UL) {
            inicioSemBolaMs = agoraLinhaMs;
        }

        const bool semBolaHaTempoSuficiente =
            (agoraLinhaMs - inicioSemBolaMs) >=
            TEMPO_SEM_BOLA_PARA_REPOSICIONAR_MS;

        float alvoX = 0.0f;
        float alvoY = 0.0f;

        if (semBolaHaTempoSuficiente &&
            obterAlvoReposicionamentoPorZona(alvoX, alvoY))
        {
            atualizarLeiturasPosicionamento(
                ultraEcm,
                ultraDcm,
                ultraFcm,
                ultraTcm,
                ultrasValidos
            );

            if (moverParaComGiro(alvoX, alvoY)) {
                return;
            }
        }
        else {
            girarNoEixo(cmdGiro);
            return;
        }
    }

    return;
}

// =============================================================================
// FUNÇÕES COMPLEMENTARES DO ATACANTE
// =============================================================================

// Parâmetros de ajuste — *** ALTERE APENAS AQUI para tunar o atacante ***
// =============================================================================

// --- Velocidades do atacante ---
const int VELOCIDADE_IR_FRONTAL_PWM         = 200;
const int VELOCIDADE_IR_FAIXA_REDUZIDA_PWM = 140;

// --- Freio ultrassônico do atacante (laterais) ---
const float ATACANTE_ULTRA_FREIO_INICIO_CM       = 70.0f;
const float ATACANTE_ULTRA_FREIO_CRITICO_CM      = 50.0f;
const int   ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN  = 80;
const int   ATACANTE_ULTRA_FREIO_PWM_POR_CM      = 3;

// --- Confirmação de linha + parede (evita falso positivo único) ---
const uint8_t ATACANTE_LINHA_PAREDE_CONFIRMACAO = 3;

// --- Tempo mínimo sem bola na câmera para iniciar busca ---
const unsigned long ATACANTE_ESPERA_SEM_BOLA_CAMERA_MS = 2000UL;


// =============================================================================
// PID DE SUAVIZAÇÃO ANGULAR ANTIGO DO ATACANTE
// =============================================================================
// Mantido no código original.
// =============================================================================

const float PID_MOVIMENTO_KP           = 2.0f;
const float PID_MOVIMENTO_KI           = 0.01f;
const float PID_MOVIMENTO_KD           = 0.8f;
const float PID_MOVIMENTO_INTEGRAL_MAX = 90.0f;
const float PID_MOVIMENTO_SAIDA_MAX    = 15.0f;
const float ALPHA_MOVIMENTO            = 0.15f;
const float ALPHA_MOVIMENTO_ALVO       = 0.11f;
const float PASSO_MAX_MOVIMENTO_ALVO_GRAUS = 18.0f;

float        pidMovimentoIntegral              = 0.0f;
float        pidMovimento                      = 0.0f;
float        erroMovimento                     = 0.0f;
float        erroAnteriorMovimento             = 0.0f;
float        anguloMovimentoAtual              = -1.0f;
float        anguloMovimentoSuavizado          = -1.0f;
float        anguloMovimentoDesejadoFiltrado   = -1.0f;
unsigned long ultimoTempoPidMovimento          = 0;
unsigned long inicioCameraSemIrMs              = 0;


// Zera o controlador angular antigo do atacante
void resetControleMovimentoAtacante() {
  pidMovimentoIntegral            = 0.0f;
  pidMovimento                    = 0.0f;
  erroMovimento                   = 0.0f;
  erroAnteriorMovimento           = 0.0f;
  anguloMovimentoAtual            = -1.0f;
  anguloMovimentoSuavizado        = -1.0f;
  anguloMovimentoDesejadoFiltrado = -1.0f;
  ultimoTempoPidMovimento         = 0;
}


// PID de transição angular antigo
float calcularPidMovimento(float erro) {
  unsigned long agora = millis();
  float dt = 0.02f;

  if (ultimoTempoPidMovimento != 0) {
    dt = (agora - ultimoTempoPidMovimento) / 1000.0f;

    if (dt < 0.005f)
        dt = 0.005f;

    if (dt > 0.2f)
        dt = 0.2f;
  }

  ultimoTempoPidMovimento = agora;

  pidMovimentoIntegral += erro * dt;

  if (pidMovimentoIntegral > PID_MOVIMENTO_INTEGRAL_MAX)
      pidMovimentoIntegral = PID_MOVIMENTO_INTEGRAL_MAX;

  if (pidMovimentoIntegral < -PID_MOVIMENTO_INTEGRAL_MAX)
      pidMovimentoIntegral = -PID_MOVIMENTO_INTEGRAL_MAX;

  float derivada =
      (erro - erroAnteriorMovimento) / dt;

  erroAnteriorMovimento = erro;

  pidMovimento =
      PID_MOVIMENTO_KP * erro
      + PID_MOVIMENTO_KI * pidMovimentoIntegral
      + PID_MOVIMENTO_KD * derivada;

  if (pidMovimento > PID_MOVIMENTO_SAIDA_MAX)
      pidMovimento = PID_MOVIMENTO_SAIDA_MAX;

  if (pidMovimento < -PID_MOVIMENTO_SAIDA_MAX)
      pidMovimento = -PID_MOVIMENTO_SAIDA_MAX;

  if (fabsf(erro) < 2.0f)
      pidMovimento = 0.0f;

  return -pidMovimento;
}


// Aplica suavização exponencial circular ao ângulo de movimento do atacante
float suavizarAnguloMovimentoAtacante(float anguloMovimentoDesejado) {

  float alvoBruto =
      normalizarAngulo360(
          anguloMovimentoDesejado + 180.0f
      );

  if ((anguloMovimentoAtual < 0.0f) ||
      (anguloMovimentoSuavizado < 0.0f)) {

    anguloMovimentoAtual            = alvoBruto;
    anguloMovimentoSuavizado        = alvoBruto;
    anguloMovimentoDesejadoFiltrado = alvoBruto;

    erroMovimento =
        erroAnteriorMovimento =
        pidMovimentoIntegral =
        pidMovimento = 0.0f;

    ultimoTempoPidMovimento = 0;

    return anguloMovimentoSuavizado;
  }

  float deltaAlvo =
      normalizarErro180(
          alvoBruto
          - anguloMovimentoDesejadoFiltrado
      );

  deltaAlvo =
      constrain(
          deltaAlvo,
          -PASSO_MAX_MOVIMENTO_ALVO_GRAUS,
          PASSO_MAX_MOVIMENTO_ALVO_GRAUS
      );

  anguloMovimentoDesejadoFiltrado =
      normalizarAngulo360(
          anguloMovimentoDesejadoFiltrado
          + deltaAlvo
      );

  anguloMovimentoDesejadoFiltrado =
      normalizarAngulo360(
          anguloMovimentoDesejadoFiltrado +
          ALPHA_MOVIMENTO_ALVO *
          normalizarErro180(
              alvoBruto
              - anguloMovimentoDesejadoFiltrado
          )
      );

  erroMovimento =
      normalizarErro180(
          anguloMovimentoDesejadoFiltrado
          - anguloMovimentoAtual
      );

  pidMovimento =
      calcularPidMovimento(erroMovimento);

  anguloMovimentoAtual =
      normalizarAngulo360(
          anguloMovimentoAtual
          + pidMovimento
      );

  anguloMovimentoSuavizado =
      anguloMovimentoSuavizado +
      ALPHA_MOVIMENTO *
      normalizarErro180(
          anguloMovimentoAtual
          - anguloMovimentoSuavizado
      );

  anguloMovimentoSuavizado =
      normalizarAngulo360(
          anguloMovimentoSuavizado
      );

  return anguloMovimentoSuavizado;
}


/*
//////////// NÃO VAMOS USAR///////////////////

// Faz rampa angular curta (100–300 ms) entre faixas do IR
float obterAnguloIrSuavizado(float anguloAlvoGraus) {
  float alvo  = normalizarAngulo360(anguloAlvoGraus);
  unsigned long agora = millis();

  if (!anguloIrSuaveInicializado) {
    anguloIrSuaveAtual = anguloIrSuaveInicio = anguloIrSuaveAlvo = alvo;
    inicioTransicaoIrMs = agora;
    duracaoTransicaoIrMs = TRANSICAO_ANGULO_IR_MIN_MS;
    anguloIrSuaveInicializado = true;
    return quantizarAnguloPasso(
        anguloIrSuaveAtual,
        PASSO_ANGULO_IR_GRAUS
    );
  }

  float erroNovoAlvo =
      fabsf(
          normalizarErro180(
              alvo - anguloIrSuaveAlvo
          )
      );

  if (erroNovoAlvo >= 1.0f) {
    anguloIrSuaveInicio = anguloIrSuaveAtual;
    anguloIrSuaveAlvo   = alvo;
    inicioTransicaoIrMs = agora;

    float delta =
        fabsf(
            normalizarErro180(
                anguloIrSuaveAlvo
                - anguloIrSuaveInicio
            )
        );

    unsigned long duracaoCalculada =
        (unsigned long)(
            delta *
            TRANSICAO_ANGULO_IR_MS_POR_GRAU
        );

    if (duracaoCalculada < TRANSICAO_ANGULO_IR_MIN_MS)
        duracaoCalculada = TRANSICAO_ANGULO_IR_MIN_MS;

    if (duracaoCalculada > TRANSICAO_ANGULO_IR_MAX_MS)
        duracaoCalculada = TRANSICAO_ANGULO_IR_MAX_MS;

    duracaoTransicaoIrMs = duracaoCalculada;
  }

  unsigned long decorridoMs =
      agora - inicioTransicaoIrMs;

  if (decorridoMs >= duracaoTransicaoIrMs) {
    anguloIrSuaveAtual =
        anguloIrSuaveAlvo;
  } else {
    float progresso =
        (float)decorridoMs /
        (float)duracaoTransicaoIrMs;

    float delta =
        normalizarErro180(
            anguloIrSuaveAlvo
            - anguloIrSuaveInicio
        );

    anguloIrSuaveAtual =
        normalizarAngulo360(
            anguloIrSuaveInicio
            + delta * progresso
        );
  }

  return quantizarAnguloPasso(
      anguloIrSuaveAtual,
      PASSO_ANGULO_IR_GRAUS
  );
}
*/


// =============================================================================
// IR
// =============================================================================

bool irNaFaixaFrontal(float anguloBolaGraus) {
  float ang = normalizarAngulo360(anguloBolaGraus);

  return (ang > 333.0f || ang < 27.0f);
}


// Retém por poucos milissegundos o último ângulo IR válido
bool obterAnguloIrDisponivel(float &anguloBolaGraus) {

  if (irDetectado && anguloIr >= 0.0f) {

    anguloBolaGraus =
        normalizarAngulo360(anguloIr);

    return true;
  }

  if (ultimoAnguloIrValido >= 0.0f &&
      (millis() - ultimoRxIrValidoMs) <=
      RETENCAO_IR_VALIDO_MS) {

    anguloBolaGraus =
        normalizarAngulo360(
            ultimoAnguloIrValido
        );

    return true;
  }

  return false;
}


// =============================================================================
// ULTRASSÔNICOS
// =============================================================================

bool ultraLateralCriticoAtacante() {

  bool ultraDireitoCritico =
      (ultraDcm >= 0.0f) &&
      (ultraDcm <=
       ATACANTE_ULTRA_FREIO_CRITICO_CM);

  bool ultraEsquerdoCritico =
      (ultraEcm >= 0.0f) &&
      (ultraEcm <=
       ATACANTE_ULTRA_FREIO_CRITICO_CM);

  return ultraDireitoCritico ||
         ultraEsquerdoCritico;
}


// Limita velocidade por freio ultrassônico frontal
int aplicarFreioUltrassonicoAtacanteFrente(
    int velocidadeDesejada
) {

  int velocidadeBase =
      constrain(
          velocidadeDesejada,
          0,
          255
      );

  bool ultrasRecentes =
      (ultimoRxUltraMs > 0) &&
      ((millis() - ultimoRxUltraMs) <=
       TIMEOUT_ULTRA_MS);

  if (!ultrasRecentes)
      return velocidadeBase;

  float menorUltraCm = -1.0f;

  float leituras[] = {
      ultraFcm
  };

  for (float leitura : leituras) {

    if (leitura < 0.0f)
        continue;

    if (ultraTcm > 150) {

      if ((menorUltraCm < 0.0f) ||
          (leitura < menorUltraCm)) {

        menorUltraCm = leitura;
      }
    }

    if ((menorUltraCm < 0.0f) ||
        (menorUltraCm >
         ATACANTE_ULTRA_FREIO_INICIO_CM)) {

      return velocidadeBase;
    }

    int velocidadeLimite =
        ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;

    if (menorUltraCm >
        ATACANTE_ULTRA_FREIO_CRITICO_CM) {

      velocidadeLimite +=
          (int)(
              (menorUltraCm -
               ATACANTE_ULTRA_FREIO_CRITICO_CM)
              *
              ATACANTE_ULTRA_FREIO_PWM_POR_CM
          );
    }

    if (velocidadeLimite >
        velocidade_maxima) {

      velocidadeLimite =
          velocidade_maxima;
    }

    return min(
        velocidadeBase,
        velocidadeLimite
    );
  }

  return velocidadeBase;
}


// Limita velocidade por freio ultrassônico lateral
int aplicarFreioUltrassonicoAtacante(
    int velocidadeDesejada
) {

  int velocidadeBase =
      constrain(
          velocidadeDesejada,
          0,
          255
      );

  bool ultrasRecentes =
      (ultimoRxUltraMs > 0) &&
      ((millis() - ultimoRxUltraMs) <=
       TIMEOUT_ULTRA_MS);

  if (!ultrasRecentes)
      return velocidadeBase;

  float menorUltraCm = -1.0f;

  float leituras[] = {
      ultraDcm,
      ultraEcm
  };

  for (float leitura : leituras) {

    if (leitura < 0.0f)
        continue;

    if ((menorUltraCm < 0.0f) ||
        (leitura < menorUltraCm)) {

      menorUltraCm = leitura;
    }
  }

  if ((menorUltraCm < 0.0f) ||
      (menorUltraCm >
       ATACANTE_ULTRA_FREIO_INICIO_CM)) {

    return velocidadeBase;
  }

  int velocidadeLimite =
      ATACANTE_ULTRA_FREIO_VELOCIDADE_MIN;

  if (menorUltraCm >
      ATACANTE_ULTRA_FREIO_CRITICO_CM) {

    velocidadeLimite +=
        (int)(
            (menorUltraCm -
             ATACANTE_ULTRA_FREIO_CRITICO_CM)
            *
            ATACANTE_ULTRA_FREIO_PWM_POR_CM
        );
  }

  if (velocidadeLimite >
      velocidade_maxima) {

    velocidadeLimite =
        velocidade_maxima;
  }

  return min(
      velocidadeBase,
      velocidadeLimite
  );
}


// =============================================================================
// MAPEAMENTO DA BOLA
// =============================================================================

float mapearAnguloBolaParaMovimento(
    float anguloBolaGraus
)
{
  float ang =
      normalizarAngulo360(
          anguloBolaGraus
      );

  if (ang >= 32.0f && ang <= 60.0f)
      return 100.0f;

  if (ang > 60.0f && ang < 90.0f)
      return 90.0f;

  if (ang >= 90.0f && ang < 135.0f)
      return 180.0f;

  if (ang >= 135.0f && ang < 180.0f)
      return 225.0f;

  if (ang >= 180.0f && ang < 225.0f)
      return 135.0f;

  if (ang >= 225.0f && ang < 270.0f)
      return 180.0f;

  if (ang >= 270.0f && ang < 300.0f)
      return 270.0f;

  if (ang >= 300.0f && ang <= 328.0f)
      return 260.0f;

  return ang;
}


// Reduz velocidade em faixas próximas do frontal
int calcularVelocidadeIrPorAngulo(
    float anguloBolaGraus
) {

  float ang =
      normalizarAngulo360(
          anguloBolaGraus
      );

  if ((ang >= 33.0f && ang <= 60.0f) ||
      (ang >= 300.0f && ang <= 328.0f)) {

    return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;
  }

  if (ang >= 140.0f && ang < 220.0f)
      return VELOCIDADE_IR_FAIXA_REDUZIDA_PWM;

  return velocidade_maxima;
}


// =============================================================================
// BUSCA SEM BOLA
// =============================================================================

float calcularAnguloBuscaSemBolaCameraAtacante() {

  bool ultrasRecentes =
      ultrasValidos &&
      (ultimoRxUltraMs > 0) &&
      ((millis() - ultimoRxUltraMs) <=
       TIMEOUT_ULTRA_MS);

  if (ultrasRecentes) {

    bool esquerdaPerto =
        (ultraEcm >= 0.0f) &&
        (ultraEcm < 60.0f);

    bool direitaPerto =
        (ultraDcm >= 0.0f) &&
        (ultraDcm < 60.0f);

    bool esquerdaLivre =
        ultraEcm > 50.0f;

    bool direitaLivre =
        ultraDcm > 50.0f;

    if (esquerdaPerto && direitaLivre)
        return 90.0f;

    if (direitaPerto && esquerdaLivre)
        return 270.0f;
  }

  return 0.0f;
}


// =============================================================================
// FILTRO MEGA ANGULO
// =============================================================================

float suavizadorMegaAnguloMovimento(
    float anguloNovo
)
{
    static float anguloFiltrado = 0.0f;

    const float ALFA = 0.18f;

    float diff =
        anguloNovo -
        anguloFiltrado;

    if (diff > 180.0f)
        diff -= 360.0f;

    if (diff < -180.0f)
        diff += 360.0f;

    anguloFiltrado +=
        ALFA * diff;

    if (anguloFiltrado < 0)
        anguloFiltrado += 360.0f;

    if (anguloFiltrado >= 360.0f)
        anguloFiltrado -= 360.0f;

    return anguloFiltrado;
}


// =============================================================================
// PID DA BÚSSOLA
// =============================================================================

float PIDZIMBUSSOLANOVINHA(
    float erro
)
{
  static float erroAnterior = 0.0f;
  static float integral = 0.0f;
  static unsigned long ultimoMs = 0;

  const float Kp = 1.2f;
  const float Ki = 0.01f;
  const float Kd = 0.8f;

  unsigned long agora = millis();

  float dt = 0.02f;

  if (ultimoMs != 0) {

    dt =
        (agora - ultimoMs)
        / 1000.0f;

    if (dt < 0.005f)
        dt = 0.005f;

    if (dt > 0.2f)
        dt = 0.2f;
  }

  ultimoMs = agora;

  integral +=
      erro * dt;

  integral =
      constrain(
          integral,
          -100.0f,
          100.0f
      );

  float derivada =
      (erro - erroAnterior)
      / dt;

  float saida =
      (Kp * erro)
    + (Ki * integral)
    + (Kd * derivada);

  erroAnterior = erro;

  return -saida;
}


// =============================================================================
// CONTROLE DE GIRO PARA O GOL DURANTE ATAQUE FRONTAL
// =============================================================================

const float PID_GOL_CAMERA_KP = 1.2f;
const float PID_GOL_CAMERA_KI = 0.0f;
const float PID_GOL_CAMERA_KD = 0.5f;

const float PID_GOL_CAMERA_INTEGRAL_MAX = 100.0f;

const int PID_GOL_CAMERA_SAIDA_MIN = 30;
const int PID_GOL_CAMERA_SAIDA_MAX = 180;

const float TOLERANCIA_GOL_CAMERA_GRAUS = 2.0f;

const unsigned long RETENCAO_GOL_CAMERA_ATAQUE_MS = 100;

const float SALTO_MAX_GOL_CAMERA_GRAUS = 20.0f;


// Estado do PID específico da câmera.
float pidGolCameraIntegral = 0.0f;
float pidGolCameraErroAnterior = 0.0f;
unsigned long pidGolCameraUltimoMs = 0;


// Estado do filtro do ângulo do gol.
float cameraGolAnguloFiltrado = 0.0f;
bool cameraGolFiltroInicializado = false;
unsigned long cameraGolUltimaLeituraValidaMs = 0;


// =============================================================================
// RESET PID DO GOL
// =============================================================================

void resetPidGolCamera()
{
  pidGolCameraIntegral = 0.0f;
  pidGolCameraErroAnterior = 0.0f;
  pidGolCameraUltimoMs = 0;
}


// =============================================================================
// RESET FILTRO E PID DO GOL
// =============================================================================

void resetControleGolCamera()
{
  cameraGolAnguloFiltrado = 0.0f;
  cameraGolFiltroInicializado = false;
  cameraGolUltimaLeituraValidaMs = 0;

  resetPidGolCamera();
}


// =============================================================================
// OBTÉM ÂNGULO DO GOL
// =============================================================================

bool obterAnguloGolCameraAtaque(
    float &anguloGol
)
{
  const unsigned long agora =
      millis();

  int16_t leituraGol = -999;

  if (cameraTemGolSelecionadoValido(
          leituraGol))
  {
    float leitura =
        normalizarErro180(
            (float)leituraGol
        );

    if (!cameraGolFiltroInicializado)
    {
      cameraGolAnguloFiltrado =
          leitura;

      cameraGolFiltroInicializado =
          true;
    }
    else
    {
      float delta =
          normalizarErro180(
              leitura -
              cameraGolAnguloFiltrado
          );

      delta =
          constrain(
              delta,
              -SALTO_MAX_GOL_CAMERA_GRAUS,
               SALTO_MAX_GOL_CAMERA_GRAUS
          );

      const float ALPHA_GOL_CAMERA =
          0.5f;

      cameraGolAnguloFiltrado =
          normalizarErro180(
              cameraGolAnguloFiltrado +
              (ALPHA_GOL_CAMERA * delta)
          );
    }

    cameraGolUltimaLeituraValidaMs =
        agora;

    anguloGol =
        cameraGolAnguloFiltrado;

    return true;
  }

  if (cameraGolFiltroInicializado &&
      cameraGolUltimaLeituraValidaMs > 0 &&
      (agora -
       cameraGolUltimaLeituraValidaMs) <=
      RETENCAO_GOL_CAMERA_ATAQUE_MS)
  {
    anguloGol =
        cameraGolAnguloFiltrado;

    return true;
  }

  return false;
}


// =============================================================================
// CALCULA CMD GIRO DO GOL PELA CÂMERA
// =============================================================================

int calcularCmdGiroGolCamera(
    float anguloGol
)
{
  const unsigned long agora =
      millis();

  float dt = 0.02f;

  if (pidGolCameraUltimoMs != 0)
  {
    dt =
        (agora - pidGolCameraUltimoMs)
        / 1000.0f;

    if (dt < 0.005f)
        dt = 0.005f;

    if (dt > 0.2f)
        dt = 0.2f;
  }

  pidGolCameraUltimoMs =
      agora;

  float erroGol =
      normalizarErro180(
          -anguloGol
      );

  if (fabsf(erroGol) <=
      TOLERANCIA_GOL_CAMERA_GRAUS)
  {
    pidGolCameraIntegral = 0.0f;
    pidGolCameraErroAnterior =
        erroGol;

    return 0;
  }

  pidGolCameraIntegral +=
      erroGol * dt;

  pidGolCameraIntegral =
      constrain(
          pidGolCameraIntegral,
          -PID_GOL_CAMERA_INTEGRAL_MAX,
           PID_GOL_CAMERA_INTEGRAL_MAX
      );

  float derivada =
      (erroGol -
       pidGolCameraErroAnterior)
      / dt;

  pidGolCameraErroAnterior =
      erroGol;

  float saida =
      PID_GOL_CAMERA_KP * erroGol
    + PID_GOL_CAMERA_KI *
      pidGolCameraIntegral
    + PID_GOL_CAMERA_KD *
      derivada;

  int magnitude =
      (int)fabsf(saida);

  magnitude =
      constrain(
          magnitude,
          PID_GOL_CAMERA_SAIDA_MIN,
          PID_GOL_CAMERA_SAIDA_MAX
      );

  magnitude =
      min(
          magnitude,
          VELOCIDADE_GIRO_ALINHAMENTO
      );

  int cmdGiro =
      (saida >= 0.0f)
      ? magnitude
      : -magnitude;

  cmdGiro *=
      SINAL_GIRO_PID;

  return constrain(
      cmdGiro,
      -255,
      255
  );
}


// =============================================================================
// FRENTE + GIRO PARA O GOL
// =============================================================================

void moverFrenteComGiroParaGol(
    int velocidade
)
{
  float anguloGol = 0.0f;

  if (obterAnguloGolCameraAtaque(
          anguloGol))
  {
    int cmdGiroGol =
        calcularCmdGiroGolCamera(
            anguloGol
        );

    moverFrenteComGiro(
        velocidade,
        cmdGiroGol
    );

    return;
  }

  // Sem gol válido na câmera:
  // fallback seguro para a bússola.
  resetPidGolCamera();

  float erroBussola =
      normalizarErro180(
          -calcularErroReferenciaBussola()
      );

  int cmdGiroBussola =
      constrain(
          (int)roundf(
              -PIDZIMBUSSOLANOVINHA(
                  erroBussola
              )
          ),
          -255,
          255
      );

  moverFrenteComGiro(
      velocidade,
      cmdGiroBussola
  );
}
