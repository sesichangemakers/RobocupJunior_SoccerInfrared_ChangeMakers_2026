#pragma once

#include <Arduino.h>

// =============================================================================
// BIBLIOTECA DE POSICIONAMENTO EM CAMPO (X, Y)
// =============================================================================
//
// Calcula a posicao (X, Y) do robo no campo a partir dos 4 ultrassonicos
// (Esquerda/Direita/Frente/Tras). E completamente standalone: nao depende do
// servidor web da Cabeca, da API HTTP (/api/ultras) nem de nenhuma outra
// placa — roda inteiramente a bordo do Musculo, bastando alimentar as
// leituras a cada ciclo.
//
// Uso:
//   PosicaoCampo::iniciar(config);
//   // a cada loop:
//   PosicaoCampo::atualizarLeituras(ultraE, ultraD, ultraF, ultraT, validas);
//   if (PosicaoCampo::atualizar()) {
//     float x = PosicaoCampo::obterX();
//     float y = PosicaoCampo::obterY();
//   }
//
// =============================================================================

namespace PosicaoCampo {

struct Config {
  // Dimensoes do campo e raio do robo usados para converter ultras -> posicao X/Y.
  float campoLarguraCm = 182.0f;
  float campoAlturaCm = 243.0f;
  float roboRaioCm = 10.5f;

  // Parametros da estimativa: tolerancia de consistencia e filtro temporal.
  float compToleranciaCm = 22.0f;
  float filtroTauRapido = 0.26f;
  float filtroTauLento = 0.48f;
  float saltoMaximoCm = 35.0f;
};

// Reseta o estado interno (posicao inicial no centro do campo configurado).
void iniciar(const Config& config = Config());

// Atualiza as leituras ultrassonicas da iteracao atual.
// Recomenda-se chamar antes de atualizar() no loop.
void atualizarLeituras(float ultraEsquerdaCm,
                       float ultraDireitaCm,
                       float ultraFrenteCm,
                       float ultraTrasCm,
                       bool leiturasValidas);

// Executa uma nova estimativa filtrada de posicao; retorna false sem dados validos.
bool atualizar();

// Leitura da ultima posicao estimada (em cm) e da confianca da estimativa (0-1).
float obterX();
float obterY();
float obterConfianca();

}  // namespace PosicaoCampo
