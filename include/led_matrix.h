#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>
#include "hardware/pio.h"

#define LED_COUNT 25
#define LED_PIN 7

// Definição de um pixel RGB (GRB na ordem do protocolo WS2812/WS2818b)
typedef struct {
    uint8_t G;
    uint8_t R;
    uint8_t B;
} npLED_t;

// Variáveis globais externas (definidas em led_matrix.c)
extern npLED_t leds[LED_COUNT];
extern PIO np_pio;
extern uint sm;  // State machine

// Inicializa a matriz de LEDs no pino especificado
void npInit(uint pin);

// Limpa o buffer de LEDs (apaga todos)
void npClear(void);

// Envia os dados do buffer para a matriz de LEDs
void npWrite(void);

// Funções auxiliares para ligar/desligar LEDs nas cores
void entrada_matriz_verm(int posicao_led);
void entrada_matriz_amar(int posicao_led);
void entrada_matriz_verd(int posicao_led);
void entrada_matriz_desl(int posicao_led);

#endif // LED_MATRIX_H
