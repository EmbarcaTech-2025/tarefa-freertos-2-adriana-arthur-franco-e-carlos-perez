#include "led_matrix.h"
#include "ws2818b.pio.h"
#include "pico/stdlib.h"       // para sleep_us()

// Definição das variáveis globais
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;

// Inicializa a máquina PIO e limpa LEDs
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
        if (sm < 0) {
            // Sem state machines livres: trate erro conforme necessidade
            while(1);
        }
    }

    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    npClear();
    npWrite();
}

// Limpa o buffer de LEDs
void npClear(void) {
    for (uint i = 0; i < LED_COUNT; i++) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

// Envia o buffer para os LEDs
void npWrite(void) {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);  // RESET sinal WS2812/WS2818b
}

// Define um LED específico com cor RGB
static void npSetLED(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < 0 || index >= LED_COUNT) return;
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Acende um LED vermelho na posição dada
void entrada_matriz_verm(int posicao_led) {
    npSetLED(posicao_led, 0x40, 0x00, 0x00);
    npWrite();
}

// Acende um LED amarelo (vermelho + verde) na posição dada
void entrada_matriz_amar(int posicao_led) {
    npSetLED(posicao_led, 0x40, 0x40, 0x00);
    npWrite();
}

// Acende um LED verde na posição dada
void entrada_matriz_verd(int posicao_led) {
    npSetLED(posicao_led, 0x00, 0x40, 0x00);
    npWrite();
}

// Apaga o LED na posição dada
void entrada_matriz_desl(int posicao_led) {
    npSetLED(posicao_led, 0x00, 0x00, 0x00);
    npWrite();
}
