// led_matrix_task.c
#include "led_matrix_task.h"
#include "car_status_data.h" // Para a estrutura car_status_t

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Inclua o arquivo .pio gerado. Certifique-se que o CMake está configurado para gerar ele.
#include "ws2818b.pio.h" 

// --- Definições e Funções do Driver NeoPixel (Copiadas do neopixel_pio.c) ---
#define LED_COUNT 25
#define LED_PIN 7 // GPIO 7 para o NeoPixel

struct pixel_t {
  uint8_t G, R, B;
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;

npLED_t leds[LED_COUNT]; // Buffer de LEDs

PIO np_pio;
uint sm;

void npInit(uint pin) {
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true);
  }

  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0; leds[i].G = 0; leds[i].B = 0;
  }
}

void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  if (index < LED_COUNT) { // Proteção para evitar acesso fora do array
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
  }
}

void npClear() {
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

void npWrite() {
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100);
}
// --- Fim das Funções do Driver NeoPixel ---

// A fila para receber o status do carro
QueueHandle_t xCarStatusQueue;

// Função para mapear a aceleração (0-2047) para uma coluna da matriz (ex: 5 LEDs)
// Se for uma matriz 5x5, você pode usar 5 LEDs para aceleração, 5 para marchas, etc.
// Digamos que a primeira coluna (indices 0 a 4) seja para aceleração.
void update_acceleration_display(int16_t acceleration_value) {
    // Escala a aceleração para um número de LEDs acesos (0 a 5)
    // Supondo aceleração de 0 a 2047 (metade do range do joystick Y positivo)
    // Eixo Y positivo é para frente no joystick original
    int num_leds_to_light = (acceleration_value * 5) / 2047; 
    
    // Garante que não exceda 5 LEDs
    if (num_leds_to_light < 0) num_leds_to_light = 0;
    if (num_leds_to_light > 5) num_leds_to_light = 5;

    // Limpa a coluna da aceleração (assumindo LEDs 0-4 para a primeira coluna)
    for (int i = 0; i < 5; i++) {
        npSetLED(i, 0, 0, 0); // Desliga todos
    }

    // Acende os LEDs proporcionalmente à aceleração
    // Usaremos verde para aceleração
    for (int i = 0; i < num_leds_to_light; i++) {
        // Mapeamento para a primeira coluna da matriz (0 a 4)
        // Se sua matriz for diferente, ajuste os índices aqui.
        // Ex: para uma matriz 5x5, LEDs 0, 5, 10, 15, 20 podem ser a primeira coluna vertical
        // Ou 0, 1, 2, 3, 4 a primeira linha horizontal.
        // Para simplificar, vamos assumir que os LEDs 0-4 são a "barra" de aceleração.
        npSetLED(i, 0, 60, 0); // Cor verde (R, G, B)
    }
}

// Função para exibir o status de ABS (ex: LED 5, ou um ícone na matriz)
void update_abs_display(bool abs_active) {
    // Exemplo: usar o LED no índice 5 para ABS
    if (abs_active) {
        npSetLED(5, 255, 0, 0); // Vermelho vibrante para ABS
    } else {
        npSetLED(5, 0, 0, 0); // Desliga
    }
}

// Função para exibir o status do Airbag (ex: LED 6)
void update_airbag_display(bool airbag_deployed) {
    // Exemplo: usar o LED no índice 6 para Airbag
    if (airbag_deployed) {
        npSetLED(6, 255, 255, 0); // Amarelo para Airbag
    } else {
        npSetLED(6, 0, 0, 0); // Desliga
    }
}


void vLedMatrixTask(void *pvParameters) {
    car_status_t received_status;

    npInit(LED_PIN); // Inicializa a matriz de LEDs
    npClear();
    npWrite(); // Garante que todos os LEDs estão desligados no início

    while (true) {
        // Tenta receber o status mais recente do carro
        if (xQueueReceive(xCarStatusQueue, &received_status, pdMS_TO_TICKS(50)) == pdPASS) {
            // Atualiza a exibição de aceleração
            update_acceleration_display(received_status.current_acceleration);
            
            // Atualiza a exibição de ABS
            update_abs_display(received_status.abs_active);

            // Atualiza a exibição do Airbag
            update_airbag_display(received_status.airbag_deployed);

            // TODO: Adicionar lógica para exibir marchas, etc. aqui

            npWrite(); // Envia os dados atualizados para a matriz de LEDs
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Atualiza a matriz a cada 20ms
    }
}