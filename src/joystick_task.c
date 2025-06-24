#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "joystick_task.h"

// Pinos ADC para o joystick
#define JOYSTICK_Y_PIN  26 // ADC0 (eixo Y vertical)

// Pinos GPIO para os botões
#define JOYSTICK_SW_PIN 22 // Botão SW (buzina)
#define BUTTON_A_PIN    5  // Botão A (ABS)
#define BUTTON_B_PIN    6  // Botão B (Airbag)

// Fila declarada em main.c
extern QueueHandle_t xJoystickQueue;

void vJoystickTask(void *pvParameters) {
    joystick_data_t current_joystick_data;
    
    // Inicializa o ADC
    adc_init();
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Inicializa os pinos dos botões com pull-up
    gpio_init(JOYSTICK_SW_PIN);
    gpio_set_dir(JOYSTICK_SW_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_SW_PIN);
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);

    // Filtro simples para suavizar leituras do ADC
    static uint16_t prev_y = 2047;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40); // Lê a cada 40ms
    const TickType_t xPrintFrequency = pdMS_TO_TICKS(500); // Imprime a cada 500ms
    TickType_t xLastPrintTime = xLastWakeTime;

    while (true) {
        // Leitura do eixo Y
        adc_select_input(0); // ADC0 para JOYSTICK_Y_PIN
        uint16_t raw_y = adc_read();
        // Filtro: média ponderada para suavizar
        raw_y = (prev_y * 3 + raw_y) / 4;
        prev_y = raw_y;
        current_joystick_data.y_axis = (int16_t)(raw_y - 2047); // Ajustado para frente positivo

        // Leitura dos botões (pressionado = LOW = true)
        current_joystick_data.sw_state = !gpio_get(JOYSTICK_SW_PIN);
        current_joystick_data.button_A_state = !gpio_get(BUTTON_A_PIN);
        current_joystick_data.button_B_state = !gpio_get(BUTTON_B_PIN);

        // Envia dados para a fila
        xQueueOverwrite(xJoystickQueue, &current_joystick_data);

        // Imprime apenas a cada 500ms
        if (xTaskGetTickCount() - xLastPrintTime >= xPrintFrequency) {
            printf("Joystick: Y=%d, SW=%d, A=%d, B=%d\n",
                   current_joystick_data.y_axis,
                   current_joystick_data.sw_state,
                   current_joystick_data.button_A_state,
                   current_joystick_data.button_B_state);
            xLastPrintTime = xTaskGetTickCount();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}