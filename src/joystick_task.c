#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h" // <-- DESCOMENTE ESTA LINHA!

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "joystick_task.h"

// Definição dos pinos (repetido aqui para clareza, mas idealmente seria em um arquivo de configuração de pinos)
#define JOYSTICK_VRX_PIN 27   // Pino para VRx (eixo X)
#define JOYSTICK_VRY_PIN 26   // Pino para VRy (eixo Y)
#define JOYSTICK_SW_PIN  22   // Pino para chave (SW)

// Definição dos canais ADC
#define ADC_VRX_CHANNEL 1     // ADC1 (GPIO27)
#define ADC_VRY_CHANNEL 0     // ADC0 (GPIO26)

// A fila será criada no main.c e acessada globalmente via extern
extern QueueHandle_t xJoystickQueue;

void vJoystickTask(void *pvParameters) {
    // Inicializa o ADC (uma única vez ao iniciar a tarefa)
    adc_init(); // <-- DESCOMENTE ESTA LINHA!
    adc_gpio_init(JOYSTICK_VRX_PIN); // <-- DESCOMENTE ESTA LINHA!
    adc_gpio_init(JOYSTICK_VRY_PIN); // <-- DESCOMENTE ESTA LINHA!
    
    // Configura o pino da chave (SW)
    gpio_init(JOYSTICK_SW_PIN);
    gpio_set_dir(JOYSTICK_SW_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_SW_PIN);

    joystick_data_t current_joystick_data;

    // Loop principal da tarefa
    while (true) {
        // Leitura do Eixo X
        adc_select_input(ADC_VRX_CHANNEL); // <-- DESCOMENTE ESTA LINHA!
        uint16_t vrx_value = adc_read();       // <-- DESCOMENTE ESTA LINHA!
        
        // Leitura do Eixo Y
        adc_select_input(ADC_VRY_CHANNEL); // <-- DESCOMENTE ESTA LINHA!
        uint16_t vry_value = adc_read();       // <-- DESCOMENTE ESTA LINHA!
        
        // Leitura do estado da chave
        bool sw_state = !gpio_get(JOYSTICK_SW_PIN); // Inverte pull-up

        // Converte os valores
        current_joystick_data.x_axis = vrx_value - 2048; // <-- DESCOMENTE ESTA LINHA!
        current_joystick_data.y_axis = vry_value - 2048; // <-- DESCOMENTE ESTA LINHA!
        current_joystick_data.sw_state = sw_state;

        // Envia os dados para a fila
        xQueueSend(xJoystickQueue, &current_joystick_data, 0); 
        
        // Opcional: imprimir no serial para depuração (pode ser removido depois)
        // Se o vMonitorJoystickTask estiver ativo em main.c, este aqui é redundante
        // Mas pode ser útil para ver o dado no ponto de origem.
        printf("Task Joystick: X:%d, Y:%d, SW:%d\n", 
               current_joystick_data.x_axis, current_joystick_data.y_axis, current_joystick_data.sw_state); // <-- DESCOMENTE ESTA LINHA (opcional)

        // Atraso para a tarefa não consumir 100% da CPU.
        vTaskDelay(pdMS_TO_TICKS(50)); // Lê a cada 50ms (20 vezes por segundo)
    }
}