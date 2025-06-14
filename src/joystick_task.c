#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

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
// QueueHandle_t xJoystickQueue;

void vJoystickTask(void *pvParameters) {
    // Inicializa o ADC (uma única vez ao iniciar a tarefa)
    adc_init();
    adc_gpio_init(JOYSTICK_VRX_PIN);
    adc_gpio_init(JOYSTICK_VRY_PIN);
    
    // Configura o pino da chave (SW)
    gpio_init(JOYSTICK_SW_PIN);
    gpio_set_dir(JOYSTICK_SW_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_SW_PIN);

    joystick_data_t current_joystick_data;

    // Loop principal da tarefa
    while (true) {
        // Leitura do Eixo X
        adc_select_input(ADC_VRX_CHANNEL);
        uint16_t vrx_value = adc_read();
        
        // Leitura do Eixo Y
        adc_select_input(ADC_VRY_CHANNEL);
        uint16_t vry_value = adc_read();
        
        // Leitura do estado da chave
        bool sw_state = !gpio_get(JOYSTICK_SW_PIN); // Inverte pull-up

        // Converte os valores
        current_joystick_data.x_axis = vrx_value - 2048;
        current_joystick_data.y_axis = vry_value - 2048;
        current_joystick_data.sw_state = sw_state;

        // Envia os dados para a fila
        // xQueueSend envia para o final da fila. 0 ms de block time
        // significa que não vai esperar se a fila estiver cheia.
        // Se a fila estiver cheia e não puder enviar, a tarefa continua.
        xQueueSend(xJoystickQueue, &current_joystick_data, 0); 
        
        // Opcional: imprimir no serial para depuração (pode ser removido depois)
        // printf("Task: Eixo X: %d, Eixo Y: %d, Chave: %d\n", 
        //        current_joystick_data.x_axis, current_joystick_data.y_axis, current_joystick_data.sw_state);

        // Atraso para a tarefa não consumir 100% da CPU.
        // Ajuste esse valor conforme a necessidade de responsividade.
        vTaskDelay(pdMS_TO_TICKS(50)); // Lê a cada 50ms (20 vezes por segundo)
    }
}
