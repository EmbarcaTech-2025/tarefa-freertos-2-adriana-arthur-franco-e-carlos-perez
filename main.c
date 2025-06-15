#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "joystick_task.h"
#include "car_indicators_task.h"
//#include "led_matrix_task.h" 
#include "car_control_task.h" 
#include "car_status_data.h"
#include "oled_task.h"

#include "hardware/i2c.h"
#include "ssd1306.h"

// Definição da fila do joystick (global para ser acessada por outras tarefas)
QueueHandle_t xJoystickQueue;
QueueHandle_t xCarStatusQueue;

// Exemplo de outra tarefa que lê os dados do joystick (para testes iniciais)
void vMonitorJoystickTask(void *pvParameters) {
    joystick_data_t received_joystick_data;
    car_status_t received_car_status;

    while (true) {
        if (xQueueReceive(xJoystickQueue, &received_joystick_data, 0) == pdPASS) {
            printf("Monitor Joystick: X:%d, Y:%d, SW:%d\n",
                   received_joystick_data.x_axis, received_joystick_data.y_axis, received_joystick_data.sw_state);
        }
        
        // Manter este bloco de recepcao de fila do car_status comentado por enquanto
        if (xQueueReceive(xCarStatusQueue, &received_car_status, 0) == pdPASS) {
            printf("Monitor Car: Accel:%d, Gear:%d, ABS:%d, Airbag:%d\n",
                   received_car_status.current_acceleration, received_car_status.current_gear,
                   received_car_status.abs_active, received_car_status.airbag_deployed);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // Para não sobrecarregar o serial
    }
}

// --- Main ---
int main() {
    stdio_init_all(); // ESSENCIAL para o printf funcionar
    // gpio_init_all(); // Esta linha está comentada. Certifique-se de que os GPIOs são inicializados individualmente nas tarefas.
    sleep_ms(10);

// Cria as filas
    xJoystickQueue = xQueueCreate(1, sizeof(joystick_data_t));
    xCarStatusQueue = xQueueCreate(1, sizeof(car_status_t));

    // Verificar apenas a fila que está ativa no momento
    if (xJoystickQueue == NULL || xCarStatusQueue == NULL) { 
        printf("Falha ao criar a fila do Joystick.\n");
        while(true);
    }

    // Cria as tarefas
    xTaskCreate(vJoystickTask,
                "JoystickTask",
                configMINIMAL_STACK_SIZE + 128,
                NULL,
                tskIDLE_PRIORITY + 3, // Alta prioridade para leitura do joystick
                NULL);

    
    xTaskCreate(vCarControlTask, // Manter comentado
                "CarControlTask",
                configMINIMAL_STACK_SIZE + 256,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    
    xTaskCreate(vCarIndicatorsTask,
                "CarIndicatorsTask",
                configMINIMAL_STACK_SIZE + 256,
                NULL,
                tskIDLE_PRIORITY + 1, // Prioridade um pouco menor para indicadores visuais
                NULL);
    /*
    xTaskCreate(vLedMatrixTask, // Manter comentado
                "LedMatrixTask",
                configMINIMAL_STACK_SIZE + 512,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
    */
    xTaskCreate(vCarIndicatorsTask,
                "CarIndicatorsTask",
                configMINIMAL_STACK_SIZE + 256,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(vOledTask, // <-- ADICIONE ESTA TAREFA!
                "OledTask",
                configMINIMAL_STACK_SIZE + 512, // OLED pode precisar de mais pilha para buffer
                NULL,
                tskIDLE_PRIORITY + 1, // Prioridade similar aos indicadores visuais
                NULL);


    xTaskCreate(vMonitorJoystickTask, // <-- DESCOMENTE ESTA TAREFA PARA VER O PRINT DO JOYSTICK
                "MonitorTask", // Nome mais genérico
                configMINIMAL_STACK_SIZE + 256,
                NULL,
                tskIDLE_PRIORITY, // Prioridade mais baixa, apenas para debug
                NULL);

    vTaskStartScheduler();

    printf("ERRO! Scheduler FreeRTOS retornou.\n");
    while (true) {
    }
    return 0;
}