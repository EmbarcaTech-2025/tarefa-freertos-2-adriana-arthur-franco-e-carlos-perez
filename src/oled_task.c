// src/oled_task.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "oled/ssd1306.h" // Seu driver SSD1306.h

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "oled_task.h"
#include "car_status_data.h" // Para a estrutura car_status_t

// A fila do status do carro será acessada globalmente via extern
extern QueueHandle_t xCarStatusQueue;

// Note: As definições de pinos e a instância do display (ssd1306_t)
// são gerenciadas internamente pelo seu driver ssd1306.c.
// Não precisamos delas aqui em oled_task.c, apenas chamar as funções.

void vOledTask(void *pvParameters) {
    car_status_t received_car_status;

    // A inicialização completa do I2C e do display SSD1306 (incluindo poweron)
    // é feita internamente pela função ssd1306_init() do seu driver.
    ssd1306_init(); // <-- CHAMADA SIMPLIFICADA!

    // Garante que o display está limpo no início
    ssd1306_clear();
    ssd1306_show();

    char speed_str[20]; // Buffer para formatar a string de velocidade

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Atualiza o OLED a cada 100ms
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Tenta receber o status mais recente do carro
        // Espera no máximo 'xFrequency' ticks. Se não houver, usa os últimos dados.
        if (xQueueReceive(xCarStatusQueue, &received_car_status, xFrequency) == pdPASS) {
            // Limpa o buffer do OLED
            ssd1306_clear();

            // Formata a string de velocidade
            // Usamos current_acceleration para a velocidade, como definido antes
            snprintf(speed_str, sizeof(speed_str), "Speed: %d Km/h", received_car_status.current_acceleration);
            
            // Escreve a string na primeira linha do OLED (coluna 0, linha 0)
            // Note: Seu driver não usa o argumento de escala.
            ssd1306_draw_string(0, 0, speed_str); // <-- CHAMADA SIMPLIFICADA!

            // Atualiza o display com o buffer
            ssd1306_show(); // <-- CHAMADA SIMPLIFICADA!
        }
        
        // Atraso para manter a frequência da tarefa
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}