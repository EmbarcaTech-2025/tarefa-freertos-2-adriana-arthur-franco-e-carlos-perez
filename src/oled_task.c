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

void vOledTask(void *pvParameters) {
    car_status_t received_car_status;

    ssd1306_init(); // Inicialização completa pelo seu driver
    ssd1306_clear();
    ssd1306_show();

    char line1_str[20]; // Velocidade
    char line2_str[20]; // RPM
    char line3_str[20]; // Marcha
    char line4_str[20]; // ABS
    char line5_str[20]; // Airbag

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Atualiza o OLED a cada 100ms
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Tenta receber o status mais recente do carro
        if (xQueueReceive(xCarStatusQueue, &received_car_status, xFrequency) == pdPASS) {
            ssd1306_clear(); // Limpa o buffer do OLED

            // --- Linha 1: Velocidade ---
            snprintf(line1_str, sizeof(line1_str), "Speed: %3d Km/h", received_car_status.current_speed_kmh);
            ssd1306_draw_string(0, 0, line1_str); // x=0, y=0 (1ª linha)

            // --- Linha 2: RPM ---
            snprintf(line2_str, sizeof(line2_str), "RPM: %5d", received_car_status.current_rpm);
            ssd1306_draw_string(0, 8, line2_str); // x=0, y=8 (2ª linha)

            // --- Linha 3: Marcha ---
            if (received_car_status.current_gear == 0) {
                snprintf(line3_str, sizeof(line3_str), "Gear: N");
            } else {
                snprintf(line3_str, sizeof(line3_str), "Gear: %d", received_car_status.current_gear);
            }
            ssd1306_draw_string(0, 16, line3_str); // x=0, y=16 (3ª linha)

            // --- Linha 4: ABS ---
            snprintf(line4_str, sizeof(line4_str), "ABS: %s", received_car_status.abs_active ? "Active" : "Inactive");
            ssd1306_draw_string(0, 24, line4_str); // x=0, y=24 (4ª linha)

            // --- Linha 5: Airbag ---
            snprintf(line5_str, sizeof(line5_str), "Airbag: %s", received_car_status.airbag_deployed ? "Deployed" : "OK");
            ssd1306_draw_string(0, 32, line5_str); // x=0, y=32 (5ª linha)

            // Atualiza o display com o buffer
            ssd1306_show();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}