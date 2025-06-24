
#include "injector_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "led_matrix.h"      // npClear(), npWrite(), entrada_matriz_verm()
#include "car_status_data.h" // car_status_t
#include "led_matrix.h"
#include <stdlib.h>

extern QueueHandle_t xCarStatusQueue;

void vInjectorTask(void *pvParameters) {
    car_status_t current_car_status;
    const uint8_t injector_positions[4] = {0, 1, 2, 3}; 

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (xQueuePeek(xCarStatusQueue, &current_car_status, 0) == pdPASS) {
            int rpm = current_car_status.current_rpm;
            if (rpm < 900) rpm = 900;

            //calculo para deixar os pulsos visiveis na matriz de led
            int pulse_interval_ms = 100 - (abs(900-rpm)/3);
            if (pulse_interval_ms < 10) pulse_interval_ms = 10;
            if (pulse_interval_ms > 100) pulse_interval_ms = 100;

            for (int i = 0; i < 4; i++) {
                entrada_matriz_verm(injector_positions[i]); 
                npWrite();
                vTaskDelay(pdMS_TO_TICKS(pulse_interval_ms));

                npClear();
                npWrite();
                vTaskDelay(pdMS_TO_TICKS(pulse_interval_ms)); // Pequena pausa entre os pulsos
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}