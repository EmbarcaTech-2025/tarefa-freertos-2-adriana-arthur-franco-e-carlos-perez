#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "joystick_task.h"
#include "car_indicators_task.h"
#include "car_control_task.h"
#include "car_status_data.h"
#include "engine_sound_task.h"
#include "oled_task.h"
#include "ssd1306.h"

QueueHandle_t xJoystickQueue;
QueueHandle_t xCarStatusQueue;

void vMonitorJoystickTask(void *pvParameters) {
    joystick_data_t received_joystick_data;
    car_status_t received_car_status;

    while (true) {
        if (xQueueReceive(xJoystickQueue, &received_joystick_data, 0) == pdPASS) {
            printf("Monitor: Y=%d, SW=%d, A=%d, B=%d\n",
                   received_joystick_data.y_axis,
                   received_joystick_data.sw_state,
                   received_joystick_data.button_A_state,
                   received_joystick_data.button_B_state);
        }
        if (xQueueReceive(xCarStatusQueue, &received_car_status, 0) == pdPASS) {
            printf("Monitor Car: Speed=%d Km/h, RPM=%d, Gear=%d, ABS=%d, Airbag=%d, Horn=%d\n",
                   received_car_status.current_speed_kmh,
                   received_car_status.current_rpm,
                   received_car_status.current_gear,
                   received_car_status.abs_active,
                   received_car_status.airbag_deployed,
                   received_car_status.horn_active,
                   received_car_status.red_led_active);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Imprime a cada 500ms
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("Main: Inicializando sistema...\n");

    // Teste inicial do OLED
    ssd1306_init();
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "Sistema Iniciado");
    ssd1306_show();
    sleep_ms(1000);
    ssd1306_clear();
    ssd1306_show();

    xJoystickQueue = xQueueCreate(5, sizeof(joystick_data_t));
    xCarStatusQueue = xQueueCreate(5, sizeof(car_status_t));

    configASSERT(xJoystickQueue);
    configASSERT(xCarStatusQueue);

    xTaskCreate(vJoystickTask, "JoystickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vCarControlTask, "CarControlTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vCarIndicatorsTask, "CarIndicatorsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vOledTask, "OledTask", configMINIMAL_STACK_SIZE + 200, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vMonitorJoystickTask, "MonitorJoystickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vEngineSoundTask, "EngineSoundTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, NULL);

    vTaskStartScheduler();

    while (true) {}
}