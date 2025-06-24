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
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "led_matrix.h"
#include "injector_task.h"

// Biblioteca gerada pelo arquivo .pio durante compilação.
//#include "ws2818b.pio.h"

QueueHandle_t xJoystickQueue;
QueueHandle_t xCarStatusQueue;

void vMonitorJoystickTask(void *pvParameters) { // Usada apenas durante o desenvolvimento
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
            printf("🚗 Monitor Car:\n");
            printf("   Velocidade:   %3d km/h\n", received_car_status.current_speed_kmh);
            printf("   RPM:          %4d\n",     received_car_status.current_rpm);
            printf("   Marcha:       %d\n",       received_car_status.current_gear);
            printf("   ABS:          %s\n",       received_car_status.abs_active ? "ATIVO" : "inativo");
            printf("   Airbag:       %s\n",       received_car_status.airbag_deployed ? "DISPARADO" : "ok");
            printf("   Buzina:       %s\n",       received_car_status.horn_active ? "LIGADA" : "desligada");
            printf("   LED Vermelho: %s\n",       received_car_status.red_led_active ? "aceso" : "apagado");
            printf("-----------------------------------\n");
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

    // Inicializa matriz de LEDs NeoPixel.
    npInit(LED_PIN);
    npClear();

    xJoystickQueue = xQueueCreate(8, sizeof(joystick_data_t));
    xCarStatusQueue = xQueueCreate(8, sizeof(car_status_t));

    configASSERT(xJoystickQueue);
    configASSERT(xCarStatusQueue);

    xTaskCreate(vJoystickTask, "JoystickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL);
    xTaskCreate(vCarControlTask, "CarControlTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vCarIndicatorsTask, "CarIndicatorsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vOledTask, "OledTask", configMINIMAL_STACK_SIZE + 200, NULL, tskIDLE_PRIORITY + 2, NULL);
    //xTaskCreate(vMonitorJoystickTask, "MonitorJoystickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vEngineSoundTask, "EngineSoundTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, NULL);
    xTaskCreate(vInjectorTask, "InjectorTask", configMINIMAL_STACK_SIZE + 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (true) {}
}