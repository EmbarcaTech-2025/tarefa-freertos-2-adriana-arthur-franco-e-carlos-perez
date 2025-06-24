#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "joystick_task.h"
#include "car_status_data.h"

// Pinos dos LEDs RGB
#define LED_RED_PIN     13
#define LED_GREEN_PIN   11
#define LED_BLUE_PIN    12
#define BUZZER_PIN      21

// Thresholds
#define ACCELERATION_THRESHOLD  200
#define BRAKE_THRESHOLD         -200

extern QueueHandle_t xJoystickQueue;
extern QueueHandle_t xCarStatusQueue;

void set_led(uint gpio_pin, bool state) {
    gpio_put(gpio_pin, state);
}

void buzzer_on() {
    gpio_put(BUZZER_PIN, 1);
}

void buzzer_off() {
    gpio_put(BUZZER_PIN, 0);
}

void vCarIndicatorsTask(void *pvParameters) {
    printf("CarIndicatorsTask: Iniciada\n");

    // Inicializa LEDs e buzzer
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, 0); // Garante estado inicial baixo
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, 0);
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);

    // Teste inicial: Piscar LEDs para confirmar funcionamento
    printf("CarIndicatorsTask: Testando LEDs...\n");
    set_led(LED_RED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_led(LED_RED_PIN, 0);
    set_led(LED_GREEN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_led(LED_GREEN_PIN, 0);
    set_led(LED_BLUE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_led(LED_BLUE_PIN, 0);
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(500));
    buzzer_off();
    printf("CarIndicatorsTask: Teste concluído\n");

    joystick_data_t current_joystick_data;
    car_status_t current_car_status;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    while (true) {
        if (xQueuePeek(xJoystickQueue, &current_joystick_data, 0) == pdPASS &&
            xQueuePeek(xCarStatusQueue, &current_car_status, 0) == pdPASS) {
            // Depuração
            printf("CarIndicatorsTask: Y=%d, SW=%d\n", 
                   current_joystick_data.y_axis, current_joystick_data.sw_state);

            // LED vermelho: ligado durante ABS ou freio normal
            if (current_car_status.red_led_active || 
                (!current_car_status.red_led_active && current_joystick_data.y_axis < BRAKE_THRESHOLD)) {
                set_led(LED_RED_PIN, 1);
                if (!current_car_status.red_led_active) {
                    printf("CarIndicatorsTask: LED Vermelho ON (freio)\n");
                }
            } else {
                set_led(LED_RED_PIN, 0);
            }

            // LED verde: ligado apenas durante aceleração e se ABS não estiver ativo
            if (!current_car_status.red_led_active && current_joystick_data.y_axis > ACCELERATION_THRESHOLD) {
                set_led(LED_GREEN_PIN, 1);
                printf("CarIndicatorsTask: LED Verde ON\n");
            } else {
                set_led(LED_GREEN_PIN, 0);
            }

            // LED azul e buzzer para buzina (botão SW)
            if (current_joystick_data.sw_state) {
                set_led(LED_BLUE_PIN, 1);
                buzzer_on();
                printf("CarIndicatorsTask: LED Azul ON, Buzzer ON\n");
            } else {
                set_led(LED_BLUE_PIN, 0);
                buzzer_off();
            }
        } else {
            printf("CarIndicatorsTask: Falha ao ler fila\n");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}