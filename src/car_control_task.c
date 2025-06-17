#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "car_status_data.h"
#include "joystick_task.h"

extern QueueHandle_t xJoystickQueue;
extern QueueHandle_t xCarStatusQueue;

#define MAX_SPEED_KMH           150
#define JOYSTICK_MAX_ABS_VAL    2048
#define NEUTRAL_THRESHOLD_JOY   100
#define ACCELERATION_RATE       0.10f
#define BRAKE_RATE              0.3f
#define DRAG_RATE               0.015f
#define MIN_RPM                 900
#define MAX_RPM                 5500
#define RPM_PER_KMH             35
#define GEAR_1_MAX_SPEED        20
#define GEAR_2_MAX_SPEED        40
#define GEAR_3_MAX_SPEED        70
#define GEAR_4_MAX_SPEED        100
#define GEAR_5_MAX_SPEED        150

static float current_speed_float = 0.0f;
static bool airbag_was_deployed_once = false;

void vCarControlTask(void *pvParameters) {
    printf("CarControlTask: Iniciada\n");

    joystick_data_t received_joystick_data;
    car_status_t current_car_status;

    current_car_status.current_speed_kmh = 0;
    current_car_status.current_rpm = MIN_RPM;
    current_car_status.current_gear = 0;
    current_car_status.abs_active = false;
    current_car_status.airbag_deployed = airbag_was_deployed_once;
    current_car_status.horn_active = false;
    current_car_status.red_led_active = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    while (true) {
        if (xQueueReceive(xJoystickQueue, &received_joystick_data, 0) == pdPASS) {
            int16_t joystick_y = received_joystick_data.y_axis;

            // Lógica de ABS (botão A)
            if (received_joystick_data.button_A_state) {
                current_speed_float = 0.0f; // Para imediatamente
                current_car_status.abs_active = true;
                current_car_status.red_led_active = true;
            } else {
                current_car_status.abs_active = false;
                current_car_status.red_led_active = false;

                // Lógica de velocidade (apenas se ABS não estiver ativo)
                if (joystick_y > NEUTRAL_THRESHOLD_JOY) {
                    float accel_input = (float)(joystick_y - NEUTRAL_THRESHOLD_JOY) / (JOYSTICK_MAX_ABS_VAL - NEUTRAL_THRESHOLD_JOY);
                    current_speed_float += accel_input * ACCELERATION_RATE * (float)xFrequency / pdMS_TO_TICKS(1);
                } else if (joystick_y < -NEUTRAL_THRESHOLD_JOY) {
                    float brake_input = (float)(-joystick_y - NEUTRAL_THRESHOLD_JOY) / (JOYSTICK_MAX_ABS_VAL - NEUTRAL_THRESHOLD_JOY);
                    current_speed_float -= brake_input * BRAKE_RATE * (float)xFrequency / pdMS_TO_TICKS(1);
                    if (current_speed_float < 0.0f) current_speed_float = 0.0f;
                } else {
                    current_speed_float -= DRAG_RATE * (float)xFrequency / pdMS_TO_TICKS(1);
                    if (current_speed_float < 0.0f) current_speed_float = 0.0f;
                }

                if (current_speed_float > MAX_SPEED_KMH) current_speed_float = MAX_SPEED_KMH;
            }

            // Lógica de RPM
            int16_t calculated_rpm;
            if (current_speed_float == 0 && joystick_y <= NEUTRAL_THRESHOLD_JOY) {
                calculated_rpm = MIN_RPM;
            } else if (current_speed_float > 0) {
                calculated_rpm = (int16_t)(MIN_RPM + (current_speed_float * RPM_PER_KMH));
                if (calculated_rpm > MAX_RPM) calculated_rpm = MAX_RPM;
            } else {
                calculated_rpm = MIN_RPM + (int16_t)(((float)joystick_y / JOYSTICK_MAX_ABS_VAL) * (MAX_RPM - MIN_RPM));
                if (calculated_rpm > MAX_RPM) calculated_rpm = MAX_RPM;
                if (calculated_rpm < MIN_RPM) calculated_rpm = MIN_RPM;
            }

            // Lógica de marcha
            int8_t calculated_gear;
            if (current_speed_float == 0) {
                calculated_gear = 0;
            } else if (current_speed_float <= GEAR_1_MAX_SPEED) {
                calculated_gear = 1;
            } else if (current_speed_float <= GEAR_2_MAX_SPEED) {
                calculated_gear = 2;
            } else if (current_speed_float <= GEAR_3_MAX_SPEED) {
                calculated_gear = 3;
            } else if (current_speed_float <= GEAR_4_MAX_SPEED) {
                calculated_gear = 4;
            } else {
                calculated_gear = 5;
            }

            // Lógica de Airbag (botão B)
            if (received_joystick_data.button_B_state && !airbag_was_deployed_once) {
                airbag_was_deployed_once = true;
            }
            current_car_status.airbag_deployed = airbag_was_deployed_once;

            // Lógica de buzina (botão SW)
            current_car_status.horn_active = received_joystick_data.sw_state;

            // Atualiza status do carro
            current_car_status.current_speed_kmh = (int16_t)current_speed_float;
            current_car_status.current_rpm = calculated_rpm;
            current_car_status.current_gear = calculated_gear;

            // Envia status para a fila
            xQueueOverwrite(xCarStatusQueue, &current_car_status);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}