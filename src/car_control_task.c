// src/car_control_task.c
#include <stdio.h> // Para debug, pode ser removido depois
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "car_status_data.h" // Para a estrutura car_status_t
#include "joystick_task.h"    // Para a estrutura joystick_data_t e a fila xJoystickQueue

// As filas serão criadas em main.c e acessadas globalmente via extern
extern QueueHandle_t xJoystickQueue;
extern QueueHandle_t xCarStatusQueue;

// Constantes para cálculo de velocidade
#define MAX_SPEED_KMH           100    // Velocidade máxima em Km/h
#define JOYSTICK_MAX_ABS_VAL    2048   // Valor máximo absoluto do joystick (aprox. 2047 do centro)
#define NEUTRAL_THRESHOLD_JOY   100    // Valores do joystick +/- esta faixa são considerados neutros

// Fatores de simulação (ajuste estes valores para o comportamento desejado)
// Quanto maior o fator, mais rápido o carro acelera/freia/desacelera
#define ACCELERATION_RATE       0.05f  // Taxa de aceleração por tick (ajustar!)
#define BRAKE_RATE              0.08f  // Taxa de frenagem por tick (ajustar!)
#define DRAG_RATE               0.02f  // Taxa de desaceleração natural (arrasto) por tick (ajustar!)

// Variável estática para manter a velocidade atual
static float current_speed_float = 0.0f; // Usamos float para cálculos mais suaves

void vCarControlTask(void *pvParameters) {
    joystick_data_t received_joystick_data;
    car_status_t current_car_status;

    // Inicializa o status do carro (valores padrão)
    current_car_status.current_acceleration = 0; // Usaremos este campo para a velocidade (Km/h)
    current_car_status.current_gear = 0;         // Começa em Neutro
    current_car_status.abs_active = false;       // ABS inativo
    current_car_status.airbag_deployed = false;  // Airbag não acionado

    // Para controlar a frequência da tarefa
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Atualiza a cada 50ms
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Tenta receber os dados mais recentes do joystick da fila
        // Não bloqueia (0 tick), se não houver dados, usa os últimos conhecidos
        if (xQueueReceive(xJoystickQueue, &received_joystick_data, 0) == pdPASS) {
            int16_t joystick_y = received_joystick_data.y_axis;

            // Lógica de simulação de velocidade
            if (joystick_y > NEUTRAL_THRESHOLD_JOY) { // Acelerando
                // Mapeia o valor do joystick para um fator de aceleração
                float accel_input = (float)(joystick_y - NEUTRAL_THRESHOLD_JOY) / (JOYSTICK_MAX_ABS_VAL - NEUTRAL_THRESHOLD_JOY);
                current_speed_float += accel_input * ACCELERATION_RATE * (float)xFrequency / pdMS_TO_TICKS(1); // Ajusta pela frequência
            } else if (joystick_y < -NEUTRAL_THRESHOLD_JOY) { // Freando
                // Mapeia o valor do joystick para um fator de frenagem
                float brake_input = (float)(-joystick_y - NEUTRAL_THRESHOLD_JOY) / (JOYSTICK_MAX_ABS_VAL - NEUTRAL_THRESHOLD_JOY);
                current_speed_float -= brake_input * BRAKE_RATE * (float)xFrequency / pdMS_TO_TICKS(1); // Ajusta pela frequência
            } else { // Joystick na zona neutra, aplica arrasto (drag)
                current_speed_float -= DRAG_RATE * (float)xFrequency / pdMS_TO_TICKS(1); // Desacelera naturalmente
            }

            // Garante que a velocidade não seja negativa
            if (current_speed_float < 0.0f) {
                current_speed_float = 0.0f;
            }

            // Garante que a velocidade não exceda o máximo
            if (current_speed_float > MAX_SPEED_KMH) {
                current_speed_float = MAX_SPEED_KMH;
            }

            // Atualiza a estrutura de status do carro
            current_car_status.current_acceleration = (int16_t)current_speed_float; // Usamos este campo para velocidade em Km/h

            // Envia o status atualizado do carro para a fila
            // Não bloqueia (0 ticks), se a fila estiver cheia, sobrescreve o item antigo
            xQueueOverwrite(xCarStatusQueue, &current_car_status);
        }
        
        // Atraso para manter a frequência da tarefa
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}