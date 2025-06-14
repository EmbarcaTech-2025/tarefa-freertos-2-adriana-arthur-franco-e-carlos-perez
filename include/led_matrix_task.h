#ifndef LED_MATRIX_TASK_H
#define LED_MATRIX_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "car_status_data.h" // Incluir a estrutura de status do carro

// Declaração da fila de status do carro
extern QueueHandle_t xCarStatusQueue;

// Protótipo da função da tarefa da matriz de LEDs
void vLedMatrixTask(void *pvParameters);

#endif // LED_MATRIX_TASK_H