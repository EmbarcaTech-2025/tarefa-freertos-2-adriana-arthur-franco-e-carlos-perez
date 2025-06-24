#ifndef CAR_INDICATORS_TASK_H
#define CAR_INDICATORS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h" // Para acessar a fila do joystick

// Protótipo da função da tarefa de indicadores do carro
void vCarIndicatorsTask(void *pvParameters);

#endif // CAR_INDICATORS_TASK_H