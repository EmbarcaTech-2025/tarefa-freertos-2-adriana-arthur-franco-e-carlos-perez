// car_control_task.h
#ifndef CAR_CONTROL_TASK_H
#define CAR_CONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Protótipo da função da tarefa de controle do carro
void vCarControlTask(void *pvParameters);

#endif // CAR_CONTROL_TASK_H