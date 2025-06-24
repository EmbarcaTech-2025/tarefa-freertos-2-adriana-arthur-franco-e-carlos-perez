// include/oled_task.h
#ifndef OLED_TASK_H
#define OLED_TASK_H

#include "FreeRTOS.h"
#include "task.h"

// Protótipo da tarefa do OLED
void vOledTask(void *pvParameters);

#endif // OLED_TASK_H