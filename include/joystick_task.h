#ifndef JOYSTICK_TASK_H
#define JOYSTICK_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h" // Certifique-se que FreeRTOS.h está incluído
#include "queue.h"    // Certifique-se que queue.h está incluído

// Estrutura para os dados do joystick
typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    bool sw_state;
} joystick_data_t;

// **APENAS DECLARAÇÃO**: Diz ao compilador que a variável existe em algum lugar.
// NÃO aloca memória.
extern QueueHandle_t xJoystickQueue;

// Protótipo da função da tarefa do joystick
void vJoystickTask(void *pvParameters);

#endif // JOYSTICK_TASK_H