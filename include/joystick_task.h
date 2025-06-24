#ifndef JOYSTICK_TASK_H
#define JOYSTICK_TASK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t y_axis;         // Eixo Y do joystick (-2047 a +2048)
    bool sw_state;          // Estado do botão SW (buzina)
    bool button_A_state;    // Estado do botão A (ABS)
    bool button_B_state;    // Estado do botão B (Airbag)
} joystick_data_t;

void vJoystickTask(void *pvParameters);

#endif // JOYSTICK_TASK_H