// Em um novo arquivo de cabeçalho, por exemplo, car_status_data.h
#ifndef CAR_STATUS_DATA_H
#define CAR_STATUS_DATA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t current_acceleration; // -2048 a +2047 do joystick Y, ou uma escala própria (0-100)
    int8_t  current_gear;         // 0 (Neutro) a 5 (Marcha 5)
    bool    abs_active;           // true se ABS está atuando
    bool    airbag_deployed;      // true se Airbag foi acionado
} car_status_t;

#endif // CAR_STATUS_DATA_H