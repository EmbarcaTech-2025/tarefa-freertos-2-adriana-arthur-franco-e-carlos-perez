#ifndef CAR_STATUS_DATA_H
#define CAR_STATUS_DATA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t current_speed_kmh;      // Velocidade atual em Km/h
    int16_t current_rpm;            // RPM atual do motor
    int8_t  current_gear;           // 0 (Neutro), 1, 2, 3, 4, 5
    bool    abs_active;             // true se ABS está atuando
    bool    airbag_deployed;        // true se Airbag foi acionado
    bool    horn_active;            // true se a buzina está sendo acionada
    bool    red_led_active;         // true se o LED vermelho está aceso
} car_status_t;

#endif // CAR_STATUS_DATA_H