#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "car_status_data.h"
#include "engine_sound_task.h"
#include "car_control_task.h"

#define BUZZER_PWM_PIN      10 // Pino para o buzzer do ronco do motor
#define PWM_WRAP_VAL        6250 


// Externs para as filas
extern QueueHandle_t xCarStatusQueue;

// Constantes de RPM (poderiam ser passadas via parâmetros ou definidas em um header compartilhado)
#define MIN_RPM_ENGINE_SOUND    900 // Replicando de car_control_task.c ou importando
#define MAX_RPM_ENGINE_SOUND    5500 // Replicando de car_control_task.c ou importando

void vEngineSoundTask(void *pvParameters) {
    printf("EngineSoundTask: Iniciada\n");

    // Configuração do PWM
    gpio_set_function(BUZZER_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PWM_PIN);
    uint channel = pwm_gpio_to_channel(BUZZER_PWM_PIN);
    pwm_set_wrap(slice_num, PWM_WRAP_VAL - 1);
    pwm_set_enabled(slice_num, true);

    car_status_t car_status;

    while (true) {
        if (xQueuePeek(xCarStatusQueue, &car_status, 0) == pdPASS) {
            int rpm = car_status.current_rpm;

            // Frequência proporcional ao RPM (ajuste a escala conforme o som desejado)
            float freq = rpm / 10.0f;  // ex: 900 RPM → 90 Hz, 5500 RPM → 550 Hz

            float clkdiv = 125000000.0f / (freq * PWM_WRAP_VAL);
            if (clkdiv < 1.0f) clkdiv = 1.0f;
            if (clkdiv > 255.0f) clkdiv = 255.0f;

            pwm_set_clkdiv(slice_num, clkdiv);
            pwm_set_chan_level(slice_num, channel, PWM_WRAP_VAL / 2); // 50% duty

            // Você pode deixar sempre ativo ou desativar se rpm < 1000
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // atualiza som a cada 50 ms
    }
}