// car_control_task.c
#include "car_control_task.h"
#include "joystick_task.h"       // Para a fila do joystick
#include "car_status_data.h"     // Para a estrutura de status do carro
#include "led_matrix_task.h"     // Para a fila de status do carro
#include "car_indicators_task.h" // Para os LEDs RGB e Buzzer

#include "pico/stdlib.h" // Para printf e sleep_ms para depuração

// Variáveis internas da lógica do carro
static int16_t g_current_acceleration = 0; // 0 a 2047 (ou 0 a 100 em escala)
static int8_t  g_current_gear = 0;         // 0 (Neutro), 1, 2, 3, 4, 5
static bool    g_abs_active = false;
static bool    g_airbag_deployed = false;

// Thresholds para a lógica de aceleração/frenagem
#define JOYSTICK_ACCEL_THRESHOLD  500  // Y > 500 para acelerar
#define JOYSTICK_BRAKE_THRESHOLD  -500 // Y < -500 para frear
#define MAX_ACCELERATION_VALUE    2000 // Valor máximo de aceleração (simulado)
#define MIN_ACCELERATION_VALUE    0    // Valor mínimo de aceleração

void vCarControlTask(void *pvParameters) {
    joystick_data_t joystick_data;
    car_status_t car_status;

    while (true) {
        // Tenta receber os dados mais recentes do joystick
        if (xQueueReceive(xJoystickQueue, &joystick_data, pdMS_TO_TICKS(50)) == pdPASS) {
            // --- Lógica de Aceleração / Frenagem ---
            if (joystick_data.y_axis > JOYSTICK_ACCEL_THRESHOLD) {
                // Acelerando
                g_current_acceleration += (joystick_data.y_axis - JOYSTICK_ACCEL_THRESHOLD) / 100; // Ajuste o divisor para a sensibilidade
                if (g_current_acceleration > MAX_ACCELERATION_VALUE) g_current_acceleration = MAX_ACCELERATION_VALUE;
                
                // Envia sinal para LED Verde (Aceleração)
                // Usamos a função auxiliar da car_indicators_task para evitar duplicação de lógica de pinos
                set_led(LED_GREEN_PIN, true); // Supondo que LED_GREEN_PIN e set_led estejam acessíveis (via include)
                set_led(LED_RED_PIN, false);
            } else if (joystick_data.y_axis < JOYSTICK_BRAKE_THRESHOLD) {
                // Freando
                g_current_acceleration += (joystick_data.y_axis - JOYSTICK_BRAKE_THRESHOLD) / 50; // Freio mais agressivo
                if (g_current_acceleration < MIN_ACCELERATION_VALUE) g_current_acceleration = MIN_ACCELERATION_VALUE;
                
                // Envia sinal para LED Vermelho (Frenagem)
                set_led(LED_RED_PIN, true);
                set_led(LED_GREEN_PIN, false);
            } else {
                // Inatividade do joystick Y - carro desacelera naturalmente (freio motor)
                g_current_acceleration -= 10; // Taxa de desaceleração natural
                if (g_current_acceleration < MIN_ACCELERATION_VALUE) g_current_acceleration = MIN_ACCELERATION_VALUE;

                set_led(LED_GREEN_PIN, false);
                set_led(LED_RED_PIN, false);
            }
            
            // --- Lógica do Botão SW (Buzina, por enquanto) ---
            g_airbag_deployed = false; // Reset airbag (para testar)
            g_abs_active = false;      // Reset ABS (para testar)

            if (joystick_data.sw_state) { // Botão pressionado
                // Como você inverteu a lógica antes: (inverta a lógica) acende o azul (buzina).
                // Se pressionar A, freio brusco para entrada do abs.
                // Se pressionar B, colisão e atuação do Air Bag.

                // Por enquanto, vamos usar o SW para a buzina/LED azul.
                // Depois, dividiremos para botões A e B.
                set_led(LED_BLUE_PIN, true);
                buzzer_on(); // Assumindo que buzzer_on() está acessível
            } else {
                set_led(LED_BLUE_PIN, false);
                buzzer_off();
            }

            // --- Lógica Temporária para ABS e Airbag com botões A e B ---
            // (Assumindo que você terá outros botões ou remapeará os do joystick)
            // Por enquanto, vamos simular:
            // Botão A (SW) para ABS (se Y negativo)
            // Botão B (não temos um pino para B ainda, mas se tivesse, seria algo assim)
            // if (joystick_data.sw_state && joystick_data.y_axis < JOYSTICK_BRAKE_THRESHOLD) {
            //     g_abs_active = true;
            //     // Frenagem ainda mais brusca se ABS ativo
            //     g_current_acceleration -= 500; 
            //     if (g_current_acceleration < MIN_ACCELERATION_VALUE) g_current_acceleration = MIN_ACCELERATION_VALUE;
            // }

            // if (joystick_data.sw_state && joystick_data.x_axis > 1000) { // Exemplo: SW + Joystick X para direita = colisão
            //     g_airbag_deployed = true;
            //     g_current_acceleration = 0; // Para imediatamente
            //     // Tocar som de colisão
            // }


            // --- Preparar e Enviar Status do Carro para Outras Tarefas ---
            car_status.current_acceleration = g_current_acceleration;
            car_status.current_gear = g_current_gear; // Por enquanto, sempre 0
            car_status.abs_active = g_abs_active;
            car_status.airbag_deployed = g_airbag_deployed;

            // Envia o status atualizado para a fila da matriz de LEDs
            xQueueSend(xCarStatusQueue, &car_status, 0); 

            // Opcional: imprimir o status atual do carro para depuração
            // printf("Car: Accel: %d, Gear: %d, ABS: %d, Airbag: %d\n", 
            //         g_current_acceleration, g_current_gear, g_abs_active, g_airbag_deployed);

        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Processa a lógica do carro frequentemente
    }
}