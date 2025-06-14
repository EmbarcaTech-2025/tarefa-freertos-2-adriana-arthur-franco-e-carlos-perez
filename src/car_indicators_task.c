// car_indicators_task.c
#include "car_indicators_task.h"
#include "joystick_task.h" // Para incluir a estrutura joystick_data_t e a fila xJoystickQueue

#include "pico/stdlib.h"
//#include "hardware/pwm.h" // Para o buzzer, se for usar PWM para o ronco

// Definição dos pinos dos LEDs RGB
#define LED_RED_PIN     13
#define LED_GREEN_PIN   11
#define LED_BLUE_PIN    12
// Definição do pino do Buzzer (exemplo)
#define BUZZER_PIN      10 

// Thresholds para ativar os LEDs de aceleração/frenagem
#define ACCELERATION_THRESHOLD  500
#define BRAKE_THRESHOLD         -500

// Funções auxiliares para controlar os LEDs e Buzzer
void set_led(uint gpio_pin, bool state) {
    gpio_put(gpio_pin, state);
}

// Inicia o buzzer com uma frequência simples para teste
void buzzer_on() {
    // Para um simples "on/off" do buzzer, basta ligar/desligar o GPIO
    // Se for um buzzer passivo e precisar de PWM, a lógica seria mais complexa aqui
    gpio_put(BUZZER_PIN, 1); 
}

void buzzer_off() {
    gpio_put(BUZZER_PIN, 0);
}

void vCarIndicatorsTask(void *pvParameters) {
    joystick_data_t current_joystick_data;

    // Inicialização dos GPIOs dos LEDs
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);

    // Inicialização do GPIO do Buzzer
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    // Assegura que o buzzer está desligado no início
    buzzer_off(); 

    while (true) {
        // Tenta receber os dados mais recentes do joystick da fila
        // Espera no máximo 100ms por novos dados. Se não houver, continua
        // para não bloquear indefinidamente.
        if (xQueueReceive(xJoystickQueue, &current_joystick_data, pdMS_TO_TICKS(100)) == pdPASS) {
            // Lógica para Aceleração/Frenagem (LED Verde/Vermelho)
            if (current_joystick_data.y_axis > ACCELERATION_THRESHOLD) {
                // Acelerando
                set_led(LED_GREEN_PIN, true);
                set_led(LED_RED_PIN, false); // Garante que o vermelho está desligado
            } else if (current_joystick_data.y_axis < BRAKE_THRESHOLD) {
                // Freando
                set_led(LED_RED_PIN, true);
                set_led(LED_GREEN_PIN, false); // Garante que o verde está desligado
            } else {
                // Joystick no centro ou dentro dos thresholds de inatividade
                set_led(LED_GREEN_PIN, false);
                set_led(LED_RED_PIN, false);
            }

            // Lógica para Buzina (LED Azul e Buzzer)
            // A lógica do botão SW é invertida porque ele está com pull-up.
            // Se o botão for pressionado, gpio_get() retorna 0, então !gpio_get() é 1.
            if (current_joystick_data.sw_state) { // Se o botão SW foi pressionado
                set_led(LED_BLUE_PIN, true);
                buzzer_on();
            } else {
                set_led(LED_BLUE_PIN, false);
                buzzer_off();
            }
        }
        
        // Pequeno atraso para a tarefa não consumir 100% da CPU.
        // Já há um atraso na tarefa do joystick, mas este é para o ciclo de atualização dos LEDs.
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}