# Tarefa: Roteiro de FreeRTOS #2 - EmbarcaTech 2025

Autores:  **Adriana Paula     -     Arthur Franco     -     Carlos Perez**  


Curso: Residência Tecnológica em Sistemas Embarcados  

Instituição: EmbarcaTech - HBr  

Campinas, junho de 2025  

---

# Simulador Veicular na BitDogLab

## Entendendo o projeto Simulador Veicular na BitDogLab

#### **Introdução**

Este projeto consiste no desenvolvimento de um simulador de painel automotivo, implementado na plataforma de hardware **BitDogLab**, que é baseada no microcontrolador Raspberry Pi Pico (RP2040). O objetivo central é simular em tempo real as dinâmicas de um veículo, como aceleração, frenagem, troca de marchas e operação do motor, gerenciando múltiplos periféricos de forma concorrente. Para alcançar a simultaneidade e a responsividade necessárias, o sistema operacional de tempo real (RTOS) **FreeRTOS** é empregado para orquestrar as diversas funcionalidades do software.  

#### **Plataforma de Hardware: BitDogLab**

A **BitDogLab** serve como a base física para a simulação, integrando os seguintes componentes essenciais:  
* **Microcontrolador RP2040:** O núcleo do processamento, responsável por executar todas as tarefas do sistema.  
* **Joystick Analógico:** Atua como a principal interface de controle do usuário, simulando o acelerador e o freio do veículo.  
* **Display OLED:** Exibe informações de telemetria em tempo real, como velocidade, RPM e marcha atual.  
* **LEDs e Buzzer:** Fornecem feedback visual e sonoro para eventos como frenagem (luz de freio) e acionamento da buzina.  
* **Matriz de LEDs:** Utilizada para a visualização dinâmica da sequência de injeção de combustível do motor.  

#### **Arquitetura de Software e Funcionalidades**

A arquitetura do software é fundamentada no FreeRTOS, que permite a divisão do sistema em tarefas independentes e concorrentes. A comunicação e sincronização entre essas tarefas são realizadas de forma segura e eficiente através de filas (*Queues*). Cada tarefa possui uma responsabilidade única:  

1.  **`vJoystickTask` (Leitura de Controles):**  
    * **Função:** Realiza a leitura contínua do joystick (eixo analógico e botões).  
    * **Comunicação:** Envia os dados de estado do controle para uma fila (`xJoystickQueue`), disponibilizando-os para outras tarefas.  

2.  **`vCarControlTask` (Lógica do Veículo):**  
    * **Função:** Constitui o núcleo da simulação. Recebe os dados do joystick e calcula a física do veículo, atualizando status como velocidade, RPM e marcha. Implementa lógicas para aceleração, frenagem, arrasto e simulação de troca de marchas.  
    * **Comunicação:** Publica o estado atualizado do veículo (`car_status_t`) em uma fila global (`xCarStatusQueue`).  

3.  **`vCarIndicatorsTask` (Indicadores Visuais e Sonoros):**  
    * **Função:** Gerencia os LEDs e o buzzer. Acende o LED de freio durante a frenagem, o LED de aceleração e aciona a buzina conforme os comandos do joystick.  
    * **Comunicação:** Lê (sem remover) os dados das filas de status do carro e do joystick para reagir em tempo real aos eventos.  

4.  **`vEngineSoundTask` (Simulação Sonora do Motor):**  
    * **Função:** Gera um som de motor cuja frequência é proporcional ao RPM atual do veículo.  
    * **Comunicação:** Lê o valor de RPM da fila de status do carro (`xCarStatusQueue`) e ajusta a frequência de um sinal PWM enviado a um buzzer.  

5.  **`vOledTask` (Interface de Exibição):**  
    * **Função:** É responsável por atualizar o display OLED com as informações de telemetria do veículo.  
    * **Comunicação:** Recebe o `car_status_t` da fila e formata os dados de velocidade, RPM, marcha e outros indicadores para exibição.  

6.  **`vInjectorTask` (Simulação dos Injetores de Combustível):**  
    * **Função:** Esta nova funcionalidade simula a operação dos injetores de combustível do motor. A frequência dos pulsos de injeção é dinamicamente ajustada com base no RPM do motor.  
    * **Comunicação:** Lê o RPM da fila `xCarStatusQueue`.  
    * **Lógica:** Calcula um intervalo de pulso que é inversamente proporcional ao RPM — quanto maior a rotação do motor, mais rápidos são os pulsos. A tarefa então aciona sequencialmente quatro LEDs na matriz de LEDs, representando o ciclo de injeção dos quatro cilindros do motor, oferecendo um feedback visual dinâmico da atividade do motor.  

7.  **`vMonitorTask` (Tarefa de Depuração)**  
    * **Função:** Esta tarefa foi criada exclusivamente para fins de depuração durante o desenvolvimento. Sua única responsabilidade é monitorar os dados que trafegam pelas principais filas de comunicação do sistema (`xJoystickQueue` e `xCarStatusQueue`), exibindo seus valores em tempo real no terminal serial. Ela não controla nenhum hardware e não faz parte da lógica funcional do simulador final.  
    * **Comunicação:** Atua como uma consumidora de dados de ambas as filas. Utiliza a função `xQueueReceive` para ler e **remover** as mensagens das filas. Isso permite inspecionar os dados exatos que estão sendo produzidos pela `vJoystickTask` e pela `vCarControlTask`.  
    * **Lógica:** A cada 500 milissegundos, a tarefa tenta ler uma mensagem de cada fila e imporime os dados colhidos no monitor serial.  

#### **Fluxo de Dados**

A comunicação entre as tarefas é desacoplada pelo uso de filas. A `vJoystickTask` atua como produtora de dados de controle, enquanto a `vCarControlTask` os consome para produzir o estado do veículo. As demais tarefas (`vOledTask`, `vCarIndicatorsTask`, `vEngineSoundTask` e `vInjectorTask`) atuam como consumidoras do estado do veículo, cada uma traduzindo esses dados em uma saída específica (visual ou sonora).  
Essa arquitetura modular, viabilizada pelo FreeRTOS, resulta em um sistema e escalável, onde funcionalidades podem ser adicionadas ou modificadas com impacto mínimo no restante do código, garantindo a operação concorrente e responsiva de todos os elementos da simulação.  

---

## Código

O código de cada módulo:  

- main.c:  
```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "joystick_task.h"
#include "car_indicators_task.h"
#include "car_control_task.h"
#include "car_status_data.h"
#include "engine_sound_task.h"
#include "oled_task.h"
#include "ssd1306.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "led_matrix.h"
#include "injector_task.h"

// Biblioteca gerada pelo arquivo .pio durante compilação.
//#include "ws2818b.pio.h"

QueueHandle_t xJoystickQueue;
QueueHandle_t xCarStatusQueue;

void vMonitorJoystickTask(void *pvParameters) { // Usada apenas durante o desenvolvimento
    joystick_data_t received_joystick_data;
    car_status_t received_car_status;

    while (true) {
        if (xQueueReceive(xJoystickQueue, &received_joystick_data, 0) == pdPASS) {
            printf("Monitor: Y=%d, SW=%d, A=%d, B=%d\n",
                   received_joystick_data.y_axis,
                   received_joystick_data.sw_state,
                   received_joystick_data.button_A_state,
                   received_joystick_data.button_B_state);
        }
        if (xQueueReceive(xCarStatusQueue, &received_car_status, 0) == pdPASS) {
            printf("🚗 Monitor Car:\n");
            printf("   Velocidade:   %3d km/h\n", received_car_status.current_speed_kmh);
            printf("   RPM:          %4d\n",     received_car_status.current_rpm);
            printf("   Marcha:       %d\n",       received_car_status.current_gear);
            printf("   ABS:          %s\n",       received_car_status.abs_active ? "ATIVO" : "inativo");
            printf("   Airbag:       %s\n",       received_car_status.airbag_deployed ? "DISPARADO" : "ok");
            printf("   Buzina:       %s\n",       received_car_status.horn_active ? "LIGADA" : "desligada");
            printf("   LED Vermelho: %s\n",       received_car_status.red_led_active ? "aceso" : "apagado");
            printf("-----------------------------------\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Imprime a cada 500ms
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("Main: Inicializando sistema...\n");

    // Teste inicial do OLED
    ssd1306_init();
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "Sistema Iniciado");
    ssd1306_show();
    sleep_ms(1000);
    ssd1306_clear();
    ssd1306_show();

    // Inicializa matriz de LEDs NeoPixel.
    npInit(LED_PIN);
    npClear();

    xJoystickQueue = xQueueCreate(8, sizeof(joystick_data_t));
    xCarStatusQueue = xQueueCreate(8, sizeof(car_status_t));

    configASSERT(xJoystickQueue);
    configASSERT(xCarStatusQueue);

    xTaskCreate(vJoystickTask, "JoystickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL);
    xTaskCreate(vCarControlTask, "CarControlTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vCarIndicatorsTask, "CarIndicatorsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vOledTask, "OledTask", configMINIMAL_STACK_SIZE + 200, NULL, tskIDLE_PRIORITY + 2, NULL);
    //xTaskCreate(vMonitorJoystickTask, "MonitorJoystickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vEngineSoundTask, "EngineSoundTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 0, NULL);
    xTaskCreate(vInjectorTask, "InjectorTask", configMINIMAL_STACK_SIZE + 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (true) {}
}
```
- CMakeLists.txt:  
```c
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)

# Escolhe a placa
set(PICO_BOARD pico CACHE STRING "Board type")

# Caminho do SDK
set(PICO_SDK_PATH "C:/Users/profc/.pico-sdk/sdk/2.1.1")

# Importa o SDK do Pico
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# FreeRTOS
if (DEFINED ENV{FREERTOS_KERNEL_PATH})
    set(FREERTOS_KERNEL_PATH $ENV{FREERTOS_KERNEL_PATH})
else()
    set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS)
endif()
message("FreeRTOS Kernel located in ${FREERTOS_KERNEL_PATH}")

# Define o projeto
project(embarcatech-tarefa-freertos-2 C CXX ASM)

# Inicializa o SDK (depois do project!)
pico_sdk_init()

# Importa o FreeRTOS (depois do pico_sdk_init!)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Adiciona o executável
add_executable(embarcatech-tarefa-freertos-2
        main.c
        oled/ssd1306.c
        src/joystick_task.c
        src/car_indicators_task.c
        src/car_control_task.c
        src/oled_task.c
        src/engine_sound_task.c
        src/injector_task.c
        src/led_matrix.c
)

# Generate PIO header
pico_generate_pio_header(embarcatech-tarefa-freertos-2 ${CMAKE_CURRENT_LIST_DIR}/ws2818b.pio)

# Ativa USB
pico_enable_stdio_usb(embarcatech-tarefa-freertos-2 1)

# Includes
target_include_directories(embarcatech-tarefa-freertos-2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/oled
)

# Liga as bibliotecas necessárias
target_link_libraries(embarcatech-tarefa-freertos-2
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_pwm
    hardware_pio
    hardware_clocks
    FreeRTOS-Kernel-Heap4
)

# Gera arquivos .uf2 e outros
pico_add_extra_outputs(embarcatech-tarefa-freertos-2)
```

- joystick_task.c:  
```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "joystick_task.h"

// Pinos ADC para o joystick
#define JOYSTICK_Y_PIN  26 // ADC0 (eixo Y vertical)

// Pinos GPIO para os botões
#define JOYSTICK_SW_PIN 22 // Botão SW (buzina)
#define BUTTON_A_PIN    5  // Botão A (ABS)
#define BUTTON_B_PIN    6  // Botão B (Airbag)

// Fila declarada em main.c
extern QueueHandle_t xJoystickQueue;

void vJoystickTask(void *pvParameters) {
    joystick_data_t current_joystick_data;
    
    // Inicializa o ADC
    adc_init();
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Inicializa os pinos dos botões com pull-up
    gpio_init(JOYSTICK_SW_PIN);
    gpio_set_dir(JOYSTICK_SW_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_SW_PIN);
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);

    // Filtro simples para suavizar leituras do ADC
    static uint16_t prev_y = 2047;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40); // Lê a cada 40ms
    const TickType_t xPrintFrequency = pdMS_TO_TICKS(500); // Imprime a cada 500ms
    TickType_t xLastPrintTime = xLastWakeTime;

    while (true) {
        // Leitura do eixo Y
        adc_select_input(0); // ADC0 para JOYSTICK_Y_PIN
        uint16_t raw_y = adc_read();
        // Filtro: média ponderada para suavizar
        raw_y = (prev_y * 3 + raw_y) / 4;
        prev_y = raw_y;
        current_joystick_data.y_axis = (int16_t)(raw_y - 2047); // Ajustado para frente positivo

        // Leitura dos botões (pressionado = LOW = true)
        current_joystick_data.sw_state = !gpio_get(JOYSTICK_SW_PIN);
        current_joystick_data.button_A_state = !gpio_get(BUTTON_A_PIN);
        current_joystick_data.button_B_state = !gpio_get(BUTTON_B_PIN);

        // Envia dados para a fila
        xQueueOverwrite(xJoystickQueue, &current_joystick_data);

        // Imprime apenas a cada 500ms
        if (xTaskGetTickCount() - xLastPrintTime >= xPrintFrequency) {
            printf("Joystick: Y=%d, SW=%d, A=%d, B=%d\n",
                   current_joystick_data.y_axis,
                   current_joystick_data.sw_state,
                   current_joystick_data.button_A_state,
                   current_joystick_data.button_B_state);
            xLastPrintTime = xTaskGetTickCount();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

- car_control_task.c:  
```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "car_status_data.h"
#include "joystick_task.h"

extern QueueHandle_t xJoystickQueue;
extern QueueHandle_t xCarStatusQueue;

#define MAX_SPEED_KMH           150
#define JOYSTICK_MAX_ABS_VAL    2048
#define NEUTRAL_THRESHOLD_JOY   100
#define ACCELERATION_RATE       0.10f
#define BRAKE_RATE              0.3f
#define DRAG_RATE               0.015f
#define MIN_RPM                 900
#define MAX_RPM                 5500
#define RPM_PER_KMH             36
#define GEAR_1_MAX_SPEED        20
#define GEAR_2_MAX_SPEED        40
#define GEAR_3_MAX_SPEED        70
#define GEAR_4_MAX_SPEED        100
#define GEAR_5_MAX_SPEED        150

static float current_speed_float = 0.0f;
static bool airbag_was_deployed_once = false;

void vCarControlTask(void *pvParameters) {
    printf("CarControlTask: Iniciada\n");

    joystick_data_t received_joystick_data;
    car_status_t current_car_status;
    int16_t calculated_rpm;

    current_car_status.current_speed_kmh = 0;
    current_car_status.current_rpm = MIN_RPM;
    current_car_status.current_gear = 0;
    current_car_status.abs_active = false;
    current_car_status.airbag_deployed = airbag_was_deployed_once;
    current_car_status.horn_active = false;
    current_car_status.red_led_active = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    while (true) {
        if (xQueueReceive(xJoystickQueue, &received_joystick_data, 0) == pdPASS) {
            int16_t joystick_y = received_joystick_data.y_axis;

            // Lógica de ABS (botão A)
            if (received_joystick_data.button_A_state) {
                current_speed_float = 0.0f; // Para imediatamente
                current_car_status.abs_active = true;
                current_car_status.red_led_active = true;
            } else {
                current_car_status.abs_active = false;
                current_car_status.red_led_active = false;

                // Lógica de velocidade (apenas se ABS não estiver ativo)
                if (joystick_y > NEUTRAL_THRESHOLD_JOY) {
                    float accel_input = (float)(joystick_y - NEUTRAL_THRESHOLD_JOY) / (JOYSTICK_MAX_ABS_VAL - NEUTRAL_THRESHOLD_JOY);
                    current_speed_float += accel_input * ACCELERATION_RATE * (float)xFrequency / pdMS_TO_TICKS(1);
                } else if (joystick_y < -NEUTRAL_THRESHOLD_JOY) {
                    float brake_input = (float)(-joystick_y - NEUTRAL_THRESHOLD_JOY) / (JOYSTICK_MAX_ABS_VAL - NEUTRAL_THRESHOLD_JOY);
                    current_speed_float -= brake_input * BRAKE_RATE * (float)xFrequency / pdMS_TO_TICKS(1);
                    if (current_speed_float < 0.0f) current_speed_float = 0.0f;
                } else {
                    current_speed_float -= DRAG_RATE * (float)xFrequency / pdMS_TO_TICKS(1);
                    if (current_speed_float < 0.0f) current_speed_float = 0.0f;
                }

                if (current_speed_float > MAX_SPEED_KMH) current_speed_float = MAX_SPEED_KMH;
            }

            /// Cálculo de marcha
            int8_t previous_gear = current_car_status.current_gear;
            int8_t calculated_gear;

            if (current_speed_float == 0) {
                calculated_gear = 0;
            } else if (current_speed_float <= GEAR_1_MAX_SPEED) {
                calculated_gear = 1;
            } else if (current_speed_float <= GEAR_2_MAX_SPEED) {
                calculated_gear = 2;
            } else if (current_speed_float <= GEAR_3_MAX_SPEED) {
                calculated_gear = 3;
            } else if (current_speed_float <= GEAR_4_MAX_SPEED) {
                calculated_gear = 4;
            } else {
                calculated_gear = 5;
            }

            // Detecta troca de marcha
            bool gear_changed = (calculated_gear != previous_gear);

            // Simula tempo de engate e corte de aceleração
            if (gear_changed && previous_gear != 0) {
                // Engate: brevíssimo delay
                vTaskDelay(pdMS_TO_TICKS(100));

                // Cut de aceleração: RPM quase zero momentaneamente
                current_car_status.current_rpm = MIN_RPM;
                xQueueOverwrite(xCarStatusQueue, &current_car_status);
                vTaskDelay(pdMS_TO_TICKS(50)); // Simula corte breve
            }

            // Atualiza a marcha após simulação
            current_car_status.current_gear = calculated_gear;

            // Novo cálculo de RPM baseado na marcha
            float gear_ratio[] = {0.0f, 5.0f, 4.0f, 3.0f, 2.2f, 1.6f}; // N, 1 a 5

            if (calculated_gear > 0) {
                calculated_rpm = MIN_RPM + (int)(current_speed_float * gear_ratio[calculated_gear]);
                if (calculated_rpm > MAX_RPM) calculated_rpm = MAX_RPM;
            } else {
                calculated_rpm = MIN_RPM;
            }

            current_car_status.current_rpm = calculated_rpm;


            // Lógica de Airbag (botão B)
            if (received_joystick_data.button_B_state && !airbag_was_deployed_once) {
                airbag_was_deployed_once = true;
            }
            current_car_status.airbag_deployed = airbag_was_deployed_once;

            // Lógica de buzina (botão SW)
            current_car_status.horn_active = received_joystick_data.sw_state;

            // Atualiza status do carro
            current_car_status.current_speed_kmh = (int16_t)current_speed_float;
            current_car_status.current_rpm = calculated_rpm;
            current_car_status.current_gear = calculated_gear;

            // Envia status para a fila
            xQueueOverwrite(xCarStatusQueue, &current_car_status);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

- car_status_data.h:  
```c
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
```

- car_indicators_task.c:  
```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "joystick_task.h"
#include "car_status_data.h"

// Pinos dos LEDs RGB
#define LED_RED_PIN     13
#define LED_GREEN_PIN   11
#define LED_BLUE_PIN    12
#define BUZZER_PIN      21

// Thresholds
#define ACCELERATION_THRESHOLD  200
#define BRAKE_THRESHOLD         -200

extern QueueHandle_t xJoystickQueue;
extern QueueHandle_t xCarStatusQueue;

void set_led(uint gpio_pin, bool state) {
    gpio_put(gpio_pin, state);
}

void buzzer_on() {
    gpio_put(BUZZER_PIN, 1);
}

void buzzer_off() {
    gpio_put(BUZZER_PIN, 0);
}

void vCarIndicatorsTask(void *pvParameters) {
    printf("CarIndicatorsTask: Iniciada\n");

    // Inicializa LEDs e buzzer
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, 0); // Garante estado inicial baixo
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, 0);
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);

    // Teste inicial: Piscar LEDs para confirmar funcionamento
    printf("CarIndicatorsTask: Testando LEDs...\n");
    set_led(LED_RED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_led(LED_RED_PIN, 0);
    set_led(LED_GREEN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_led(LED_GREEN_PIN, 0);
    set_led(LED_BLUE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    set_led(LED_BLUE_PIN, 0);
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(500));
    buzzer_off();
    printf("CarIndicatorsTask: Teste concluído\n");

    joystick_data_t current_joystick_data;
    car_status_t current_car_status;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    while (true) {
        if (xQueuePeek(xJoystickQueue, &current_joystick_data, 0) == pdPASS &&
            xQueuePeek(xCarStatusQueue, &current_car_status, 0) == pdPASS) {
            // Depuração
            printf("CarIndicatorsTask: Y=%d, SW=%d\n", 
                   current_joystick_data.y_axis, current_joystick_data.sw_state);

            // LED vermelho: ligado durante ABS ou freio normal
            if (current_car_status.red_led_active || 
                (!current_car_status.red_led_active && current_joystick_data.y_axis < BRAKE_THRESHOLD)) {
                set_led(LED_RED_PIN, 1);
                if (!current_car_status.red_led_active) {
                    printf("CarIndicatorsTask: LED Vermelho ON (freio)\n");
                }
            } else {
                set_led(LED_RED_PIN, 0);
            }

            // LED verde: ligado apenas durante aceleração e se ABS não estiver ativo
            if (!current_car_status.red_led_active && current_joystick_data.y_axis > ACCELERATION_THRESHOLD) {
                set_led(LED_GREEN_PIN, 1);
                printf("CarIndicatorsTask: LED Verde ON\n");
            } else {
                set_led(LED_GREEN_PIN, 0);
            }

            // LED azul e buzzer para buzina (botão SW)
            if (current_joystick_data.sw_state) {
                set_led(LED_BLUE_PIN, 1);
                buzzer_on();
                printf("CarIndicatorsTask: LED Azul ON, Buzzer ON\n");
            } else {
                set_led(LED_BLUE_PIN, 0);
                buzzer_off();
            }
        } else {
            printf("CarIndicatorsTask: Falha ao ler fila\n");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

- engine_sound_task.c:  
```c
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

// Constantes de RPM
#define MIN_RPM_ENGINE_SOUND    900
#define MAX_RPM_ENGINE_SOUND    5500

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

            // Frequência proporcional ao RPM
            float freq = rpm / 10.0f;  // ex: 900 RPM → 90 Hz, 5500 RPM → 550 Hz

            float clkdiv = 125000000.0f / (freq * PWM_WRAP_VAL);
            if (clkdiv < 1.0f) clkdiv = 1.0f;
            if (clkdiv > 255.0f) clkdiv = 255.0f;

            pwm_set_clkdiv(slice_num, clkdiv);
            pwm_set_chan_level(slice_num, channel, PWM_WRAP_VAL / 2); // 50% duty
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // atualiza som a cada 50 ms
    }
}
```

- oled_task.c:  
```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "oled/ssd1306.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "oled_task.h"
#include "car_status_data.h" // Para a estrutura car_status_t

// A fila do status do carro será acessada globalmente via extern
extern QueueHandle_t xCarStatusQueue;

void vOledTask(void *pvParameters) {
    car_status_t received_car_status;

    ssd1306_init(); // Inicialização completa pelo seu driver
    ssd1306_clear();
    ssd1306_show();

    char line1_str[20]; // Velocidade
    char line2_str[20]; // RPM
    char line3_str[20]; // Marcha
    char line4_str[20]; // ABS
    char line5_str[20]; // Airbag

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Atualiza o OLED a cada 100ms
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Tenta receber o status mais recente do carro
        if (xQueueReceive(xCarStatusQueue, &received_car_status, xFrequency) == pdPASS) {
            ssd1306_clear(); // Limpa o buffer do OLED

            // --- Linha 1: Velocidade ---
            snprintf(line1_str, sizeof(line1_str), "Speed: %3d Km/h", received_car_status.current_speed_kmh);
            ssd1306_draw_string(0, 0, line1_str); // x=0, y=0 (1ª linha)

            // --- Linha 2: RPM ---
            snprintf(line2_str, sizeof(line2_str), "RPM: %5d", received_car_status.current_rpm);
            ssd1306_draw_string(0, 8, line2_str); // x=0, y=8 (2ª linha)

            // --- Linha 3: Marcha ---
            if (received_car_status.current_gear == 0) {
                snprintf(line3_str, sizeof(line3_str), "Gear: N");
            } else {
                snprintf(line3_str, sizeof(line3_str), "Gear: %d", received_car_status.current_gear);
            }
            ssd1306_draw_string(0, 16, line3_str); // x=0, y=16 (3ª linha)

            // --- Linha 4: ABS ---
            snprintf(line4_str, sizeof(line4_str), "ABS: %s", received_car_status.abs_active ? "Active" : "Inactive");
            ssd1306_draw_string(0, 24, line4_str); // x=0, y=24 (4ª linha)

            // --- Linha 5: Airbag ---
            snprintf(line5_str, sizeof(line5_str), "Airbag: %s", received_car_status.airbag_deployed ? "Deployed" : "OK");
            ssd1306_draw_string(0, 32, line5_str); // x=0, y=32 (5ª linha)

            // Atualiza o display com o buffer
            ssd1306_show();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

- ssd1306.c:  
```c
// ssd1306.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "oled/ssd1306.h"

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT  64
#define SSD1306_I2C i2c1
#define SSD1306_I2C_ADDR 0x3C
#define SSD1306_SDA_PIN 14
#define SSD1306_SCL_PIN 15
#define SSD1306_BUF_LEN (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

static uint8_t framebuffer[SSD1306_BUF_LEN]; // buffer estático (ideal para RTOS) da tela toda
static uint8_t tx_buffer[SSD1306_WIDTH + 1]; // buffer estático (idem) de uma página para ssd1306_write_data()

// Fonte 8x8 - o bit mais significativo do byte representa o pixel mais à esquerda
// e cada byte representa uma linha horizontal
// 0x20 - 0x7F (Caracteres Imprimíveis)
static const uint8_t font8x8_basic[128][8] = {
    // 0x20 - 0x7F (Caracteres Imprimíveis ASCII)
    [' '] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Espaço
    ['!'] = {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // exclamação
    ['"'] = {0x6C,0x6C,0x48,0x00,0x00,0x00,0x00,0x00}, // aspas (aplique escape ao usar)
    ['#'] = {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // cerqiolha
    ['$'] = {0x10,0x7E,0x90,0x7C,0x12,0xFC,0x10,0x00}, // cifrão
    ['%'] = {0xC2,0xC6,0x0C,0x18,0x30,0x66,0xC6,0x00}, // porcento
    ['&'] = {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // E comercial
    ['\''] = {0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00}, // apóstrofe (precisa escape aqui)
    ['('] = {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // abre parênteses
    [')'] = {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // fecha parênteses
    ['*'] = {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // multiplicação ou asterisco
    ['+'] = {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // adição ou mais
    [','] = {0x00,0x00,0x00,0x00,0x18,0x18,0x30,0x00}, // vírgila
    ['-'] = {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // subtração ou menos
    ['.'] = {0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00}, // ponto
    ['/'] = {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, // divisão ou barra
    [':'] = {0x00,0x00,0x18,0x18,0x00,0x18,0x18,0x00}, // dois pontos
    [';'] = {0x00,0x00,0x18,0x18,0x00,0x18,0x30,0x00}, // ´ponto e vírgula
    ['<'] = {0x18,0x30,0x60,0xC0,0x60,0x30,0x18,0x00}, // menor do que
    ['='] = {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // igral a
    ['>'] = {0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00}, // maior do que
    ['?'] = {0x3C,0x66,0x0C,0x18,0x30,0x00,0x18,0x00}, // interrogação
    ['@'] = {0x3C,0x66,0x7E,0x7E,0x7E,0x42,0x3C,0x00}, // arroba
    ['['] = {0x7E,0x60,0x60,0x60,0x60,0x60,0x7E,0x00}, // abre colchetes
    ['\\'] = {0xC0,0x60,0x30,0x18,0x0C,0x06,0x03,0x00}, // barra invertida (precisa escape aqui e ao usar)
    [']'] = {0x7E,0x06,0x06,0x06,0x06,0x06,0x7E,0x00}, // fecha colchetes
    ['^'] = {0x00,0x30,0x6C,0xC6,0x00,0x00,0x00,0x00}, // potenciação ou circunflexo
    ['_'] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE}, // underline ou sublinhado
    ['`'] = {0x00,0x60,0x30,0x00,0x00,0x00,0x00,0x00}, // crase
    ['{'] = {0x0C,0x18,0x18,0x60,0x18,0x18,0x0C,0x00}, // abre chaves
    ['|'] = {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // barra vertical
    ['}'] = {0x60,0x30,0x30,0x0C,0x30,0x30,0x60,0x00}, // fecha chaves
    ['~'] = {0x00,0x00,0x60,0x99,0x99,0x06,0x00,0x00}, // til (aproximadamente)
    ['0'] = {0x3C,0x66,0xCE,0xD6,0xE6,0x66,0x3C,0x00}, // 0
    ['1'] = {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 1
    ['2'] = {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00}, // 2
    ['3'] = {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00}, // 3
    ['4'] = {0x0C,0x1C,0x3C,0x6C,0xFE,0x0C,0x0C,0x00}, // 4
    ['5'] = {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00}, // 5
    ['6'] = {0x3C,0x66,0x60,0x7C,0x66,0x66,0x3C,0x00}, // 6
    ['7'] = {0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0x00}, // 7
    ['8'] = {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, // 8
    ['9'] = {0x3C,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00}, // 9
    ['A'] = {0x18,0x3C,0x66,0x66,0x7E,0x66,0x66,0x00}, // A
    ['B'] = {0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00}, // B
    ['C'] = {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00}, // BC
    ['D'] = {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, // CD
    ['E'] = {0x7E,0x60,0x60,0x7C,0x60,0x60,0x7E,0x00}, // DE
    ['F'] = {0x7E,0x60,0x60,0x7C,0x60,0x60,0x60,0x00}, // F
    ['G'] = {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00}, // G
    ['H'] = {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00}, // H
    ['I'] = {0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00}, // I
    ['J'] = {0x0E,0x06,0x06,0x06,0x66,0x66,0x3C,0x00}, // J
    ['K'] = {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, // K
    ['L'] = {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00}, // L
    ['M'] = {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, // M
    ['N'] = {0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00}, // N
    ['O'] = {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // O
    ['P'] = {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // P
    ['Q'] = {0x3C,0x66,0x66,0x66,0x6E,0x3C,0x0E,0x00}, // Q
    ['R'] = {0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00}, // R
    ['S'] = {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00}, // S
    ['T'] = {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // T
    ['U'] = {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00}, // U
    ['V'] = {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // V
    ['W'] = {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    ['X'] = {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // X
    ['Y'] = {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, // Y
    ['Z'] = {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00}, // Z
    ['a'] = {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3A,0x00}, // a
    ['b'] = {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // b
    ['c'] = {0x00,0x00,0x3C,0x66,0x60,0x66,0x3C,0x00}, // c
    ['d'] = {0x06,0x06,0x1E,0x66,0x66,0x66,0x3E,0x00}, // d
    ['e'] = {0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00}, // e
    ['f'] = {0x1C,0x30,0x30,0x7E,0x30,0x30,0x30,0x00}, // f
    ['g'] = {0x00,0x00,0x3C,0x66,0x66,0x3E,0x06,0x7C}, // g
    ['h'] = {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // h
    ['i'] = {0x18,0x00,0x38,0x10,0x10,0x10,0x3C,0x00}, // i
    ['j'] = {0x18,0x00,0x30,0x00,0x30,0x30,0x30,0x60}, // j
    ['k'] = {0x60,0x64,0x6C,0x78,0x6C,0x66,0x62,0x00}, // k
    ['l'] = {0x38,0x10,0x10,0x10,0x10,0x10,0x7C,0x00}, // l
    ['m'] = {0x00,0x00,0xFF,0xDB,0xDB,0xDB,0xDB,0x00}, // m
    ['n'] = {0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x00}, // n
    ['o'] = {0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00}, // o
    ['p'] = {0x00,0x7C,0x66,0x66,0x7C,0x60,0x60,0x00}, // p
    ['q'] = {0x00,0x3C,0x66,0x66,0x7C,0x06,0x06,0x00}, // q
    ['r'] = {0x00,0x60,0x7C,0x66,0x60,0x60,0x60,0x00}, // r
    ['s'] = {0x00,0x00,0x3C,0x60,0x3C,0x06,0x7C,0x00}, // s
    ['t'] = {0x20,0x7C,0x20,0x20,0x20,0x26,0x1C,0x00}, // t
    ['u'] = {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00}, // u
    ['v'] = {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // v
    ['w'] = {0x00,0x00,0x63,0x6B,0x7F,0x77,0x63,0x00}, // w
    ['x'] = {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // x
    ['y'] = {0x00,0x00,0x66,0x66,0x3E,0x06,0x06,0x7C}, // y
    ['z'] = {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}, // z
};

// Envia um comando de controle ao display
static void ssd1306_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(SSD1306_I2C, SSD1306_I2C_ADDR, buf, 2, false);
}

// Envia os dados aos display. É chamada pela função ssd1306_show()
static void ssd1306_write_data(const uint8_t *data, size_t len) {
    // Verifica se o buffer estático é grande o suficiente
    if (len + 1 > sizeof(tx_buffer)) {
        return;
    }
    tx_buffer[0] = 0x40;
    memcpy(&tx_buffer[1], data, len);
    i2c_write_blocking(SSD1306_I2C, SSD1306_I2C_ADDR, tx_buffer, len + 1, false);
}

// Inicializa o display e o barramento I2C
void ssd1306_init(void) {
    i2c_init(SSD1306_I2C, 400 * 1000);
    gpio_set_function(SSD1306_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SSD1306_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SSD1306_SDA_PIN);
    gpio_pull_up(SSD1306_SCL_PIN);
    sleep_ms(100);

    const uint8_t init_cmds[] = {
        0xAE,       // Display OFF
        0x20, 0x00, // Set Memory Addressing Mode = Horizontal
        0x40,       // Set Display Start Line = 0
        0xA1,       // Set Segment Re-map (espelha horizontalmente)
        0xA8, 0x3F, // Set Multiplex Ratio = 63 (para 64 linhas)
        0xC8,       // COM Output Scan Direction remap (espelha vertical)
        0xD3, 0x00, // Set Display Offset = 0
        0xDA, 0x12, // Set COM Pins hardware configuration
        0x81, 0x7F, // Set Contrast Control = 0x7F
        0xA4,       // Resume to RAM content (normal display mode)
        0xA6,       // Set Normal Display (não invertido)
        0xD5, 0x80, // Set Display Clock Divide Ratio/Oscillator Frequency
        0x8D, 0x14, // Charge Pump = Enable
        0xDB, 0x20, // Set VCOMH Deselect Level
        0xD9, 0xF1, // Set Pre-charge Period
        0xAF        // Display ON
};


    for (uint8_t i = 0; i < sizeof(init_cmds); i++) {
        ssd1306_write_cmd(init_cmds[i]);
    }
    ssd1306_clear();
    ssd1306_show();
}

// Limpa o framebuffer (é necessário chamar ssd1306_show() para refletir no display)/ 
void ssd1306_clear(void) {
    memset(framebuffer, 0, sizeof(framebuffer));
}

// Desenha um pixel nas coordenadas (x,y) no framebuffer
void ssd1306_draw_pixel(int x, int y, bool on) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    int byte_index = (y / 8) * SSD1306_WIDTH + x;
    if (on)
        framebuffer[byte_index] |= (1 << (y % 8));
    else
        framebuffer[byte_index] &= ~(1 << (y % 8));
}

// Desenha um caractere 8x8 no framebuffer (é necessário chamar ssd1306_show() para refletir no display)
void ssd1306_draw_char(int x, int y, char c) {
    const uint8_t *bitmap = font8x8_basic[(unsigned char)c];
    for (int row = 0; row < 8; row++) {
        uint8_t bits = bitmap[row];
        for (int col = 0; col < 8; col++) {
            // bit mais significativo (7 - col) representa pixel mais à esquerda
            bool on = (bits & (1 << (7 - col))) != 0;
            ssd1306_draw_pixel(x + col, y + row, on);
        }
    }
}

// Desenha uma string no framebuffer (é necessário chamar ssd1306_show() para refletir no display)
void ssd1306_draw_string(int x, int y, const char *str) {
    while (*str) {
        ssd1306_draw_char(x, y, *str++);
        x += 8;
    }
}

// Envia o conteúdo atual do framebuffer para o display via I2C
void ssd1306_show(void) {
    for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++) {
        ssd1306_write_cmd(0xB0 + page);
        ssd1306_write_cmd(0x00);
        ssd1306_write_cmd(0x10);
        ssd1306_write_data(&framebuffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
    }
}

// Desenha uma linha entre dois pontos no framebuffer de (x0,y0) até (x1,y1) (necessita ssd1306_show() após)
void ssd1306_draw_line(int x0, int y0, int x1, int y1, bool on) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (true) {
        ssd1306_draw_pixel(x0, y0, on);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}


// Anima um pixel caminhando horizontalmente pelo meio do display em 2 segundos
void ssd1306_walk_horizontal_pixel(void) {
    ssd1306_clear();
    int y = SSD1306_HEIGHT / 2; // linha central vertical (y = 32)
    const int delay_ms = 2000 / SSD1306_WIDTH; // ≈ 15.6ms por passo

    for (int x = 0; x < SSD1306_WIDTH; x++) {
        ssd1306_draw_pixel(x, y, true);
        ssd1306_show();
        sleep_ms(delay_ms);
    }
}
```

- injector_task.c:  
```c
#include "injector_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "led_matrix.h"      // npClear(), npWrite(), entrada_matriz_verm()
#include "car_status_data.h" // car_status_t
#include "led_matrix.h"
#include <stdlib.h>

extern QueueHandle_t xCarStatusQueue;

void vInjectorTask(void *pvParameters) {
    car_status_t current_car_status;
    const uint8_t injector_positions[4] = {0, 1, 2, 3}; 

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (xQueuePeek(xCarStatusQueue, &current_car_status, 0) == pdPASS) {
            int rpm = current_car_status.current_rpm;
            if (rpm < 900) rpm = 900;

            //calculo para deixar os pulsos visiveis na matriz de led
            int pulse_interval_ms = 100 - (abs(900-rpm)/3);
            if (pulse_interval_ms < 10) pulse_interval_ms = 10;
            if (pulse_interval_ms > 100) pulse_interval_ms = 100;

            for (int i = 0; i < 4; i++) {
                entrada_matriz_verm(injector_positions[i]); 
                npWrite();
                vTaskDelay(pdMS_TO_TICKS(pulse_interval_ms));

                npClear();
                npWrite();
                vTaskDelay(pdMS_TO_TICKS(pulse_interval_ms)); // Pequena pausa entre os pulsos
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}
```

- led_matrix.c:  
```c
#include "led_matrix.h"
#include "ws2818b.pio.h"
#include "pico/stdlib.h"       // para sleep_us()

// Definição das variáveis globais
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;

// Inicializa a máquina PIO e limpa LEDs
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;

    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
        if (sm < 0) {
            // Sem state machines livres: trate erro conforme necessidade
            while(1);
        }
    }

    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    npClear();
    npWrite();
}

// Limpa o buffer de LEDs
void npClear(void) {
    for (uint i = 0; i < LED_COUNT; i++) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

// Envia o buffer para os LEDs
void npWrite(void) {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);  // RESET sinal WS2812/WS2818b
}

// Define um LED específico com cor RGB
static void npSetLED(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < 0 || index >= LED_COUNT) return;
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Acende um LED vermelho na posição dada
void entrada_matriz_verm(int posicao_led) {
    npSetLED(posicao_led, 0x40, 0x00, 0x00);
    npWrite();
}

// Acende um LED amarelo (vermelho + verde) na posição dada
void entrada_matriz_amar(int posicao_led) {
    npSetLED(posicao_led, 0x40, 0x40, 0x00);
    npWrite();
}

// Acende um LED verde na posição dada
void entrada_matriz_verd(int posicao_led) {
    npSetLED(posicao_led, 0x00, 0x40, 0x00);
    npWrite();
}

// Apaga o LED na posição dada
void entrada_matriz_desl(int posicao_led) {
    npSetLED(posicao_led, 0x00, 0x00, 0x00);
    npWrite();
}
```

---

## Detalhes da implementação

**Tabela com as prioridades e temporizações das tarefas do projeto:**  


| **Tarefa**               | **Função**                         | **Prioridade**         | **Período / Delay**         | **Fonte de Dados**             |
|--------------------------|------------------------------------|-------------------------|------------------------------|--------------------------------|
| `vJoystickTask`          | Leitura do joystick                | `tskIDLE_PRIORITY + 5`  | 40 ms (`vTaskDelayUntil`)    | Leitura direta dos pinos/ADC   |
| `vCarControlTask`        | Lógica de controle do carro        | `tskIDLE_PRIORITY + 4`  | 50 ms (`vTaskDelayUntil`)    | Fila do joystick (`Receive`)   |
| `vCarIndicatorsTask`     | Controle de LEDs e buzina          | `tskIDLE_PRIORITY + 3`  | 20 ms (`vTaskDelayUntil`)    | Fila do carro (`Peek`)         |
| `vEngineSoundTask`       | Geração de som do motor (PWM)      | `tskIDLE_PRIORITY + 0`  | 50 ms (`vTaskDelay`)         | Fila do carro (`Peek`)         |
| `vInjectorTask`          | Simulação dos injetores (LEDs)     | `tskIDLE_PRIORITY + 1`  | 10–100 ms (depende do RPM)   | Fila do carro (`Peek`)         |
| `vOledTask`              | Atualização do display OLED        | `tskIDLE_PRIORITY + 2`  | 100 ms (`vTaskDelayUntil`)   | Fila do carro (`Receive`)      |

**Observações:**  

* A prioridade mais alta é da tarefa **JoystickTask** já que ela coleta os comandos do usuário.  
* A tarefa **CarControlTask** vem logo em seguida, para reagir rapidamente aos comandos.  
* As tarefas **CarIndicatorsTask** e **EngineSoundTask** usam `Peek`, ou seja, não consomem a fila, apenas observam.  
* A tarefa **InjectorTask** alterna LEDs com um delay variável que depende do RPM (entre 10–100 ms).  
* O **display OLED** é atualizado com menor frequência (100 ms).  

---

## Glossário 

ADC (Analog-to-Digital Converter): Conversor Analógico-Digital. Um componente que converte um sinal analógico (como a tensão de um joystick) em um valor digital que o microcontrolador pode processar.  

API (Application Programming Interface): Conjunto de definições e protocolos que permitem que softwares se comuniquem uns com os outros. No FreeRTOS, são as funções como xTaskCreate(), xQueueSend(), etc.  

CMake: Uma ferramenta de código aberto usada para gerenciar o processo de compilação de software usando uma abordagem independente de plataforma. Gera makefiles ou outros arquivos de projeto.  

Duty Cycle: Em PWM, é a proporção do tempo em que um sinal está "ligado" (HIGH) em relação ao período total do sinal. Expresso em porcentagem.  

Embedded System (Sistema Embarcado): Um sistema computacional com uma função dedicada dentro de um sistema mecânico ou elétrico maior. Projetado para uma tarefa específica.  

extern: Palavra-chave em C que declara que uma variável ou função é definida em outro arquivo fonte, permitindo que ela seja usada no arquivo atual.  

Framebuffer: Uma área da memória que contém uma representação em pixels do que deve ser exibido na tela. O driver SSD1306 escreve neste buffer e depois o transfere para o display.  

FreeRTOS: Um sistema operacional em tempo real (RTOS) de código aberto para microcontroladores. Ele gerencia as tarefas, filas, semáforos e outros recursos de tempo real.  

GPIO (General Purpose Input/Output): Pinos de entrada/saída de uso geral no microcontrolador que podem ser configurados como entradas ou saídas digitais, ou para funções especiais (I2C, PWM, ADC).  

Heap: Uma área de memória onde programas podem alocar memória dinamicamente em tempo de execução (ex: com malloc() ou xTaskCreate() do FreeRTOS).  

I2C (Inter-Integrated Circuit): Um protocolo de comunicação serial de dois fios (SDA e SCL) amplamente usado para comunicação de curta distância entre componentes, como microcontroladores e displays OLED.  

Kernel: O núcleo de um sistema operacional que gerencia os recursos do sistema e as interações entre hardware e software. No FreeRTOS, é o responsável pelo agendamento das tarefas.  

Mutex (Mutual Exclusion): Um objeto de sincronização usado para proteger recursos compartilhados, garantindo que apenas uma tarefa por vez possa acessá-lo.  

Pico SDK (Software Development Kit): O kit de desenvolvimento de software oficial da Raspberry Pi Foundation para o Raspberry Pi Pico. Fornece bibliotecas e ferramentas para programar o RP2040.  

PIO (Programmable I/O): Um subsistema no RP2040 que permite aos desenvolvedores definir interfaces de hardware personalizadas programando pequenos "state machines".  

PWM (Pulse Width Modulation): Modulação por Largura de Pulso. Uma técnica para controlar a quantidade de energia entregue a uma carga, variando a largura de um pulso digital. Usada para controle de brilho de LEDs, velocidade de motores e geração de áudio.  

Queue (Fila): Um mecanismo de comunicação entre tarefas no FreeRTOS que permite a troca segura de dados. Os dados são enviados para o final da fila e lidos do início (FIFO - First In, First Out).  

Raspberry Pi Pico: Uma placa de microcontrolador pequena, rápida e versátil construída no chip RP2040 da Raspberry Pi.  

README.md: Um arquivo de texto comum em projetos de software que fornece uma visão geral do projeto, instruções de build, uso e outras informações importantes.  

RTOS (Real-Time Operating System): Sistema Operacional em Tempo Real. Um sistema operacional que garante que certas operações serão executadas dentro de prazos definidos.  

Scheduler (Agendador): A parte do kernel do RTOS que decide qual tarefa deve ser executada a qualquer momento, com base em suas prioridades e outros critérios.  

Semáforo: Um objeto de sincronização que pode ser usado para controlar o acesso a recursos ou para sinalizar a ocorrência de eventos entre tarefas.  

SSD1306: Um chip controlador amplamente utilizado em pequenos displays OLED monocromáticos.  

Stack (Pilha): Uma área de memória reservada para cada tarefa onde variáveis locais, parâmetros de função e endereços de retorno são armazenados temporariamente.  

static: Palavra-chave em C que, quando aplicada a variáveis globais ou em escopo de arquivo, garante que elas sejam visíveis apenas dentro do arquivo em que foram definidas. Quando aplicada a variáveis locais dentro de uma função, garante que a variável preserve seu valor entre chamadas da função.  

Task (Tarefa): No FreeRTOS, uma função independente que pode ser executada em concorrência com outras tarefas. Cada tarefa tem sua própria pilha e prioridade.  

Tick: Uma interrupção periódica gerada por um timer do sistema, usada pelo FreeRTOS para manter o controle do tempo e para o agendamento de tarefas.  

Time Slicing (Fatiamento de Tempo): Um modo de operação do scheduler onde tarefas de mesma prioridade compartilham o tempo da CPU, cada uma recebendo uma "fatia" de tempo para executar antes que o scheduler mude para a próxima.  

vTaskDelay() / vTaskDelayUntil(): Funções do FreeRTOS para atrasar (suspender) a execução de uma tarefa por um período de tempo especificado. vTaskDelayUntil é preferível para atrasos periódicos precisos.  

xQueueOverwrite() / xQueuePeek() / xQueueReceive(): Funções do FreeRTOS para interagir com filas. Overwrite substitui o item mais antigo se a fila estiver cheia. Peek lê sem remover. Receive lê e remove.  

---

## Referências 

#### 🔹 Raspberry Pi Pico SDK
- [Getting Started with Raspberry Pi Pico (PDF)](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)  
- [C/C++ SDK Documentation (PDF)](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)  
- [Repositório oficial no GitHub](https://github.com/raspberrypi/pico-sdk)

#### 🔹 FreeRTOS
- [Documentação oficial do FreeRTOS](https://www.freertos.org/Documentation/RTOS_book.html)  
- [API Reference – v10.x](https://www.freertos.org/a00106.html)  
- [Exemplo de uso com Raspberry Pi Pico + FreeRTOS (GitHub)](https://github.com/sekigon-gonnoc/pico-freertos)


---

## Resultado obtido

[👉 Clique aqui e baixe o vídeo do projeto](video/video.mp4)  

---

## 📜 Licença  
GNU GPL-3.0.  
