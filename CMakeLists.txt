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
        # ws2818b.pio
        oled/ssd1306.c
        src/joystick_task.c
        src/car_indicators_task.c
        # src/car_status_data.h
        src/car_control_task.c
        src/oled_task.c
        #src/led_matrix_task.c
        src/engine_sound_task.c
)

# Agora o pico_add_pio_program deve funcionar
# pico_add_pio_program(embarcatech-tarefa-freertos-2 ${CMAKE_CURRENT_LIST_DIR}/ws2818b.pio)

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
    #hardware_pio
    #hardware_clocks
    FreeRTOS-Kernel-Heap4
)

# Gera arquivos .uf2 e outros
pico_add_extra_outputs(embarcatech-tarefa-freertos-2)
