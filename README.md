# Tarefa: Roteiro de FreeRTOS #2 - EmbarcaTech 2025

Autores:  **Adriana Paula          Arthur Franco          Carlos Perez**  


Curso: Residência Tecnológica em Sistemas Embarcados  

Instituição: EmbarcaTech - HBr  

Campinas, junho de 2025  

---
# Simulador de automóvel na BitDogLab

## Introdução

A ideia é construir um sistema simples e modular, com FreeRTOS, que simule um carro automático com acelerador e freio, som de motor, luz para indicar aceleração (verde) e frenagem (vermelha), buzina com led azul e buzzer, indicação no display oled de velocidade, RPM, marcha e atuação do ABS e do airbag. A aceleração e o freio são controlados pelo eixo vertical do joystick, a buzina é controlada pela chave do joystick. O botão A controla o uso do ABS e o botão B controla o disparo do airbag.

O sistema foi desenvolvido de modo que cada módulo cuide de uma funcionalidade específica e o desafio é estabelecer as prioridades e temporizações de cada funcionalidade para que o funcionamento seja o mais real prossível, sem atrasos ou perda de controle.

Segue uma breve descrição de cada módulo:

- main.c: Inicializa o sistema, cria as filas (xJoystickQueue e xCarStatusQueue) e as tarefas (vJoystickTask, vCarControlTask, vCarIndicatorsTask, vOledTask, vEngineSoundTask e vMonitorJoystickTask). A tarefa de monitoramento é útil para debug.  
- CMakeLists.txt: Configura o projeto para a Raspberry Pi Pico (RP2040), com suporte ao FreeRTOS, I2C, ADC, PWM e bibliotecas padrão.  
- joystick_task.c: Lê o joystick (eixo Y para acelerador e freio e o botão SW para buzina) e os botões A (ABS) e B (Airbag), enviando os dados para xJoystickQueue. Usa ADC para o eixo do joystick e GPIO para os botões.  
- car_control_task.c: Processa os dados do joystick para simular a dinâmica do carro (velocidade, RPM, marchas, ABS, Airbag, buzina) e envia o status para xCarStatusQueue.  
- car_indicators_task.c: Controla o LED RGB (verde para aceleração, vermelho para freio, azul para buzina) com base nos dados do joystick. O buzzer da buzina está implementado de forma simples (liga/desliga).  
- engine_sound_task.c: Simula o ronco do motor com PWM.  
- oled_task.c: Exibe no display OLED informações do carro (velocidade, RPM, marcha, ABS, Airbag) usando a biblioteca ssd1306.c.  
- ssd1306.c / ssd1306.h: Driver para o display OLED SSD1306, com funções para inicializar, limpar, desenhar pixels, caracteres e strings.  
- car_status_data.h: Define a estrutura car_status_t para armazenar o estado do carro.  

## Código

O código de cada módulo:

---

## 📜 Licença  
GNU GPL-3.0.  
