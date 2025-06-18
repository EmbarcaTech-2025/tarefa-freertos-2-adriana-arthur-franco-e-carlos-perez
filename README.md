# Tarefa: Roteiro de FreeRTOS #2 - EmbarcaTech 2025

Autor:  **Adriana Paula**  
        **Arthur Franco**  
        **Carlos Perez**  


Curso: Residência Tecnológica em Sistemas Embarcados  

Instituição: EmbarcaTech - HBr  

Campinas, junho de 2025  

---
UEBA  
main.c: Inicializa o sistema, cria as filas (xJoystickQueue e xCarStatusQueue) e as tarefas (vJoystickTask, vCarControlTask, vCarIndicatorsTask, vOledTask, vMonitorJoystickTask). A tarefa de monitoramento é útil para debug.
CMakeLists.txt: Configura o projeto para a Raspberry Pi Pico (RP2040), com suporte ao FreeRTOS, I2C, ADC, e bibliotecas padrão. A matriz de LEDs está comentada (provavelmente porque você deixou para o final).
joystick_task.c: Lê o joystick (eixos X e Y, botão SW) e os botões A (ABS) e B (Airbag), enviando os dados para xJoystickQueue. Usa ADC para os eixos e GPIO para os botões.
car_indicators_task.c: Controla o LED RGB (verde para aceleração, vermelho para freio, azul para buzina) com base nos dados do joystick. O buzzer está implementado de forma simples (liga/desliga).
car_control_task.c: Processa os dados do joystick para simular a dinâmica do carro (velocidade, RPM, marchas, ABS, Airbag, buzina) e envia o status para xCarStatusQueue.
oled_task.c: Exibe no display OLED informações do carro (velocidade, RPM, marcha, ABS, Airbag) usando a biblioteca ssd1306.c.
ssd1306.c / ssd1306.h: Driver para o display OLED SSD1306, com funções para inicializar, limpar, desenhar pixels, caracteres e strings.
car_status_data.h: Define a estrutura car_status_t para armazenar o estado do carro.


Primeira para segunda marcha: entre 15 a 25 km/h. Segunda para terceira marcha: entre 25 a 40 km/h. Terceira para quarta marcha: entre 40 a 60 km/h. Quarta para quinta marcha: acima de 60 km/h.
Para carros a gasolina, a troca de marchas é geralmente recomendada em torno de 2000 a 2500 RPM. A RPM máxima pode ficar nuns 5000 RPM.



---

## 📜 Licença  
GNU GPL-3.0.  
