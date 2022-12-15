# Sistemas-Embarcados-UTFPR

## STM32 Utilizado
- F401CDU6

## Portas Utiliadas

- USB -> PA11; PA12
- Led -> PC13
- SYS -> PA13; PA14
- ADC -> ADC1_IN8 -> PB0 Com ADM2;
- PWM -> A7;A6;B0 -> Frequencia 60Hz utilizado para RGB de um led

## Problema a ser solucionado:

### Trabalho final da disciplina de sistemas embarcados

1. [x] Substituir a fila de recepção de dados da USB por um objeto do FreeRTOS que seja mais eficiente para receber dados em blocos, com tamanho variável;

2. [x] Implementar uma tarefa servidor, que será responsável por gerenciar a impressão de dados no terminal, eliminando a necessidade de um mutex para gerenciar o recurso compartilhado;

3. [X] Implementar um console/terminal a partir do FreeRTOS+CLI

4. [X] Implementar pelo menos 3 comandos no terminal, sendo obrigatoriamente um deles a listagem das tarefas instaladas no sistema e outro a listagem de harmônicas de um sinal recebido por um conversor A/D do microcontrolador. O terceiro comando é de livre escolha.
