#include "main.h"
#include "stm32l1xx_hal.h"
#include <string.h>
#define API_WRITE_KEY "4FWY2IBWEKYH83FF"
#define HOST "184.106.153.149"

void ESP8366_Init(UART_HandleTypeDef *uart1);
void Send_To_Server(UART_HandleTypeDef *uart1, uint16_t temperature, uint16_t humidity);