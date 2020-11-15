/*
 * debug.c
 *
 *  Created on: Sep 5, 2020
 *      Author: Brinza
 */

#include "debug.h"

void dbgUARTVal(unsigned char outVal)
{
    static UART_Handle uart;
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = UART_BAUD_RATE;

    if (uart == NULL)
        uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        dbgFailRoutine();
    }

    UART_writePolling(uart, &outVal, 1);
//    UART_close(uart);
}

void dbgOutputLoc(unsigned int outLoc)
{
    if (outLoc > 127)
    {
        dbgFailRoutine();
    }
    else
    {
//        GPIO_toggle(DEBUG_GPIO_0);
//        GPIO_write(DEBUG_GPIO_7, 0);
//        GPIO_write(DEBUG_GPIO_0, outLoc & 0b00000001);
//        GPIO_write(DEBUG_GPIO_1, outLoc & 0b00000010);
//        GPIO_write(DEBUG_GPIO_2, outLoc & 0b00000100);
//        GPIO_write(DEBUG_GPIO_3, outLoc & 0b00001000);
//        GPIO_write(DEBUG_GPIO_4, outLoc & 0b00010000);
//        GPIO_write(DEBUG_GPIO_5, outLoc & 0b00100000);
//        GPIO_write(DEBUG_GPIO_6, outLoc & 0b01000000);
//
//        GPIO_write(DEBUG_GPIO_7, 1);
        dbgWriteNumToGPIO(outLoc);

    }
}

// TODO: disable interrupts
// TODO: check timing
// TODO: comment
void dbgFailRoutine(void)
{
    dbgWriteNumToGPIO(0xAA);
    IntMasterDisable();
    while(1)
    {
        int i;
        for(i = 0; i < 3e5; i++)
        {
            i = i;
            // Nothing
        }
        GPIO_toggle(CONFIG_GPIO_LED_0);
    }
}

void dbgWriteNumToGPIO(unsigned int num)
{
    //        GPIO_toggle(DEBUG_GPIO_0);
            GPIO_write(DEBUG_GPIO_7, 0);
            GPIO_write(DEBUG_GPIO_0, num & 0b00000001);
            GPIO_write(DEBUG_GPIO_1, num & 0b00000010);
            GPIO_write(DEBUG_GPIO_2, num & 0b00000100);
            GPIO_write(DEBUG_GPIO_3, num & 0b00001000);
            GPIO_write(DEBUG_GPIO_4, num & 0b00010000);
            GPIO_write(DEBUG_GPIO_5, num & 0b00100000);
            GPIO_write(DEBUG_GPIO_6, num & 0b01000000);

            GPIO_write(DEBUG_GPIO_7, 1);
}

void printHelper(unsigned char str[]){
    int i = 0;
    for (i = 0; str[i] != '\0'; i++){
        dbgUARTVal(str[i]);
    }
}

void printHelperNum(unsigned int num){
//    while (num != 0) {
//        dbgUARTVal((char) num % 10 + 0x30);
//        num = num / 10;
//    }
    int r = 0;

    if (num == 0){
        return;
    }

    r = num % 10;
    printHelperNum(num / 10);

    dbgUARTVal((char) r + 0x30);

}
