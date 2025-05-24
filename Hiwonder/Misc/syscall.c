/**
 * @file syscall.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief Redirection of standard library stub functions
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include <stdio.h>
#include "usart.h"
#include "global_conf.h"

#if (0 == ENABLE_DEBUG_UART)	//Printing logs using JLink
#include "SEGGER_RTT.h"
#endif

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */

#pragma import(__use_no_semihosting) /* The following code does not apply to semihost mode */

void _sys_exit(int x)
{
}

void _ttywrch(int ch)
{
}

struct __FILE {
    int handle;
};
FILE __stdout;


PUTCHAR_PROTOTYPE {
#if (1 == ENABLE_DEBUG_UART)	//Printing logs to UART
    HAL_UART_Transmit(DEBUG_UART_REFERENCE, (uint8_t*)&ch, 1, 0xFFFF); /* Redirect to debug UART for printing */
#endif
	
#if (0 == ENABLE_DEBUG_UART)	//Printing logs using JLink
    SEGGER_RTT_Write(0, &ch, 1);   /* Redirect to JLINK RTT printing. When using RTT, you need to initialize RTT first. */
#endif
	
    return (ch);
}


