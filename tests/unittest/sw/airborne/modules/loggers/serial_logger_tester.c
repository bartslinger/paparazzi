#include "unity.h"
#include "mcu_periph/Mockuart.h"
#include "loggers/serial_logger.h"

// Externs defined in uart.h
#if USE_UART1
struct uart_periph uart1;
#endif

void setUp(void)
{
    Mockuart_Init();
}

void tearDown(void)
{
    Mockuart_Verify();
    Mockuart_Destroy();
}

void testIfInitalizationOfModuleAlsoInitializesUart(void)
{
    uart_periph_init_Expect(&SERIAL_LOG_UART);
    serial_logger_start();
}
