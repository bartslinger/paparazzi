#include "unity.h"
#include "mcu_periph/Mockuart.h"
#include "loggers/serial_logger.h"


// Externs normally defined in uart.c (not included in this test)
struct uart_periph SERIAL_LOG_UART; // struct uart_periph uart1;

struct serial_logger_struct serial_logger_original;

void setUp(void)
{
    // Store initial (unset) value of serial_logger struct
    serial_logger_original = serial_logger;
    Mockuart_Init();
}

void tearDown(void)
{
    // Reset struct value to (unset) initial state
    serial_logger = serial_logger_original;
    Mockuart_Verify();
    Mockuart_Destroy();
}

void test_WriteStartByteWith12DataBytesPeriodicly(void)
{
    uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 104, TRUE);
    uart_transmit_Expect(&SERIAL_LOG_UART, 0xFF);
    for (uint8_t i=0; i<12; i++)
    {
        uart_transmit_Expect(&SERIAL_LOG_UART, i); // i represents sort of random data
        uart_transmit_IgnoreArg_data();
    }

    serial_logger_periodic();
}

void test_PreventToWriteTooMuchData(void)
{
    uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 104, FALSE);

    serial_logger_periodic();
}

void test_SendErrorMessageAfterUnableTooWrite(void)
{
    test_PreventToWriteTooMuchData();

    uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 104, FALSE);
    uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 104, TRUE);
    uart_transmit_Expect(&SERIAL_LOG_UART, 0xF0);
    for (uint8_t i=0; i<12; i++)
    {
        uart_transmit_Expect(&SERIAL_LOG_UART, 0x00); // Explicitly require 0x00 for error
    }

    serial_logger_periodic(); // First time, still no space
    serial_logger_periodic();
}

void test_SendNewDataAfterErrorMessage(void)
{
    test_SendErrorMessageAfterUnableTooWrite();
    test_WriteStartByteWith12DataBytesPeriodicly();
}
