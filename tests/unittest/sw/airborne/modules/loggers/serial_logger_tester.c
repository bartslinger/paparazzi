#include "unity.h"
#include "mcu_periph/Mockuart.h"
#include "loggers/serial_logger.h"


// Externs normally defined in uart.c (not included in this test)
struct uart_periph SERIAL_LOG_UART; // struct uart_periph uart1;
// Externs normally defined in imu.c
struct Imu imu;

struct serial_logger_struct serial_logger_original;
struct Imu imu_original;

void setUp(void)
{
  // Store initial (unset) value of serial_logger struct
  serial_logger_original = serial_logger;
  imu_original = imu;
  Mockuart_Init();
}

void tearDown(void)
{
  // Reset struct value to (unset) initial state
  serial_logger = serial_logger_original;
  imu = imu_original;
  Mockuart_Verify();
  Mockuart_Destroy();
}

void test_WriteStartByteWithUnscaledAccelerationsPeriodically(void)
{
  imu.accel_unscaled.x = 0x11223344;
  imu.accel_unscaled.y = 0x55667788;
  imu.accel_unscaled.z = 0x99AABBCC;
  uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 13, TRUE);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0xFF);

  // Imu values are 32 bit, need to be send in 4 bytes each.
  // Least significant bit first
  // Send x
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x44);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x33);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x22);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x11);

  // Send y
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x88);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x77);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x66);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x55);

  // Send z
  uart_transmit_Expect(&SERIAL_LOG_UART, 0xCC);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0xBB);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0xAA);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0x99);

  serial_logger_periodic();
}

void test_PreventToWriteTooMuchData(void)
{
  uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 13, FALSE);

  serial_logger_periodic();
}

void test_SendErrorMessageAfterUnableTooWrite(void)
{
  test_PreventToWriteTooMuchData();

  uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 5, FALSE);
  uart_check_free_space_ExpectAndReturn(&SERIAL_LOG_UART, 5, TRUE);
  uart_transmit_Expect(&SERIAL_LOG_UART, 0xF0);
  for (uint8_t i=0; i<4; i++)
  {
    uart_transmit_Expect(&SERIAL_LOG_UART, 0x00); // Explicitly require 0x00 for error
  }
  serial_logger_periodic(); // First time, still no space
  serial_logger_periodic();
}

void test_SendNewDataAfterErrorMessage(void)
{
  test_SendErrorMessageAfterUnableTooWrite();
  test_WriteStartByteWithUnscaledAccelerationsPeriodically();
}
