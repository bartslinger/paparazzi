#include "unity.h"
#include "mcu_periph/Mockuart.h"
#include "mcu_periph/Mockspi.h"
#include "loggers/sd_logger.h"


// Externs normally defined in uart.c (not included in this test)
struct uart_periph SD_LOG_UART; // struct uart_periph uart1;
// Externs normally defined in imu.c

// Externs normally defined in spi.c
struct spi_periph spi2;

struct Imu imu;

struct SdLogger sd_logger_original;
struct Imu imu_original;

void setUp(void)
{
  // Store initial (unset) value of sd_logger struct
  sd_logger_original = sd_logger;
  imu_original = imu;
  Mockuart_Init();
  Mockspi_Init();
}

void tearDown(void)
{
  // Reset struct value to (unset) initial state
  sd_logger = sd_logger_original;
  imu = imu_original;
  Mockuart_Verify();
  Mockuart_Destroy();
  Mockspi_Verify();
  Mockspi_Destroy();
}

void test_SerialPrintLine(void)
{
  uart_transmit_Expect(&SD_LOG_UART, 0x68); // h
  uart_transmit_Expect(&SD_LOG_UART, 0x6F); // o
  uart_transmit_Expect(&SD_LOG_UART, 0x69); // i
  uart_transmit_Expect(&SD_LOG_UART, 0x0A); // \n

  sd_logger_serial_println("hoi");

  uart_transmit_Expect(&SD_LOG_UART, 0x64); // d
  uart_transmit_Expect(&SD_LOG_UART, 0x6F); // o
  uart_transmit_Expect(&SD_LOG_UART, 0x65); // e
  uart_transmit_Expect(&SD_LOG_UART, 0x69); // i
  uart_transmit_Expect(&SD_LOG_UART, 0x0A); // \n

  sd_logger_serial_println("doei");
}

//! Initialize the SD card with special procedure
/*!
 * For the SD card to switch to SPI mode, both the CS and
 * the MOSI line need to be held HIGH while sending at least
 * 74 clock pulses. This is accomplished by using SPINoSelect
 * to keep the line high and sending 0xFF to keep MOSI
 * high.
 */
void test_InitializeSdCard(void)
{
  struct spi_transaction first_transaction = sd_logger.spi_t; // copy initial values
  first_transaction.select = SPINoSelect;
  first_transaction.status = SPITransDone;
  first_transaction.cpol = SPICpolIdleLow;
  first_transaction.cpha = SPICphaEdge1;
  first_transaction.dss = SPIDss8bit;
  first_transaction.bitorder = SPIMSBFirst;
  first_transaction.cdiv = SPIDiv64;
  first_transaction.slave_idx = SD_LOGGER_SPI_LINK_SLAVE_NUMBER;
  first_transaction.input_length = 0;
  first_transaction.output_length = 10;
  first_transaction.output_buf = sd_logger.output_buf;

  spi_submit_ExpectAndReturn(&(SD_LOGGER_SPI_LINK_DEVICE), &sd_logger.spi_t, TRUE);
  sd_logger_start();

  TEST_ASSERT_EQUAL_PTR(&(SD_LOGGER_SPI_LINK_DEVICE), &sd_logger.spi_p);

  for(uint8_t i=0; i<10; i++){
    TEST_ASSERT_EQUAL_HEX8(0xFF, sd_logger.output_buf[i]);
  }

  /* Second transaction should be the reset command CMD0, which is deliverd in the first periodic cycle. */
  struct spi_transaction second_transaction = first_transaction;
  second_transaction.select = SPISelectUnselect;
  second_transaction.input_length = 0;

  //sd_logger_periodic();
}
