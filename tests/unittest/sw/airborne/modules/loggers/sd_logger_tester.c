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

uint8_t MySpiSubmitCallbackSendCMD0NrCalls;
uint8_t MySpiSubmitCallbackStartSdCardNrCalls;
uint8_t MySpiSubmitCallbackReceiveCorrectResponseCMD0NrCalls;
uint8_t MySpiSubmitCallbackProcessCmd8CorrectResponseNrCalls;

void setUp(void)
{
  // Store initial (unset) value of sd_logger struct
  sd_logger_original = sd_logger;
  sd_logger_setup_spi();
  MySpiSubmitCallbackSendCMD0NrCalls                   = 0;
  MySpiSubmitCallbackStartSdCardNrCalls                = 0;
  MySpiSubmitCallbackReceiveCorrectResponseCMD0NrCalls = 0;
  MySpiSubmitCallbackProcessCmd8CorrectResponseNrCalls = 0;
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

bool_t MySpiSubmitCallbackStartSdCard(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) cmock_num_calls; // ignore unused
  MySpiSubmitCallbackStartSdCardNrCalls++;

  /* First call, this sends 80 (>74) clock pulses with CS and MOSI high */
  TEST_ASSERT_EQUAL_PTR(&(SD_LOGGER_SPI_LINK_DEVICE), p);
  TEST_ASSERT_EQUAL(SPINoSelect, t->select);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);
  TEST_ASSERT_EQUAL(SPICpolIdleLow, t->cpol);
  TEST_ASSERT_EQUAL(SPICphaEdge1, t->cpha);
  TEST_ASSERT_EQUAL(SPIDss8bit, t->dss);
  TEST_ASSERT_EQUAL(SPIMSBFirst, t->bitorder);
  TEST_ASSERT_EQUAL(SPIDiv64, t->cdiv);
  TEST_ASSERT_EQUAL(SD_LOGGER_SPI_LINK_SLAVE_NUMBER, t->slave_idx);
  TEST_ASSERT_EQUAL(10, t->output_length);
  TEST_ASSERT_EQUAL(0, t->input_length);
  TEST_ASSERT_EQUAL_PTR(&sd_logger.output_buf, t->output_buf);
  TEST_ASSERT_EQUAL_PTR(&sd_logger.input_buf, t->input_buf);
  for (uint8_t i=0; i<10; i++){
    TEST_ASSERT_EQUAL(0xFF, t->output_buf[i]);
  }

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sd_logger_send_CMD0, t->after_cb);

  return TRUE;
}

//! Initialize the SD card with special procedure
/*!
 * For the SD card to switch to SPI mode, both the CS and
 * the MOSI line need to be held HIGH while sending at least
 * 74 clock pulses. This is accomplished by using SPINoSelect
 * to keep the line high and sending 0xFF to keep MOSI
 * high.
 */
void test_StartSdCard(void)
{
  // Main start function
  spi_submit_StubWithCallback(MySpiSubmitCallbackStartSdCard);
  sd_logger_start();
  TEST_ASSERT_EQUAL_MESSAGE(1, MySpiSubmitCallbackStartSdCardNrCalls, "spi_submit called too often.");
}


bool_t MySpiSubmitCallbackSendCMD0(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  MySpiSubmitCallbackSendCMD0NrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6+8+1, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x40, t->output_buf[0]); // CMD byte
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // paramter bytes (ignored)
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x95, t->output_buf[5]); // CRC7 for CMD0(0x00000000)

  // Set reply
  t->input_buf[6] = 0xFF;
  t->input_buf[7] = 0x01;

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sd_logger_get_CMD0_response, t->after_cb);

  return TRUE;
}

//! Send CMD0 after the 74 pulses
/*!
 * Function is called by callback from start procedure. The
 * 74 clock pulses have just been fired.
 */
void test_SendCMD0(void)
{
  // From callback:
  spi_submit_StubWithCallback(MySpiSubmitCallbackSendCMD0);
  sd_logger_send_CMD0(&sd_logger.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, MySpiSubmitCallbackSendCMD0NrCalls, "spi_submit called too often.");
}

bool_t MySpiSubmitCallbackReceiveCorrectResponseCMD0(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  MySpiSubmitCallbackReceiveCorrectResponseCMD0NrCalls++;
  /* Next step is to send CMD8.
   * Response type is R7 (R1 + 32bit) = 5 bytes
   */
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6+8+5, t->input_length); // CMD + Ncr_max + response

  TEST_ASSERT_EQUAL_HEX8(0x48, t->output_buf[0]); // CMD8
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0xAA, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x87, t->output_buf[5]); // CRC7 for CMD8(0x000001AA)
  TEST_ASSERT_EQUAL_PTR(&sd_logger_process_CMD8, t->after_cb); // Continue with processing the response

  return TRUE;
}

//! Check response from the CMD0
/*!
 * This is callback function from sending CMD0 (reset SD card).
 * Ncr (response time) is variable, can be 0-8 bytes.
 * MISO is high (0xFF) while SD is working.
 * Test for two different cases.
 */
void test_ReceiveResponseCMD0Case1(void)
{
  // Bytes 0-5 = command
  sd_logger.input_buf[6] = 0xFF;
  sd_logger.input_buf[7] = 0x01; // Reply means in idle state
  sd_logger.input_buf[8] = 0xFF;
  sd_logger.input_buf[9] = 0xFF;
  sd_logger.input_buf[10] = 0xFF;
  sd_logger.input_buf[11] = 0xFF;

  spi_submit_StubWithCallback(MySpiSubmitCallbackReceiveCorrectResponseCMD0);
  // Function call:
  sd_logger_get_CMD0_response(&sd_logger.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, MySpiSubmitCallbackReceiveCorrectResponseCMD0NrCalls, "spi_submit call count mismatch.");
}

/* Changed Ncr (response time) */
void test_ReceiveResponseCMD0Case2(void)
{
  // Bytes 0-5 = command
  sd_logger.input_buf[6] = 0xFF;
  sd_logger.input_buf[7] = 0xFF;
  sd_logger.input_buf[8] = 0xFF;
  sd_logger.input_buf[9] = 0xFF;
  sd_logger.input_buf[10] = 0xFF;
  sd_logger.input_buf[11] = 0xFF;
  sd_logger.input_buf[12] = 0xFF;
  sd_logger.input_buf[13] = 0xFF;
  sd_logger.input_buf[14] = 0x01; // more delay

  spi_submit_StubWithCallback(MySpiSubmitCallbackReceiveCorrectResponseCMD0);
  sd_logger_get_CMD0_response(&sd_logger.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, MySpiSubmitCallbackReceiveCorrectResponseCMD0NrCalls, "spi_submit call count mismatch.");
}

/* No response at all */
void test_ReceiveResponseCMD0Case3(void)
{
  // Bytes 0-5 = command
  sd_logger.input_buf[6] = 0xFF;
  sd_logger.input_buf[7] = 0xFF;
  sd_logger.input_buf[8] = 0xFF;
  sd_logger.input_buf[9] = 0xFF;
  sd_logger.input_buf[10] = 0xFF;
  sd_logger.input_buf[11] = 0xFF;
  sd_logger.input_buf[12] = 0xFF;
  sd_logger.input_buf[13] = 0xFF;
  sd_logger.input_buf[14] = 0xFF;


  sd_logger_get_CMD0_response(&sd_logger.spi_t);
  TEST_ASSERT_EQUAL(CardUnknown, sd_logger.card_type);
}

/* WRONG response */
void test_ReceiveResponseCMD0Case4(void)
{
  // Bytes 0-5 = command
  sd_logger.input_buf[6] = 0xFF;
  sd_logger.input_buf[7] = 0xFF;
  sd_logger.input_buf[8] = 0xFF;
  sd_logger.input_buf[9] = 0x02; // wrong value/error
  sd_logger.input_buf[10] = 0xFF;
  sd_logger.input_buf[11] = 0xFF;
  sd_logger.input_buf[12] = 0xFF;
  sd_logger.input_buf[13] = 0xFF;
  sd_logger.input_buf[14] = 0xFF;

  sd_logger_get_CMD0_response(&sd_logger.spi_t);
  TEST_ASSERT_EQUAL(CardUnknown, sd_logger.card_type);
}

bool_t MySpiSubmitCallbackProcessCmd8CorrectResponse(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  MySpiSubmitCallbackProcessCmd8CorrectResponseNrCalls++;

  // Perform ACMD41 call with argument 0x40000000
  TEST_ASSERT_EQUAL(6+8+1+6, t->output_length); // CMD55 + Ncr (max 8) + R1 + CMD41
  TEST_ASSERT_EQUAL(6+8+1+6+8+1, t->input_length); // CMD41 responds also with R1

  // CMD55
  TEST_ASSERT_EQUAL_HEX8(0x77, t->output_buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]);

  // Response time CMD55 (8+1)
  for(uint8_t i=0; i<9; i++){
    TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[i+6]);
  }

  // CMD41(0x40000000)
  TEST_ASSERT_EQUAL_HEX8(0x69, t->output_buf[15]);
  TEST_ASSERT_EQUAL_HEX8(0x40, t->output_buf[16]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[17]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[18]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[19]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[20]);

  return TRUE;
}

void test_ProcessResponseToCMD8CaseMatched(void)
{
  sd_logger.input_buf[6]  = 0xFF;
  sd_logger.input_buf[7]  = 0xFF;
  sd_logger.input_buf[8]  = 0x01; // R1 response (idle state)
  sd_logger.input_buf[9]  = 0x00; // matching value 0x000001AA
  sd_logger.input_buf[10] = 0x00;
  sd_logger.input_buf[11] = 0x01;
  sd_logger.input_buf[12] = 0xAA;
  // additional data from buffer is irrelevant

  // It matches, so should call ACMD41 with argument 0x40000000
  spi_submit_StubWithCallback(MySpiSubmitCallbackProcessCmd8CorrectResponse);
  sd_logger_process_CMD8(&sd_logger.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, MySpiSubmitCallbackProcessCmd8CorrectResponseNrCalls, "spi_submit call count mismatch.");
}
