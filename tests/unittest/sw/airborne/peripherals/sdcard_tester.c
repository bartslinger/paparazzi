#include "unity.h"
#include "mcu_periph/Mockspi.h"
#include "peripherals/sdcard.h"


uint8_t SpiSubmitCall_SendCMD0NrCalls;
uint8_t SpiSubmitCall_SendDummyClockNrCalls;
uint8_t SpiSubmitCall_RequestNBytesNrCalls;
uint8_t SpiSubmitCall_SendCMD8NrCalls;
uint8_t SpiSubmitCall_SendACMDNrCalls;

uint8_t NBytesToRequest;
uint8_t ACMD_CMD;
uint32_t ACMD_ARG;

// Actually declared in spi.c
struct spi_periph spi2;

// Actually declared in module (for example) that uses an sd card.
struct SdCard sdcard;

struct SdCard sdcard_original;

void setUp(void)
{
  sdcard_original = sdcard;
  SpiSubmitCall_SendCMD0NrCalls                     = 0;
  SpiSubmitCall_SendDummyClockNrCalls               = 0;
  SpiSubmitCall_RequestNBytesNrCalls                = 0;
  SpiSubmitCall_SendCMD8NrCalls                     = 0;
  SpiSubmitCall_SendACMDNrCalls                     = 0;

  NBytesToRequest = 1;
  ACMD_CMD = 57;
  ACMD_ARG = 57;

  Mockspi_Init();

  // The init function should called before use of any other function
  sdcard_init(&sdcard, &spi2, SPI_SLAVE3); // works also with other peripheral or slave
  sdcard.response_counter = 57; // To make sure this gets set to zero everywhere
}

void tearDown(void)
{
  sdcard = sdcard_original; // revert back to original state
  Mockspi_Verify();
  Mockspi_Destroy();
}

void test_SdCardInitializeStructInitialValues(void)
{
  // First, set some random variables to the values in the struct
  struct spi_periph random_spip;
  sdcard.spi_p = &random_spip;
  sdcard.status = 0x57;
  sdcard.spi_t.slave_idx = 0x57;
  sdcard.spi_t.select = 0x57;
  sdcard.spi_t.status = 0x57;
  sdcard.spi_t.cpol = 0x57;
  sdcard.spi_t.cpha = 0x57;
  sdcard.spi_t.dss = 0x57;
  sdcard.spi_t.bitorder = 0x57;
  sdcard.spi_t.cdiv = 0x57;
  sdcard.spi_t.input_buf = NULL;
  sdcard.spi_t.output_buf = NULL;
  sdcard.spi_t.input_length = 57;
  sdcard.spi_t.output_length = 57;


  // Call the function
  sdcard_init(&sdcard, &spi2, SPI_SLAVE3);

  // Then, verify the initial values after initialization are correct
  TEST_ASSERT_EQUAL_PTR(&spi2, sdcard.spi_p);
  TEST_ASSERT_EQUAL(SPI_SLAVE3, sdcard.spi_t.slave_idx);
  TEST_ASSERT_EQUAL(SPISelectUnselect, sdcard.spi_t.select);
  TEST_ASSERT_EQUAL(SPITransDone, sdcard.spi_t.status);
  TEST_ASSERT_EQUAL(SPICpolIdleLow, sdcard.spi_t.cpol);
  TEST_ASSERT_EQUAL(SPICphaEdge1, sdcard.spi_t.cpha);
  TEST_ASSERT_EQUAL(SPIDss8bit, sdcard.spi_t.dss);
  TEST_ASSERT_EQUAL(SPIMSBFirst, sdcard.spi_t.bitorder);
  TEST_ASSERT_EQUAL(SPIDiv64, sdcard.spi_t.cdiv);
  TEST_ASSERT_EQUAL_PTR(&sdcard.input_buf, sdcard.spi_t.input_buf);
  TEST_ASSERT_EQUAL_PTR(&sdcard.output_buf, sdcard.spi_t.output_buf);
  TEST_ASSERT_EQUAL(0, sdcard.spi_t.input_length);
  TEST_ASSERT_EQUAL(0, sdcard.spi_t.output_length);

  // Also, the state for upcoming periodic loop is set
  TEST_ASSERT_EQUAL(SdCard_BeforeDummyClock, sdcard.status);
}

bool_t SpiSubmitCall_SendDummyClock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  SpiSubmitCall_SendDummyClockNrCalls++;

  /* First call, this sends 80 (>74) clock pulses with CS and MOSI high */
  TEST_ASSERT_EQUAL(SPINoSelect, t->select);
  TEST_ASSERT_EQUAL(10, t->output_length);
  TEST_ASSERT_EQUAL(0, t->input_length);

  for (uint8_t i=0; i<10; i++){
    TEST_ASSERT_EQUAL(0xFF, t->output_buf[i]);
  }

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

//! Initialize the SD card with special sequence
/*!
 * For the SD card to switch to SPI mode, both the CS and
 * the MOSI line need to be held HIGH while sending at least
 * 74 clock pulses. This is accomplished by using SPINoSelect
 * to keep the line high and sending 0xFF to keep MOSI
 * high.
 */
void test_SendDummyClockPulses(void)
{
  sdcard.status = SdCard_BeforeDummyClock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendDummyClock);

  // Call period loop
  sdcard_periodic(&sdcard);

  TEST_ASSERT_EQUAL(SdCard_SendingDummyClock, sdcard.status);

  // Also, test that nothing is done in the next periodic loop
  sdcard_periodic(&sdcard);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendDummyClockNrCalls, "spi_submit call count mismatch.");
}

void test_DoNothingWhenTransactionInProgress(void)
{
  sdcard.status = SdCard_BeforeDummyClock;
  sdcard.spi_t.status = SPITransPending;

  // Call the periodic loop
  sdcard_periodic(&sdcard);

  // Test also for this status
  sdcard.spi_t.status = SPITransRunning;

  // Call the periodic loop again
  sdcard_periodic(&sdcard);
}

void test_DoNothingInPeriodicLoopWhenNotInitialized(void)
{
  sdcard.status = SdCard_UnInit;

  // Call the periodic loop
  sdcard_periodic(&sdcard);
}

bool_t SpiSubmitCall_SendCMD0(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  SpiSubmitCall_SendCMD0NrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);

  TEST_ASSERT_EQUAL_HEX8(0x40, t->output_buf[0]); // CMD byte
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // paramter bytes (ignored)
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x95, t->output_buf[5]); // CRC7 for CMD0(0x00000000)

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_DummyClockPulsesCallback(void)
{
  sdcard.status = SdCard_SendingDummyClock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD0);

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD0NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD0, sdcard.status);
}

bool_t SpiSubmitCall_RequestNBytes(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused warning
  SpiSubmitCall_RequestNBytesNrCalls++;

  TEST_ASSERT_EQUAL(NBytesToRequest, t->output_length);
  TEST_ASSERT_EQUAL(NBytesToRequest, t->input_length);  // reading response later

  for (uint8_t i=0; i<NBytesToRequest; i++) {
    TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[i]);
  }

  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);
  return TRUE;
}

void test_ReadySendingCMD0(void)
{
  sdcard.status = SdCard_SendingCMD0;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard.response_counter); // Is already one because first byte has been requested
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD0Resp, sdcard.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

bool_t SpiSubmitCall_SendCMD8(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  SpiSubmitCall_SendCMD8NrCalls++;

  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);

  TEST_ASSERT_EQUAL_HEX8(0x48, t->output_buf[0]); // CMD8
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0xAA, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x87, t->output_buf[5]); // CRC7 for CMD8(0x000001AA)

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_PollingCMD0ResponseDataReady(void)
{
  sdcard.status = SdCard_ReadingCMD0Resp;
  sdcard.input_buf[0] = 0x01;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD8);

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD8NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD8, sdcard.status);
}

void test_PollingCMD0ResponseLater(void)
{
  sdcard.status = SdCard_ReadingCMD0Resp;
  sdcard.response_counter = 1;
  sdcard.input_buf[0] = 0xFF; // Not ready
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

void helper_ResponseTimeout(uint8_t limit)
{
  sdcard.response_counter = 1;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Delay is maximal 9 bytes, abort after this
  for (uint8_t i=0; i<limit; i++) {
    sdcard.input_buf[0] = 0xFF; // Not ready

    // Run the callback function
    sdcard_spicallback(&sdcard.spi_t);
  }
  // The last time, don't call spi_submit again. (therefore expect 8 instead of 9)
  TEST_ASSERT_EQUAL_MESSAGE(limit-1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_Error, sdcard.status);
}

void test_PollingCMD0ResponseTimeout(void)
{
  sdcard.status = SdCard_ReadingCMD0Resp;
  helper_ResponseTimeout(9);
}

//! Callback of CMD8
/*!
 * CMD8 sending has completed. Start polling bytes for response.
 */
void test_ReadySendingCMD8(void)
{
  sdcard.status = SdCard_SendingCMD8;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard.response_counter); // Is already one because first byte has been requested
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD8Resp, sdcard.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

void test_PollingCMD8ResponseLater(void)
{
  sdcard.status = SdCard_ReadingCMD8Resp;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);
  sdcard.response_counter = 5; // somewhat random
  sdcard.input_buf[0] = 0xFF;

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);
  TEST_ASSERT_EQUAL(6, sdcard.response_counter);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

//! When 0x01 received, the next four bytes is the 32bit parameter value
void test_PollingCMD8ResponseReady(void)
{
  sdcard.status = SdCard_ReadingCMD8Resp;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes); NBytesToRequest = 4;
  sdcard.response_counter = 5; // somewhat random
  sdcard.input_buf[0] = 0x01;

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD8Parameter, sdcard.status);
}

void test_PollingCMD8Timeout(void)
{
  sdcard.status = SdCard_ReadingCMD8Resp;
  helper_ResponseTimeout(9);
}

//! Reading parameter in response to CMD8, 0x1AA mismatch case
void test_ReadCMD8ParameterMismatch(void)
{
  sdcard.status = SdCard_ReadingCMD8Parameter;
  sdcard.input_buf[0] = 0x00;
  sdcard.input_buf[1] = 0x00;
  sdcard.input_buf[2] = 0x01;
  sdcard.input_buf[3] = 0xBB; // << Mismatch!

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);
  TEST_ASSERT_EQUAL(SdCard_Error, sdcard.status);
}

//! Reading parameter in response to CMD8, 0x1AA match case
void test_ReadCMD8ParameterMatch(void)
{
  sdcard.status = SdCard_ReadingCMD8Parameter;
  sdcard.input_buf[0] = 0x00;
  sdcard.input_buf[1] = 0x00;
  sdcard.input_buf[2] = 0x01;
  sdcard.input_buf[3] = 0xAA; // << Match!

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);

  TEST_ASSERT_EQUAL(SdCard_SendingACMD41v2, sdcard.status);
}

bool_t SpiSubmitCall_SendACMD(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused parameters
  SpiSubmitCall_SendACMDNrCalls++;

  // Perform ACMD call with argument ACMD_ARG
  TEST_ASSERT_EQUAL(6+8+1+6, t->output_length); // CMD55 + Ncr (max 8) + R1 + CMD41
  TEST_ASSERT_EQUAL(6+8+1+6, t->input_length);

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

  //ACMD_CMD with ACMD_ARG
  TEST_ASSERT_EQUAL_HEX8(0x40 | ACMD_CMD, t->output_buf[15]);
  TEST_ASSERT_EQUAL_HEX8(ACMD_ARG >> 24, t->output_buf[16]);
  TEST_ASSERT_EQUAL_HEX8(ACMD_ARG >> 16, t->output_buf[17]);
  TEST_ASSERT_EQUAL_HEX8(ACMD_ARG >> 8, t->output_buf[18]);
  TEST_ASSERT_EQUAL_HEX8(ACMD_ARG >> 0, t->output_buf[19]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[20]);

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_SendACMD41NextPeriodicLoop(void)
{
  sdcard.status = SdCard_SendingACMD41v2;
  ACMD_CMD = 41;
  ACMD_ARG = 0x40000000;
  spi_submit_StubWithCallback(SpiSubmitCall_SendACMD);

  // Function is called in the periodic loop
  sdcard_periodic(&sdcard);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendACMDNrCalls, "spi_submit call count mismatch.");
  // No need to change state to capture event
}

void test_ReadySendingACMD41v2(void)
{
  sdcard.status = SdCard_SendingACMD41v2;

  // Run the callback function
  sdcard_spicallback(&sdcard.spi_t);

}

void test_SendErrorMessage(void)
{
  TEST_IGNORE();
}
