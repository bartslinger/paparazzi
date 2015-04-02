#include "unity.h"
#include "mcu_periph/Mockspi.h"
#include "peripherals/sdcard.h"


uint8_t SpiSubmitCall_SendCMD0NrCalls;
uint8_t SpiSubmitCall_SendDummyClockNrCalls;
uint8_t SpiSubmitCall_RequestNBytesNrCalls;
uint8_t SpiSubmitCall_SendCMD8NrCalls;
uint8_t SpiSubmitCall_SendACMDNrCalls;
uint8_t SpiSubmitCall_SendCMD58NrCalls;
uint8_t SpiSubmitCall_SendCMD16NrCalls;
uint8_t SpiSubmitCall_SendCMD24NrCalls;
uint8_t SpiSubmitCall_SendDataBlockNrCalls;
uint8_t SpiSubmitCall_SendCMD17NrCalls;
uint8_t SpiSubmitCall_ReadDataBlockNrCalls;

uint8_t NBytesToRequest;
uint8_t ACMD_CMD;
uint32_t ACMD_ARG;

// Actually declared in spi.c
struct spi_periph spi2;

// Actually declared in module (for example) that uses an sd card.
struct SdCard sdcard1;

struct SdCard sdcard_original;

void setUp(void)
{
  sdcard_original = sdcard1;
  SpiSubmitCall_SendCMD0NrCalls                     = 0;
  SpiSubmitCall_SendDummyClockNrCalls               = 0;
  SpiSubmitCall_RequestNBytesNrCalls                = 0;
  SpiSubmitCall_SendCMD8NrCalls                     = 0;
  SpiSubmitCall_SendACMDNrCalls                     = 0;
  SpiSubmitCall_SendCMD58NrCalls                    = 0;
  SpiSubmitCall_SendCMD16NrCalls                    = 0;
  SpiSubmitCall_SendCMD24NrCalls                    = 0;
  SpiSubmitCall_SendDataBlockNrCalls                = 0;
  SpiSubmitCall_SendCMD17NrCalls                    = 0;
  SpiSubmitCall_ReadDataBlockNrCalls                = 0;

  NBytesToRequest = 1;
  ACMD_CMD = 57;
  ACMD_ARG = 57;

  Mockspi_Init();

  // The init function should called before use of any other function
  sdcard_init(&sdcard1, &spi2, SPI_SLAVE3); // works also with other peripheral or slave
  sdcard1.response_counter = 57; // To make sure this gets set to zero everywhere
  sdcard1.timeout_counter = 5700; // To make sure this gets set to zero everywhere
}

void tearDown(void)
{
  sdcard1 = sdcard_original; // revert back to original state
  Mockspi_Verify();
  Mockspi_Destroy();
}

void test_SdCardInitializeStructInitialValues(void)
{
  // First, set some random variables to the values in the struct
  struct spi_periph random_spip;
  sdcard1.spi_p = &random_spip;
  sdcard1.status = 0x57;
  sdcard1.spi_t.slave_idx = 0x57;
  sdcard1.spi_t.select = 0x57;
  sdcard1.spi_t.status = 0x57;
  sdcard1.spi_t.cpol = 0x57;
  sdcard1.spi_t.cpha = 0x57;
  sdcard1.spi_t.dss = 0x57;
  sdcard1.spi_t.bitorder = 0x57;
  sdcard1.spi_t.cdiv = 0x57;
  sdcard1.spi_t.input_buf = NULL;
  sdcard1.spi_t.output_buf = NULL;
  sdcard1.spi_t.input_length = 57;
  sdcard1.spi_t.output_length = 57;
  sdcard1.card_type = 57;


  // Call the function
  sdcard_init(&sdcard1, &spi2, SPI_SLAVE3);

  // Then, verify the initial values after initialization are correct
  TEST_ASSERT_EQUAL_PTR(&spi2, sdcard1.spi_p);
  TEST_ASSERT_EQUAL(SPI_SLAVE3, sdcard1.spi_t.slave_idx);
  TEST_ASSERT_EQUAL(SPISelectUnselect, sdcard1.spi_t.select);
  TEST_ASSERT_EQUAL(SPITransDone, sdcard1.spi_t.status);
  TEST_ASSERT_EQUAL(SPICpolIdleLow, sdcard1.spi_t.cpol);
  TEST_ASSERT_EQUAL(SPICphaEdge1, sdcard1.spi_t.cpha);
  TEST_ASSERT_EQUAL(SPIDss8bit, sdcard1.spi_t.dss);
  TEST_ASSERT_EQUAL(SPIMSBFirst, sdcard1.spi_t.bitorder);
  TEST_ASSERT_EQUAL(SPIDiv64, sdcard1.spi_t.cdiv);
  TEST_ASSERT_EQUAL_PTR(&sdcard1.input_buf, sdcard1.spi_t.input_buf);
  TEST_ASSERT_EQUAL_PTR(&sdcard1.output_buf, sdcard1.spi_t.output_buf);
  TEST_ASSERT_EQUAL(0, sdcard1.spi_t.input_length);
  TEST_ASSERT_EQUAL(0, sdcard1.spi_t.output_length);
  TEST_ASSERT_EQUAL(SdCardType_Unknown, sdcard1.card_type);

  // Also, the state for upcoming periodic loop is set
  TEST_ASSERT_EQUAL(SdCard_BeforeDummyClock, sdcard1.status);
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
  sdcard1.status = SdCard_BeforeDummyClock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendDummyClock);

  // Call period loop
  sdcard_periodic(&sdcard1);

  TEST_ASSERT_EQUAL(SdCard_SendingDummyClock, sdcard1.status);

  // Also, test that nothing is done in the next periodic loop
  sdcard_periodic(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendDummyClockNrCalls, "spi_submit call count mismatch.");
}

void test_DoNothingWhenTransactionInProgress(void)
{
  sdcard1.status = SdCard_BeforeDummyClock;
  sdcard1.spi_t.status = SPITransPending;

  // Call the periodic loop
  sdcard_periodic(&sdcard1);

  // Test also for this status
  sdcard1.spi_t.status = SPITransRunning;

  // Call the periodic loop again
  sdcard_periodic(&sdcard1);
}

void test_DoNothingInPeriodicLoopWhenNotInitialized(void)
{
  sdcard1.status = SdCard_UnInit;

  // Call the periodic loop
  sdcard_periodic(&sdcard1);
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
  sdcard1.status = SdCard_SendingDummyClock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD0);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD0NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD0, sdcard1.status);
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
  sdcard1.status = SdCard_SendingCMD0;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); // Is already one because first byte has been requested
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD0Resp, sdcard1.status);
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
  sdcard1.status = SdCard_ReadingCMD0Resp;
  sdcard1.input_buf[0] = 0x01;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD8);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD8NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD8, sdcard1.status);
}

void helper_RequestFirstResponseByte(void)
{
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); // Is already one because first byte has been requested
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");

}

void helper_ResponseLater(void)
{
  sdcard1.response_counter = 3; // random
  sdcard1.input_buf[0] = 0xFF; // Not ready
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(4, sdcard1.response_counter);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

void helper_ResponseTimeout(uint8_t limit)
{
  sdcard1.response_counter = 1;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Delay is maximal 9 bytes, abort after this
  for (uint8_t i=0; i<limit; i++) {
    sdcard1.input_buf[0] = 0xFF; // Not ready

    // Run the callback function
    sdcard_spicallback(&sdcard1.spi_t);
  }
  // The last time, don't call spi_submit again. (therefore expect 8 instead of 9)
  TEST_ASSERT_EQUAL_MESSAGE(limit-1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_Error, sdcard1.status);
}

void test_PollingCMD0ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingCMD0Resp;
  helper_ResponseLater();
}

void test_PollingCMD0ResponseTimeout(void)
{
  sdcard1.status = SdCard_ReadingCMD0Resp;
  helper_ResponseTimeout(9);
}

//! Callback of CMD8
/*!
 * CMD8 sending has completed. Start polling bytes for response.
 */
void test_ReadySendingCMD8(void)
{
  sdcard1.status = SdCard_SendingCMD8;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); // Is already one because first byte has been requested
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD8Resp, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

void test_PollingCMD8ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingCMD8Resp;
  helper_ResponseLater();
}

//! When 0x01 received, the next four bytes is the 32bit parameter value
void test_PollingCMD8ResponseReady(void)
{
  sdcard1.status = SdCard_ReadingCMD8Resp;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes); NBytesToRequest = 4;
  sdcard1.response_counter = 5; // somewhat random
  sdcard1.input_buf[0] = 0x01;

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD8Parameter, sdcard1.status);
}

void test_PollingCMD8Timeout(void)
{
  sdcard1.status = SdCard_ReadingCMD8Resp;
  helper_ResponseTimeout(9);
}

//! Reading parameter in response to CMD8, 0x1AA mismatch case
void test_ReadCMD8ParameterMismatch(void)
{
  sdcard1.status = SdCard_ReadingCMD8Parameter;
  sdcard1.input_buf[0] = 0x00;
  sdcard1.input_buf[1] = 0x00;
  sdcard1.input_buf[2] = 0x01;
  sdcard1.input_buf[3] = 0xBB; // << Mismatch!

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL(SdCard_Error, sdcard1.status);
}

//! Reading parameter in response to CMD8, 0x1AA match case
void test_ReadCMD8ParameterMatch(void)
{
  sdcard1.status = SdCard_ReadingCMD8Parameter;
  sdcard1.input_buf[0] = 0x00;
  sdcard1.input_buf[1] = 0x00;
  sdcard1.input_buf[2] = 0x01;
  sdcard1.input_buf[3] = 0xAA; // << Match!

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_SendingACMD41v2, sdcard1.status);
  TEST_ASSERT_EQUAL(0, sdcard1.timeout_counter); // reset the timout counter for ACMD41
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
  sdcard1.status = SdCard_SendingACMD41v2;
  sdcard1.timeout_counter = 0;
  ACMD_CMD = 41;
  ACMD_ARG = 0x40000000;
  spi_submit_StubWithCallback(SpiSubmitCall_SendACMD);

  // Function is called in the periodic loop
  sdcard_periodic(&sdcard1);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendACMDNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(1, sdcard1.timeout_counter);
  // No need to change state to capture event
}

void test_ReadySendingACMD41v2(void)
{
  sdcard1.status = SdCard_SendingACMD41v2;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(1, sdcard1.response_counter); // Is already one because first byte has been requested
  TEST_ASSERT_EQUAL(SdCard_ReadingACMD41v2Resp, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

void test_PollingACMD41v2ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingACMD41v2Resp;
  helper_ResponseLater();
}

void test_PollingACMD41v2ResponseTimeout(void)
{
  sdcard1.status = SdCard_ReadingACMD41v2Resp;
  helper_ResponseTimeout(9);
}

//! ACMD41 response is 0x01, try again next periodic loop
void test_PollingACMD41v2Response0x01(void)
{
  sdcard1.timeout_counter = 0;
  sdcard1.status = SdCard_ReadingACMD41v2Resp;
  sdcard1.input_buf[0] = 0x01;

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_SendingACMD41v2, sdcard1.status);
}

//! ACMD41 command try only 500 times (this command checks status until it is initialized (or not))
void test_TryACMD41OnlyLimitedNumberOfTimes(void)
{
  sdcard1.status = SdCard_ReadingACMD41v2Resp;
  sdcard1.timeout_counter = 499; // Already tried 499 times
  sdcard1.input_buf[0] = 0x01; // Response is still not 0x00

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Error, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD58(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls; // ignore unused
  SpiSubmitCall_SendCMD58NrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  // R3 response
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x7A, t->output_buf[0]); // CMD byte
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // paramter bytes (ignored)
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); // Stop bit

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);


  return TRUE;
}

//! Sd card ready with initialization, start CMD58
void test_PollingACMD41v2Response0x00(void)
{
  sdcard1.status = SdCard_ReadingACMD41v2Resp;
  sdcard1.timeout_counter = 57; // Tried limited number of times, not exceeded timeout
  sdcard1.input_buf[0] = 0x00;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD58);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_SendingCMD58, sdcard1.status);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD58NrCalls, "spi_submit call count mismatch.");
}

void test_ReadySendingCMD58(void)
{
  sdcard1.status = SdCard_SendingCMD58;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD58Resp, sdcard1.status);
}

void test_PollingCMD58ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingCMD58Resp;
  helper_ResponseLater();
}

void test_PollingCMD58Timeout(void)
{
  sdcard1.status = SdCard_ReadingCMD58Resp;
  helper_ResponseTimeout(9);
}

//! CMD58 has responded with 0x00, then read the next 4 bytes (OCR register)
void test_PollingCMD58DataReady(void)
{
  sdcard1.status = SdCard_ReadingCMD58Resp;
  sdcard1.input_buf[0] = 0x00;
  sdcard1.response_counter = 3;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes); NBytesToRequest = 4;

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD58Parameter, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD16(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitCall_SendCMD16NrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  // R1 response
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x50, t->output_buf[0]); // CMD byte
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // paramter bytes
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x02, t->output_buf[3]); // force blocksize 512 bytes.
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); // Stop bit

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

//! Check 32-bit OCR register for CCS bit
void test_ReadCMD58ParameterCCSBitSet(void)
{
  sdcard1.status = SdCard_ReadingCMD58Parameter;
  sdcard1.input_buf[0] = 0xCF; // bit 30 and 31 set
  sdcard1.input_buf[1] = 0xFF;
  sdcard1.input_buf[2] = 0xFF;
  sdcard1.input_buf[3] = 0xFF; // last byte

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Idle, sdcard1.status);
  TEST_ASSERT_EQUAL(SdCardType_SdV2block, sdcard1.card_type);

}

void test_ReadCMD58ParameterCCSBitUnSet(void)
{
  sdcard1.status = SdCard_ReadingCMD58Parameter;
  sdcard1.input_buf[0] = 0x8F; // bit 30 (CCS) not set, bit 31 set
  sdcard1.input_buf[1] = 0xFF;
  sdcard1.input_buf[2] = 0xFF;
  sdcard1.input_buf[3] = 0xFF; // last byte
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD16);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_SendingCMD16, sdcard1.status);
  TEST_ASSERT_EQUAL(SdCardType_SdV1, sdcard1.card_type);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD16NrCalls, "spi_submit call count mismatch.");
}

//! If bit 31 is not set, the CCS bit is not valid. Abort initialization
void test_ReadCMD58ParameterBit31NotSet(void)
{
  sdcard1.status = SdCard_ReadingCMD58Parameter;
  sdcard1.input_buf[0] = 0x4F; // bit 31 not set (then bit 30 is not valid)
  sdcard1.input_buf[1] = 0xFF;
  sdcard1.input_buf[2] = 0xFF;
  sdcard1.input_buf[3] = 0xFF; // last byte

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Error, sdcard1.status);
  TEST_ASSERT_EQUAL(SdCardType_Unknown, sdcard1.card_type);
}

void test_ReadySendingCMD16(void)
{
  sdcard1.status = SdCard_SendingCMD16;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD16Resp, sdcard1.status);
}

void test_PollingCMD16ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingCMD16Resp;
  helper_ResponseLater();
}

void test_PollingCMD16Timeout(void)
{
  sdcard1.status = SdCard_ReadingCMD16Resp;
  helper_ResponseTimeout(9);
}

void test_PollingCMD16ResponseDataReady(void)
{
  sdcard1.status = SdCard_ReadingCMD16Resp;
  sdcard1.response_counter = 4;
  sdcard1.input_buf[0] = 0x00; // correct response = ready

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Idle, sdcard1.status);
}

void test_DoNotWriteDataIfNotIdle(void)
{
  sdcard1.status = SdCard_Error;

  // Call the write data function
  sdcard_write_block(&sdcard1, 0x00000000);

  // Expect zero calls to spi_submit
}

bool_t SpiSubmitCall_SendCMD24(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitCall_SendCMD24NrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv8, t->cdiv);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  // R1 response
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x58, t->output_buf[0]); // CMD byte
  if (sdcard1.card_type == SdCardType_SdV2block) {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // 4 bytes for the address
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x14, t->output_buf[4]);
  }
  else {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // 4 bytes for the address
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x28, t->output_buf[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  }
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); // Stop bit

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_WriteDataBlockWithBlockAddress(void)
{
  sdcard1.status = SdCard_Idle;
  sdcard1.card_type = SdCardType_SdV2block;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD24);

  // Call the write data function
  sdcard_write_block(&sdcard1, 0x00000014); // is decimal 20 * 512

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD24NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD24, sdcard1.status);
}


void test_WriteDataBlockWithByteAddress(void)
{
  sdcard1.status = SdCard_Idle;
  sdcard1.card_type = SdCardType_SdV2byte;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD24);

  // Call the write data function
  sdcard_write_block(&sdcard1, 0x00000014); // = decimal 20

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD24NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD24, sdcard1.status);
}

void test_ReadySendingCMD24(void) {
  sdcard1.status = SdCard_SendingCMD24;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD24Resp, sdcard1.status);
}

void test_PollingCMD24ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingCMD24Resp;
  helper_ResponseLater();
}

void test_PollingCMD24Timeout(void)
{
  sdcard1.status = SdCard_ReadingCMD24Resp;
  helper_ResponseTimeout(9);
}

//! When CMD24 responds, another dummy byte needs to be requested before the block with data is transferred
void test_PollingCMD24DataReady(void)
{
  sdcard1.status = SdCard_ReadingCMD24Resp;
  sdcard1.input_buf[0] = 0x00; // Ready
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_BeforeSendingDataBlock, sdcard1.status);
  // Value of the response counter does not matter any more
}

bool_t SpiSubmitCall_SendDataBlock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitCall_SendDataBlockNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(516, t->output_length);
  TEST_ASSERT_EQUAL(516, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0xFE, t->output_buf[0]); // Data Token
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[256]);
  TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[257]);
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[258]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[512]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[513]); // CRC byte 1
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[514]); // CRC byte 2
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[515]); // Request data response

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_SendDataBlock(void)
{
  sdcard1.status = SdCard_BeforeSendingDataBlock;
  spi_submit_StubWithCallback(SpiSubmitCall_SendDataBlock);

  for (uint16_t i=0; i<256; i++) {
    sdcard1.output_buf[i+1] = 0x00;
    sdcard1.output_buf[i+256+1] = i;
  }

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendDataBlockNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingDataBlock, sdcard1.status);
}

//!
void test_ReadySendingDataBlockAccepted(void)
{
  sdcard1.status = SdCard_SendingDataBlock;
  sdcard1.input_buf[515] = 0x05; // B00000101 = data accepted

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Busy, sdcard1.status);
}

void test_ReadySendingDataBlockRejected(void)
{
  sdcard1.status = SdCard_SendingDataBlock;
  sdcard1.input_buf[515] = 0x0D; // B00001101 = data rejected

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Error, sdcard1.status);
}

void test_RequestBytePeriodicallyWhileBusy(void)
{
  sdcard1.status = SdCard_Busy;
  sdcard1.input_buf[0] = 0x00; // LOW = busy
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Run the periodic function
  sdcard_periodic(&sdcard1);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_Busy, sdcard1.status);
}

void test_RevertToIdleWhenNoLongerBusy(void)
{
  sdcard1.status = SdCard_Busy;
  sdcard1.input_buf[0] = 0xFF; // line = high = no longer busy

  // Run the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Idle, sdcard1.status);
}

bool_t SpiSubmitCall_SendCMD17(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitCall_SendCMD17NrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv8, t->cdiv);
  TEST_ASSERT_EQUAL(6, t->output_length);
  TEST_ASSERT_EQUAL(6, t->input_length);  // R1 response
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  TEST_ASSERT_EQUAL_HEX8(0x51, t->output_buf[0]); // CMD byte
  if (sdcard1.card_type == SdCardType_SdV2block) {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // 4 bytes for the address
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[3]);  // is just 20
    TEST_ASSERT_EQUAL_HEX8(0x14, t->output_buf[4]);
  }
  else {
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[1]); // 4 bytes for the address
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x28, t->output_buf[3]); // is 20 * 512
    TEST_ASSERT_EQUAL_HEX8(0x00, t->output_buf[4]);
  }
  TEST_ASSERT_EQUAL_HEX8(0x01, t->output_buf[5]); // Stop bit

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_DoNotReadDataIfNotIdle(void)
{
  sdcard1.status = SdCard_Busy;

  // Call the read data function
  sdcard_read_block(&sdcard1, 0x00000000);

  // Expect zero calls to spi_submit
}

void test_ReadDataBlockWithBlockAddress(void)
{
  sdcard1.status = SdCard_Idle;
  sdcard1.card_type = SdCardType_SdV2block;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD17);

  // Call the read data function
  sdcard_read_block(&sdcard1, 0x00000014);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD17NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD17, sdcard1.status);
}


void test_ReadDataBlockWithByteAddress(void)
{
  sdcard1.status = SdCard_Idle;
  sdcard1.card_type = SdCardType_SdV2byte;
  spi_submit_StubWithCallback(SpiSubmitCall_SendCMD17);

  // Call the read data function
  sdcard_read_block(&sdcard1, 0x00000014);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_SendCMD17NrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_SendingCMD17, sdcard1.status);
}

void test_ReadySendingCMD17(void) {
  sdcard1.status = SdCard_SendingCMD17;
  helper_RequestFirstResponseByte();
  TEST_ASSERT_EQUAL(SdCard_ReadingCMD17Resp, sdcard1.status);
}

void test_PollingCMD17ResponseLater(void)
{
  sdcard1.status = SdCard_ReadingCMD17Resp;
  helper_ResponseLater();
}

void test_PollingCMD17Timeout(void)
{
  sdcard1.status = SdCard_ReadingCMD17Resp;
  helper_ResponseTimeout(9);
}

//! When CMD17 response is ready, switch to mode waiting for data token
void test_PollingCMD17DataReady(void)
{
  sdcard1.status = SdCard_ReadingCMD17Resp;
  sdcard1.input_buf[0] = 0x00; // data ready
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Call the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(0, sdcard1.timeout_counter); // reset the timout counter for data token response
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_WaitingForDataToken, sdcard1.status);
}

void test_PollDataTokenPeriodically(void)
{
  sdcard1.status = SdCard_WaitingForDataToken;
  sdcard1.timeout_counter = 5;
  spi_submit_StubWithCallback(SpiSubmitCall_RequestNBytes);

  // Call the periodic function
  sdcard_periodic(&sdcard1);

  TEST_ASSERT_EQUAL(6, sdcard1.timeout_counter);
  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_RequestNBytesNrCalls, "spi_submit call count mismatch.");
}

void test_PollingDataTokenTimeout(void)
{
  sdcard1.status = SdCard_WaitingForDataToken;
  sdcard1.timeout_counter = 499; // Already tried 499 times
  sdcard1.input_buf[0] = 0xFF; // Still no data token

  // Call the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Error, sdcard1.status);
}

void test_PollingDataTokenNotReady(void)
{
  sdcard1.status = SdCard_WaitingForDataToken;
  sdcard1.timeout_counter = 5;
  sdcard1.input_buf[0] = 0xFF; // Not ready

  // Call the callback function
  sdcard_spicallback(&sdcard1.spi_t);
  TEST_ASSERT_EQUAL(SdCard_WaitingForDataToken, sdcard1.status);
}

bool_t SpiSubmitCall_ReadDataBlock(struct spi_periph *p, struct spi_transaction *t, int cmock_num_calls)
{
  (void) p; (void) cmock_num_calls;
  SpiSubmitCall_ReadDataBlockNrCalls++;

  TEST_ASSERT_EQUAL(SPISelectUnselect, t->select);
  TEST_ASSERT_EQUAL(SPIDiv8, t->cdiv);
  TEST_ASSERT_EQUAL(512+2, t->output_length);
  TEST_ASSERT_EQUAL(512+2, t->input_length);
  TEST_ASSERT_EQUAL(SPITransDone, t->status);

  for (uint16_t i=0; i<512; i++) {
    TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[i]);
  }
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[512]); // CRC byte 1
  TEST_ASSERT_EQUAL_HEX8(0xFF, t->output_buf[513]); // CRC byte 2

  // Callback
  TEST_ASSERT_EQUAL_PTR(&sdcard_spicallback, t->after_cb);

  return TRUE;
}

void test_PollingDataTokenReady(void)
{
  sdcard1.status = SdCard_WaitingForDataToken;
  sdcard1.input_buf[0] = 0xFE; // Data token
  spi_submit_StubWithCallback(SpiSubmitCall_ReadDataBlock);

  // Call the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL_MESSAGE(1, SpiSubmitCall_ReadDataBlockNrCalls, "spi_submit call count mismatch.");
  TEST_ASSERT_EQUAL(SdCard_ReadingDataBlock, sdcard1.status);
}

void test_ReadDataBlockContent(void)
{
  sdcard1.status = SdCard_ReadingDataBlock;

  // Call the callback function
  sdcard_spicallback(&sdcard1.spi_t);

  TEST_ASSERT_EQUAL(SdCard_Idle, sdcard1.status);

}

void test_SendErrorMessage(void)
{
  //TEST_IGNORE();
}
