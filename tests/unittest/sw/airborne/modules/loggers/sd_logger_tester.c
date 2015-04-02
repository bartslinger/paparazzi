#include "unity.h"
#include "subsystems/datalink/Mocktelemetry.h"
#include "peripherals/Mocksdcard.h"
#include "subsystems/Mockimu.h"
#include "loggers/sd_logger.h"

// Actually defined in sdcard.c
struct SdCard sdcard1;

// Actually defined in spi.c
struct spi_periph spi2;

// Actually defined in imu.c
struct Imu imu;

// Actually defined in uart.c
struct uart_periph uart1;

// Actually defined in pprz_transport.c
struct pprz_transport pprz_tp;



void setUp(void)
{
  Mocksdcard_Init();
}

void tearDown(void)
{
  Mocksdcard_Verify();
  Mocksdcard_Destroy();
}
