#include "unity.h"
#include "subsystems/datalink/Mocktelemetry.h"
#include "Mockmessages_testable.h"
#include "subsystems/sensors/Mockrpm_sensor_arch.h"
#include "subsystems/sensors/rpm_sensor.h"


/* Actually defined in telemetry.c */
struct periodic_telemetry pprz_telemetry;
/* This function has no prototype in telemetry.h */
bool_t register_periodic_telemetry(struct periodic_telemetry *_pt, const char *_msg, telemetry_cb _cb) {
  (void) _pt; (void) _msg; (void) _cb; return TRUE;
}

struct RpmSensor rpm_sensor_original;


void setUp(void)
{
  rpm_sensor_original = rpm_sensor;
  Mockrpm_sensor_arch_Init();
}

void tearDown(void)
{
  Mockrpm_sensor_arch_Verify();
  Mockrpm_sensor_arch_Destroy();
  rpm_sensor = rpm_sensor_original;
}

void test_CallArchInitFunctionWhenInitializing(void)
{
  rpm_sensor_arch_init_Expect();
  rpm_sensor_init();
}

void test_SetPreviousCountAfterProcessingPulse(void)
{
  rpm_sensor_process_pulse(1234, 1);
  TEST_ASSERT_EQUAL(1234, rpm_sensor.previous_cnt);
}

void test_Count2LargerThanCount1WithoutOverflow(void)
{
  rpm_sensor.previous_cnt = 100;
  uint16_t cnt = 500;
  uint8_t overflow_cnt = 0;
  rpm_sensor_process_pulse(cnt, overflow_cnt);
  TEST_ASSERT_EQUAL_FLOAT(117.1875, rpm_sensor.motor_frequency);
}

void test_Count1LargerThanCount2WithOverflow(void)
{
  rpm_sensor.previous_cnt = 500;
  uint16_t cnt = 400;
  uint8_t overflow_cnt = 1;
  rpm_sensor_process_pulse(cnt, overflow_cnt);
  char msg[10]; sprintf(msg, "%f", rpm_sensor.motor_frequency);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(281250./65436/6, rpm_sensor.motor_frequency, msg);
}

/*
 * In this case, the period between two pulses is larger than one period.
 * It would be possible to measure larger values by including overflows,
 * but the frequency is already very low so it is set to zero.
 */
void test_Count2LargerThanCount1WithOverflow(void)
{
  rpm_sensor.previous_cnt = 500;
  uint16_t cnt = 600;
  uint8_t overflow_cnt = 1;
  rpm_sensor_process_pulse(cnt, overflow_cnt);
  char msg[10]; sprintf(msg, "%f", rpm_sensor.motor_frequency);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0., rpm_sensor.motor_frequency, msg);
}

/*
 * T1 > T2 would stil be valid with a single overflow, but with 2 or more overflows, set frequency to zero
 */
void test_Count1LargerThanCount2WithMultipleOverflow(void)
{
  rpm_sensor.previous_cnt = 500;
  uint16_t cnt = 400;
  uint8_t overflow_cnt = 2;
  rpm_sensor_process_pulse(cnt, overflow_cnt);
  char msg[10]; sprintf(msg, "%f", rpm_sensor.motor_frequency);
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0., rpm_sensor.motor_frequency, msg);
}
