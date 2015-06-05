#include "unity.h"
#include "filters/notch_filter.h"

struct SecondOrderNotchFilter acc_x_notch;
struct SecondOrderNotchFilter acc_x_notch_orig;

void setUp (void)
{
  acc_x_notch_orig = acc_x_notch;
}


void tearDown (void)
{
  acc_x_notch = acc_x_notch_orig;
}

void test_setSamplingFrequency (void)
{
  notch_filter_set_sampling_frequency(&acc_x_notch, 1000);
  TEST_ASSERT_EQUAL_FLOAT(0.001, acc_x_notch.Ts);
}

void test_setBandWidth (void)
{
  notch_filter_set_sampling_frequency(&acc_x_notch, 1000);
  notch_filter_set_bandwidth(&acc_x_notch, 10.);
  TEST_ASSERT_EQUAL_FLOAT(0.9391, acc_x_notch.d2); /* Verification from matlab */
}

void test_setFilterFrequency(void)
{
  notch_filter_set_sampling_frequency(&acc_x_notch, 1000);
  notch_filter_set_filter_frequency(&acc_x_notch, 60.5);
  char msg[10]; sprintf(msg, "%f", acc_x_notch.costheta);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0001, 0.9286, acc_x_notch.costheta, msg);
}

void test_updateFilter(void)
{
  notch_filter_set_sampling_frequency(&acc_x_notch, 1000);
  notch_filter_set_bandwidth(&acc_x_notch, 10.);
  notch_filter_set_filter_frequency(&acc_x_notch, 60.5);

  /* Verification values from matlab */
  int32_t x[6] = {-728, -1152, -1530, -1421, -775, 123};
  int32_t y[6] = {1046, 1070, 829, 748, 892, 1040};

  acc_x_notch.xn2 = x[0];
  acc_x_notch.xn1 = x[1];
  acc_x_notch.yn2 = y[0];
  acc_x_notch.yn1 = y[1];

  int32_t output;
  for (uint8_t i = 2; i<6; i++) {
    notch_filter_update(&acc_x_notch, &x[i], &output);
    TEST_ASSERT_EQUAL_INT32(y[i], output);
  }
}
