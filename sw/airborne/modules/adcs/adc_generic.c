#include "adc_generic.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/telemetry.h"
#include BOARD_CONFIG

uint16_t adc_generic_val1;
uint16_t adc_generic_val2;

#ifndef ADC_CHANNEL_GENERIC1
#ifndef ADC_CHANNEL_GENERIC2
#error "at least one ADC_CHANNEL_GENERIC1/2 needs to be defined to use the generic_adc module"
#endif
#endif

#ifndef ADC_CHANNEL_GENERIC_NB_SAMPLES
#define ADC_CHANNEL_GENERIC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif


#ifdef ADC_CHANNEL_GENERIC1
static struct adc_buf buf_generic1;
#endif

#ifdef ADC_CHANNEL_GENERIC2
static struct adc_buf buf_generic2;
#endif

static void send_adc_generic(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t adc_watikwil = buf_generic1.sum;
  pprz_msg_send_ADC_GENERIC(trans, dev, AC_ID,
                                  &adc_watikwil,
                                  &adc_generic_val2);
}

void adc_generic_init(void)
{
#ifdef ADC_CHANNEL_GENERIC1
  adc_buf_channel(ADC_CHANNEL_GENERIC1, &buf_generic1, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
#ifdef ADC_CHANNEL_GENERIC2
  adc_buf_channel(ADC_CHANNEL_GENERIC2, &buf_generic2, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ADC_GENERIC, send_adc_generic);
}

void adc_generic_periodic(void)
{
#ifdef ADC_CHANNEL_GENERIC1
  adc_generic_val1 = buf_generic1.sum / buf_generic1.av_nb_sample;
#endif
#ifdef ADC_CHANNEL_GENERIC2
  adc_generic_val2 = buf_generic2.sum / buf_generic2.av_nb_sample;
#endif

  //DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, DefaultDevice, &adc_generic_val1, &adc_generic_val2);
}

