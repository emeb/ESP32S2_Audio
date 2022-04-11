/*
 * eb_adc.h - my high-level ADC driver. Mimics background ADC operation.
 * 01-16-22 E. Brombaugh
 */

#ifndef __eb_adc__
#define __eb_adc__

#define ADC_NUMVALS 2
#define ADC_NUMPARAMS 4

extern volatile int16_t adc_val[ADC_NUMVALS], adc_param[ADC_NUMPARAMS];

esp_err_t eb_adc_init(void);
void eb_adc_setactparam(uint8_t idx);
void eb_adc_setparamval(uint8_t idx, int16_t val);
void eb_adc_forceactparam(void);

#endif
