/*
 * eb_adc.c - my high-level ADC driver. Mimics background ADC operation.
 * 01-16-22 E. Brombaugh
 */
 
#include <stdio.h>
#include "main.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "hal/i2s_hal.h"
#include "hal/i2s_types.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/spi_reg.h>
#include <soc/spi_struct.h>
#include "eb_adc.h"

static const char* TAG = "eb_adc";
static uint8_t adc_idx, adc_hyst_state, adc_param_idx;
static int16_t adc_hyst_val;
int32_t adc_iir[ADC_NUMVALS];
volatile int16_t adc_val[ADC_NUMVALS], adc_param[ADC_NUMPARAMS];

#define GET_UNIT(x)			((x>>3) & 0x1)
#define TIMES				8
#define CV0					ADC_CHANNEL_8
#define CV1					ADC_CHANNEL_9

/* uncomment this to do PIO (not DMA) */
#define ADC_PIO

#ifdef ADC_PIO
/*
 * low-level version of adc1_get_raw() - cuts time from 45us to 15us
 */
int16_t IRAM_ATTR my_adc1_get_raw(uint8_t chl)
{
	int16_t adc_value;
	
	SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << chl);
	/* note that this reg is undocumented! */
	while (HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_slave_addr1, meas_status) != 0) {}
	SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
	SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
    while (SENS.sar_meas1_ctrl2.meas1_done_sar == 0);
    adc_value = HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas1_ctrl2, meas1_data_sar);
    return adc_value;
}

/*
 * low-level version of adc1_get_raw() - cuts time from 45us to 15us
 */
void IRAM_ATTR my_adc1_get_raw_begin(uint8_t chl)
{
	SENS.sar_meas1_ctrl2.sar1_en_pad = (1 << chl);
	/* note that this reg is undocumented! */
	while (HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_slave_addr1, meas_status) != 0) {}
	SENS.sar_meas1_ctrl2.meas1_start_sar = 0;
	SENS.sar_meas1_ctrl2.meas1_start_sar = 1;
}

/*
 * fetch result as 13-bit
 */
int16_t IRAM_ATTR my_adc1_get_raw_end(void)
{
    while (SENS.sar_meas1_ctrl2.meas1_done_sar == 0);
    return 8191-HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas1_ctrl2, meas1_data_sar);
}

/*
 * IIR filter routine for 13-bit ADC values - convert to 12 bit on output
 */
#define IIR_COEF 4
inline int16_t adc_IIR_filter(int32_t *filt_state, int16_t in)
{
	*filt_state += ((in<<IIR_COEF) - *filt_state )>>IIR_COEF;
	return *filt_state >> (IIR_COEF+1);		// convert to 12-bit
}

/*
 * ADC timer callback
 */
void IRAM_ATTR adc_timer_callback(void *arg)
{
	/* set ISR active flag */
	//gpio_set_level(DIAG_1, 1);
	
#if 0
	/* wait for conversion */
	if(adc_idx==0)
		adc_val[0] = my_adc1_get_raw(ADC_CHANNEL_8); //adc1_get_raw(ADC_CHANNEL_8);
	else
		adc_val[1] = my_adc1_get_raw(ADC_CHANNEL_9); //adc1_get_raw(ADC_CHANNEL_9);
	
	adc_idx ^= 1;
#else
	/* don't wait for conversion */
	/* get previous result */
	if(adc_idx==0)
	{
		adc_val[0] = adc_IIR_filter(&adc_iir[0], my_adc1_get_raw_end());
	}
	else
	{
		adc_val[1] = adc_IIR_filter(&adc_iir[1], my_adc1_get_raw_end());
	
		/* update parameter */
		switch(adc_hyst_state)
		{
			case 0:	/* reset */
				adc_hyst_val = adc_val[1];
				adc_hyst_state = 1;
				break;
			
			case 1: /* locked & waiting for unlock */
				if(abs(adc_val[1]-adc_hyst_val) > 200)
					adc_hyst_state = 2;
				break;
			
			case 2:	/* tracking */
				adc_param[adc_param_idx] = adc_val[1];
				break;
		}
	}
	adc_idx ^= 1;
	
	/* start next one */
	if(adc_idx==0)
		my_adc1_get_raw_begin(CV0);
	else
		my_adc1_get_raw_begin(CV1);
#endif
	
	/* clear ISR active flag */
	//gpio_set_level(DIAG_1, 0);
}
#else
/*
 * ADC timer callback for DMA
 */
void IRAM_ATTR adc_timer_callback(void *arg)
{
	uint8_t result[4] = {0};
	esp_err_t ret;
    uint32_t ret_num = 0;
	int i;
	
	/* set ISR active flag */
	gpio_set_level(DIAG_1_1, 1);
	
	/* acquire a buffer of adc data */
	ret = adc_digi_read_bytes(result, 4, &ret_num, 1000);
	
#if 0
	/* filter it into the running array */
	for(i=0;i<2;i++)
		adc_val[i&1] += ((*(uint16_t *)(&result[2*i])&0xfff) - adc_val[i&1])>>4;
#else
	adc_val[0] = *(uint16_t *)(&result[0]);
	adc_val[1] = *(uint16_t *)(&result[2]);
#endif
	
	/* clear ISR active flag */
	gpio_set_level(DIAG_1_1, 0);
}
#endif

esp_err_t eb_adc_init(void)
{
	esp_timer_handle_t timer_handle;
	
	/* setup state */
	adc_idx = 0;
	adc_hyst_state = 0;
	adc_param_idx = 0;
	
#ifdef ADC_PIO
	/* single conversions w/ timer */
    ESP_LOGI(TAG, "PIO mode");
	adc1_config_width(ADC_WIDTH_BIT_13);
	adc1_config_channel_atten(CV0, ADC_ATTEN_DB_6);
	adc1_config_channel_atten(CV1, ADC_ATTEN_DB_6);
	adc1_get_raw(CV0);
#else
	/* multi conversions w/ DMA */
    ESP_LOGI(TAG, "DMA mode");
	uint16_t adc1_chan_mask = BIT(8) | BIT(9);
	adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 4*TIMES,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = 0,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));
	
    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = 0,
        .conv_limit_num = 250,
        .sample_freq_hz = 10 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
	
	adc_channel_t channel[2] = {ADC1_CHANNEL_8, ADC1_CHANNEL_9};
	adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 2;
    for (int i = 0; i < 2; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0xf;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = 0;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
    adc_digi_start();
#if 1
	uint32_t first_desc = GPSPI3.dma_in_link.addr | 0x3FF00000;
	printf("first desc: 0x%08X\n", first_desc);
	uint32_t next_desc = first_desc;
	/* walk the descriptor list */
	do
	{
		printf("head = 0x%08X\n", *((uint32_t *)next_desc));
		printf("buff = 0x%08X\n", *((uint32_t *)next_desc + 1));
		next_desc = *((uint32_t *)next_desc + 2);
		printf("next_desc = 0x%08X\n", next_desc);
	} while((next_desc != first_desc) && (next_desc!= 0));
	
	//while(1)
	//{
	//	printf("SPI3 in desc: 0x%08X\n", GPSPI3.dma_in_suc_eof_des_addr);
	//	vTaskDelay(1);
	//}
	//uint16_t *adc_buf = (uint16_t *)*((uint32_t *)first_desc + 1);
	//printf("adc_buf @ 0x%08X\n", (uint32_t)adc_buf);
#endif
#endif
	esp_timer_create_args_t timer_args = {
		.callback = adc_timer_callback,
		.arg = NULL,
		.name = "adc_timer",
		.skip_unhandled_events = true
	};
	esp_timer_create(&timer_args, &timer_handle);
	esp_timer_start_periodic(timer_handle, 1000);
	
	return ESP_OK;
}

/*
 * set adc[1] parameter destination
 */
void eb_adc_setactparam(uint8_t idx)
{
	if(idx>=ADC_NUMPARAMS)
		return;
	
	if(idx != adc_param_idx)
	{
		/* this has to be atomic */
		taskDISABLE_INTERRUPTS();
		adc_param_idx = idx;
		adc_hyst_state = 0;
		taskENABLE_INTERRUPTS();
	}
}

/*
 * set adc[1] parameter value
 */
void eb_adc_setparamval(uint8_t idx, int16_t val)
{
	if(idx>=ADC_NUMPARAMS)
		return;
	
	adc_param[idx] = val;
}

/*
 * force adc[1] active parameter hysteresis to track live value
 */
void eb_adc_forceactparam(void)
{
	adc_hyst_state = 2;
}
